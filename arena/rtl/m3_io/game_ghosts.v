`timescale 1ns / 1ps
`include "arena_defs.vh"

// Game mode
module game_ghosts #(
    parameter COLS = `ARENA_COLS, parameter NUM_CELLS = `ARENA_NUM_CELLS,
    parameter INDEX_W = `ARENA_INDEX_W, parameter DIST_W = `ARENA_DIST_W,
    parameter RELEASE_CYCLES = 300_000_000
) (
    input  wire                   clk, input wire rst,
    input  wire                   spawn,
    input  wire [INDEX_W-1:0]     spawn_pos,
    input  wire [2:0]             nghost,
    input  wire                   commit_all,
    input  wire                   enable,
    input  wire                   tick,
    input  wire [4*INDEX_W-1:0]   target,
    input  wire [NUM_CELLS-1:0]   open,
    output reg                    eng_start,
    output reg  [INDEX_W-1:0]     eng_start_idx,
    output wire [INDEX_W-1:0]     eng_goal_idx,
    output reg  [INDEX_W-1:0]     eng_path_addr,
    input  wire                   eng_done,
    input  wire                   eng_found,
    input  wire [DIST_W-1:0]      eng_path_cells,
    input  wire [INDEX_W-1:0]     eng_path_data,
    output reg  [4*INDEX_W-1:0]   pos,
    output reg  [4*INDEX_W-1:0]   prev_pos,
    output wire [3:0]             present,
    output wire [3:0]             released,
    output reg                    moved,
    output wire                   idle
);
    localparam [2:0] S_PICK = 3'd0, S_START = 3'd1, S_WAIT = 3'd2, S_RESULT = 3'd3, S_COPY = 3'd4;
    localparam RW = $clog2(3*RELEASE_CYCLES + 2);
    reg [2:0]          st;
    reg [1:0]          cur, rr;
    reg [INDEX_W-1:0]  spos;
    reg [INDEX_W-1:0]  goal_r;
    reg                stale;
    reg [INDEX_W-1:0]  copy_last;
    reg [RW-1:0]       rel_cnt;
    reg [INDEX_W-1:0]  len [0:3];
    reg [INDEX_W-1:0]  rd  [0:3];

    assign present  = (nghost >= 4) ? 4'b1111 : (nghost == 3) ? 4'b0111 : (nghost == 2) ? 4'b0011 : 4'b0001;
    assign released = present & {rel_cnt >= 3*RELEASE_CYCLES, rel_cnt >= 2*RELEASE_CYCLES,
                                 rel_cnt >= RELEASE_CYCLES, 1'b1};
    assign idle     = (st == S_PICK) && !eng_start;
    assign eng_goal_idx = goal_r;

    function [INDEX_W-1:0] gpos; input [4*INDEX_W-1:0] v; input [1:0] g;
        gpos = v[g*INDEX_W +: INDEX_W];
    endfunction
    function adjacent; input [INDEX_W-1:0] a, b;
        adjacent = ((b == a + 1'b1) && (a % COLS != COLS-1)) || ((b + 1'b1 == a) && (a % COLS != 0)) ||
                   (b == a + COLS) || (b + COLS == a);
    endfunction

    wire [3:0]           have = {rd[3] != len[3], rd[2] != len[2], rd[1] != len[1], rd[0] != len[0]};
    wire [4*INDEX_W-1:0] nxt;
    wire                 buf_we = (st == S_COPY);
    wire [INDEX_W-1:0]   buf_wa = eng_path_addr - 1'b1;
    genvar gb;
    generate for (gb = 0; gb < 4; gb = gb + 1) begin : g_route
        grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_r (.clk(clk), .we(buf_we && cur == gb),
            .waddr(buf_wa), .wdata(eng_path_data), .raddr(rd[gb]), .rdata(nxt[gb*INDEX_W +: INDEX_W]));
    end endgenerate

    wire [3:0] want = released & (commit_all ? ~have : 4'b1111);
    wire [1:0] c0 = rr, c1 = rr + 2'd1, c2 = rr + 2'd2, c3 = rr + 2'd3;
    wire [1:0] pick = want[c0] ? c0 : want[c1] ? c1 : want[c2] ? c2 : c3;
    wire       any  = |want;

    wire [INDEX_W-1:0] cur_pos = gpos(pos, cur);
    wire result_ok = !stale && eng_found && (eng_path_cells >= 2) && (spos == cur_pos);
    wire [DIST_W-1:0] n_copy = commit_all ? eng_path_cells - 1'b1 : 1;

    integer g;
    always @(posedge clk) begin
        eng_start <= 1'b0;
        moved     <= 1'b0;
        if (rst) begin
            st <= S_PICK; cur <= 0; rr <= 0; spos <= 0; goal_r <= 0; stale <= 1'b0;
            copy_last <= 0; rel_cnt <= 0; pos <= 0; prev_pos <= 0; eng_start_idx <= 0;
            eng_path_addr <= 1;
            for (g = 0; g < 4; g = g + 1) begin len[g] <= 0; rd[g] <= 0; end
        end else begin
            if (enable && rel_cnt < 3*RELEASE_CYCLES) rel_cnt <= rel_cnt + 1'b1;

            case (st)
                S_PICK: if (enable && any && !spawn) begin
                    cur <= pick; spos <= gpos(pos, pick);
                    eng_start_idx <= gpos(pos, pick); goal_r <= gpos(target, pick);
                    stale <= 1'b0;
                    eng_start <= 1'b1;
                    st <= S_START;
                end
                S_START: st <= S_WAIT;
                S_WAIT:  if (eng_done) st <= S_RESULT;
                S_RESULT: begin
                    rr <= cur + 2'd1;
                    if (result_ok) begin
                        eng_path_addr <= 1;
                        copy_last <= n_copy[INDEX_W-1:0];
                        st <= S_COPY;
                    end else begin
                        len[cur] <= 0; rd[cur] <= 0;
                        st <= S_PICK;
                    end
                end
                default: begin
                    if (eng_path_addr == copy_last) begin
                        len[cur] <= copy_last; rd[cur] <= 0;
                        eng_path_addr <= 1;
                        st <= S_PICK;
                    end else
                        eng_path_addr <= eng_path_addr + 1'b1;
                end
            endcase

            if (spawn) begin
                for (g = 0; g < 4; g = g + 1) begin
                    pos[g*INDEX_W +: INDEX_W]      <= spawn_pos;
                    prev_pos[g*INDEX_W +: INDEX_W] <= spawn_pos;
                    len[g] <= 0; rd[g] <= 0;
                end
                rel_cnt <= 0; rr <= 0;
                if (st != S_PICK) stale <= 1'b1;
                if (st == S_COPY) st <= S_PICK;
            end else if (enable && tick) begin
                for (g = 0; g < 4; g = g + 1) begin
                    prev_pos[g*INDEX_W +: INDEX_W] <= gpos(pos, g);
                    if (released[g] && have[g] && !(st == S_COPY && cur == g)) begin
                        if (adjacent(gpos(pos, g), gpos(nxt, g)) && open[gpos(nxt, g)]) begin
                            pos[g*INDEX_W +: INDEX_W] <= gpos(nxt, g);
                            rd[g] <= rd[g] + 1'b1;
                            moved <= 1'b1;
                        end else begin
                            len[g] <= 0; rd[g] <= 0;
                        end
                    end
                end
            end
        end
    end
endmodule
