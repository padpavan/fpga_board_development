`timescale 1ns / 1ps
`include "arena_defs.vh"

// BFS engine
module bfs_engine #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter COST_W  = `ARENA_COST_W,
    parameter DIST_W  = `ARENA_DIST_W,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter CNT_W   = `ARENA_CNT_W
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 step_en,

    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    input  wire                 start,
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    output reg                  busy,
    output reg                  done,
    output reg                  found,
    output reg                  no_path,

    input  wire [INDEX_W-1:0]   path_rd_addr,
    output wire [INDEX_W-1:0]   path_rd_data,
    output reg  [DIST_W-1:0]    path_cells,
    output reg  [DIST_W-1:0]    path_cost,

    output reg  [INDEX_W-1:0]   current,
    input  wire [INDEX_W-1:0]   cell_rd_addr,
    output wire [1:0]           cell_state,

    output reg  [CNT_W-1:0]     cycles,
    output reg  [DIST_W-1:0]    expansions
);
    localparam NUM_CELLS = COLS * ROWS;
    localparam [INDEX_W-1:0] NONE     = {INDEX_W{1'b1}};
    localparam [CNT_W-1:0]   CNT_MAX  = {CNT_W{1'b1}};
    localparam [DIST_W-1:0]  DIST_MAX = {DIST_W{1'b1}};

    localparam [INDEX_W-1:0] COLS_W = COLS;
    localparam [INDEX_W-1:0] ROWS_W = ROWS;

    localparam [2:0] S_IDLE        = 3'd0,
                     S_CLEAR       = 3'd1,
                     S_START_ENQ   = 3'd2,
                     S_POP         = 3'd3,
                     S_EXPAND      = 3'd4,
                     S_RECON_COUNT = 3'd5,
                     S_RECON_STORE = 3'd6,
                     S_FINISH      = 3'd7;

    reg [2:0]        state;
    reg [INDEX_W-1:0] clr_addr, head, tail;
    reg [INDEX_W-1:0] s_idx, g_idx;
    reg [INDEX_W-1:0] recon_cur, recon_idx;
    reg [DIST_W-1:0]  recon_len;
    reg [1:0]         dir;
    reg [INDEX_W:0]   count;
    reg               ep_wall;

    reg               vis_we;      reg [INDEX_W-1:0] vis_waddr;  reg              vis_wdata;
    reg               par_we;      reg [INDEX_W-1:0] par_waddr;  reg [INDEX_W-1:0] par_wdata;
    reg               q_we;        reg [INDEX_W-1:0] q_waddr;    reg [INDEX_W-1:0] q_wdata;
    reg               path_we;     reg [INDEX_W-1:0] path_waddr; reg [INDEX_W-1:0] path_wdata;
    reg               cs_we;       reg [INDEX_W-1:0] cs_waddr;   reg [1:0]         cs_wdata;

    wire              vis_rdata;
    wire [COST_W-1:0] cost_rdata;
    wire [INDEX_W-1:0] q_rdata, par_rdata;

    wire [INDEX_W-1:0] q_head = (q_we && q_waddr == head) ? q_wdata : q_rdata;

    wire [INDEX_W-1:0] cur_row = current / COLS;
    wire [INDEX_W-1:0] cur_col = current % COLS;

    wire [INDEX_W-1:0] nb_sel =
        (dir == 2'd0) ? (current - COLS_W) :
        (dir == 2'd1) ? (current + COLS_W) :
        (dir == 2'd2) ? (current - 1'b1)   :
                        (current + 1'b1);

    wire nb_ok =
        (dir == 2'd0) ? (cur_row > 0)          :
        (dir == 2'd1) ? (cur_row < (ROWS - 1)) :
        (dir == 2'd2) ? (cur_col > 0)          :
                        (cur_col < (COLS - 1));

    wire [INDEX_W-1:0] cost_raddr = (state == S_CLEAR) ? clr_addr : nb_sel;
    wire ep_hit = ((clr_addr == s_idx) || (clr_addr == g_idx)) &&
                  (cost_rdata == `ARENA_COST_WALL);
    wire ep_out = (s_idx >= NUM_CELLS) || (g_idx >= NUM_CELLS);

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_cost (
        .clk(clk), .we(load_we), .waddr(load_addr), .wdata(load_data),
        .raddr(cost_raddr), .rdata(cost_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(1)) u_visited (
        .clk(clk), .we(vis_we && step_en), .waddr(vis_waddr), .wdata(vis_wdata),
        .raddr(nb_sel), .rdata(vis_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_parent (
        .clk(clk), .we(par_we && step_en), .waddr(par_waddr), .wdata(par_wdata),
        .raddr(recon_cur), .rdata(par_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_queue (
        .clk(clk), .we(q_we && step_en), .waddr(q_waddr), .wdata(q_wdata),
        .raddr(head), .rdata(q_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_path (
        .clk(clk), .we(path_we && step_en), .waddr(path_waddr), .wdata(path_wdata),
        .raddr(path_rd_addr), .rdata(path_rd_data)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(`ARENA_CS_W)) u_cell_state (
        .clk(clk), .we(cs_we && step_en), .waddr(cs_waddr), .wdata(cs_wdata),
        .raddr(cell_rd_addr), .rdata(cell_state)
    );

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            found       <= 1'b0;
            no_path     <= 1'b0;
            current     <= 0;
            path_cells  <= 0;
            path_cost   <= 0;
            cycles      <= 0;
            expansions  <= 0;
            vis_we      <= 1'b0;
            par_we      <= 1'b0;
            q_we        <= 1'b0;
            path_we     <= 1'b0;
            cs_we       <= 1'b0;
        end else begin
            if (step_en) begin
                vis_we  <= 1'b0;
                par_we  <= 1'b0;
                q_we    <= 1'b0;
                path_we <= 1'b0;
                cs_we   <= 1'b0;
            end

            if (busy && step_en && cycles != CNT_MAX)
                cycles <= cycles + 1'b1;

            if (state == S_IDLE) begin
                if (start) begin
                    s_idx      <= start_idx;
                    g_idx      <= goal_idx;
                    busy       <= 1'b1;
                    done       <= 1'b0;
                    found      <= 1'b0;
                    no_path    <= 1'b0;
                    head       <= 0;
                    tail       <= 0;
                    count      <= 0;
                    clr_addr   <= 0;
                    ep_wall    <= 1'b0;
                    cycles     <= 0;
                    expansions <= 0;
                    path_cells <= 0;
                    path_cost  <= 0;
                    state      <= S_CLEAR;
                end
            end else if (step_en) begin
                case (state)
                    S_CLEAR: begin
                        vis_we    <= 1'b1;
                        vis_waddr <= clr_addr;
                        vis_wdata <= 1'b0;
                        par_we    <= 1'b1;
                        par_waddr <= clr_addr;
                        par_wdata <= NONE;
                        cs_we     <= 1'b1;
                        cs_waddr  <= clr_addr;
                        cs_wdata  <= `ARENA_CS_UNSEEN;
                        if (ep_hit)
                            ep_wall <= 1'b1;
                        if (clr_addr == NUM_CELLS - 1) begin
                            if (ep_wall || ep_hit || ep_out) begin
                                no_path <= 1'b1;
                                state   <= S_FINISH;
                            end else begin
                                state   <= S_START_ENQ;
                            end
                        end else begin
                            clr_addr <= clr_addr + 1'b1;
                        end
                    end

                    S_START_ENQ: begin
                        vis_we    <= 1'b1;
                        vis_waddr <= s_idx;
                        vis_wdata <= 1'b1;
                        par_we    <= 1'b1;
                        par_waddr <= s_idx;
                        par_wdata <= NONE;
                        q_we      <= 1'b1;
                        q_waddr   <= tail;
                        q_wdata   <= s_idx;
                        cs_we     <= 1'b1;
                        cs_waddr  <= s_idx;
                        cs_wdata  <= `ARENA_CS_FRONTIER;
                        tail      <= tail + 1'b1;
                        count     <= 1;
                        state     <= S_POP;
                    end

                    S_POP: begin
                        if (count == 0) begin
                            no_path <= 1'b1;
                            state   <= S_FINISH;
                        end else begin
                            current  <= q_head;
                            head     <= head + 1'b1;
                            count    <= count - 1'b1;
                            cs_we    <= 1'b1;
                            cs_waddr <= q_head;
                            cs_wdata <= `ARENA_CS_VISITED;
                            if (expansions != DIST_MAX)
                                expansions <= expansions + 1'b1;
                            if (q_head == g_idx) begin
                                found     <= 1'b1;
                                recon_cur <= g_idx;
                                recon_len <= 0;
                                state     <= S_RECON_COUNT;
                            end else begin
                                dir   <= 0;
                                state <= S_EXPAND;
                            end
                        end
                    end

                    S_EXPAND: begin
                        if (nb_ok && !vis_rdata && (cost_rdata != `ARENA_COST_WALL)) begin
                            vis_we    <= 1'b1;
                            vis_waddr <= nb_sel;
                            vis_wdata <= 1'b1;
                            par_we    <= 1'b1;
                            par_waddr <= nb_sel;
                            par_wdata <= current;
                            q_we      <= 1'b1;
                            q_waddr   <= tail;
                            q_wdata   <= nb_sel;
                            cs_we     <= 1'b1;
                            cs_waddr  <= nb_sel;
                            cs_wdata  <= `ARENA_CS_FRONTIER;
                            tail      <= tail + 1'b1;
                            count     <= count + 1'b1;
                        end
                        if (dir == 2'd3)
                            state <= S_POP;
                        else
                            dir <= dir + 1'b1;
                    end

                    S_RECON_COUNT: begin
                        if (recon_cur != s_idx) begin
                            recon_len <= recon_len + 1'b1;
                            recon_cur <= par_rdata;
                        end else begin
                            recon_cur <= g_idx;
                            recon_idx <= recon_len;
                            state     <= S_RECON_STORE;
                        end
                    end

                    S_RECON_STORE: begin
                        path_we    <= 1'b1;
                        path_waddr <= recon_idx;
                        path_wdata <= recon_cur;
                        cs_we      <= 1'b1;
                        cs_waddr   <= recon_cur;
                        cs_wdata   <= `ARENA_CS_PATH;
                        if (recon_cur == s_idx) begin
                            path_cells <= recon_len + 1'b1;
                            path_cost  <= recon_len;
                            state      <= S_FINISH;
                        end else begin
                            recon_idx <= recon_idx - 1'b1;
                            recon_cur <= par_rdata;
                        end
                    end

                    S_FINISH: begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end
endmodule
