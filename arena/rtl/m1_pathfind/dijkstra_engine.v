`timescale 1ns / 1ps
`include "arena_defs.vh"

// Dijkstra engine
module dijkstra_engine #(
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

    output wire [INDEX_W-1:0]   current,
    input  wire [INDEX_W-1:0]   cell_rd_addr,
    output wire [1:0]           cell_state,

    output reg  [CNT_W-1:0]     cycles,
    output reg  [DIST_W-1:0]    expansions
);
    localparam NUM_CELLS = COLS * ROWS;
    localparam [INDEX_W-1:0] NONE     = {INDEX_W{1'b1}};
    localparam [DIST_W-1:0]  INF      = {DIST_W{1'b1}};
    localparam [CNT_W-1:0]   CNT_MAX  = {CNT_W{1'b1}};
    localparam [DIST_W-1:0]  DIST_MAX = {DIST_W{1'b1}};

    localparam [INDEX_W-1:0] COLS_W = COLS;
    localparam [INDEX_W-1:0] ROWS_W = ROWS;

    localparam [3:0] S_IDLE        = 4'd0,
                     S_INIT        = 4'd1,
                     S_SELECT      = 4'd2,
                     S_RELAX       = 4'd3,
                     S_RECON_COUNT = 4'd4,
                     S_RECON_STORE = 4'd5,
                     S_FINISH      = 4'd6,
                     S_RELAX_WR    = 4'd7,
                     S_SEED        = 4'd8;

    reg [3:0]         state;
    reg [INDEX_W-1:0] clr_addr, sel;
    reg [INDEX_W-1:0] s_idx, g_idx;
    reg [INDEX_W-1:0] recon_cur, recon_idx;
    reg [DIST_W-1:0]  sel_dist, recon_len;
    reg [1:0]         dir;
    reg               ep_wall;

    reg               rx_ok;
    reg [INDEX_W-1:0] rx_idx;
    reg [DIST_W-1:0]  rx_new, rx_old;

    reg               dist_we;    reg [INDEX_W-1:0] dist_waddr; reg [DIST_W-1:0]  dist_wdata;
    reg               vis_we;     reg [INDEX_W-1:0] vis_waddr;  reg               vis_wdata;
    reg               par_we;     reg [INDEX_W-1:0] par_waddr;  reg [INDEX_W-1:0] par_wdata;
    reg               path_we;    reg [INDEX_W-1:0] path_waddr; reg [INDEX_W-1:0] path_wdata;
    reg               cs_we;      reg [INDEX_W-1:0] cs_waddr;   reg [1:0]         cs_wdata;

    wire               vis_rdata;
    wire [COST_W-1:0]  cost_rdata;
    wire [DIST_W-1:0]  dist_rdata;
    wire [INDEX_W-1:0] par_rdata;

    wire [INDEX_W-1:0] sel_row = sel / COLS;
    wire [INDEX_W-1:0] sel_col = sel % COLS;

    wire [INDEX_W-1:0] nb_sel =
        (dir == 2'd0) ? (sel - COLS_W) :
        (dir == 2'd1) ? (sel + COLS_W) :
        (dir == 2'd2) ? (sel - 1'b1)   :
                        (sel + 1'b1);

    wire nb_ok =
        (dir == 2'd0) ? (sel_row > 0)          :
        (dir == 2'd1) ? (sel_row < (ROWS - 1)) :
        (dir == 2'd2) ? (sel_col > 0)          :
                        (sel_col < (COLS - 1));

    wire [INDEX_W-1:0] cost_raddr = (state == S_INIT)   ? clr_addr  : nb_sel;
    wire [INDEX_W-1:0] par_raddr  = recon_cur;

    wire ep_hit = ((clr_addr == s_idx) || (clr_addr == g_idx)) &&
                  (cost_rdata == `ARENA_COST_WALL);
    wire ep_out = (s_idx >= NUM_CELLS) || (g_idx >= NUM_CELLS);

    wire [DIST_W-1:0] cost_ext = {{(DIST_W-COST_W){1'b0}}, cost_rdata};
    wire [DIST_W-1:0] new_dist = sel_dist + cost_ext;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_cost (
        .clk(clk), .we(load_we), .waddr(load_addr), .wdata(load_data),
        .raddr(cost_raddr), .rdata(cost_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(DIST_W)) u_dist (
        .clk(clk), .we(dist_we && step_en), .waddr(dist_waddr), .wdata(dist_wdata),
        .raddr(nb_sel), .rdata(dist_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(1)) u_visited (
        .clk(clk), .we(vis_we && step_en), .waddr(vis_waddr), .wdata(vis_wdata),
        .raddr(nb_sel), .rdata(vis_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_parent (
        .clk(clk), .we(par_we && step_en), .waddr(par_waddr), .wdata(par_wdata),
        .raddr(par_raddr), .rdata(par_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_path (
        .clk(clk), .we(path_we && step_en), .waddr(path_waddr), .wdata(path_wdata),
        .raddr(path_rd_addr), .rdata(path_rd_data)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(`ARENA_CS_W)) u_cell_state (
        .clk(clk), .we(cs_we && step_en), .waddr(cs_waddr), .wdata(cs_wdata),
        .raddr(cell_rd_addr), .rdata(cell_state)
    );

    wire               heap_ready, heap_empty;
    wire [INDEX_W-1:0] heap_min_idx;
    wire [DIST_W-1:0]  heap_min_key;

    wire init_last   = (state == S_INIT) && (clr_addr == NUM_CELLS - 1);
    wire rx_improve  = rx_ok && (rx_new < rx_old);
    wire heap_clr    = init_last;
    wire heap_push   = (state == S_SEED) ||
                       ((state == S_RELAX_WR) && rx_improve && heap_ready);
    wire heap_upd    = (state == S_RELAX_WR) && (rx_old != INF);
    wire [INDEX_W-1:0] heap_push_idx = (state == S_SEED) ? s_idx : rx_idx;
    wire [DIST_W-1:0]  heap_push_key = (state == S_SEED) ? {DIST_W{1'b0}} : rx_new;
    wire heap_pop    = (state == S_SELECT) && heap_ready && !heap_empty;

    dij_min_heap #(.NUM_CELLS(NUM_CELLS), .INDEX_W(INDEX_W), .DIST_W(DIST_W)) u_heap (
        .clk(clk), .rst(rst), .step_en(step_en),
        .clr(heap_clr),
        .push(heap_push), .push_upd(heap_upd),
        .push_idx(heap_push_idx), .push_key(heap_push_key),
        .pop(heap_pop),
        .ready(heap_ready), .empty(heap_empty),
        .min_idx(heap_min_idx), .min_key(heap_min_key)
    );

    assign current = sel;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            found       <= 1'b0;
            no_path     <= 1'b0;
            sel         <= 0;
            path_cells  <= 0;
            path_cost   <= 0;
            cycles      <= 0;
            expansions  <= 0;
            dist_we     <= 1'b0;
            vis_we      <= 1'b0;
            par_we      <= 1'b0;
            path_we     <= 1'b0;
            cs_we       <= 1'b0;
        end else begin
            if (step_en) begin
                dist_we <= 1'b0;
                vis_we  <= 1'b0;
                par_we  <= 1'b0;
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
                    clr_addr   <= 0;
                    ep_wall    <= 1'b0;
                    cycles     <= 0;
                    expansions <= 0;
                    path_cells <= 0;
                    path_cost  <= 0;
                    state      <= S_INIT;
                end
            end else if (step_en) begin
                case (state)
                    S_INIT: begin
                        dist_we    <= 1'b1;
                        dist_waddr <= clr_addr;
                        dist_wdata <= (clr_addr == s_idx) ? {DIST_W{1'b0}} : INF;
                        vis_we     <= 1'b1;
                        vis_waddr  <= clr_addr;
                        vis_wdata  <= 1'b0;
                        par_we     <= 1'b1;
                        par_waddr  <= clr_addr;
                        par_wdata  <= NONE;
                        cs_we      <= 1'b1;
                        cs_waddr   <= clr_addr;
                        cs_wdata   <= (clr_addr == s_idx) ? `ARENA_CS_FRONTIER
                                                          : `ARENA_CS_UNSEEN;
                        if (ep_hit)
                            ep_wall <= 1'b1;
                        if (clr_addr == NUM_CELLS - 1) begin
                            if (ep_wall || ep_hit || ep_out) begin
                                no_path <= 1'b1;
                                state   <= S_FINISH;
                            end else begin
                                state   <= S_SEED;
                            end
                        end else begin
                            clr_addr <= clr_addr + 1'b1;
                        end
                    end

                    S_SEED: begin
                        state <= S_SELECT;
                    end

                    S_SELECT: begin
                        if (heap_ready) begin
                            if (heap_empty) begin
                                no_path <= 1'b1;
                                state   <= S_FINISH;
                            end else begin
                                sel       <= heap_min_idx;
                                sel_dist  <= heap_min_key;
                                vis_we    <= 1'b1;
                                vis_waddr <= heap_min_idx;
                                vis_wdata <= 1'b1;
                                cs_we     <= 1'b1;
                                cs_waddr  <= heap_min_idx;
                                cs_wdata  <= `ARENA_CS_VISITED;
                                if (expansions != DIST_MAX)
                                    expansions <= expansions + 1'b1;
                                if (heap_min_idx == g_idx) begin
                                    found     <= 1'b1;
                                    path_cost <= heap_min_key;
                                    recon_cur <= g_idx;
                                    recon_len <= 0;
                                    state     <= S_RECON_COUNT;
                                end else begin
                                    dir   <= 0;
                                    state <= S_RELAX;
                                end
                            end
                        end
                    end

                    S_RELAX: begin
                        rx_ok  <= nb_ok && !vis_rdata && (cost_rdata != `ARENA_COST_WALL);
                        rx_idx <= nb_sel;
                        rx_new <= new_dist;
                        rx_old <= dist_rdata;
                        state  <= S_RELAX_WR;
                    end

                    S_RELAX_WR: begin
                        if (!rx_improve || heap_ready) begin
                            if (rx_improve) begin
                                dist_we    <= 1'b1;
                                dist_waddr <= rx_idx;
                                dist_wdata <= rx_new;
                                par_we     <= 1'b1;
                                par_waddr  <= rx_idx;
                                par_wdata  <= sel;
                                cs_we      <= 1'b1;
                                cs_waddr   <= rx_idx;
                                cs_wdata   <= `ARENA_CS_FRONTIER;
                            end
                            if (dir == 2'd3) begin
                                state <= S_SELECT;
                            end else begin
                                dir   <= dir + 1'b1;
                                state <= S_RELAX;
                            end
                        end
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
