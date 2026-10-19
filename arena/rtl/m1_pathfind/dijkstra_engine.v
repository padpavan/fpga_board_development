`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Member 1 - Dijkstra engine (weighted shortest path)
//
//   - distance RAM, visited set and parent/predecessor memory.
//   - minimum unsettled-distance selection by linear scan over all cells.
//   - non-negative cost relaxation:  d[v] <- min(d[v], d[u] + w(u,v)).
//   - predecessor-based path reconstruction.
//
// Usage identical to bfs_engine (load maze while idle, pulse `start`, wait for
// `done`).  Dijkstra uses the cell costs, so `path_cost` reports the minimum
// weighted cost while `path_cells` reports the number of cells on that route.
//////////////////////////////////////////////////////////////////////////////
module dijkstra_engine #(
    parameter COLS    = 16,
    parameter ROWS    = 12,
    parameter COST_W  = 4,
    parameter DIST_W  = 16,
    parameter INDEX_W = 8
) (
    input  wire                 clk,
    input  wire                 rst,

    // maze load interface (driven while idle)
    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    // run control
    input  wire                 start,
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    // status
    output reg                  busy,
    output reg                  done,
    output reg                  found,
    output reg                  no_path,

    // path readback (valid after done && found)
    input  wire [INDEX_W-1:0]   path_rd_addr,
    output wire [INDEX_W-1:0]   path_rd_data,
    output reg  [DIST_W-1:0]    path_cells,   // cells in path (goal inclusive)
    output reg  [DIST_W-1:0]    path_cost,    // minimum weighted cost

    // live search view (node currently selected / being relaxed)
    output wire [INDEX_W-1:0]   current,

    // statistics
    output reg  [DIST_W-1:0]    cycles,
    output reg  [DIST_W-1:0]    expansions
);
    localparam NUM_CELLS = COLS * ROWS;
    localparam [INDEX_W-1:0] NONE = {INDEX_W{1'b1}};
    localparam [DIST_W-1:0]  INF  = {DIST_W{1'b1}};

    localparam [INDEX_W-1:0] COLS_W = COLS;
    localparam [INDEX_W-1:0] ROWS_W = ROWS;

    // FSM states
    localparam [2:0] S_IDLE        = 3'd0,
                     S_INIT        = 3'd1,
                     S_SELECT      = 3'd2,
                     S_RELAX       = 3'd3,
                     S_RECON_COUNT = 3'd4,
                     S_RECON_STORE = 3'd5,
                     S_DONE        = 3'd6;

    reg [2:0]         state;
    reg [INDEX_W-1:0] clr_addr, scan_addr, best_idx, sel;
    reg [INDEX_W-1:0] s_idx, g_idx;
    reg [INDEX_W-1:0] recon_cur, recon_idx;
    reg [DIST_W-1:0]  best_dist, sel_dist, recon_len;
    reg [1:0]         dir;
    reg               start_d;

    // memory write ports
    reg               dist_we;    reg [INDEX_W-1:0] dist_waddr; reg [DIST_W-1:0]  dist_wdata;
    reg               vis_we;     reg [INDEX_W-1:0] vis_waddr;  reg               vis_wdata;
    reg               par_we;     reg [INDEX_W-1:0] par_waddr;  reg [INDEX_W-1:0] par_wdata;
    reg               path_we;    reg [INDEX_W-1:0] path_waddr; reg [INDEX_W-1:0] path_wdata;

    // memory read data
    wire               vis_rdata;
    wire [COST_W-1:0]  cost_rdata;
    wire [DIST_W-1:0]  dist_rdata;
    wire [INDEX_W-1:0] par_rdata, path_rdata;

    // -------- combinational neighbour generation (from `sel`) --------------
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

    // -------- read-address muxes -------------------------------------------
    wire [INDEX_W-1:0] dist_raddr = (state == S_SELECT) ? scan_addr : nb_sel;
    wire [INDEX_W-1:0] vis_raddr  = (state == S_SELECT) ? scan_addr : nb_sel;
    wire [INDEX_W-1:0] cost_raddr = nb_sel;
    wire [INDEX_W-1:0] par_raddr  = recon_cur;

    // -------- relaxation arithmetic ----------------------------------------
    wire [DIST_W-1:0] cost_ext = {{(DIST_W-COST_W){1'b0}}, cost_rdata};
    wire [DIST_W-1:0] new_dist = sel_dist + cost_ext;

    // -------- memories -----------------------------------------------------
    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_cost (
        .clk(clk), .we(load_we), .waddr(load_addr), .wdata(load_data),
        .raddr(cost_raddr), .rdata(cost_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(DIST_W)) u_dist (
        .clk(clk), .we(dist_we), .waddr(dist_waddr), .wdata(dist_wdata),
        .raddr(dist_raddr), .rdata(dist_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(1)) u_visited (
        .clk(clk), .we(vis_we), .waddr(vis_waddr), .wdata(vis_wdata),
        .raddr(vis_raddr), .rdata(vis_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_parent (
        .clk(clk), .we(par_we), .waddr(par_waddr), .wdata(par_wdata),
        .raddr(par_raddr), .rdata(par_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_path (
        .clk(clk), .we(path_we), .waddr(path_waddr), .wdata(path_wdata),
        .raddr(path_rd_addr), .rdata(path_rd_data)
    );

    assign current = sel;

    // -------- FSM + registers ----------------------------------------------
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
            start_d     <= 1'b0;
        end else begin
            start_d <= start;

            // defaults: no memory writes this cycle
            dist_we <= 1'b0;
            vis_we  <= 1'b0;
            par_we  <= 1'b0;
            path_we <= 1'b0;

            case (state)
                S_IDLE: begin
                    done    <= 1'b0;
                    found   <= 1'b0;
                    no_path <= 1'b0;
                    if (start && !start_d) begin
                        s_idx      <= start_idx;
                        g_idx      <= goal_idx;
                        busy       <= 1'b1;
                        clr_addr   <= 0;
                        scan_addr  <= 0;
                        best_dist  <= INF;
                        cycles     <= 0;
                        expansions <= 0;
                        path_cells <= 0;
                        path_cost  <= 0;
                        state      <= S_INIT;
                    end
                end

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
                    if (clr_addr == NUM_CELLS - 1) begin
                        scan_addr <= 0;
                        best_dist <= INF;
                        best_idx  <= 0;
                        state     <= S_SELECT;
                    end else begin
                        clr_addr <= clr_addr + 1'b1;
                    end
                end

                S_SELECT: begin
                    if (scan_addr < NUM_CELLS) begin
                        if (!vis_rdata && (dist_rdata < best_dist)) begin
                            best_dist <= dist_rdata;
                            best_idx  <= scan_addr;
                            sel_dist  <= dist_rdata;
                        end
                        scan_addr <= scan_addr + 1'b1;
                    end else begin
                        if (best_dist == INF) begin
                            no_path <= 1'b1;
                            state   <= S_DONE;
                        end else begin
                            sel         <= best_idx;
                            vis_we      <= 1'b1;
                            vis_waddr   <= best_idx;
                            vis_wdata   <= 1'b1;
                            expansions  <= expansions + 1'b1;
                            if (best_idx == g_idx) begin
                                found     <= 1'b1;
                                path_cost <= sel_dist;
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
                    if (nb_ok && !vis_rdata && (cost_rdata != 0)) begin
                        if (new_dist < dist_rdata) begin
                            dist_we    <= 1'b1;
                            dist_waddr <= nb_sel;
                            dist_wdata <= new_dist;
                            par_we     <= 1'b1;
                            par_waddr  <= nb_sel;
                            par_wdata  <= sel;
                        end
                    end
                    if (dir == 2'd3) begin
                        scan_addr <= 0;
                        best_dist <= INF;
                        state     <= S_SELECT;
                    end else begin
                        dir <= dir + 1'b1;
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
                    if (recon_cur == s_idx) begin
                        path_cells <= recon_len + 1'b1;
                        state      <= S_DONE;
                    end else begin
                        recon_idx <= recon_idx - 1'b1;
                        recon_cur <= par_rdata;
                    end
                end

                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            endcase

            if (state != S_IDLE && state != S_DONE)
                cycles <= cycles + 1'b1;
        end
    end
endmodule
