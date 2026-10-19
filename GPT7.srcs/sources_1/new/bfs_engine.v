`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Member 1 - BFS engine (unweighted baseline shortest path)
//
//   - FIFO frontier queue implemented as a ring buffer in a grid_mem.
//   - visited bitmap + parent/predecessor memory for path reconstruction.
//   - Reports busy/done/found/no_path, path readback, and stats
//     (cycles, expansions, path cost / path cell count).
//
// Usage:
//   1. While the engine is idle, load the maze into its local cost RAM using
//      load_we / load_addr / load_data (4-bit cost, 0 = wall, 1..15 = cost).
//   2. Pulse `start` once with start_idx / goal_idx held.
//   3. Wait for `done`; read `found` / `no_path`, then read the path back
//      through path_rd_addr -> path_rd_data (path_cells entries).
//
// BFS ignores cost magnitudes and treats every non-zero cost cell as
// traversable with a single step, giving minimum-step routes.
//////////////////////////////////////////////////////////////////////////////
module bfs_engine #(
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
    output reg  [DIST_W-1:0]    path_cost,    // number of steps (edges)

    // live search view (node currently being expanded / popped)
    output reg  [INDEX_W-1:0]   current,

    // statistics
    output reg  [DIST_W-1:0]    cycles,
    output reg  [DIST_W-1:0]    expansions
);
    localparam NUM_CELLS = COLS * ROWS;
    localparam [INDEX_W-1:0] NONE = {INDEX_W{1'b1}};

    localparam [INDEX_W-1:0] COLS_W = COLS;
    localparam [INDEX_W-1:0] ROWS_W = ROWS;

    // FSM states
    localparam [2:0] S_IDLE        = 3'd0,
                     S_CLEAR       = 3'd1,
                     S_START_ENQ   = 3'd2,
                     S_POP         = 3'd3,
                     S_EXPAND      = 3'd4,
                     S_RECON_COUNT = 3'd5,
                     S_RECON_STORE = 3'd6,
                     S_DONE        = 3'd7;

    reg [2:0]        state;
    reg [INDEX_W-1:0] clr_addr, head, tail;
    reg [INDEX_W-1:0] s_idx, g_idx;
    reg [INDEX_W-1:0] recon_cur, recon_idx;
    reg [DIST_W-1:0]  recon_len;
    reg [1:0]         dir;
    reg [INDEX_W:0]   count;
    reg               start_d;

    // memory write ports
    reg               vis_we;      reg [INDEX_W-1:0] vis_waddr;  reg              vis_wdata;
    reg               par_we;      reg [INDEX_W-1:0] par_waddr;  reg [INDEX_W-1:0] par_wdata;
    reg               q_we;        reg [INDEX_W-1:0] q_waddr;    reg [INDEX_W-1:0] q_wdata;
    reg               path_we;     reg [INDEX_W-1:0] path_waddr; reg [INDEX_W-1:0] path_wdata;

    // memory read data
    wire              vis_rdata;
    wire [COST_W-1:0] cost_rdata;
    wire [INDEX_W-1:0] q_rdata, par_rdata, path_rdata;

    // -------- combinational neighbour generation (from `current`) ----------
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

    // -------- memories -----------------------------------------------------
    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_cost (
        .clk(clk), .we(load_we), .waddr(load_addr), .wdata(load_data),
        .raddr(nb_sel), .rdata(cost_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(1)) u_visited (
        .clk(clk), .we(vis_we), .waddr(vis_waddr), .wdata(vis_wdata),
        .raddr(nb_sel), .rdata(vis_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_parent (
        .clk(clk), .we(par_we), .waddr(par_waddr), .wdata(par_wdata),
        .raddr(recon_cur), .rdata(par_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_queue (
        .clk(clk), .we(q_we), .waddr(q_waddr), .wdata(q_wdata),
        .raddr(head), .rdata(q_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_path (
        .clk(clk), .we(path_we), .waddr(path_waddr), .wdata(path_wdata),
        .raddr(path_rd_addr), .rdata(path_rd_data)
    );

    // -------- FSM + registers ----------------------------------------------
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
            start_d     <= 1'b0;
        end else begin
            start_d <= start;

            // defaults: no memory writes this cycle
            vis_we  <= 1'b0;
            par_we  <= 1'b0;
            q_we    <= 1'b0;
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
                        head       <= 0;
                        tail       <= 0;
                        count      <= 0;
                        clr_addr   <= 0;
                        cycles     <= 0;
                        expansions <= 0;
                        path_cells <= 0;
                        path_cost  <= 0;
                        state      <= S_CLEAR;
                    end
                end

                S_CLEAR: begin
                    vis_we    <= 1'b1;
                    vis_waddr <= clr_addr;
                    vis_wdata <= 1'b0;
                    par_we    <= 1'b1;
                    par_waddr <= clr_addr;
                    par_wdata <= NONE;
                    if (clr_addr == NUM_CELLS - 1)
                        state <= S_START_ENQ;
                    else
                        clr_addr <= clr_addr + 1'b1;
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
                    tail      <= tail + 1'b1;
                    count     <= 1;
                    state     <= S_POP;
                end

                S_POP: begin
                    if (count == 0) begin
                        no_path <= 1'b1;
                        state   <= S_DONE;
                    end else begin
                        current    <= q_rdata;
                        head       <= head + 1'b1;
                        count      <= count - 1'b1;
                        expansions <= expansions + 1'b1;
                        if (q_rdata == g_idx) begin
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
                    if (nb_ok && !vis_rdata && (cost_rdata != 0)) begin
                        vis_we    <= 1'b1;
                        vis_waddr <= nb_sel;
                        vis_wdata <= 1'b1;
                        par_we    <= 1'b1;
                        par_waddr <= nb_sel;
                        par_wdata <= current;
                        q_we      <= 1'b1;
                        q_waddr   <= tail;
                        q_wdata   <= nb_sel;
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
                    if (recon_cur == s_idx) begin
                        path_cells <= recon_len + 1'b1;
                        path_cost  <= recon_len;
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
