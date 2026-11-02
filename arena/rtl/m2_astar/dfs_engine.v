`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Member 2 - DFS engine (depth-first search, the "blind" baseline)
//
// Same port list and handshake as every engine (CONTRACT section 4).
//
// How it searches
//   - An explicit stack holds the current branch: stack[0] = start,
//     stack[sp] = the cell being expanded (`current`).
//   - S_PROBE tries one direction per clock in the fixed order
//     up, down, left, right (dir 0..3, the same order as the other engines).
//     The first open, unvisited neighbour is pushed and probing restarts at
//     dir 0 on that new cell.
//   - If all four directions fail the cell is a dead end: it is popped
//     (cell_state VISITED) and its parent is probed again from dir 0.
//     Already-visited neighbours are skipped, so no per-level direction
//     memory is needed.
//   - The search stops the moment the goal is pushed.  The stack then IS
//     the path (start .. goal), so there is no parent RAM and no walk-back:
//     a second copy of the stack RAM (u_stack_path) is the path RAM read by
//     path_rd_addr.  S_MARK just walks the stack once to colour the path
//     cells PATH and add up path_cost.
//
// DFS returns *a* path, not a shortest one (CONTRACT 4.5).
//
// Metrics (CONTRACT 4.3)
//   expansions = cells pushed (each becomes `current` once; start and goal
//                included).
//   path_cost  = sum of cost[v] over the cells entered after start
//                (weighted, like Dijkstra / A*).
//   cycles     = CLEAR (NUM_CELLS) + PUSH_START (1) + 1 per probe
//                + MARK (path_cells) + FINISH (1).  ref/arena_ref.py dfs()
//                reproduces it exactly.
//
// cell_state: FRONTIER = on the current stack branch, VISITED = backtracked
// dead end, PATH = final path.
//
// Memory write ports are registered and gated by step_en (same scheme as
// bfs_engine): a write issued on one step lands on the next step edge.
//////////////////////////////////////////////////////////////////////////////
module dfs_engine #(
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

    // maze load interface (only while not busy)
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
    output reg  [DIST_W-1:0]    path_cells,
    output reg  [DIST_W-1:0]    path_cost,

    // live search view
    output reg  [INDEX_W-1:0]   current,
    input  wire [INDEX_W-1:0]   cell_rd_addr,
    output wire [1:0]           cell_state,

    // statistics
    output reg  [CNT_W-1:0]     cycles,
    output reg  [DIST_W-1:0]    expansions
);
    localparam NUM_CELLS = COLS * ROWS;
    localparam [INDEX_W-1:0] NONE      = {INDEX_W{1'b1}};
    localparam [CNT_W-1:0]   CNT_MAX   = {CNT_W{1'b1}};
    localparam [DIST_W-1:0]  DIST_MAX  = {DIST_W{1'b1}};
    localparam [INDEX_W-1:0] COLS_W    = COLS;
    localparam [COST_W-1:0]  WALL      = `ARENA_COST_WALL;
    localparam [1:0]         CS_UNSEEN = `ARENA_CS_UNSEEN;
    localparam [1:0]         CS_FRONT  = `ARENA_CS_FRONTIER;
    localparam [1:0]         CS_VISIT  = `ARENA_CS_VISITED;
    localparam [1:0]         CS_PATH   = `ARENA_CS_PATH;

    // FSM states
    localparam [2:0] S_IDLE       = 3'd0,
                     S_CLEAR      = 3'd1,
                     S_PUSH_START = 3'd2,
                     S_PROBE      = 3'd3,
                     S_MARK       = 3'd4,
                     S_FINISH     = 3'd5;

    reg [2:0]         state;
    reg [INDEX_W-1:0] clr_addr;
    reg [INDEX_W-1:0] s_idx, g_idx;
    reg [INDEX_W-1:0] sp;          // depth of the top of the stack
    reg [INDEX_W-1:0] mark_ptr;    // S_MARK walk position
    reg [1:0]         dir;
    reg               ep_wall;     // start or goal seen as a wall during S_CLEAR

    // memory write ports (registered)
    reg               vis_we;  reg [INDEX_W-1:0] vis_waddr;  reg               vis_wdata;
    reg               stk_we;  reg [INDEX_W-1:0] stk_waddr;  reg [INDEX_W-1:0] stk_wdata;
    reg               cs_we;   reg [INDEX_W-1:0] cs_waddr;   reg [1:0]         cs_wdata;

    // memory read data
    wire              vis_rdata;
    wire [COST_W-1:0] cost_rdata;
    wire [INDEX_W-1:0] stk_rdata;

    // -------- neighbour of `current` in direction `dir` --------------------
    wire [INDEX_W-1:0] cur_row = current / COLS;
    wire [INDEX_W-1:0] cur_col = current % COLS;

    wire [INDEX_W-1:0] nb_sel = (dir == 2'd0) ? (current - COLS_W) :
                                (dir == 2'd1) ? (current + COLS_W) :
                                (dir == 2'd2) ? (current - 1'b1)   :
                                                (current + 1'b1);

    wire nb_ok = (dir == 2'd0) ? (cur_row > 0)          :
                 (dir == 2'd1) ? (cur_row < (ROWS - 1)) :
                 (dir == 2'd2) ? (cur_col > 0)          :
                                 (cur_col < (COLS - 1));

    // -------- stack read: parent on a pop, or the S_MARK walk --------------
    // A pending (registered) write to the same slot is forwarded.  It only
    // happens for start == goal: S_PUSH_START writes stack[0] and S_MARK
    // reads it on the very next step.
    wire [INDEX_W-1:0] stk_raddr = (state == S_MARK) ? mark_ptr : (sp - 1'b1);
    wire [INDEX_W-1:0] stk_q     = (stk_we && (stk_waddr == stk_raddr)) ? stk_wdata : stk_rdata;

    // S_MARK reads the cost of the stacked cell through its own RAM copy
    // (u_cost_mark), so the probe path (sp -> stack read -> cost read ->
    // can_push -> sp) doesn't exist for static timing.
    wire [INDEX_W-1:0] cost_raddr = (state == S_CLEAR) ? clr_addr : nb_sel;
    wire [COST_W-1:0]  mark_cost;

    wire can_push = nb_ok && !vis_rdata && (cost_rdata != WALL);

    // -------- start / goal validity (checked during S_CLEAR) ---------------
    wire ep_hit = ((clr_addr == s_idx) || (clr_addr == g_idx)) && (cost_rdata == WALL);
    wire ep_out = (s_idx >= NUM_CELLS) || (g_idx >= NUM_CELLS);

    // -------- memories -----------------------------------------------------
    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_cost (
        .clk(clk), .we(load_we), .waddr(load_addr), .wdata(load_data),
        .raddr(cost_raddr), .rdata(cost_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_cost_mark (
        .clk(clk), .we(load_we), .waddr(load_addr), .wdata(load_data),
        .raddr(stk_q), .rdata(mark_cost)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(1)) u_visited (
        .clk(clk), .we(vis_we && step_en), .waddr(vis_waddr), .wdata(vis_wdata),
        .raddr(nb_sel), .rdata(vis_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_stack (
        .clk(clk), .we(stk_we && step_en), .waddr(stk_waddr), .wdata(stk_wdata),
        .raddr(stk_raddr), .rdata(stk_rdata)
    );

    // second copy of the stack = path RAM (addr 0 = start ... goal)
    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_stack_path (
        .clk(clk), .we(stk_we && step_en), .waddr(stk_waddr), .wdata(stk_wdata),
        .raddr(path_rd_addr), .rdata(path_rd_data)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(2)) u_cell_state (
        .clk(clk), .we(cs_we && step_en), .waddr(cs_waddr), .wdata(cs_wdata),
        .raddr(cell_rd_addr), .rdata(cell_state)
    );

    // -------- FSM ----------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            busy       <= 1'b0;
            done       <= 1'b0;
            found      <= 1'b0;
            no_path    <= 1'b0;
            current    <= NONE;
            path_cells <= 0;
            path_cost  <= 0;
            cycles     <= 0;
            expansions <= 0;
            vis_we     <= 1'b0;
            stk_we     <= 1'b0;
            cs_we      <= 1'b0;
        end else begin
            // default: no memory write on the next step (held while paused)
            if (step_en) begin
                vis_we <= 1'b0;
                stk_we <= 1'b0;
                cs_we  <= 1'b0;
            end

            // cycles: every step after start, up to and including the
            // cycle done rises (busy is still 1 on that edge)
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
                    current    <= NONE;
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
                    // clear visited + cell_state, check the endpoints
                    S_CLEAR: begin
                        vis_we    <= 1'b1;
                        vis_waddr <= clr_addr;
                        vis_wdata <= 1'b0;
                        cs_we     <= 1'b1;
                        cs_waddr  <= clr_addr;
                        cs_wdata  <= CS_UNSEEN;
                        if (ep_hit)
                            ep_wall <= 1'b1;
                        if (clr_addr == NUM_CELLS - 1) begin
                            if (ep_wall || ep_hit || ep_out) begin
                                no_path <= 1'b1;
                                state   <= S_FINISH;
                            end else begin
                                state   <= S_PUSH_START;
                            end
                        end else begin
                            clr_addr <= clr_addr + 1'b1;
                        end
                    end

                    // stack[0] = start
                    S_PUSH_START: begin
                        vis_we     <= 1'b1;
                        vis_waddr  <= s_idx;
                        vis_wdata  <= 1'b1;
                        stk_we     <= 1'b1;
                        stk_waddr  <= 0;
                        stk_wdata  <= s_idx;
                        cs_we      <= 1'b1;
                        cs_waddr   <= s_idx;
                        cs_wdata   <= CS_FRONT;
                        current    <= s_idx;
                        sp         <= 0;
                        dir        <= 0;
                        expansions <= 1;
                        if (s_idx == g_idx) begin
                            found    <= 1'b1;
                            mark_ptr <= 0;
                            state    <= S_MARK;
                        end else begin
                            state    <= S_PROBE;
                        end
                    end

                    // try one direction of the top cell
                    S_PROBE: begin
                        if (can_push) begin
                            vis_we    <= 1'b1;
                            vis_waddr <= nb_sel;
                            vis_wdata <= 1'b1;
                            stk_we    <= 1'b1;
                            stk_waddr <= sp + 1'b1;
                            stk_wdata <= nb_sel;
                            cs_we     <= 1'b1;
                            cs_waddr  <= nb_sel;
                            cs_wdata  <= CS_FRONT;
                            current   <= nb_sel;
                            sp        <= sp + 1'b1;
                            dir       <= 0;
                            if (expansions != DIST_MAX)
                                expansions <= expansions + 1'b1;
                            if (nb_sel == g_idx) begin
                                found    <= 1'b1;
                                mark_ptr <= 0;
                                state    <= S_MARK;
                            end
                        end else if (dir == 2'd3) begin
                            // dead end: pop
                            cs_we    <= 1'b1;
                            cs_waddr <= current;
                            cs_wdata <= CS_VISIT;
                            dir      <= 0;
                            if (sp == 0) begin
                                no_path <= 1'b1;
                                state   <= S_FINISH;
                            end else begin
                                sp      <= sp - 1'b1;
                                current <= stk_q;      // stack[sp-1]
                            end
                        end else begin
                            dir <= dir + 1'b1;
                        end
                    end

                    // walk the stack: colour PATH, add up the cost
                    S_MARK: begin
                        cs_we    <= 1'b1;
                        cs_waddr <= stk_q;
                        cs_wdata <= CS_PATH;
                        if (mark_ptr != 0)
                            path_cost <= path_cost + mark_cost;
                        if (mark_ptr == sp) begin
                            path_cells <= sp + 1'b1;
                            state      <= S_FINISH;
                        end else begin
                            mark_ptr <= mark_ptr + 1'b1;
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
