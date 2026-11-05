`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Member 2 - A* search engine  (CONTRACT.md section 4 port list, unchanged)
//
// Finds a minimum-cost 4-connected path on the 16x12 cost grid, using
//     f = g + h,   h = Manhattan distance to the goal (manhattan.v)
//
// Building blocks (each has its own self-checking testbench):
//     ast_open_set    priority queue of discovered cells, pops lowest f
//                     (then highest g, then lowest index)
//     ast_neighbour   cell + direction -> neighbour cell, in-grid flag
//     ast_relax       g_new / f_new / "is this a better route?" for a neighbour
//     manhattan       heuristic h(v, goal)
//     ast_sat_counter cycles / expansions statistics (CONTRACT 4.3)
//
// Private memories (common/grid_mem, async-read LUTRAM):
//     cost     maze copy, written only by the load bus
//     g        best known cost from start   (only trusted if cell is seen)
//     parent   predecessor on that best route
//     cs_eng   2-bit search state, the engine's own read port
//     cs_vga   identical copy (same writes) whose read port is cell_rd_addr,
//              so VGA can read at any time without disturbing the search
//     path     reconstructed path, stored goal-first (see "path readback")
//
// FSM (every state advances only when step_en = 1):
//
//   IDLE/DONE --start--> INIT      clear cs_* to UNSEEN, clear open set
//                        CHK_S     start on a wall?            -> FAIL
//                        CHK_G     goal on a wall?             -> FAIL
//                        SEED      g[s]=0, push s with f = h(s)
//                     +> POP       request the best open cell
//                     |  POP_WAIT  set empty                   -> FAIL
//                     |            else close it: u = cell, expansions++
//                     |            u == goal                   -> RECON
//                     |  NB x4     evaluate up/down/left/right neighbour of u,
//                     |            register the relax result
//                     |  NB_WR     write g/parent/state, push if improved
//                     +----------- after dir 3
//                        RECON     walk parent[] from goal to start,
//                                  mark PATH, write path RAM   -> DONE (found)
//                        FAIL      -> DONE (no_path)
//
// NB / NB_WR are split so the long read -> add -> compare chain (neighbour
// index, cost/state/g LUTRAM reads, g_new, g_new < g_old) ends in a register
// instead of running on into the open-set and RAM write enables: one cycle
// at 100 MHz could not hold both.
//
// Closed cells are never reopened: Manhattan distance is consistent when
// every cost >= 1, so the first time a cell is popped its g is optimal.
// ref/arena_ref.py astar() mirrors these exact rules, so the expected
// path_cells and expansions are known exactly for every test maze.
//////////////////////////////////////////////////////////////////////////////
module astar_engine #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter COST_W  = `ARENA_COST_W,
    parameter DIST_W  = `ARENA_DIST_W,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter CNT_W   = `ARENA_CNT_W
) (
    input  wire                 clk,
    input  wire                 rst,          // sync, active-high
    input  wire                 step_en,      // 1 = run, 0 = freeze

    // maze load (only while not busy)
    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    // run control
    input  wire                 start,        // 1-cycle pulse
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    // status
    output reg                  busy    = 1'b0,
    output reg                  done    = 1'b0,
    output reg                  found   = 1'b0,
    output reg                  no_path = 1'b0,

    // path readback: addr 0 = start ... addr path_cells-1 = goal
    input  wire [INDEX_W-1:0]   path_rd_addr,
    output wire [INDEX_W-1:0]   path_rd_data,
    output wire [DIST_W-1:0]    path_cells,
    output wire [DIST_W-1:0]    path_cost,

    // live view for VGA
    output wire [INDEX_W-1:0]   current,
    input  wire [INDEX_W-1:0]   cell_rd_addr,
    output wire [1:0]           cell_state,

    // statistics
    output wire [CNT_W-1:0]     cycles,
    output wire [DIST_W-1:0]    expansions,

    // debug (CONTRACT 0: optional dbg_ outputs, may be left unconnected)
    output wire [3:0]           dbg_state
);
    localparam NUM_CELLS = COLS * ROWS;
    localparam [INDEX_W-1:0] LAST = NUM_CELLS - 1;
    localparam [INDEX_W-1:0] NONE = `ARENA_IDX_NONE;

    // Shared codes from arena_defs.vh, given plain names for the code below.
    localparam [COST_W-1:0] COST_WALL   = `ARENA_COST_WALL;
    localparam [1:0]        CS_UNSEEN   = `ARENA_CS_UNSEEN,
                            CS_FRONTIER = `ARENA_CS_FRONTIER,
                            CS_VISITED  = `ARENA_CS_VISITED,
                            CS_PATH     = `ARENA_CS_PATH;

    // ---------------------------------------------------------------- FSM --
    localparam [3:0] S_IDLE     = 4'd0,
                     S_INIT     = 4'd1,
                     S_CHK_S    = 4'd2,
                     S_CHK_G    = 4'd3,
                     S_SEED     = 4'd4,
                     S_POP      = 4'd5,
                     S_POP_WAIT = 4'd6,
                     S_NB       = 4'd7,
                     S_RECON    = 4'd8,
                     S_FAIL     = 4'd9,
                     S_DONE     = 4'd10,
                     S_NB_WR    = 4'd11;

    reg [3:0]         state = S_IDLE;
    reg [INDEX_W-1:0] ptr   = {INDEX_W{1'b0}};  // INIT clear address
    reg [INDEX_W-1:0] s_idx = {INDEX_W{1'b0}};  // start, sampled on `start`
    reg [INDEX_W-1:0] t_idx = {INDEX_W{1'b0}};  // goal (target), sampled on `start`
    reg [INDEX_W-1:0] cur   = {INDEX_W{1'b0}};  // u: node being expanded
    reg [DIST_W-1:0]  g_cur = {DIST_W{1'b0}};   // g[u]
    reg [1:0]         dir   = 2'd0;             // neighbour being relaxed
    reg [INDEX_W-1:0] rc_cur = {INDEX_W{1'b0}}; // reconstruction walker
    reg [DIST_W-1:0]  rc_len = {DIST_W{1'b0}};  // cells written so far
    reg [DIST_W-1:0]  path_cells_r = {DIST_W{1'b0}};
    reg [DIST_W-1:0]  path_cost_r  = {DIST_W{1'b0}};
    reg [INDEX_W-1:0] current_r    = NONE;

    // Neighbour result registered in NB, used by NB_WR.
    reg [INDEX_W-1:0] nb_v_r   = {INDEX_W{1'b0}};
    reg               nb_imp_r = 1'b0;
    reg [DIST_W-1:0]  nb_g_r   = {DIST_W{1'b0}};
    reg [DIST_W-1:0]  nb_f_r   = {DIST_W{1'b0}};

    assign path_cells = path_cells_r;
    assign path_cost  = path_cost_r;
    assign current    = current_r;
    assign dbg_state  = state;

    // ------------------------------------------------- memory read data --
    wire [COST_W-1:0]  cost_rdata;
    wire [DIST_W-1:0]  g_rdata;
    wire [INDEX_W-1:0] par_rdata;
    wire [1:0]         cs_rdata;

    // --------------------------------------------- open set connections --
    wire               os_ready, os_pop_valid, os_pop_empty;
    wire [INDEX_W-1:0] os_pop_idx;

    // ------------------------------------------- neighbour of u in `dir` --
    // Registered one cycle early, from the values cur/dir are about to take
    // (POP_WAIT takes a cell -> its dir 0; NB_WR advances dir), so in NB the
    // neighbour index comes straight from flops instead of cur/dir -> adder
    // -> LUTRAM reads -> relax.  Same cycle count; NB is always entered from
    // one of those two updates, so nb_v is always for the current cur/dir.
    wire               pw_take = (state == S_POP_WAIT) && os_pop_valid && !os_pop_empty;
    wire               nw_adv  = step_en && (state == S_NB_WR) && (os_ready || !nb_imp_r);
    wire [INDEX_W-1:0] cur_nx  = pw_take ? os_pop_idx : cur;
    wire [1:0]         dir_nx  = pw_take ? 2'd0 : nw_adv ? dir + 1'b1 : dir;
    wire [INDEX_W-1:0] nb_v_nx;
    wire               nb_valid_nx;
    reg  [INDEX_W-1:0] nb_v     = {INDEX_W{1'b0}};
    reg                nb_valid = 1'b0;

    ast_neighbour #(.COLS(COLS), .ROWS(ROWS), .INDEX_W(INDEX_W)) u_nb (
        .u_idx(cur_nx), .dir(dir_nx), .v_idx(nb_v_nx), .v_valid(nb_valid_nx)
    );
    always @(posedge clk) begin
        nb_v     <= nb_v_nx;
        nb_valid <= nb_valid_nx;
    end

    // ------------------------------------------------ heuristic h(x, goal) --
    // SEED needs h(start); every NB cycle needs h(neighbour).
    wire [INDEX_W-1:0] h_idx = (state == S_SEED) ? s_idx : nb_v;
    wire [DIST_W-1:0]  h;

    manhattan #(.COLS(COLS), .ROWS(ROWS), .INDEX_W(INDEX_W), .DIST_W(DIST_W)) u_h (
        .a_idx(h_idx), .b_idx(t_idx), .h(h)
    );

    // ------------------------------------------- relaxation of neighbour --
    wire [DIST_W-1:0] rx_g_new, rx_f_new;
    wire              rx_improve;

    ast_relax #(.COST_W(COST_W), .DIST_W(DIST_W)) u_rx (
        .g_u(g_cur), .cost_v(cost_rdata), .v_valid(nb_valid), .cs_v(cs_rdata),
        .g_v_old(g_rdata), .h_v(h),
        .g_new(rx_g_new), .f_new(rx_f_new), .improve(rx_improve)
    );

    // ------------------------------------------------- read-address muxes --
    wire [INDEX_W-1:0] cost_raddr = (state == S_CHK_S) ? s_idx :
                                    (state == S_CHK_G) ? t_idx : nb_v;
    // POP_WAIT reads the popped cell's g; NB reads v's state and g.
    wire [INDEX_W-1:0] cs_raddr   = nb_v;
    wire [INDEX_W-1:0] g_raddr    = (state == S_POP_WAIT) ? os_pop_idx : nb_v;
    wire [INDEX_W-1:0] par_raddr  = rc_cur;

    // -------------------------------------- events (all need step_en = 1) --
    // INIT waits at ptr 0 until the open set can take its clr, so the two
    // clears always start together.
    wire init_go   = step_en && (state == S_INIT) && (ptr != 0 || os_ready);
    wire seed_go   = step_en && (state == S_SEED) && os_ready;
    wire pop_got   = (state == S_POP_WAIT) && os_pop_valid;  // pop_valid includes step_en
    wire pop_fresh = pop_got && !os_pop_empty;              // a cell to expand
    // Only an improving neighbour needs the open set, so the others don't wait
    // for a re-sort still running after the last pop or push.
    wire nb_go     = step_en && (state == S_NB_WR) && (os_ready || !nb_imp_r);
    wire nb_write  = nb_go && nb_imp_r;
    wire recon_go  = step_en && (state == S_RECON);

    wire start_ok  = start && (state == S_IDLE || state == S_DONE);

    // ------------------------------------------------- open set (queue) --
    wire               os_clr      = (state == S_INIT) && (ptr == 0);
    wire               os_push     = seed_go || nb_write;
    wire [INDEX_W-1:0] os_push_idx = (state == S_SEED) ? s_idx : nb_v_r;
    wire [DIST_W-1:0]  os_push_f   = (state == S_SEED) ? h : nb_f_r;
    wire [DIST_W-1:0]  os_push_g   = (state == S_SEED) ? {DIST_W{1'b0}} : nb_g_r;
    wire               os_pop_req  = (state == S_POP);

    ast_open_set #(.NUM_CELLS(NUM_CELLS), .INDEX_W(INDEX_W), .DIST_W(DIST_W)) u_os (
        .clk(clk), .rst(rst), .step_en(step_en),
        .clr(os_clr), .push(os_push), .push_idx(os_push_idx),
        .push_f(os_push_f), .push_g(os_push_g), .pop_req(os_pop_req),
        .ready(os_ready), .pop_valid(os_pop_valid),
        .pop_idx(os_pop_idx), .pop_empty(os_pop_empty)
    );

    // ------------------------------------------- memory write ports --------
    // cell state: one writer per FSM state.
    //     INIT     ptr        <= UNSEEN
    //     SEED     start      <= FRONTIER
    //     POP_WAIT popped u   <= VISITED
    //     NB_WR    improved v <= FRONTIER
    //     RECON    path cell  <= PATH
    wire cs_we = init_go || seed_go || pop_fresh || nb_write || recon_go;

    wire [INDEX_W-1:0] cs_waddr = (state == S_SEED)     ? s_idx      :
                                  (state == S_POP_WAIT) ? os_pop_idx :
                                  (state == S_NB_WR)    ? nb_v_r     :
                                  (state == S_RECON)    ? rc_cur     : ptr;

    wire [1:0]         cs_wdata = (state == S_POP_WAIT) ? CS_VISITED :
                                  (state == S_RECON)    ? CS_PATH    :
                                  (state == S_INIT)     ? CS_UNSEEN  : CS_FRONTIER;

    // g and parent: written when the start is seeded and on every improvement.
    wire               gp_we    = seed_go || nb_write;
    wire [INDEX_W-1:0] gp_waddr = (state == S_SEED) ? s_idx : nb_v_r;
    wire [DIST_W-1:0]  g_wdata  = (state == S_SEED) ? {DIST_W{1'b0}} : nb_g_r;
    wire [INDEX_W-1:0] p_wdata  = (state == S_SEED) ? NONE : cur;

    // ------------------------------------------------------- memories ------
    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_cost (
        .clk(clk), .we(load_we), .waddr(load_addr), .wdata(load_data),
        .raddr(cost_raddr), .rdata(cost_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(DIST_W)) u_g (
        .clk(clk), .we(gp_we), .waddr(gp_waddr), .wdata(g_wdata),
        .raddr(g_raddr), .rdata(g_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_par (
        .clk(clk), .we(gp_we), .waddr(gp_waddr), .wdata(p_wdata),
        .raddr(par_raddr), .rdata(par_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(2)) u_cs_eng (
        .clk(clk), .we(cs_we), .waddr(cs_waddr), .wdata(cs_wdata),
        .raddr(cs_raddr), .rdata(cs_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(2)) u_cs_vga (
        .clk(clk), .we(cs_we), .waddr(cs_waddr), .wdata(cs_wdata),
        .raddr(cell_rd_addr), .rdata(cell_state)
    );

    // Path readback.  RECON walks goal -> start, so the RAM holds the path
    // goal-first (path[0] = goal).  Flipping the read address gives the
    // CONTRACT order (addr 0 = start) without a second pass.
    wire [INDEX_W-1:0] path_raddr = path_cells_r[INDEX_W-1:0] - 1'b1 - path_rd_addr;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_path (
        .clk(clk), .we(recon_go), .waddr(rc_len[INDEX_W-1:0]), .wdata(rc_cur),
        .raddr(path_raddr), .rdata(path_rd_data)
    );

    // ----------------------------------------------- statistics (4.3) ------
    ast_sat_counter #(.W(CNT_W)) u_cycles (
        .clk(clk), .rst(rst), .clr(start_ok), .inc(busy && step_en), .q(cycles)
    );

    ast_sat_counter #(.W(DIST_W)) u_expansions (
        .clk(clk), .rst(rst), .clr(start_ok), .inc(pop_fresh), .q(expansions)
    );

    // ------------------------------------------------------ state machine --
    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            found        <= 1'b0;
            no_path      <= 1'b0;
            path_cells_r <= {DIST_W{1'b0}};
            path_cost_r  <= {DIST_W{1'b0}};
            current_r    <= NONE;
            ptr          <= {INDEX_W{1'b0}};

        end else if (start_ok) begin
            // New run (accepted from IDLE or DONE, even while paused, so
            // busy always rises the cycle after start: CONTRACT 4.1.2).
            s_idx        <= start_idx;
            t_idx        <= goal_idx;
            busy         <= 1'b1;
            done         <= 1'b0;
            found        <= 1'b0;
            no_path      <= 1'b0;
            path_cells_r <= {DIST_W{1'b0}};
            path_cost_r  <= {DIST_W{1'b0}};
            current_r    <= NONE;
            ptr          <= {INDEX_W{1'b0}};
            state        <= S_INIT;

        end else if (step_en) begin
            case (state)
                S_INIT: if (init_go) begin
                    ptr <= ptr + 1'b1;
                    if (ptr == LAST)
                        state <= S_CHK_S;
                end

                S_CHK_S: state <= (cost_rdata == COST_WALL) ? S_FAIL : S_CHK_G;
                S_CHK_G: state <= (cost_rdata == COST_WALL) ? S_FAIL : S_SEED;

                S_SEED: if (os_ready) state <= S_POP;      // push issued by seed_go
                S_POP:  if (os_ready) state <= S_POP_WAIT; // pop_req taken this cycle

                S_POP_WAIT: if (os_pop_valid) begin
                    if (os_pop_empty)
                        state <= S_FAIL;                   // nothing left: unreachable
                    else begin
                        cur       <= os_pop_idx;           // close u (cs write above)
                        g_cur     <= g_rdata;
                        current_r <= os_pop_idx;
                        dir       <= 2'd0;
                        if (os_pop_idx == t_idx) begin
                            rc_cur <= os_pop_idx;
                            rc_len <= {DIST_W{1'b0}};
                            state  <= S_RECON;
                        end else
                            state  <= S_NB;
                    end
                end

                S_NB: begin                                // evaluate v = nb(u, dir)
                    nb_v_r   <= nb_v;
                    nb_imp_r <= rx_improve;
                    nb_g_r   <= rx_g_new;
                    nb_f_r   <= rx_f_new;
                    state    <= S_NB_WR;
                end

                S_NB_WR: if (os_ready || !nb_imp_r) begin  // relax/push done by nb_write
                    dir   <= dir + 1'b1;
                    state <= S_NB;
                    if (dir == 2'd3)
                        state <= S_POP;
                end

                S_RECON: begin                             // path[rc_len] = rc_cur (above)
                    rc_len <= rc_len + 1'b1;
                    if (rc_cur == s_idx) begin
                        busy         <= 1'b0;              // busy falls, done rises together
                        done         <= 1'b1;
                        found        <= 1'b1;
                        path_cells_r <= rc_len + 1'b1;
                        path_cost_r  <= g_cur;             // g[goal]
                        state        <= S_DONE;
                    end else
                        rc_cur <= par_rdata;
                end

                S_FAIL: begin
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    no_path <= 1'b1;                       // path_cells/cost stay 0
                    state   <= S_DONE;
                end

                default: ;                                 // IDLE, DONE: wait for start
            endcase
        end
    end
endmodule
