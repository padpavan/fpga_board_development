`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_astar_engine : acceptance test for the A* engine (CONTRACT 4 + 10)
//
// For every shared maze in sim/mazes/ (written by ref/gen_mazes.py):
//   load <name>.mem over the load bus, then for each line of
//   <name>.ast.expect  (start goal found cost cells expansions)  run A* and
//   check
//     * found / no_path exactly one set, matches expectation
//     * path_cost, path_cells, expansions == reference model (exact)
//     * cycles == number of busy && step_en cycles counted here (CONTRACT 4.3)
//     * busy rises the cycle after start; busy falls as done rises
//     * path RAM: addr 0 = start, last = goal, every step adjacent, no walls,
//       sum of entered costs == path_cost
//     * cell_state: every path cell = PATH, and #VISITED + #PATH == expansions
//       (every closed cell is either on the path or plain visited)
// Runs are back to back with NO rst in between (restart from DONE, 4.1.2).
//
// Pass 0 runs everything at full speed.  Pass 1 repeats it with step_en
// randomly toggling (pause / animation speed): all results, including
// cycles, must be identical to pass 0.
//
// Then handshake edge cases:
//     * start pulsed while busy is ignored
//     * done and all results hold until the next start
//     * rst mid-run returns to idle with zeroed status/counters, then a
//       fresh run still works
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
`ifndef TB_USE_HEAP
`define TB_USE_HEAP 0
`endif
// Open set under test: sim/run_m2.sh tb_astar_engine -DTB_USE_HEAP=1  for the heap.

module tb_astar_engine;
    localparam COLS      = `ARENA_COLS;
    localparam ROWS      = `ARENA_ROWS;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam COST_W    = `ARENA_COST_W;
    localparam DIST_W    = `ARENA_DIST_W;
    localparam CNT_W     = `ARENA_CNT_W;
    localparam MAX_RUNS  = 128;

    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    // ---- DUT ports ----------------------------------------------------------
    reg                rst = 1'b1;
    reg                step_en = 1'b1;
    reg                load_we = 1'b0;
    reg  [INDEX_W-1:0] load_addr = 0;
    reg  [COST_W-1:0]  load_data = 0;
    reg                start = 1'b0;
    reg  [INDEX_W-1:0] start_idx = 0, goal_idx = 0;
    wire               busy, done, found, no_path;
    reg  [INDEX_W-1:0] path_rd_addr = 0;
    wire [INDEX_W-1:0] path_rd_data;
    wire [DIST_W-1:0]  path_cells, path_cost, expansions;
    wire [INDEX_W-1:0] current;
    reg  [INDEX_W-1:0] cell_rd_addr = 0;
    wire [1:0]         cell_state;
    wire [CNT_W-1:0]   cycles;
    wire [3:0]         dbg_state;

    astar_engine #(.USE_HEAP(`TB_USE_HEAP)) dut (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(busy), .done(done), .found(found), .no_path(no_path),
        .path_rd_addr(path_rd_addr), .path_rd_data(path_rd_data),
        .path_cells(path_cells), .path_cost(path_cost),
        .current(current), .cell_rd_addr(cell_rd_addr), .cell_state(cell_state),
        .cycles(cycles), .expansions(expansions),
        .dbg_state(dbg_state)
    );

    // ---- step_en driver: 1, or random (~70% high) when rand_step ----------
    reg rand_step = 1'b0;
    always @(negedge clk)
        step_en <= rand_step ? (($unsigned($random) % 10) < 7) : 1'b1;

    // ---- bookkeeping --------------------------------------------------------
    reg [COST_W-1:0] cost [0:NUM_CELLS-1];      // testbench copy of the maze
    integer errors = 0;
    integer n_runs = 0;
    integer run_no;                             // index within a pass
    integer pass;
    integer ref_cycles [0:MAX_RUNS-1];          // pass-0 cycles, compared in pass 1

    task fail(input [8*72-1:0] msg);
        begin
            errors = errors + 1;
            if (errors <= 25) $display("  t=%0t FAIL %0s", $time, msg);
        end
    endtask

    // ---- load the maze in `cost` into the engine (busy must be 0) ---------
    task load_maze;
        integer i;
        begin
            if (busy) fail("load while busy");
            for (i = 0; i < NUM_CELLS; i = i + 1) begin
                @(negedge clk);
                load_we = 1'b1; load_addr = i; load_data = cost[i];
            end
            @(negedge clk) load_we = 1'b0;
        end
    endtask

    // ---- start a run and wait for done, counting busy&&step_en cycles -----
    task launch_and_wait(input integer s, input integer g, output integer n_cyc);
        integer timeout;
        begin
            @(negedge clk);
            start_idx = s; goal_idx = g; start = 1'b1;
            @(posedge clk); #1;                 // start sampled at this edge
            if (!busy) fail("busy did not rise the cycle after start");
            if (done)  fail("done still high after start");
            @(negedge clk) start = 1'b0;

            // Count every later edge whose PRE-edge busy && step_en is 1; the
            // edge that raises done still sees busy = 1, so it is included.
            n_cyc = 0;
            timeout = 0;
            begin : wait_done
                forever begin
                    @(posedge clk);
                    if (busy && step_en) n_cyc = n_cyc + 1;   // pre-edge values
                    #1;
                    if (done) disable wait_done;
                    timeout = timeout + 1;
                    if (timeout > 400000) begin
                        fail("timeout waiting for done");
                        disable wait_done;
                    end
                end
            end
            if (busy) fail("busy still high when done rose");
        end
    endtask

    // ---- check a finished run -----------------------------------------------
    task check_run(input integer s, input integer g, input integer e_found,
                   input integer e_cost, input integer e_cells, input integer e_exp,
                   input integer n_cyc);
        integer k, p, prev, dr, dc, sum, n_vis, n_path, bad_path_state;
        begin
            if (found + no_path !== 1)        fail("exactly one of found/no_path must be 1");
            if (found !== e_found[0])         fail("found mismatch");
            if (path_cost !== e_cost)         fail("path_cost mismatch");
            if (path_cells !== e_cells)       fail("path_cells mismatch");
            if (expansions !== e_exp)         fail("expansions mismatch");
            if (cycles !== n_cyc)             fail("cycles != busy&&step_en count");

            // path RAM walk
            sum = 0;
            bad_path_state = 0;
            prev = -1;
            for (k = 0; k < path_cells; k = k + 1) begin
                path_rd_addr = k;
                #1;
                p = path_rd_data;
                if (k == 0 && p != s)                fail("path[0] != start");
                if (k == path_cells - 1 && p != g)   fail("path[last] != goal");
                if (cost[p] == 0)                    fail("path crosses a wall");
                if (k > 0) begin
                    dr = (p / COLS) - (prev / COLS);
                    dc = (p % COLS) - (prev % COLS);
                    if (dr < 0) dr = -dr;
                    if (dc < 0) dc = -dc;
                    if (dr + dc != 1)                fail("path step not adjacent");
                    sum = sum + cost[p];
                end
                cell_rd_addr = p;
                #1;
                if (cell_state !== `ARENA_CS_PATH) bad_path_state = bad_path_state + 1;
                prev = p;
            end
            if (found && sum !== path_cost)  fail("sum of path costs != path_cost");
            if (bad_path_state)              fail("path cell not marked PATH");

            // cell_state census
            n_vis = 0; n_path = 0;
            for (k = 0; k < NUM_CELLS; k = k + 1) begin
                cell_rd_addr = k;
                #1;
                if (cell_state === `ARENA_CS_VISITED) n_vis  = n_vis + 1;
                if (cell_state === `ARENA_CS_PATH)    n_path = n_path + 1;
            end
            if (n_path !== path_cells)         fail("#PATH cells != path_cells");
            if (n_vis + n_path !== expansions) fail("#VISITED + #PATH != expansions");

            // full-speed vs random-step cycles must agree
            if (pass == 0)
                ref_cycles[run_no] = cycles;
            else if (cycles !== ref_cycles[run_no])
                fail("cycles differ between full speed and paused run");

            if (pass == 0)
                $display("  %4d -> %4d  found=%0d cost=%3d cells=%3d exp=%3d cycles=%6d",
                         s, g, found, path_cost, path_cells, expansions, cycles);
            run_no = run_no + 1;
            n_runs = n_runs + 1;
        end
    endtask

    // ---- one maze file + its expectations ---------------------------------
    task run_maze(input [8*48-1:0] mem_file, input [8*48-1:0] exp_file);
        integer fd, r, s, g, ef, ec, ecells, eexp, n_cyc, err0;
        begin
            $readmemh(mem_file, cost);
            load_maze;
            fd = $fopen(exp_file, "r");
            if (fd == 0) begin
                fail("cannot open expectation file");
            end else begin
                if (pass == 0) $display("  %0s", mem_file);
                r = $fscanf(fd, "%d %d %d %d %d %d\n", s, g, ef, ec, ecells, eexp);
                while (r == 6) begin
                    err0 = errors;
                    launch_and_wait(s, g, n_cyc);
                    check_run(s, g, ef, ec, ecells, eexp, n_cyc);
                    if (errors != err0)
                        $display("    ^ run %0d->%0d: expected found=%0d cost=%0d cells=%0d exp=%0d, got found=%0d cost=%0d cells=%0d exp=%0d",
                                 s, g, ef, ec, ecells, eexp, found, path_cost, path_cells, expansions);
                    r = $fscanf(fd, "%d %d %d %d %d %d\n", s, g, ef, ec, ecells, eexp);
                end
                $fclose(fd);
            end
        end
    endtask

    task run_all_mazes;
        begin
            run_no = 0;
            run_maze("sim/mazes/open.mem",        "sim/mazes/open.ast.expect");
            run_maze("sim/mazes/weighted.mem",    "sim/mazes/weighted.ast.expect");
            run_maze("sim/mazes/unreachable.mem", "sim/mazes/unreachable.ast.expect");
            run_maze("sim/mazes/corridor.mem",    "sim/mazes/corridor.ast.expect");
            run_maze("sim/mazes/rand0.mem",       "sim/mazes/rand0.ast.expect");
            run_maze("sim/mazes/rand1.mem",       "sim/mazes/rand1.ast.expect");
            run_maze("sim/mazes/rand2.mem",       "sim/mazes/rand2.ast.expect");
            run_maze("sim/mazes/rand3.mem",       "sim/mazes/rand3.ast.expect");
            run_maze("sim/mazes/rand4.mem",       "sim/mazes/rand4.ast.expect");
            run_maze("sim/mazes/rand5.mem",       "sim/mazes/rand5.ast.expect");
            run_maze("sim/mazes/rand6.mem",       "sim/mazes/rand6.ast.expect");
            run_maze("sim/mazes/rand7.mem",       "sim/mazes/rand7.ast.expect");
        end
    endtask

    integer n_cyc, k;
    reg [DIST_W-1:0] hold_cost, hold_exp;
    reg [CNT_W-1:0]  hold_cyc;

    initial begin
        // reset >= 2 cycles (CONTRACT 10.5)
        repeat (3) @(posedge clk);
        @(negedge clk) rst = 1'b0;
        if (busy || done || found || no_path) fail("status not clear after rst");

        $display("open set: %0s", `TB_USE_HEAP ? "binary heap (ast_open_set_heap)" : "linear scan (ast_open_set)");

        // ---------------- pass 0: full speed --------------------------------
        pass = 0;
        $display("pass 0: full speed");
        run_all_mazes;

        // ---------------- pass 1: step_en random ----------------------------
        pass = 1;
        $display("pass 1: step_en randomly toggling (results must match pass 0)");
        rand_step = 1'b1;
        run_all_mazes;
        rand_step = 1'b0;
        @(negedge clk);

        // ---------------- handshake edge cases (open maze) ------------------
        $display("handshake edge cases");
        pass = 2;
        $readmemh("sim/mazes/open.mem", cost);
        load_maze;

        // start while busy must be ignored: run 0 -> 191 (cost 26, exp 27)
        @(negedge clk);
        start_idx = 0; goal_idx = NUM_CELLS - 1; start = 1'b1;
        @(negedge clk) start = 1'b0;
        repeat (300) @(negedge clk);
        start_idx = 5; goal_idx = 6; start = 1'b1;  // stray start mid-run
        @(negedge clk) start = 1'b0;
        while (!done) @(negedge clk);
        if (path_cost !== 26 || expansions !== 27 || !found)
            fail("start while busy was not ignored");

        // done and results hold until the next start
        hold_cost = path_cost; hold_exp = expansions; hold_cyc = cycles;
        repeat (100) @(negedge clk);
        if (!done || busy || path_cost !== hold_cost || expansions !== hold_exp || cycles !== hold_cyc)
            fail("results did not hold while done");

        // rst mid-run
        @(negedge clk);
        start_idx = 0; goal_idx = NUM_CELLS - 1; start = 1'b1;
        @(negedge clk) start = 1'b0;
        repeat (500) @(negedge clk);
        rst = 1'b1;
        repeat (2) @(negedge clk);
        rst = 1'b0;
        #1;
        if (busy || done || found || no_path || cycles !== 0 || expansions !== 0)
            fail("rst mid-run did not return to idle with zeroed counters");

        // and a fresh run afterwards still works
        launch_and_wait(17, 17, n_cyc);       // start == goal
        if (!found || path_cells !== 1 || path_cost !== 0 || expansions !== 1 || cycles !== n_cyc)
            fail("start==goal after rst");
        path_rd_addr = 0; #1;
        if (path_rd_data !== 17) fail("start==goal path[0]");

        $display("  %0d maze runs checked (2 passes)", n_runs);
        if (errors == 0)
            $display("PASS: tb_astar_engine (%0s, 0 errors)", `TB_USE_HEAP ? "heap" : "scan");
        else
            $display("FAIL: tb_astar_engine (%0d errors)", errors);
        $finish;
    end
endmodule
