`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_m2_crosscheck : A* (M2) vs Dijkstra (M1) on the shared mazes (CONTRACT 10.3)
//
// Both engines get the same maze over the same load bus and the same start
// pulse, exactly as pathfind_top will drive them.  For every line of the
// shared <name>.expect (start goal found bfs_cost dij_cost ast_cost cells_min):
//     * both engines agree on found / no_path, and match the reference
//     * dij.path_cost == ast.path_cost == reference dij_cost   (hard rule)
//     * ast.expansions <= dij.expansions                        (reported only)
//
// M1's dijkstra_engine predates CONTRACT 1.0 (see CONTRACT 12):
//   * no step_en / cell_rd_addr ports, 16-bit cycles, needs rst to rerun,
//     start==goal and wall endpoints not yet handled.
// By default this bench drives that CURRENT port list, pulses rst before
// each run, and SKIPS the CONTRACT-12 edge cases (counted and reported).
// Once M1's engine meets CONTRACT 4, compile with  -DDIJ_CONTRACT_PORTS
// to use the frozen port list and check every run with no skips:
//     sim/run_m2.sh tb_m2_crosscheck rtl/m1_pathfind/dijkstra_engine.v -DDIJ_CONTRACT_PORTS
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_m2_crosscheck;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam COST_W    = `ARENA_COST_W;
    localparam DIST_W    = `ARENA_DIST_W;
    localparam CNT_W     = `ARENA_CNT_W;

    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    // ---- shared inputs (what pathfind_top fans out) ------------------------
    reg                rst = 1'b1;
    reg                step_en = 1'b1;
    reg                load_we = 1'b0;
    reg  [INDEX_W-1:0] load_addr = 0;
    reg  [COST_W-1:0]  load_data = 0;
    reg                start = 1'b0;
    reg  [INDEX_W-1:0] start_idx = 0, goal_idx = 0;
    reg  [INDEX_W-1:0] cell_rd_addr = 0;

    // ---- A* ------------------------------------------------------------------
    wire               ast_busy, ast_done, ast_found, ast_no_path;
    wire [INDEX_W-1:0] ast_path_rd_data, ast_current;
    wire [DIST_W-1:0]  ast_path_cells, ast_path_cost, ast_expansions;
    wire [CNT_W-1:0]   ast_cycles;
    wire [1:0]         ast_cell_state;

    astar_engine u_ast (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(ast_busy), .done(ast_done), .found(ast_found), .no_path(ast_no_path),
        .path_rd_addr({INDEX_W{1'b0}}), .path_rd_data(ast_path_rd_data),
        .path_cells(ast_path_cells), .path_cost(ast_path_cost),
        .current(ast_current), .cell_rd_addr(cell_rd_addr), .cell_state(ast_cell_state),
        .cycles(ast_cycles), .expansions(ast_expansions),
        .dbg_state()
    );

    // ---- Dijkstra (M1) -------------------------------------------------------
    wire               dij_busy, dij_done, dij_found, dij_no_path;
    wire [INDEX_W-1:0] dij_path_rd_data, dij_current;
    wire [DIST_W-1:0]  dij_path_cells, dij_path_cost, dij_expansions;

`ifdef DIJ_CONTRACT_PORTS
    wire [CNT_W-1:0]   dij_cycles;
    wire [1:0]         dij_cell_state;

    dijkstra_engine u_dij (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(dij_busy), .done(dij_done), .found(dij_found), .no_path(dij_no_path),
        .path_rd_addr({INDEX_W{1'b0}}), .path_rd_data(dij_path_rd_data),
        .path_cells(dij_path_cells), .path_cost(dij_path_cost),
        .current(dij_current), .cell_rd_addr(cell_rd_addr), .cell_state(dij_cell_state),
        .cycles(dij_cycles), .expansions(dij_expansions)
    );
    localparam LEGACY_DIJ = 0;
`else
    wire [DIST_W-1:0]  dij_cycles;              // pre-contract: 16 bits

    dijkstra_engine u_dij (
        .clk(clk), .rst(rst),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(dij_busy), .done(dij_done), .found(dij_found), .no_path(dij_no_path),
        .path_rd_addr({INDEX_W{1'b0}}), .path_rd_data(dij_path_rd_data),
        .path_cells(dij_path_cells), .path_cost(dij_path_cost),
        .current(dij_current),
        .cycles(dij_cycles), .expansions(dij_expansions)
    );
    localparam LEGACY_DIJ = 1;
`endif

    // ---- bookkeeping ---------------------------------------------------------
    reg [COST_W-1:0] cost [0:NUM_CELLS-1];
    integer errors = 0, n_runs = 0, n_skipped = 0, n_exp_worse = 0;
    integer sum_ast_exp = 0, sum_dij_exp = 0;

    task fail(input [8*72-1:0] msg);
        begin
            errors = errors + 1;
            if (errors <= 25) $display("  t=%0t FAIL %0s", $time, msg);
        end
    endtask

    task pulse_rst;
        begin
            @(negedge clk) rst = 1'b1;
            repeat (2) @(negedge clk);
            rst = 1'b0;
        end
    endtask

    task load_maze;
        integer i;
        begin
            for (i = 0; i < NUM_CELLS; i = i + 1) begin
                @(negedge clk);
                load_we = 1'b1; load_addr = i; load_data = cost[i];
            end
            @(negedge clk) load_we = 1'b0;
        end
    endtask

    // Pulse start, wait for BOTH engines to finish.
    task run_both(input integer s, input integer g);
        integer timeout;
        begin
            if (LEGACY_DIJ) pulse_rst;          // old Dijkstra can't restart from DONE
            @(negedge clk);
            start_idx = s; goal_idx = g; start = 1'b1;
            @(negedge clk) start = 1'b0;
            timeout = 0;
            while (!(ast_done && dij_done) && timeout < 500000) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 500000) fail("timeout waiting for both engines");
        end
    endtask

    task run_file(input [8*48-1:0] mem_file, input [8*48-1:0] exp_file);
        integer fd, r, s, g, ef, eb, ed, ea, ecells;
        reg legacy_gap;
        begin
            $readmemh(mem_file, cost);
            load_maze;
            $display("  %0s", mem_file);
            fd = $fopen(exp_file, "r");
            if (fd == 0) fail("cannot open expectation file");
            r = (fd == 0) ? 0 : $fscanf(fd, "%d %d %d %d %d %d %d\n", s, g, ef, eb, ed, ea, ecells);
            while (r == 7) begin
                // CONTRACT-12 cases the pre-contract Dijkstra doesn't handle yet.
                legacy_gap = LEGACY_DIJ && (s == g || cost[s] == 0 || cost[g] == 0);
                if (legacy_gap) begin
                    n_skipped = n_skipped + 1;
                    $display("    %4d -> %4d  skipped (M1 CONTRACT-12 gap: start==goal / wall endpoint)", s, g);
                end else begin
                    run_both(s, g);
                    n_runs = n_runs + 1;
                    if (ast_found !== ef[0] || dij_found !== ef[0])
                        fail("found disagrees (ast / dij / reference)");
                    if (dij_path_cost !== ast_path_cost)
                        fail("dij.path_cost != ast.path_cost");
                    if (ast_path_cost !== ed)
                        fail("path_cost != reference dij_cost");
                    if (ast_found && ast_expansions > dij_expansions)
                        n_exp_worse = n_exp_worse + 1;
                    if (ast_found) begin
                        sum_ast_exp = sum_ast_exp + ast_expansions;
                        sum_dij_exp = sum_dij_exp + dij_expansions;
                    end
                    $display("    %4d -> %4d  cost ast=%3d dij=%3d ref=%3d | expansions ast=%3d dij=%3d | cycles ast=%6d dij=%6d",
                             s, g, ast_path_cost, dij_path_cost, ed,
                             ast_expansions, dij_expansions, ast_cycles, dij_cycles);
                end
                r = $fscanf(fd, "%d %d %d %d %d %d %d\n", s, g, ef, eb, ed, ea, ecells);
            end
            if (fd != 0) $fclose(fd);
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        @(negedge clk) rst = 1'b0;
        if (LEGACY_DIJ)
            $display("Dijkstra: pre-contract port list (compile with -DDIJ_CONTRACT_PORTS once M1 meets CONTRACT 4)");

        run_file("sim/mazes/open.mem",        "sim/mazes/open.expect");
        run_file("sim/mazes/weighted.mem",    "sim/mazes/weighted.expect");
        run_file("sim/mazes/unreachable.mem", "sim/mazes/unreachable.expect");
        run_file("sim/mazes/corridor.mem",    "sim/mazes/corridor.expect");
        run_file("sim/mazes/rand0.mem",       "sim/mazes/rand0.expect");
        run_file("sim/mazes/rand1.mem",       "sim/mazes/rand1.expect");
        run_file("sim/mazes/rand2.mem",       "sim/mazes/rand2.expect");
        run_file("sim/mazes/rand3.mem",       "sim/mazes/rand3.expect");
        run_file("sim/mazes/rand4.mem",       "sim/mazes/rand4.expect");
        run_file("sim/mazes/rand5.mem",       "sim/mazes/rand5.expect");
        run_file("sim/mazes/rand6.mem",       "sim/mazes/rand6.expect");
        run_file("sim/mazes/rand7.mem",       "sim/mazes/rand7.expect");

        $display("  %0d runs cross-checked, %0d skipped (M1 gaps)", n_runs, n_skipped);
        $display("  expansions on found runs: A* total %0d vs Dijkstra total %0d; A* worse on %0d runs (report only)",
                 sum_ast_exp, sum_dij_exp, n_exp_worse);
        if (errors == 0)
            $display("PASS: tb_m2_crosscheck (0 errors)");
        else
            $display("FAIL: tb_m2_crosscheck (%0d errors)", errors);
        $finish;
    end
endmodule
