`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Self-test for pathfind_top : all four engines run concurrently on the
// same frozen maze and start signal (CONTRACT.md sections 5 and 10.3).
//
//   Scenario A (open grid): unit cost -> bfs.path_cost == dij.path_cost (26).
//   Scenario B (weighted) : BFS reports minimum steps (15), Dijkstra reports
//                           minimum weighted cost (17), demonstrating
//                           "fewest steps != least cost".
//   Scenario C (corridor) : unit cost, unique path -> both 100.
//   Every scenario also checks A*: ast.path_cost == dij.path_cost (10.3),
//   and DFS: found, dfs.path_cost >= dij.path_cost (any path, 4.5).
//   Shared mazes (10.2): every run in sim/mazes/<name>.expect is replayed
//   through pathfind_top and all four engines are checked against M2's
//   reference model (found / no_path, bfs/dij/ast cost, BFS path_cells, and
//   the exact DFS cost/cells from <name>.dfs.expect).
// Scenarios run back-to-back without rst (restart from DONE).
//////////////////////////////////////////////////////////////////////////////
module tb_pathfind;
    parameter COLS = `ARENA_COLS, ROWS = `ARENA_ROWS, COST_W = `ARENA_COST_W;
    parameter DIST_W = `ARENA_DIST_W, INDEX_W = `ARENA_INDEX_W, CNT_W = `ARENA_CNT_W;
    localparam NUM_CELLS = COLS * ROWS;

    reg        clk = 0;
    reg        rst = 0;
    reg        step_en = 1;
    reg        load_we = 0;
    reg  [INDEX_W-1:0] load_addr = 0;
    reg  [COST_W-1:0]  load_data = 0;
    reg        start = 0;
    reg  [3:0] run_mask = 4'b1111;
    reg  [INDEX_W-1:0] start_idx = 0;
    reg  [INDEX_W-1:0] goal_idx  = 0;
    reg  [INDEX_W-1:0] cell_rd_addr = 0;

    wire bfs_busy, bfs_done, bfs_found, bfs_no_path;
    reg  [INDEX_W-1:0] bfs_path_rd_addr = 0;
    wire [INDEX_W-1:0] bfs_path_rd_data;
    wire [DIST_W-1:0]  bfs_path_cells, bfs_path_cost, bfs_expansions;
    wire [CNT_W-1:0]   bfs_cycles;
    wire [INDEX_W-1:0] bfs_current;
    wire [1:0]         bfs_cell_state;

    wire dij_busy, dij_done, dij_found, dij_no_path;
    reg  [INDEX_W-1:0] dij_path_rd_addr = 0;
    wire [INDEX_W-1:0] dij_path_rd_data;
    wire [DIST_W-1:0]  dij_path_cells, dij_path_cost, dij_expansions;
    wire [CNT_W-1:0]   dij_cycles;
    wire [INDEX_W-1:0] dij_current;
    wire [1:0]         dij_cell_state;

    wire ast_busy, ast_done, ast_found, ast_no_path;
    reg  [INDEX_W-1:0] ast_path_rd_addr = 0;
    wire [INDEX_W-1:0] ast_path_rd_data;
    wire [DIST_W-1:0]  ast_path_cells, ast_path_cost, ast_expansions;
    wire [CNT_W-1:0]   ast_cycles;
    wire [INDEX_W-1:0] ast_current;
    wire [1:0]         ast_cell_state;
    wire [3:0]         ast_dbg_state;

    wire dfs_busy, dfs_done, dfs_found, dfs_no_path;
    reg  [INDEX_W-1:0] dfs_path_rd_addr = 0;
    wire [INDEX_W-1:0] dfs_path_rd_data;
    wire [DIST_W-1:0]  dfs_path_cells, dfs_path_cost, dfs_expansions;
    wire [CNT_W-1:0]   dfs_cycles;
    wire [INDEX_W-1:0] dfs_current;
    wire [1:0]         dfs_cell_state;

    pathfind_top #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) dut (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .run_mask(run_mask), .start_idx(start_idx), .goal_idx(goal_idx),
        .cell_rd_addr(cell_rd_addr),
        .bfs_busy(bfs_busy), .bfs_done(bfs_done), .bfs_found(bfs_found), .bfs_no_path(bfs_no_path),
        .bfs_path_rd_addr(bfs_path_rd_addr), .bfs_path_rd_data(bfs_path_rd_data),
        .bfs_path_cells(bfs_path_cells), .bfs_path_cost(bfs_path_cost),
        .bfs_current(bfs_current), .bfs_cell_state(bfs_cell_state),
        .bfs_cycles(bfs_cycles), .bfs_expansions(bfs_expansions),
        .dij_busy(dij_busy), .dij_done(dij_done), .dij_found(dij_found), .dij_no_path(dij_no_path),
        .dij_path_rd_addr(dij_path_rd_addr), .dij_path_rd_data(dij_path_rd_data),
        .dij_path_cells(dij_path_cells), .dij_path_cost(dij_path_cost),
        .dij_current(dij_current), .dij_cell_state(dij_cell_state),
        .dij_cycles(dij_cycles), .dij_expansions(dij_expansions),
        .ast_busy(ast_busy), .ast_done(ast_done), .ast_found(ast_found), .ast_no_path(ast_no_path),
        .ast_path_rd_addr(ast_path_rd_addr), .ast_path_rd_data(ast_path_rd_data),
        .ast_path_cells(ast_path_cells), .ast_path_cost(ast_path_cost),
        .ast_current(ast_current), .ast_cell_state(ast_cell_state),
        .ast_cycles(ast_cycles), .ast_expansions(ast_expansions),
        .ast_dbg_state(ast_dbg_state),
        .dfs_busy(dfs_busy), .dfs_done(dfs_done), .dfs_found(dfs_found), .dfs_no_path(dfs_no_path),
        .dfs_path_rd_addr(dfs_path_rd_addr), .dfs_path_rd_data(dfs_path_rd_data),
        .dfs_path_cells(dfs_path_cells), .dfs_path_cost(dfs_path_cost),
        .dfs_current(dfs_current), .dfs_cell_state(dfs_cell_state),
        .dfs_cycles(dfs_cycles), .dfs_expansions(dfs_expansions)
    );

    reg [COST_W-1:0] cost [0:NUM_CELLS-1];
    integer errors;

    always #5 clk = ~clk;

    task reset;
        begin
            rst = 1'b1;
            @(posedge clk); #1;
            @(posedge clk); #1;
            rst = 1'b0;
            @(posedge clk); #1;
        end
    endtask

    task load_all;
        integer i;
        begin
            for (i = 0; i < NUM_CELLS; i = i + 1) begin
                load_we   = 1'b1;
                load_addr = i;
                load_data = cost[i];
                @(posedge clk); #1;
            end
            load_we = 1'b0;
        end
    endtask

    task launch;
        begin
            start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0;
        end
    endtask

    task wait_all_done;
        integer timeout;
        begin
            timeout = 0;
            while (!(bfs_done && dij_done && ast_done && dfs_done) && timeout < 500000) begin
                @(posedge clk); #1;
                timeout = timeout + 1;
            end
            if (timeout >= 500000) begin
                $display("TIMEOUT waiting for all engines");
                $finish;
            end
        end
    endtask

    task run;
        input [INDEX_W-1:0] s;
        input [INDEX_W-1:0] g;
        begin
            load_all();
            start_idx = s; goal_idx = g;
            launch();
            wait_all_done();
        end
    endtask

    task expect_costs;
        input [8*16-1:0] name;
        input integer exp_bfs;
        input integer exp_dij;
        begin
            if (bfs_found && dij_found && ast_found && dfs_found &&
                bfs_path_cost == exp_bfs && dij_path_cost == exp_dij &&
                ast_path_cost == exp_dij && dfs_path_cost >= exp_dij) begin
                $display("PASS %0s: BFS steps=%0d (%0d cyc, %0d exp), Dijkstra cost=%0d (%0d cyc, %0d exp), A* cost=%0d (%0d cyc, %0d exp), DFS cost=%0d (%0d cyc, %0d exp)",
                         name, bfs_path_cost, bfs_cycles, bfs_expansions,
                         dij_path_cost, dij_cycles, dij_expansions,
                         ast_path_cost, ast_cycles, ast_expansions,
                         dfs_path_cost, dfs_cycles, dfs_expansions);
            end else begin
                $display("FAIL %0s: bfs(found=%b cost=%0d) dij(found=%b cost=%0d) ast(found=%b cost=%0d) dfs(found=%b cost=%0d)",
                         name, bfs_found, bfs_path_cost, dij_found, dij_path_cost,
                         ast_found, ast_path_cost, dfs_found, dfs_path_cost);
                errors = errors + 1;
            end
        end
    endtask

    task open_grid;
        integer i;
        begin
            for (i = 0; i < NUM_CELLS; i = i + 1) cost[i] = `ARENA_COST_OPEN;
        end
    endtask

    task corridor_grid;
        integer r, c;
        begin
            for (r = 0; r < ROWS; r = r + 1)
                for (c = 0; c < COLS; c = c + 1)
                    cost[r*COLS + c] = (r % 2 == 0) ? `ARENA_COST_OPEN : `ARENA_COST_WALL;
            for (r = 1; r < ROWS - 1; r = r + 2)
                cost[r*COLS + ((r % 4 == 1) ? COLS - 1 : 0)] = `ARENA_COST_OPEN;
            for (c = 0; c < COLS; c = c + 1)
                cost[(ROWS-1)*COLS + c] = `ARENA_COST_WALL;
        end
    endtask

    // replay every run of a shared maze (sim/mazes/README.md format)
    task run_maze_file;
        input [8*40-1:0] mem_file;
        input [8*40-1:0] exp_file;
        input [8*40-1:0] dfs_file;
        integer fd, fd2, r, r2, rs, rg, ef, eb, ed, ea, ec, n, bad;
        integer ds, dg, dfnd, dcost, dcells, dexp, dcyc;
        begin
            $readmemh(mem_file, cost);
            fd  = $fopen(exp_file, "r");
            fd2 = $fopen(dfs_file, "r");
            if (fd == 0 || fd2 == 0) begin
                $display("FAIL cannot open %0s", exp_file);
                errors = errors + 1;
            end else begin
                n = 0;
                r = $fscanf(fd, "%d %d %d %d %d %d %d\n", rs, rg, ef, eb, ed, ea, ec);
                while (r == 7) begin
                    r2 = $fscanf(fd2, "%d %d %d %d %d %d %d\n", ds, dg, dfnd, dcost, dcells, dexp, dcyc);
                    run(rs, rg);
                    bad = 0;
                    // DFS: same runs, same order as <name>.expect
                    if (r2 != 7 || ds != rs || dg != rg || dfnd != ef) bad = bad + 1;
                    if (dfs_found != ef[0] || dfs_no_path != !ef[0]) bad = bad + 1;
                    if (dfs_path_cost != dcost || dfs_path_cells != dcells) bad = bad + 1;
                    if (ef && dfs_path_cost < dij_path_cost) bad = bad + 1;
                    if (ef) begin
                        if (!(bfs_found && dij_found && ast_found)) bad = bad + 1;
                        if (bfs_path_cost != eb || bfs_path_cells != ec) bad = bad + 1;
                        if (dij_path_cost != ed || ast_path_cost != ea) bad = bad + 1;
                        if (dij_path_cost != ast_path_cost) bad = bad + 1;
                    end else begin
                        if (!(bfs_no_path && dij_no_path && ast_no_path)) bad = bad + 1;
                        if (bfs_found || dij_found || ast_found) bad = bad + 1;
                    end
                    if (bad != 0) begin
                        $display("FAIL %0s %0d->%0d: exp found=%0d bfs=%0d dij=%0d ast=%0d dfs=%0d | got bfs(%b,%0d) dij(%b,%0d) ast(%b,%0d) dfs(%b,%0d)",
                                 mem_file, rs, rg, ef, eb, ed, ea, dcost,
                                 bfs_found, bfs_path_cost, dij_found, dij_path_cost,
                                 ast_found, ast_path_cost, dfs_found, dfs_path_cost);
                        errors = errors + 1;
                    end
                    n = n + 1;
                    r = $fscanf(fd, "%d %d %d %d %d %d %d\n", rs, rg, ef, eb, ed, ea, ec);
                end
                $fclose(fd);
                $fclose(fd2);
                if (n == 0) begin
                    $display("FAIL %0s: no runs read", exp_file);
                    errors = errors + 1;
                end else begin
                    $display("DONE %0s: %0d runs", mem_file, n);
                end
            end
        end
    endtask

    initial begin
        $display("=== PATHFIND_TOP SELF-TEST ===");
        errors = 0;
        reset();

        // ---- Scenario A: open grid, concurrent run ----------------------
        open_grid();
        run(0, 191);
        expect_costs("open", 26, 26);

        // ---- Scenario B: weighted cell, steps != cost --------------------
        open_grid();
        cost[88] = 4'd10;   // (row5, col8) high cost on the direct route
        run(80, 95);
        expect_costs("weighted", 15, 17);

        // ---- Scenario C: corridor, unit cost ------------------------------
        corridor_grid();
        run(0, 160);
        expect_costs("corridor", 100, 100);

        // shared cell_rd_addr reaches all four engines' cell_state ports
        cell_rd_addr = 0; #1;
        if (bfs_cell_state != `ARENA_CS_PATH || dij_cell_state != `ARENA_CS_PATH ||
            ast_cell_state != `ARENA_CS_PATH || dfs_cell_state != `ARENA_CS_PATH) begin
            $display("FAIL cell_state: bfs=%0d dij=%0d ast=%0d dfs=%0d at start cell",
                     bfs_cell_state, dij_cell_state, ast_cell_state, dfs_cell_state);
            errors = errors + 1;
        end

        // ---- Shared mazes vs the reference model (CONTRACT 10.2/10.3) -----
        run_maze_file("sim/mazes/open.mem",        "sim/mazes/open.expect", "sim/mazes/open.dfs.expect");
        run_maze_file("sim/mazes/weighted.mem",    "sim/mazes/weighted.expect", "sim/mazes/weighted.dfs.expect");
        run_maze_file("sim/mazes/unreachable.mem", "sim/mazes/unreachable.expect", "sim/mazes/unreachable.dfs.expect");
        run_maze_file("sim/mazes/corridor.mem",    "sim/mazes/corridor.expect", "sim/mazes/corridor.dfs.expect");
        run_maze_file("sim/mazes/rand0.mem",       "sim/mazes/rand0.expect", "sim/mazes/rand0.dfs.expect");
        run_maze_file("sim/mazes/rand1.mem",       "sim/mazes/rand1.expect", "sim/mazes/rand1.dfs.expect");
        run_maze_file("sim/mazes/rand2.mem",       "sim/mazes/rand2.expect", "sim/mazes/rand2.dfs.expect");
        run_maze_file("sim/mazes/rand3.mem",       "sim/mazes/rand3.expect", "sim/mazes/rand3.dfs.expect");
        run_maze_file("sim/mazes/rand4.mem",       "sim/mazes/rand4.expect", "sim/mazes/rand4.dfs.expect");
        run_maze_file("sim/mazes/rand5.mem",       "sim/mazes/rand5.expect", "sim/mazes/rand5.dfs.expect");
        run_maze_file("sim/mazes/rand6.mem",       "sim/mazes/rand6.expect", "sim/mazes/rand6.dfs.expect");
        run_maze_file("sim/mazes/rand7.mem",       "sim/mazes/rand7.expect", "sim/mazes/rand7.dfs.expect");

        // ---- run_mask: only Dijkstra + DFS selected -----------------------
        // BFS and A* must not start: they keep their previous (done) status
        // and counters; Dijkstra and DFS run normally.
        begin : mask_test
            reg [CNT_W-1:0] bfs_c0, ast_c0;
            open_grid();
            load_all();
            bfs_c0 = bfs_cycles; ast_c0 = ast_cycles;
            run_mask = 4'b1010;
            start_idx = 0; goal_idx = 191;
            launch();
            if (bfs_busy || ast_busy || !dij_busy || !dfs_busy) begin
                $display("FAIL run_mask: busy bfs=%b dij=%b ast=%b dfs=%b (expected 0 1 0 1)",
                         bfs_busy, dij_busy, ast_busy, dfs_busy);
                errors = errors + 1;
            end
            while (!(dij_done && dfs_done)) begin @(posedge clk); #1; end
            if (bfs_cycles != bfs_c0 || ast_cycles != ast_c0 || bfs_busy || ast_busy ||
                dij_path_cost != 26 || dfs_path_cost != 180) begin
                $display("FAIL run_mask: unselected engines changed or selected results wrong");
                errors = errors + 1;
            end else
                $display("PASS run_mask: only Dijkstra (%0d cyc) and DFS (%0d cyc) ran", dij_cycles, dfs_cycles);
            run_mask = 4'b1111;
        end

        if (errors == 0)
            $display("TB_PATHFIND PASS");
        else
            $display("TB_PATHFIND FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
