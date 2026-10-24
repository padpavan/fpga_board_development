`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Self-test for bfs_engine (CONTRACT.md sections 4 and 10) on hand-worked
// 16x12 mazes.  Runs back-to-back without rst (restart from DONE):
//   1. open grid (all cost 1)  : 0 -> 191, expect 26 steps, 27 cells
//   2. weighted cell           : 80 -> 95, cell (5,8) cost 10
//                                (BFS ignores cost, still 15 steps)
//   3. blocked grid (wall col) : 0 -> 191, expect no_path
//   4. start == goal           : 37 -> 37, expect 1 cell, cost 0
//   5. start on wall / goal on wall : expect no_path
//   6. corridor (serpentine)   : 0 -> 160, unique path of 100 steps
//   7. step_en pause mid-run   : state frozen, same final cycles
//   8. rst clears status / counters
// Every found path is checked for endpoints, adjacency, no walls, path_cost
// and cell_state == PATH; `cycles` is checked against a TB cycle count.
//////////////////////////////////////////////////////////////////////////////
module tb_bfs;
    parameter COLS = `ARENA_COLS, ROWS = `ARENA_ROWS, COST_W = `ARENA_COST_W;
    parameter DIST_W = `ARENA_DIST_W, INDEX_W = `ARENA_INDEX_W, CNT_W = `ARENA_CNT_W;
    localparam NUM_CELLS = COLS * ROWS;
    localparam IS_DIJ    = 0;      // BFS: path_cost = steps

    reg        clk = 0;
    reg        rst = 0;
    reg        step_en = 1;
    reg        load_we = 0;
    reg  [INDEX_W-1:0] load_addr = 0;
    reg  [COST_W-1:0]  load_data = 0;
    reg        start = 0;
    reg  [INDEX_W-1:0] start_idx = 0;
    reg  [INDEX_W-1:0] goal_idx  = 0;
    wire       busy, done, found, no_path;
    reg  [INDEX_W-1:0] path_rd_addr = 0;
    wire [INDEX_W-1:0] path_rd_data;
    wire [DIST_W-1:0]  path_cells, path_cost, expansions;
    wire [CNT_W-1:0]   cycles;
    wire [INDEX_W-1:0] current;
    reg  [INDEX_W-1:0] cell_rd_addr = 0;
    wire [1:0]         cell_state;

    bfs_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) dut (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(busy), .done(done), .found(found), .no_path(no_path),
        .path_rd_addr(path_rd_addr), .path_rd_data(path_rd_data),
        .path_cells(path_cells), .path_cost(path_cost),
        .current(current),
        .cell_rd_addr(cell_rd_addr), .cell_state(cell_state),
        .cycles(cycles), .expansions(expansions)
    );

    reg [COST_W-1:0] cost [0:NUM_CELLS-1];
    integer errors;
    integer tb_cycles;     // TB-side count of clk edges from accept to done
    reg [INDEX_W-1:0] exp_s, exp_g;   // start/goal of the current run

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
            tb_cycles = 0;
            if (!busy || done) begin
                $display("FAIL handshake: busy=%b done=%b after start", busy, done);
                errors = errors + 1;
            end
        end
    endtask

    task wait_done;
        begin
            while (!done && tb_cycles < 500000) begin
                @(posedge clk); #1;
                if (step_en) tb_cycles = tb_cycles + 1;
            end
            if (tb_cycles >= 500000) begin
                $display("TIMEOUT waiting for done");
                $finish;
            end
        end
    endtask

    // status / metric checks common to every finished run
    task check_finish;
        input [8*16-1:0] name;
        begin
            if (busy || (found == no_path)) begin
                $display("FAIL %0s: busy=%b found=%b no_path=%b", name, busy, found, no_path);
                errors = errors + 1;
            end
            if (cycles != tb_cycles) begin
                $display("FAIL %0s: cycles=%0d expected %0d", name, cycles, tb_cycles);
                errors = errors + 1;
            end
            if (no_path && (path_cells != 0 || path_cost != 0)) begin
                $display("FAIL %0s: no_path but cells=%0d cost=%0d", name, path_cells, path_cost);
                errors = errors + 1;
            end
        end
    endtask

    // walk the path RAM: endpoints, 4-adjacency, no walls, cost, cell_state
    task check_path;
        input [8*16-1:0] name;
        integer i, a, b, sum, bad;
        begin
            bad = 0;
            sum = 0;
            b   = 0;
            for (i = 0; i < path_cells; i = i + 1) begin
                path_rd_addr = i; #1;
                a = b;
                b = path_rd_data;
                if (i == 0 && b != exp_s) bad = bad + 1;
                if (i == path_cells - 1 && b != exp_g) bad = bad + 1;
                if (b >= NUM_CELLS || cost[b] == `ARENA_COST_WALL) bad = bad + 1;
                if (i > 0) begin
                    if (!((a - b == COLS) || (b - a == COLS) ||
                          ((a - b == 1 || b - a == 1) && (a / COLS == b / COLS))))
                        bad = bad + 1;
                    sum = sum + cost[b];
                end
                cell_rd_addr = b; #1;
                if (cell_state != `ARENA_CS_PATH) bad = bad + 1;
            end
            if (!IS_DIJ) sum = path_cells - 1;
            if (sum != path_cost) bad = bad + 1;
            if (bad != 0) begin
                $display("FAIL %0s: path check (%0d problems, cost=%0d, sum=%0d)",
                         name, bad, path_cost, sum);
                errors = errors + 1;
            end
        end
    endtask

    task expect_found;
        input [8*16-1:0] name;
        input integer exp_cost;
        input integer exp_cells;
        begin
            check_finish(name);
            if (found && path_cost == exp_cost && path_cells == exp_cells) begin
                check_path(name);
                $display("PASS %0s: cost=%0d cells=%0d cycles=%0d expansions=%0d",
                         name, path_cost, path_cells, cycles, expansions);
            end else begin
                $display("FAIL %0s: found=%b cost=%0d (exp %0d) cells=%0d (exp %0d)",
                         name, found, path_cost, exp_cost, path_cells, exp_cells);
                errors = errors + 1;
            end
        end
    endtask

    task expect_no_path;
        input [8*16-1:0] name;
        begin
            check_finish(name);
            if (!found && no_path) begin
                $display("PASS %0s: no_path, cycles=%0d", name, cycles);
            end else begin
                $display("FAIL %0s: found=%b no_path=%b", name, found, no_path);
                errors = errors + 1;
            end
        end
    endtask

    task run;
        input [INDEX_W-1:0] s;
        input [INDEX_W-1:0] g;
        begin
            load_all();
            start_idx = s; goal_idx = g;
            exp_s = s; exp_g = g;
            launch();
            start_idx = 0; goal_idx = 0;   // engine must have sampled on start
            wait_done();
        end
    endtask

    task open_grid;
        integer i;
        begin
            for (i = 0; i < NUM_CELLS; i = i + 1) cost[i] = `ARENA_COST_OPEN;
        end
    endtask

    task blocked_grid;
        integer r;
        begin
            open_grid();
            for (r = 0; r < ROWS; r = r + 1) cost[r*COLS + 8] = `ARENA_COST_WALL;
        end
    endtask

    // even rows open; odd rows walled except one gap alternating at the
    // right / left edge; last row walled.  Unique path 0 -> 160, 100 steps.
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

    integer ref_cycles;
    reg [CNT_W-1:0]   p_cycles;
    reg [DIST_W-1:0]  p_exp;
    reg [INDEX_W-1:0] p_cur;
    reg [1:0]         p_cs;
    integer k, frozen_bad;

    initial begin
        $display("=== BFS ENGINE SELF-TEST ===");
        errors = 0;
        reset();

        // ---- 1: open grid -------------------------------------------------
        open_grid();
        run(0, 191);
        expect_found("open", 26, 27);
        ref_cycles = cycles;

        // done must hold until the next start
        repeat (5) @(posedge clk); #1;
        if (!done || !found) begin
            $display("FAIL done did not hold");
            errors = errors + 1;
        end

        // ---- 2: weighted cell (restart from DONE, no rst) ----------------
        open_grid();
        cost[88] = 4'd10;
        run(80, 95);
        if (IS_DIJ) expect_found("weighted", 17, 18);
        else        expect_found("weighted", 15, 16);

        // ---- 3: blocked grid ----------------------------------------------
        blocked_grid();
        run(0, 191);
        expect_no_path("blocked");
        cell_rd_addr = 8; #1;
        if (cell_state != `ARENA_CS_UNSEEN) begin
            $display("FAIL blocked: wall cell_state=%0d", cell_state);
            errors = errors + 1;
        end

        // ---- 4: start == goal ---------------------------------------------
        open_grid();
        run(37, 37);
        expect_found("start==goal", 0, 1);

        // ---- 5: start / goal on a wall ------------------------------------
        open_grid();
        cost[0] = `ARENA_COST_WALL;
        run(0, 191);
        expect_no_path("start wall");
        open_grid();
        cost[191] = `ARENA_COST_WALL;
        run(0, 191);
        expect_no_path("goal wall");
        run(191, 191);
        expect_no_path("s==g wall");

        // ---- 6: corridor ----------------------------------------------------
        corridor_grid();
        run(0, 160);
        expect_found("corridor", 100, 101);
        cell_rd_addr = COLS; #1;   // (1,0) is a wall
        if (cell_state != `ARENA_CS_UNSEEN) begin
            $display("FAIL corridor: wall cell_state=%0d", cell_state);
            errors = errors + 1;
        end

        // ---- 7: step_en pause ----------------------------------------------
        open_grid();
        load_all();
        start_idx = 0; goal_idx = 191;
        exp_s = 0; exp_g = 191;
        launch();
        repeat (250) begin @(posedge clk); #1; tb_cycles = tb_cycles + 1; end
        step_en = 1'b0;
        cell_rd_addr = 17; #1;
        p_cycles = cycles; p_exp = expansions; p_cur = current; p_cs = cell_state;
        frozen_bad = 0;
        for (k = 0; k < 40; k = k + 1) begin
            @(posedge clk); #1;
            if (cycles != p_cycles || expansions != p_exp || current != p_cur ||
                cell_state != p_cs || !busy || done)
                frozen_bad = frozen_bad + 1;
        end
        step_en = 1'b1;
        wait_done();
        if (frozen_bad != 0) begin
            $display("FAIL pause: state changed while step_en=0 (%0d cycles)", frozen_bad);
            errors = errors + 1;
        end
        expect_found("pause", 26, 27);
        if (cycles != ref_cycles) begin
            $display("FAIL pause: cycles=%0d, unpaused run=%0d", cycles, ref_cycles);
            errors = errors + 1;
        end

        // ---- 8: rst clears status ------------------------------------------
        reset();
        if (busy || done || found || no_path || cycles != 0 || expansions != 0) begin
            $display("FAIL rst: busy=%b done=%b found=%b no_path=%b cycles=%0d",
                     busy, done, found, no_path, cycles);
            errors = errors + 1;
        end else begin
            $display("PASS rst");
        end

        if (errors == 0)
            $display("TB_BFS PASS");
        else
            $display("TB_BFS FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
