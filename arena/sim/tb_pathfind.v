`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Self-test for pathfind_top : all three engines run concurrently on the
// same frozen maze and start signal (CONTRACT.md sections 5 and 10.3).
//
//   Scenario A (open grid): unit cost -> bfs.path_cost == dij.path_cost (26).
//   Scenario B (weighted) : BFS reports minimum steps (15), Dijkstra reports
//                           minimum weighted cost (17), demonstrating
//                           "fewest steps != least cost".
//   Scenario C (corridor) : unit cost, unique path -> both 100.
// A* is still M2's stub: it is only required to reach `done`.  Once the real
// engine lands, add the dij.path_cost == ast.path_cost cross-check here.
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

    pathfind_top #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) dut (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
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
        .ast_cycles(ast_cycles), .ast_expansions(ast_expansions)
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
            while (!(bfs_done && dij_done && ast_done) && timeout < 500000) begin
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
            if (bfs_found && dij_found &&
                bfs_path_cost == exp_bfs && dij_path_cost == exp_dij) begin
                $display("PASS %0s: BFS steps=%0d (%0d cyc, %0d exp), Dijkstra cost=%0d (%0d cyc, %0d exp)",
                         name, bfs_path_cost, bfs_cycles, bfs_expansions,
                         dij_path_cost, dij_cycles, dij_expansions);
            end else begin
                $display("FAIL %0s: bfs(found=%b cost=%0d) dij(found=%b cost=%0d)",
                         name, bfs_found, bfs_path_cost, dij_found, dij_path_cost);
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

        // shared cell_rd_addr reaches both engines' cell_state ports
        cell_rd_addr = 0; #1;
        if (bfs_cell_state != `ARENA_CS_PATH || dij_cell_state != `ARENA_CS_PATH) begin
            $display("FAIL cell_state: bfs=%0d dij=%0d at start cell",
                     bfs_cell_state, dij_cell_state);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_PATHFIND PASS");
        else
            $display("TB_PATHFIND FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
