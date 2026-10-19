`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Self-test for pathfind_top : both engines run concurrently on the same
// frozen maze and start signal.
//
//   Scenario A (open grid): both find a route; equal unit weights -> equal
//                           optimal length (BFS steps == Dijkstra cost == 26).
//   Scenario B (weighted) : BFS reports minimum steps (15), Dijkstra reports
//                           minimum weighted cost (17), demonstrating
//                           "fewest steps != least cost".
//////////////////////////////////////////////////////////////////////////////
module tb_pathfind;
    parameter COLS=16, ROWS=12, COST_W=4, DIST_W=16, INDEX_W=8;
    localparam NUM_CELLS = COLS * ROWS;

    reg        clk = 0;
    reg        rst = 0;
    reg        load_we = 0;
    reg  [INDEX_W-1:0] load_addr = 0;
    reg  [COST_W-1:0]  load_data = 0;
    reg        start = 0;
    reg  [INDEX_W-1:0] start_idx = 0;
    reg  [INDEX_W-1:0] goal_idx  = 0;

    wire bfs_busy, bfs_done, bfs_found, bfs_no_path;
    reg  [INDEX_W-1:0] bfs_path_rd_addr = 0;
    wire [INDEX_W-1:0] bfs_path_rd_data;
    wire [DIST_W-1:0]  bfs_path_cells, bfs_path_cost, bfs_cycles, bfs_expansions;
    wire [INDEX_W-1:0] bfs_current;

    wire dij_busy, dij_done, dij_found, dij_no_path;
    reg  [INDEX_W-1:0] dij_path_rd_addr = 0;
    wire [INDEX_W-1:0] dij_path_rd_data;
    wire [DIST_W-1:0]  dij_path_cells, dij_path_cost, dij_cycles, dij_expansions;
    wire [INDEX_W-1:0] dij_current;

    pathfind_top #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W)
    ) dut (
        .clk(clk), .rst(rst),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .bfs_busy(bfs_busy), .bfs_done(bfs_done), .bfs_found(bfs_found), .bfs_no_path(bfs_no_path),
        .bfs_path_rd_addr(bfs_path_rd_addr), .bfs_path_rd_data(bfs_path_rd_data),
        .bfs_path_cells(bfs_path_cells), .bfs_path_cost(bfs_path_cost),
        .bfs_current(bfs_current), .bfs_cycles(bfs_cycles), .bfs_expansions(bfs_expansions),
        .dij_busy(dij_busy), .dij_done(dij_done), .dij_found(dij_found), .dij_no_path(dij_no_path),
        .dij_path_rd_addr(dij_path_rd_addr), .dij_path_rd_data(dij_path_rd_data),
        .dij_path_cells(dij_path_cells), .dij_path_cost(dij_path_cost),
        .dij_current(dij_current), .dij_cycles(dij_cycles), .dij_expansions(dij_expansions)
    );

    reg [COST_W-1:0] cost [0:NUM_CELLS-1];
    integer errors;

    always #5 clk = ~clk;

    task reset;
        begin
            rst = 1'b1;
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

    task wait_both_done;
        integer timeout;
        begin
            timeout = 0;
            while (!(bfs_done && dij_done) && timeout < 500000) begin
                @(posedge clk); #1;
                timeout = timeout + 1;
            end
            if (timeout >= 500000) begin
                $display("TIMEOUT waiting for both engines");
                $finish;
            end
        end
    endtask

    task open_grid;
        integer i;
        begin
            for (i = 0; i < NUM_CELLS; i = i + 1) cost[i] = 4'd1;
        end
    endtask

    initial begin
        $display("=== PATHFIND_TOP SELF-TEST ===");
        errors = 0;

        // ---- Scenario A: open grid, concurrent run ----------------------
        open_grid();
        reset();
        load_all();
        start_idx = 0; goal_idx = 191;
        launch();
        wait_both_done();
        if (bfs_found && dij_found && bfs_path_cost == 26 && dij_path_cost == 26) begin
            $display("PASS open: BFS steps=%0d, Dijkstra cost=%0d",
                     bfs_path_cost, dij_path_cost);
        end else begin
            $display("FAIL open: bfs(found=%b cost=%0d) dij(found=%b cost=%0d)",
                     bfs_found, bfs_path_cost, dij_found, dij_path_cost);
            errors = errors + 1;
        end

        // ---- Scenario B: weighted cell, steps != cost --------------------
        open_grid();
        cost[88] = 4'd10;   // (row5, col8) high cost on the direct route
        reset();
        load_all();
        start_idx = 80; goal_idx = 95;
        launch();
        wait_both_done();
        if (bfs_found && dij_found && bfs_path_cost == 15 && dij_path_cost == 17) begin
            $display("PASS weighted: BFS steps=%0d, Dijkstra cost=%0d",
                     bfs_path_cost, dij_path_cost);
        end else begin
            $display("FAIL weighted: bfs(found=%b cost=%0d) dij(found=%b cost=%0d)",
                     bfs_found, bfs_path_cost, dij_found, dij_path_cost);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_PATHFIND PASS");
        else
            $display("TB_PATHFIND FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
