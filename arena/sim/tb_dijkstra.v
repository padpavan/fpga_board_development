`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Self-test for dijkstra_engine on three hand-worked 16x12 mazes:
//   1. open grid (all cost 1)  : start 0 -> goal 191, expect cost 26
//   2. weighted cell           : start 80 -> goal 95, cell (5,8) cost 10
//                                expect min cost 17 (detour +2, all cost 1)
//   3. blocked grid (wall col) : start 0 -> goal 191, expect no_path
//////////////////////////////////////////////////////////////////////////////
module tb_dijkstra;
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
    wire       busy, done, found, no_path;
    reg  [INDEX_W-1:0] path_rd_addr = 0;
    wire [INDEX_W-1:0] path_rd_data;
    wire [DIST_W-1:0]  path_cells, path_cost, cycles, expansions;
    wire [INDEX_W-1:0] current;

    dijkstra_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W)
    ) dut (
        .clk(clk), .rst(rst),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(busy), .done(done), .found(found), .no_path(no_path),
        .path_rd_addr(path_rd_addr), .path_rd_data(path_rd_data),
        .path_cells(path_cells), .path_cost(path_cost),
        .current(current),
        .cycles(cycles), .expansions(expansions)
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

    task wait_done;
        integer timeout;
        begin
            timeout = 0;
            while (!done && timeout < 500000) begin
                @(posedge clk); #1;
                timeout = timeout + 1;
            end
            if (timeout >= 500000) begin
                $display("TIMEOUT waiting for done");
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

    task blocked_grid;
        integer r;
        begin
            for (r = 0; r < NUM_CELLS; r = r + 1) cost[r] = 4'd1;
            for (r = 0; r < ROWS; r = r + 1) cost[r*COLS + 8] = 4'd0;
        end
    endtask

    initial begin
        $display("=== DIJKSTRA ENGINE SELF-TEST ===");
        errors = 0;

        // ---- Scenario 1: open grid, expect cost 26 ----------------------
        open_grid();
        reset();
        load_all();
        start_idx = 0; goal_idx = 191;
        launch();
        wait_done();
        if (found && !no_path && path_cost == 26) begin
            $display("PASS open grid: found, cost=%0d", path_cost);
        end else begin
            $display("FAIL open grid: found=%b no_path=%b cost=%0d",
                     found, no_path, path_cost);
            errors = errors + 1;
        end

        // ---- Scenario 2: weighted cell -> expect cost 17 ----------------
        open_grid();
        cost[88] = 4'd10;   // (row5, col8) high cost on the direct route
        reset();
        load_all();
        start_idx = 80; goal_idx = 95;
        launch();
        wait_done();
        if (found && path_cost == 17) begin
            $display("PASS weighted: found, min cost=%0d (detour +2)", path_cost);
        end else begin
            $display("FAIL weighted: found=%b cost=%0d", found, path_cost);
            errors = errors + 1;
        end

        // ---- Scenario 3: blocked grid -> no path ------------------------
        blocked_grid();
        reset();
        load_all();
        start_idx = 0; goal_idx = 191;
        launch();
        wait_done();
        if (!found && no_path) begin
            $display("PASS blocked: no_path detected");
        end else begin
            $display("FAIL blocked: found=%b no_path=%b", found, no_path);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_DIJKSTRA PASS");
        else
            $display("TB_DIJKSTRA FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
