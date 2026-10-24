`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_test_maze : the board-test maze ROM must equal the shared mazes
//
// For each sel, every cell of ast_test_maze is compared with the matching
// sim/mazes/<name>.mem written by ref/gen_mazes.py, and start/goal are
// compared with the first line of <name>.ast.expect.  This is what lets the
// numbers seen on the board be checked against the reference model.
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_ast_test_maze;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam COST_W    = `ARENA_COST_W;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                       // 100 MHz (CONTRACT 10.5)

    reg  [1:0]         sel = 0;
    reg  [INDEX_W-1:0] idx = 0;
    wire [COST_W-1:0]  cost;
    wire [INDEX_W-1:0] start_idx, goal_idx;

    ast_test_maze dut (.sel(sel), .idx(idx), .cost(cost),
                       .start_idx(start_idx), .goal_idx(goal_idx));

    reg [COST_W-1:0] ref_cost [0:NUM_CELLS-1];
    integer errors = 0;

    task check_maze(input [1:0] s, input [8*48-1:0] mem_file, input [8*48-1:0] exp_file);
        integer i, fd, r, es, eg, x1, x2, x3, x4, bad;
        begin
            sel = s;
            $readmemh(mem_file, ref_cost);
            bad = 0;
            for (i = 0; i < NUM_CELLS; i = i + 1) begin
                idx = i;
                #1;
                if (cost !== ref_cost[i]) begin
                    bad = bad + 1;
                    if (bad <= 5)
                        $display("  sel %0d cell %0d: got %0d, file has %0d", s, i, cost, ref_cost[i]);
                end
            end
            fd = $fopen(exp_file, "r");
            r  = $fscanf(fd, "%d %d %d %d %d %d", es, eg, x1, x2, x3, x4);
            $fclose(fd);
            if (r != 6 || start_idx !== es || goal_idx !== eg) begin
                bad = bad + 1;
                $display("  sel %0d: start/goal %0d/%0d, expect file says %0d/%0d", s, start_idx, goal_idx, es, eg);
            end
            $display("  sel %0d %0s: %0s", s, mem_file, bad ? "MISMATCH" : "matches");
            errors = errors + bad;
        end
    endtask

    initial begin
        repeat (2) @(posedge clk);
        rst = 1'b0;
        check_maze(0, "sim/mazes/open.mem",        "sim/mazes/open.ast.expect");
        check_maze(1, "sim/mazes/weighted.mem",    "sim/mazes/weighted.ast.expect");
        check_maze(2, "sim/mazes/unreachable.mem", "sim/mazes/unreachable.ast.expect");
        check_maze(3, "sim/mazes/corridor.mem",    "sim/mazes/corridor.ast.expect");
        if (errors == 0)
            $display("PASS: tb_ast_test_maze (4 mazes, 0 errors)");
        else
            $display("FAIL: tb_ast_test_maze (%0d errors)", errors);
        $finish;
    end
endmodule
