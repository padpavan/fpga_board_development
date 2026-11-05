`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_perf_probe : closest approach to the goal (CONTRACT 8a)
//   * starts empty (all-ones) after rst and after clr
//   * en = 0 ignores current (the stale cell of a previous run)
//   * IDX_NONE ignored
//   * best tracks the minimum and never rises
//   * every cell against every goal matches an independent Manhattan
//////////////////////////////////////////////////////////////////////////////
module tb_ast_perf_probe;
    localparam COLS      = `ARENA_COLS;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam BW        = `ARENA_PG_BEST_W;
    localparam [BW-1:0] EMPTY = {BW{1'b1}};

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst = 1'b1, clr = 1'b0, en = 1'b0;
    reg [INDEX_W-1:0] current = 0, goal = 0;
    wire [BW-1:0] best;

    ast_perf_probe dut (.clk(clk), .rst(rst), .clr(clr), .en(en),
                        .current(current), .goal_idx(goal), .best(best));

    integer errors = 0, c, g, h, expect_best;

    task check(input [BW-1:0] want, input [8*40-1:0] msg);
        begin
            if (best !== want) begin
                errors = errors + 1;
                if (errors <= 20) $display("  FAIL %0s: best=%0d want=%0d", msg, best, want);
            end
        end
    endtask

    task apply(input e, input [INDEX_W-1:0] c_idx);
        begin
            @(negedge clk); en = e; current = c_idx;
            @(negedge clk); en = 1'b0;
        end
    endtask

    function integer dist(input integer a, input integer b);
        integer dr, dc;
        begin
            dr = a / COLS - b / COLS; if (dr < 0) dr = -dr;
            dc = a % COLS - b % COLS; if (dc < 0) dc = -dc;
            dist = dr + dc;
        end
    endfunction

    initial begin
        repeat (3) @(negedge clk);
        rst = 1'b0;
        @(negedge clk) check(EMPTY, "after rst");

        goal = 8'd191;
        apply(1'b0, 8'd191);                 check(EMPTY, "en=0 ignores current");
        apply(1'b1, `ARENA_IDX_NONE);        check(EMPTY, "IDX_NONE ignored");
        apply(1'b1, 8'd0);                   check(5'd26, "corner");
        apply(1'b1, 8'd190);                 check(5'd1,  "closer");
        apply(1'b1, 8'd0);                   check(5'd1,  "never rises");
        apply(1'b1, `ARENA_IDX_NONE);        check(5'd1,  "IDX_NONE keeps best");
        apply(1'b1, 8'd191);                 check(5'd0,  "goal");

        @(negedge clk) clr = 1'b1;
        @(negedge clk) clr = 1'b0;           check(EMPTY, "clr");
        apply(1'b1, 8'd0);                   check(5'd26, "after clr");
        @(negedge clk) rst = 1'b1;
        @(negedge clk) rst = 1'b0;           check(EMPTY, "rst");

        // every goal: walk all cells, best must equal the running minimum
        for (g = 0; g < NUM_CELLS; g = g + 1) begin
            @(negedge clk) clr = 1'b1; goal = g;
            @(negedge clk) clr = 1'b0; en = 1'b1;
            expect_best = EMPTY;
            for (c = NUM_CELLS - 1; c >= 0; c = c - 1) begin
                current = (c * 37) % NUM_CELLS;     // not in a neat order
                h = dist(current, g);
                if (h < expect_best) expect_best = h;
                @(negedge clk) check(expect_best[BW-1:0], "running minimum");
            end
            en = 1'b0;
        end

        if (errors == 0) $display("PASS: tb_ast_perf_probe");
        else             $display("FAIL: tb_ast_perf_probe errors=%0d", errors);
        $finish;
    end
endmodule
