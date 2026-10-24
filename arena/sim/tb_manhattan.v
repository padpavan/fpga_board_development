`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_manhattan : exhaustive self-check of manhattan.v
//
// For every pair of cells (a, b) in the 16x12 grid (192 x 192 = 36 864
// pairs) it checks:
//   1. h == |row_a - row_b| + |col_a - col_b|  (integer model computed here)
//   2. h(a, a) == 0
//   3. symmetry: h(a, b) == h(b, a)   (second instance with swapped inputs)
// and reports the largest h seen (should be (ROWS-1) + (COLS-1) = 26).
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//
// manhattan is combinational; clk/rst follow the CONTRACT 10.5 convention.
//////////////////////////////////////////////////////////////////////////////
module tb_manhattan;
    localparam COLS      = `ARENA_COLS;
    localparam ROWS      = `ARENA_ROWS;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam DIST_W    = `ARENA_DIST_W;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                       // 100 MHz

    reg  [INDEX_W-1:0] a_idx, b_idx;
    wire [DIST_W-1:0]  h_ab, h_ba;

    // Same module twice, inputs swapped, to check symmetry in one pass.
    manhattan dut_ab (.a_idx(a_idx), .b_idx(b_idx), .h(h_ab));
    manhattan dut_ba (.a_idx(b_idx), .b_idx(a_idx), .h(h_ba));

    integer a, b, dr, dc, expect, h_max, errors, checks;

    initial begin
        errors = 0;
        checks = 0;
        h_max  = 0;
        a_idx  = 0;
        b_idx  = 0;
        repeat (2) @(posedge clk);
        rst = 1'b0;

        for (a = 0; a < NUM_CELLS; a = a + 1)
            for (b = 0; b < NUM_CELLS; b = b + 1) begin
                a_idx = a;
                b_idx = b;
                #1;                             // let combinational logic settle

                // Reference model: split index into row/col with integers.
                dr = (a / COLS) - (b / COLS);
                dc = (a % COLS) - (b % COLS);
                if (dr < 0) dr = -dr;
                if (dc < 0) dc = -dc;
                expect = dr + dc;
                checks = checks + 1;

                if (h_ab !== expect) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("  mismatch h(%0d,%0d) got %0d expect %0d", a, b, h_ab, expect);
                end
                if (h_ab !== h_ba) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("  asymmetric h(%0d,%0d)=%0d but h(%0d,%0d)=%0d", a, b, h_ab, b, a, h_ba);
                end
                if (a == b && h_ab !== 0) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("  h(%0d,%0d) should be 0, got %0d", a, a, h_ab);
                end
                if (h_ab > h_max) h_max = h_ab;
            end

        if (h_max !== (ROWS - 1) + (COLS - 1)) begin
            errors = errors + 1;
            $display("  max h was %0d, expected %0d", h_max, (ROWS - 1) + (COLS - 1));
        end

        $display("  checked %0d pairs, max h = %0d", checks, h_max);
        if (errors == 0)
            $display("PASS: tb_manhattan (%0d pairs, 0 errors)", checks);
        else
            $display("FAIL: tb_manhattan (%0d pairs, %0d errors)", checks, errors);
        $finish;
    end
endmodule
