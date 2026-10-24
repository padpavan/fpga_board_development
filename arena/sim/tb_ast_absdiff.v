`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// tb_ast_absdiff : exhaustive self-check of ast_absdiff
//
// Tries every (a, b) pair for a 4-bit and an 8-bit instance and compares
// against an integer model computed here in the testbench.  Ends with
// "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//
// ast_absdiff is combinational, so clk/rst are only here to follow the
// CONTRACT 10.5 testbench convention; inputs change on the falling edge and
// are checked just before the next one.
//////////////////////////////////////////////////////////////////////////////
module tb_ast_absdiff;
    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                       // 100 MHz

    reg  [3:0] a4, b4;
    wire [3:0] d4;
    reg  [7:0] a8, b8;
    wire [7:0] d8;

    ast_absdiff #(.W(4)) dut4 (.a(a4), .b(b4), .d(d4));
    ast_absdiff #(.W(8)) dut8 (.a(a8), .b(b8), .d(d8));

    integer i, j, expect, errors, checks;

    initial begin
        errors = 0;
        checks = 0;
        a4 = 0; b4 = 0; a8 = 0; b8 = 0;
        repeat (2) @(posedge clk);
        rst = 1'b0;

        // ---- 4-bit: all 16 x 16 pairs ------------------------------------
        for (i = 0; i < 16; i = i + 1)
            for (j = 0; j < 16; j = j + 1) begin
                a4 = i; b4 = j;
                #1;
                expect = (i >= j) ? i - j : j - i;
                checks = checks + 1;
                if (d4 !== expect) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("  mismatch W=4: |%0d-%0d| got %0d expect %0d", i, j, d4, expect);
                end
            end

        // ---- 8-bit: all 256 x 256 pairs ----------------------------------
        for (i = 0; i < 256; i = i + 1)
            for (j = 0; j < 256; j = j + 1) begin
                a8 = i; b8 = j;
                #1;
                expect = (i >= j) ? i - j : j - i;
                checks = checks + 1;
                if (d8 !== expect) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("  mismatch W=8: |%0d-%0d| got %0d expect %0d", i, j, d8, expect);
                end
            end

        if (errors == 0)
            $display("PASS: tb_ast_absdiff (%0d checks, 0 errors)", checks);
        else
            $display("FAIL: tb_ast_absdiff (%0d checks, %0d errors)", checks, errors);
        $finish;
    end
endmodule
