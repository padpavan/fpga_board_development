`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_sat_counter : self-check of the saturating profiling counter
//
// Two DUTs share the same rst/clr/inc:
//   dut4  : W = 4, so saturation (15) is reachable in a few cycles
//   dut32 : W = ARENA_CNT_W, the real `cycles` width (never saturates here,
//           so it must behave like a plain counter)
//
// A software model (m4 / m32) is updated on the same clock edges, and the
// outputs are compared after every edge.  Directed phases first, then 2000
// cycles of random inc/clr (plus occasional rst).
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//
// Timing convention: inputs change on the FALLING edge, so they are stable
// at the rising edge the DUT samples them on; outputs are checked #1 after
// the rising edge.
//////////////////////////////////////////////////////////////////////////////
module tb_ast_sat_counter;
    localparam CNT_W = `ARENA_CNT_W;

    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    reg rst = 1'b1;
    reg clr = 1'b0;
    reg inc = 1'b0;

    wire [3:0]       q4;
    wire [CNT_W-1:0] q32;

    ast_sat_counter #(.W(4))     dut4  (.clk(clk), .rst(rst), .clr(clr), .inc(inc), .q(q4));
    ast_sat_counter #(.W(CNT_W)) dut32 (.clk(clk), .rst(rst), .clr(clr), .inc(inc), .q(q32));

    // ---- reference model (same rules, written as plain integer code) -----
    integer m4, m32;
    always @(posedge clk) begin
        if (rst || clr) begin
            m4  <= 0;
            m32 <= 0;
        end else if (inc) begin
            if (m4 < 15) m4 <= m4 + 1;          // saturate at 2^4 - 1
            m32 <= m32 + 1;                     // 32-bit never reaches MAX here
        end
    end

    // ---- compare DUT vs model after every rising edge --------------------
    integer errors = 0;
    integer checks = 0;
    always @(posedge clk) begin
        #1;
        checks = checks + 1;
        if (q4 !== m4 || q32 !== m32) begin
            errors = errors + 1;
            if (errors <= 10)
                $display("  t=%0t mismatch: q4=%0d (model %0d) q32=%0d (model %0d)  rst=%b clr=%b inc=%b",
                         $time, q4, m4, q32, m32, rst, clr, inc);
        end
    end

    // ---- helpers: drive inputs for n cycles, changing on the falling edge -
    task drive(input r, input c, input i, input integer n);
        begin
            @(negedge clk);
            rst = r; clr = c; inc = i;
            repeat (n - 1) @(negedge clk);
        end
    endtask

    // Directed check of an exact value (on top of the model compare).
    task expect4(input integer v, input [255:0] what);
        begin
            @(posedge clk); #2;
            if (q4 !== v) begin
                errors = errors + 1;
                $display("  %0s: q4 = %0d, expected %0d", what, q4, v);
            end
        end
    endtask

    integer k;
    initial begin
        // reset held for 2 cycles (CONTRACT 10.5)
        drive(1, 0, 1, 2);                      // inc high during rst: must stay 0
        expect4(0, "during rst with inc=1");

        drive(0, 0, 1, 5);                      // count 5
        drive(0, 0, 0, 3);                      // hold 3 cycles
        expect4(5, "count 5 then hold");

        drive(0, 1, 0, 1);                      // clear
        drive(0, 0, 0, 1);
        expect4(0, "after clr");

        drive(0, 0, 1, 20);                     // 20 incs on a 4-bit counter
        expect4(15, "saturate at 15");
        drive(0, 0, 1, 5);                      // keep pushing: must not wrap
        expect4(15, "still 15 after more incs");

        drive(0, 1, 1, 1);                      // clr and inc together: clr wins
        drive(0, 0, 0, 1);
        expect4(0, "clr wins over inc");

        drive(0, 0, 1, 7);
        drive(1, 0, 1, 1);                      // rst mid-count
        drive(0, 0, 0, 1);
        expect4(0, "rst mid-count");

        // ---- random phase: model compare catches any divergence ----------
        for (k = 0; k < 2000; k = k + 1) begin
            @(negedge clk);
            inc = ($random & 3) != 0;           // inc ~75% of cycles
            clr = ($random % 64) == 0;          // occasional new run
            rst = ($random % 512) == 0;         // rare reset
        end
        @(negedge clk);
        rst = 0; clr = 0; inc = 0;
        @(posedge clk); #2;

        if (errors == 0)
            $display("PASS: tb_ast_sat_counter (%0d cycle checks, 0 errors)", checks);
        else
            $display("FAIL: tb_ast_sat_counter (%0d cycle checks, %0d errors)", checks, errors);
        $finish;
    end
endmodule
