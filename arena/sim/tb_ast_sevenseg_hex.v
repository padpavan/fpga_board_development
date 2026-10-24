`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// tb_ast_sevenseg_hex : self-check of the 4-digit hex display driver
//
// Values 0123, 4567, 89AB, CDEF together use every hex digit once per
// position group.  For each value the bench watches a full refresh period
// and checks that:
//   * exactly one anode is active (low) at a time, and all 4 get a turn
//   * the active digit shows the right nibble (an[0] = value[3:0])
//   * segment patterns match the standard active-high hex table below,
//     inverted (Basys 3 segments are active-low); dp stays off
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_ast_sevenseg_hex;
    localparam REFRESH_W = 4;                   // tiny: digit changes every 4 clocks

    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz
    reg rst = 1'b1;

    reg  [15:0] value = 16'h0000;
    wire [6:0]  seg;
    wire [3:0]  an;
    wire        dp;

    ast_sevenseg_hex #(.REFRESH_W(REFRESH_W)) dut (
        .clk(clk), .rst(rst), .value(value), .seg(seg), .an(an), .dp(dp)
    );

    // Standard hex -> segments, ACTIVE-HIGH, bit0 = a ... bit6 = g.
    function [6:0] hex_segs(input [3:0] n);
        case (n)
            4'h0: hex_segs = 7'h3F; 4'h1: hex_segs = 7'h06; 4'h2: hex_segs = 7'h5B; 4'h3: hex_segs = 7'h4F;
            4'h4: hex_segs = 7'h66; 4'h5: hex_segs = 7'h6D; 4'h6: hex_segs = 7'h7D; 4'h7: hex_segs = 7'h07;
            4'h8: hex_segs = 7'h7F; 4'h9: hex_segs = 7'h6F; 4'hA: hex_segs = 7'h77; 4'hB: hex_segs = 7'h7C;
            4'hC: hex_segs = 7'h39; 4'hD: hex_segs = 7'h5E; 4'hE: hex_segs = 7'h79; default: hex_segs = 7'h71;
        endcase
    endfunction

    integer errors = 0;

    task check_value(input [15:0] v);
        integer k, pos;
        reg [3:0] seen;
        reg [3:0] nib;
        begin
            value = v;
            seen  = 4'b0000;
            for (k = 0; k < (1 << REFRESH_W) + 4; k = k + 1) begin
                @(posedge clk); #1;
                case (an)
                    4'b1110: pos = 0;
                    4'b1101: pos = 1;
                    4'b1011: pos = 2;
                    4'b0111: pos = 3;
                    default: pos = -1;
                endcase
                if (pos < 0) begin
                    errors = errors + 1;
                    $display("  value %h: anodes %b not one-hot-low", v, an);
                end else begin
                    seen[pos] = 1'b1;
                    nib = v >> (4 * pos);
                    if (seg !== ~hex_segs(nib)) begin
                        errors = errors + 1;
                        $display("  value %h digit %0d (%h): seg %b, expected %b", v, pos, nib, seg, ~hex_segs(nib));
                    end
                end
                if (dp !== 1'b1) begin
                    errors = errors + 1;
                    $display("  dp should be off (1)");
                end
            end
            if (seen !== 4'b1111) begin
                errors = errors + 1;
                $display("  value %h: not every digit was driven (%b)", v, seen);
            end
        end
    endtask

    initial begin
        repeat (2) @(posedge clk);
        @(negedge clk) rst = 1'b0;
        check_value(16'h0123);
        check_value(16'h4567);
        check_value(16'h89AB);
        check_value(16'hCDEF);
        if (errors == 0)
            $display("PASS: tb_ast_sevenseg_hex (16 digits x 4 positions, 0 errors)");
        else
            $display("FAIL: tb_ast_sevenseg_hex (%0d errors)", errors);
        $finish;
    end
endmodule
