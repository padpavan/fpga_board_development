`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// clk_div : the single shared clock divider (common/, contract section 2).
//
// clk_out = clk / DIV with 50% duty.  DIV must be even and >= 2.
// The default DIV = 16 gives 100 MHz / 16 = 6.25 MHz for Oled_Display.
//
// Only Oled_Display may be clocked by a divided clock.  Everything else runs
// on the 100 MHz clock and uses a 1-cycle clock-enable tick instead.
//////////////////////////////////////////////////////////////////////////////
module clk_div #(
    parameter DIV = 16
) (
    input  wire clk,
    output reg  clk_out = 1'b0
);
    localparam HALF = DIV / 2;
    localparam CW   = (HALF < 2) ? 1 : $clog2(HALF);

    reg [CW-1:0] count = 0;

    always @(posedge clk) begin
        if (count == HALF - 1) begin
            count   <= 0;
            clk_out <= ~clk_out;
        end else begin
            count <= count + 1'b1;
        end
    end
endmodule
