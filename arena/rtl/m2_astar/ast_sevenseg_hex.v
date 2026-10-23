`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// ast_sevenseg_hex : show a 16-bit value as 4 hex digits on the Basys 3
//
// The 4 digits share one set of segment lines, so they are lit one at a time
// ("multiplexed") fast enough that the eye sees all four.  A free-running
// counter's top 2 bits pick the digit:
//     REFRESH_W = 18  ->  each digit for 2^16 cycles = 0.66 ms  (~380 Hz/digit)
// an[0] = rightmost digit = value[3:0].
// Basys 3 segments and anodes are ACTIVE-LOW; seg[0] = a ... seg[6] = g.
// Used only by m2_test_top.
//////////////////////////////////////////////////////////////////////////////
module ast_sevenseg_hex #(
    parameter REFRESH_W = 18
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] value,
    output reg  [6:0]  seg,
    output reg  [3:0]  an,
    output wire        dp
);
    reg [REFRESH_W-1:0] refresh = {REFRESH_W{1'b0}};
    always @(posedge clk)
        refresh <= rst ? {REFRESH_W{1'b0}} : refresh + 1'b1;

    wire [1:0] digit = refresh[REFRESH_W-1:REFRESH_W-2];

    reg [3:0] nibble;
    always @(*) begin
        case (digit)
            2'd0: begin an = 4'b1110; nibble = value[3:0];   end
            2'd1: begin an = 4'b1101; nibble = value[7:4];   end
            2'd2: begin an = 4'b1011; nibble = value[11:8];  end
            default: begin an = 4'b0111; nibble = value[15:12]; end
        endcase

        //                      gfedcba  (0 = segment on)
        case (nibble)
            4'h0: seg = 7'b1000000;
            4'h1: seg = 7'b1111001;
            4'h2: seg = 7'b0100100;
            4'h3: seg = 7'b0110000;
            4'h4: seg = 7'b0011001;
            4'h5: seg = 7'b0010010;
            4'h6: seg = 7'b0000010;
            4'h7: seg = 7'b1111000;
            4'h8: seg = 7'b0000000;
            4'h9: seg = 7'b0010000;
            4'hA: seg = 7'b0001000;
            4'hB: seg = 7'b0000011;             // lower-case b
            4'hC: seg = 7'b1000110;
            4'hD: seg = 7'b0100001;             // lower-case d
            4'hE: seg = 7'b0000110;
            default: seg = 7'b0001110;          // F
        endcase
    end

    assign dp = 1'b1;                           // decimal point off
endmodule
