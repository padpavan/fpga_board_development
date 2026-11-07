`timescale 1ns / 1ps

// Search engines
module ast_absdiff #(
    parameter W = 4
) (
    input  wire [W-1:0] a,
    input  wire [W-1:0] b,
    output wire [W-1:0] d
);
    assign d = (a >= b) ? (a - b) : (b - a);
endmodule
