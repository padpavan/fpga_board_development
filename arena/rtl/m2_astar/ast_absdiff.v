`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// ast_absdiff : combinational absolute difference  d = |a - b|
//
// Unsigned inputs.  One comparator picks which way round to subtract, so the
// result is never negative and needs no extra sign bit:
//     a >= b  ->  d = a - b
//     a <  b  ->  d = b - a
// Used by manhattan.v for |row_a - row_b| and |col_a - col_b|.
//////////////////////////////////////////////////////////////////////////////
module ast_absdiff #(
    parameter W = 4
) (
    input  wire [W-1:0] a,
    input  wire [W-1:0] b,
    output wire [W-1:0] d
);
    assign d = (a >= b) ? (a - b) : (b - a);
endmodule
