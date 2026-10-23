`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// manhattan : combinational Manhattan-distance heuristic for A*
//
//     h = |row_a - row_b| + |col_a - col_b|
//
// Takes two cell indices (idx = row*COLS + col, CONTRACT 3), splits each into
// row/col, and adds the two absolute differences.  Pure combinational logic:
// h is valid in the same cycle the indices are applied.
//
// Admissible and consistent for this maze model because every non-wall cell
// costs >= 1 and each 4-connected move changes h by exactly 1.
//
// With COLS a power of two (16) the divide/modulo are just bit slices
// (row = idx[7:4], col = idx[3:0]), so this costs only two small subtractors,
// two muxes and one adder.  Usable by anyone (CONTRACT 6).
//////////////////////////////////////////////////////////////////////////////
module manhattan #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter ROW_W   = `ARENA_ROW_W,
    parameter COL_W   = `ARENA_COL_W,
    parameter DIST_W  = `ARENA_DIST_W
) (
    input  wire [INDEX_W-1:0] a_idx,
    input  wire [INDEX_W-1:0] b_idx,
    output wire [DIST_W-1:0]  h
);
    // ---- index -> (row, col) ---------------------------------------------
    // Full-width quotient/remainder first, then keep only the bits a row or
    // column number can occupy (row < ROWS, col < COLS).
    wire [INDEX_W-1:0] a_row_full = a_idx / COLS;
    wire [INDEX_W-1:0] a_col_full = a_idx % COLS;
    wire [INDEX_W-1:0] b_row_full = b_idx / COLS;
    wire [INDEX_W-1:0] b_col_full = b_idx % COLS;

    wire [ROW_W-1:0] a_row = a_row_full[ROW_W-1:0];
    wire [COL_W-1:0] a_col = a_col_full[COL_W-1:0];
    wire [ROW_W-1:0] b_row = b_row_full[ROW_W-1:0];
    wire [COL_W-1:0] b_col = b_col_full[COL_W-1:0];

    // ---- |dr| and |dc| ---------------------------------------------------
    wire [ROW_W-1:0] d_row;
    wire [COL_W-1:0] d_col;

    ast_absdiff #(.W(ROW_W)) u_abs_row (.a(a_row), .b(b_row), .d(d_row));
    ast_absdiff #(.W(COL_W)) u_abs_col (.a(a_col), .b(b_col), .d(d_col));

    // ---- h = |dr| + |dc| --------------------------------------------------
    // Zero-extend both to DIST_W before adding so the carry is never lost
    // (max h = (ROWS-1) + (COLS-1) = 26, far below 2^DIST_W).
    assign h = {{(DIST_W-ROW_W){1'b0}}, d_row} + {{(DIST_W-COL_W){1'b0}}, d_col};
endmodule
