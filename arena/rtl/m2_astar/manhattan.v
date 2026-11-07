`timescale 1ns / 1ps
`include "arena_defs.vh"

// A* engine
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
    wire [INDEX_W-1:0] a_row_full = a_idx / COLS;
    wire [INDEX_W-1:0] a_col_full = a_idx % COLS;
    wire [INDEX_W-1:0] b_row_full = b_idx / COLS;
    wire [INDEX_W-1:0] b_col_full = b_idx % COLS;

    wire [ROW_W-1:0] a_row = a_row_full[ROW_W-1:0];
    wire [COL_W-1:0] a_col = a_col_full[COL_W-1:0];
    wire [ROW_W-1:0] b_row = b_row_full[ROW_W-1:0];
    wire [COL_W-1:0] b_col = b_col_full[COL_W-1:0];

    wire [ROW_W-1:0] d_row;
    wire [COL_W-1:0] d_col;

    ast_absdiff #(.W(ROW_W)) u_abs_row (.a(a_row), .b(b_row), .d(d_row));
    ast_absdiff #(.W(COL_W)) u_abs_col (.a(a_col), .b(b_col), .d(d_col));

    assign h = {{(DIST_W-ROW_W){1'b0}}, d_row} + {{(DIST_W-COL_W){1'b0}}, d_col};
endmodule
