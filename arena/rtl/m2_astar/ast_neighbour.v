`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_neighbour : combinational 4-connected neighbour generator
//
// Given a cell u and a direction, returns the neighbour v and whether it is
// inside the grid (CONTRACT 3: no diagonals, no wrap-around at the edges).
//
//     dir 0 = up    (row - 1)      dir 2 = left  (col - 1)
//     dir 1 = down  (row + 1)      dir 3 = right (col + 1)
//
// This order is mirrored by ref/arena_ref.py neighbours(), which is what lets
// the reference model predict the hardware's exact path.
// When the move would leave the grid, v_valid = 0 and v_idx = IDX_NONE.
//////////////////////////////////////////////////////////////////////////////
module ast_neighbour #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter INDEX_W = `ARENA_INDEX_W
) (
    input  wire [INDEX_W-1:0] u_idx,
    input  wire [1:0]         dir,
    output wire [INDEX_W-1:0] v_idx,
    output wire               v_valid
);
    localparam [1:0] DIR_UP = 2'd0, DIR_DOWN = 2'd1, DIR_LEFT = 2'd2, DIR_RIGHT = 2'd3;
    localparam [INDEX_W-1:0] COLS_I = COLS;     // COLS at index width, for +/-
    localparam [INDEX_W-1:0] NONE   = `ARENA_IDX_NONE;

    // Row/col of u (bit slices when COLS is a power of two).
    wire [INDEX_W-1:0] u_row = u_idx / COLS;
    wire [INDEX_W-1:0] u_col = u_idx % COLS;

    // Is there a cell in that direction?
    assign v_valid = (dir == DIR_UP)    ? (u_row != 0)        :
                     (dir == DIR_DOWN)  ? (u_row != ROWS - 1) :
                     (dir == DIR_LEFT)  ? (u_col != 0)        :
                                          (u_col != COLS - 1);

    // Moving one row = +/- COLS in index space, one column = +/- 1.
    wire [INDEX_W-1:0] v_raw = (dir == DIR_UP)    ? (u_idx - COLS_I) :
                               (dir == DIR_DOWN)  ? (u_idx + COLS_I) :
                               (dir == DIR_LEFT)  ? (u_idx - 1'b1)   :
                                                    (u_idx + 1'b1);

    assign v_idx = v_valid ? v_raw : NONE;
endmodule
