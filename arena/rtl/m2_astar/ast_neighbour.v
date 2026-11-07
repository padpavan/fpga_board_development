`timescale 1ns / 1ps
`include "arena_defs.vh"

// Search engines
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
    localparam [INDEX_W-1:0] COLS_I = COLS;
    localparam [INDEX_W-1:0] NONE   = `ARENA_IDX_NONE;

    wire [INDEX_W-1:0] u_row = u_idx / COLS;
    wire [INDEX_W-1:0] u_col = u_idx % COLS;

    assign v_valid = (dir == DIR_UP)    ? (u_row != 0)        :
                     (dir == DIR_DOWN)  ? (u_row != ROWS - 1) :
                     (dir == DIR_LEFT)  ? (u_col != 0)        :
                                          (u_col != COLS - 1);

    wire [INDEX_W-1:0] v_raw = (dir == DIR_UP)    ? (u_idx - COLS_I) :
                               (dir == DIR_DOWN)  ? (u_idx + COLS_I) :
                               (dir == DIR_LEFT)  ? (u_idx - 1'b1)   :
                                                    (u_idx + 1'b1);

    assign v_idx = v_valid ? v_raw : NONE;
endmodule
