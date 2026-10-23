`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_test_maze : combinational "maze ROM" for M2's solo board test
//
// Produces the cost of any cell of four built-in mazes, cell-for-cell
// identical to ref/gen_mazes.py (tb_m2_test_top checks this against the
// sim/mazes/*.mem files), plus the first start/goal pair of each maze's
// .ast.expect, so on-board results can be compared with the reference.
//
//   sel 0  open         all cost 1                      start 0     -> goal 191
//   sel 1  weighted     heavy (15) middle column        start (5,2) -> goal (5,13)
//   sel 2  unreachable  goal boxed in by walls          start 0     -> goal (6,12)
//   sel 3  corridor     serpentine single corridor      start 0     -> goal (10,0)
//
// Computed from row/col with comparators instead of a $readmemh ROM, so the
// board build needs no data files and no `initial` blocks.
// Used only by m2_test_top; not part of the arena build.
//////////////////////////////////////////////////////////////////////////////
module ast_test_maze #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter COST_W  = `ARENA_COST_W
) (
    input  wire [1:0]         sel,
    input  wire [INDEX_W-1:0] idx,
    output wire [COST_W-1:0]  cost,
    output wire [INDEX_W-1:0] start_idx,
    output wire [INDEX_W-1:0] goal_idx
);
    localparam [COST_W-1:0] WALL  = `ARENA_COST_WALL;
    localparam [COST_W-1:0] OPEN  = `ARENA_COST_OPEN;
    localparam [COST_W-1:0] HEAVY = 4'd15;

    // Geometry shared with gen_mazes.py.
    localparam MID_COL  = COLS / 2;             // weighted: heavy column
    localparam BOX_ROW  = ROWS / 2;             // unreachable: boxed goal
    localparam BOX_COL  = COLS - 4;
    localparam LAST_ROW = (ROWS % 2 == 0) ? ROWS - 2 : ROWS - 1;  // corridor end row

    // Start / goal of each maze.
    localparam [INDEX_W-1:0] OPEN_GOAL     = COLS * ROWS - 1;
    localparam [INDEX_W-1:0] WEIGHT_START  = 5 * COLS + 2;
    localparam [INDEX_W-1:0] WEIGHT_GOAL   = 5 * COLS + COLS - 3;
    localparam [INDEX_W-1:0] UNREACH_GOAL  = BOX_ROW * COLS + BOX_COL;
    localparam [INDEX_W-1:0] CORRIDOR_GOAL = LAST_ROW * COLS + (((LAST_ROW / 2) % 2 == 1) ? 0 : COLS - 1);

    wire [INDEX_W-1:0] row = idx / COLS;
    wire [INDEX_W-1:0] col = idx % COLS;

    // ---- per-maze cell cost ------------------------------------------------
    // weighted: heavy column except its bottom cell, plus a few mid-weight
    // cells (same list as gen_mazes.maze_weighted)
    wire [COST_W-1:0] c_weighted =
        ((row == 2 && col == 3) || (row == 3 && col == 3)) ? 4'd4  :
        (row == 8  && col == 11)                           ? 4'd3  :
        (row == 9  && col == 12)                           ? 4'd7  :
        (row == 10 && col == 5)                            ? 4'd2  :
        (col == MID_COL && row != ROWS - 1)                ? HEAVY : OPEN;

    // unreachable: 3x3 ring of walls around (BOX_ROW, BOX_COL)
    wire in_box     = (row + 1 >= BOX_ROW) && (row <= BOX_ROW + 1) &&
                      (col + 1 >= BOX_COL) && (col <= BOX_COL + 1);
    wire box_centre = (row == BOX_ROW) && (col == BOX_COL);
    wire [COST_W-1:0] c_unreach = (in_box && !box_centre) ? WALL : OPEN;

    // corridor: odd rows are walls except one gap, alternating right/left
    wire [INDEX_W-1:0] gap_col    = row[1] ? 0 : COLS - 1;   // (row/2) odd -> left edge
    wire [COST_W-1:0]  c_corridor = (row[0] && col != gap_col) ? WALL : OPEN;

    // ---- select the maze -----------------------------------------------------
    assign cost      = (sel == 2'd0) ? OPEN       :
                       (sel == 2'd1) ? c_weighted :
                       (sel == 2'd2) ? c_unreach  : c_corridor;

    assign start_idx = (sel == 2'd1) ? WEIGHT_START : 0;

    assign goal_idx  = (sel == 2'd0) ? OPEN_GOAL    :
                       (sel == 2'd1) ? WEIGHT_GOAL  :
                       (sel == 2'd2) ? UNREACH_GOAL : CORRIDOR_GOAL;
endmodule
