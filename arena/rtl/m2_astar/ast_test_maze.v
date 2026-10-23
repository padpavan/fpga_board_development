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
    output reg  [COST_W-1:0]  cost,
    output reg  [INDEX_W-1:0] start_idx,
    output reg  [INDEX_W-1:0] goal_idx
);
    localparam [COST_W-1:0] WALL  = `ARENA_COST_WALL;
    localparam [COST_W-1:0] OPEN  = `ARENA_COST_OPEN;
    localparam [COST_W-1:0] HEAVY = 4'd15;

    // Geometry shared with gen_mazes.py.
    localparam MID_COL  = COLS / 2;             // weighted: heavy column
    localparam BOX_ROW  = ROWS / 2;             // unreachable: boxed goal
    localparam BOX_COL  = COLS - 4;
    localparam LAST_ROW = (ROWS % 2 == 0) ? ROWS - 2 : ROWS - 1;  // corridor end row

    wire [INDEX_W-1:0] row = idx / COLS;
    wire [INDEX_W-1:0] col = idx % COLS;

    // Small helper: is (rw, cl) the cell (r, c)?  row/col are passed in
    // explicitly so always @(*) sees them as inputs of the block.
    function at(input [INDEX_W-1:0] rw, input [INDEX_W-1:0] cl,
                input integer r, input integer c);
        at = (rw == r) && (cl == c);
    endfunction

    // ---- per-maze cell cost ------------------------------------------------
    reg [COST_W-1:0] c_weighted, c_unreach, c_corridor;
    reg              in_box;
    reg [INDEX_W-1:0] gap_col;

    always @(*) begin
        // weighted: heavy column except its bottom cell, plus a few
        // mid-weight cells (same list as gen_mazes.maze_weighted)
        c_weighted = OPEN;
        if (col == MID_COL && row != ROWS - 1)        c_weighted = HEAVY;
        if (at(row, col, 2, 3) || at(row, col, 3, 3)) c_weighted = 4'd4;
        if (at(row, col, 8, 11))                      c_weighted = 4'd3;
        if (at(row, col, 9, 12))                      c_weighted = 4'd7;
        if (at(row, col, 10, 5))                      c_weighted = 4'd2;

        // unreachable: 3x3 ring of walls around (BOX_ROW, BOX_COL)
        in_box    = (row + 1 >= BOX_ROW) && (row <= BOX_ROW + 1) &&
                    (col + 1 >= BOX_COL) && (col <= BOX_COL + 1);
        c_unreach = (in_box && !at(row, col, BOX_ROW, BOX_COL)) ? WALL : OPEN;

        // corridor: odd rows are walls except one gap, alternating right/left
        gap_col    = row[1] ? 0 : COLS - 1;     // (row/2) odd -> left edge
        c_corridor = (row[0] && col != gap_col) ? WALL : OPEN;

        case (sel)
            2'd0: begin cost = OPEN;       start_idx = 0;            goal_idx = COLS * ROWS - 1;             end
            2'd1: begin cost = c_weighted; start_idx = 5 * COLS + 2; goal_idx = 5 * COLS + COLS - 3;         end
            2'd2: begin cost = c_unreach;  start_idx = 0;            goal_idx = BOX_ROW * COLS + BOX_COL;    end
            default: begin cost = c_corridor; start_idx = 0;
                           goal_idx = LAST_ROW * COLS + (((LAST_ROW / 2) % 2 == 1) ? 0 : COLS - 1);       end
        endcase
    end
endmodule
