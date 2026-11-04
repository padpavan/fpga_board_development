//////////////////////////////////////////////////////////////////////////////
// arena_defs.vh : shared constants for the FPGA Pathfinding Arena (S1_18)
//
// This file is CONTRACT.md in code form (Tier 1).  Do not edit it without a
// contract change (see CONTRACT.md section 0).  Every module that crosses a
// member boundary must `include this file instead of hard-coding numbers.
//
//   `include "arena_defs.vh"
//
// Contract version: 1.6
//////////////////////////////////////////////////////////////////////////////
`ifndef ARENA_DEFS_VH
`define ARENA_DEFS_VH

// ---------------------------------------------------------------- clocks ---
`define ARENA_CLK_HZ        100_000_000   // system clock (W5)
`define ARENA_OLED_CLK_DIV  16            // 100 MHz / 16 = 6.25 MHz for Oled_Display
`define ARENA_PIX_CLK_HZ    148_500_000   // VGA pixel clock (MMCM in m3_io/vga_clkgen), 1080p60

// ------------------------------------------------------------- maze grid ---
`define ARENA_COLS          16
`define ARENA_ROWS          12
`define ARENA_NUM_CELLS     (`ARENA_COLS * `ARENA_ROWS)   // 192
`define ARENA_INDEX_W       8     // cell index width;  idx = row*COLS + col
`define ARENA_COL_W         4     // width of a column number (0..COLS-1)
`define ARENA_ROW_W         4     // width of a row number    (0..ROWS-1)
`define ARENA_IDX_NONE      8'hFF // "no cell" (invalid parent / unused); must be >= NUM_CELLS

// ---------------------------------------------------------- cell costs ---
`define ARENA_COST_W        4
`define ARENA_COST_WALL     4'd0  // impassable
`define ARENA_COST_OPEN     4'd1  // default traversable cell, unit cost
// 4'd2 .. 4'd15 : optional weighted cell (stretch goal)

// ------------------------------------------------- engine result widths ---
`define ARENA_DIST_W        16    // path_cost, path_cells, expansions, g/h/f values
`define ARENA_CNT_W         32    // cycles counter (saturating)

// --------------------------------------- per-cell search state (2 bits) ---
`define ARENA_CS_W          2
`define ARENA_CS_UNSEEN     2'd0
`define ARENA_CS_FRONTIER   2'd1  // discovered / in queue, not yet expanded
`define ARENA_CS_VISITED    2'd2  // expanded / settled
`define ARENA_CS_PATH       2'd3  // on the final reconstructed path

// ------------------------------------------------------------ engine IDs ---
`define ARENA_ENG_BFS       2'd0
`define ARENA_ENG_DIJ       2'd1
`define ARENA_ENG_AST       2'd2
`define ARENA_ENG_DFS       2'd3

// ------------------------------------------------------- display sizes ---
`define ARENA_VGA_H         1920
`define ARENA_VGA_V         1080
`define ARENA_OLED_W        96
`define ARENA_OLED_H        64

`endif
