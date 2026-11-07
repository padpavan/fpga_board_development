// Shared settings
`ifndef ARENA_DEFS_VH
`define ARENA_DEFS_VH

`define ARENA_CLK_HZ        100_000_000
`define ARENA_OLED_CLK_DIV  16
`define ARENA_PIX_CLK_HZ    148_500_000

`define ARENA_COLS          16
`define ARENA_ROWS          12
`define ARENA_NUM_CELLS     (`ARENA_COLS * `ARENA_ROWS)
`define ARENA_INDEX_W       8
`define ARENA_COL_W         4
`define ARENA_ROW_W         4
`define ARENA_IDX_NONE      8'hFF

`define ARENA_COST_W        4
`define ARENA_COST_WALL     4'd0
`define ARENA_COST_OPEN     4'd1

`define ARENA_DIST_W        16
`define ARENA_CNT_W         32

`define ARENA_CS_W          2
`define ARENA_CS_UNSEEN     2'd0
`define ARENA_CS_FRONTIER   2'd1
`define ARENA_CS_VISITED    2'd2
`define ARENA_CS_PATH       2'd3

`define ARENA_ENG_BFS       2'd0
`define ARENA_ENG_DIJ       2'd1
`define ARENA_ENG_AST       2'd2
`define ARENA_ENG_DFS       2'd3

`define ARENA_VGA_H         1920
`define ARENA_VGA_V         1080
`define ARENA_OLED_W        96
`define ARENA_OLED_H        64

`define ARENA_PG_ADDR_W     7
`define ARENA_PG_ROT_W      3
`define ARENA_PG_SAMPLES    94
`define ARENA_PG_LEVEL_W    6
`define ARENA_PG_LEVEL_MAX  46
`define ARENA_PG_BEST_W     5

`endif
