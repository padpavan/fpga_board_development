`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// oled_map : the VGA picture, shrunk onto the 96x64 OLED (sw[8] mirror mode,
// for testing without a monitor).  CONTRACT 8, v1.7.
//
// Combinational pixel generator for Oled_Display (6.25 MHz domain).  Like
// ast_oled_map4 it puts out the cell under the pixel (cell_addr) and gets
// that cell's cost and every engine's cell_state back (arena_top routes the
// VGA read ports here while mirroring).  Everything else is a slow 100 MHz
// register; a change costs at most one glitched frame.
//
//   view 0  EDIT : one map, 16 x 12 cells of 6 x 5 px (rows 2..61): walls,
//                  start (magenta), goal (red), weighted (brown), the mouse
//                  cursor cell (yellow outline).
//   view 1  RUN  : the 4-engine race, M2's ast_oled_map4 (2 x 2 quadrants).
//   view 2  GAME : one map like EDIT with blue walls, pellets, Pac-Man (mouth
//                  facing his direction) and the ghosts (red, pink, cyan, orange;
//                  ghosts sharing a cell are drawn split in half).
// No text or status: that stays on the VGA side panels / the normal OLED page.
//////////////////////////////////////////////////////////////////////////////
module oled_map #(
    parameter COLS = `ARENA_COLS, parameter ROWS = `ARENA_ROWS,
    parameter INDEX_W = `ARENA_INDEX_W, parameter COST_W = `ARENA_COST_W,
    parameter OLED_W = `ARENA_OLED_W
) (
    input  wire [12:0]            pixel_index,
    input  wire [1:0]             view,           // 0 EDIT, 1 RUN, 2 GAME
    output wire [INDEX_W-1:0]     cell_addr,
    input  wire [COST_W-1:0]      cell_cost,
    // RUN (ast_oled_map4)
    input  wire [4*2-1:0]         cell_state,     // per engine at cell_addr
    input  wire [4*INDEX_W-1:0]   current,
    input  wire [3:0]             busy,
    input  wire [3:0]             run_mask,
    input  wire [4*3-1:0]         place,
    // EDIT
    input  wire [INDEX_W-1:0]     start_idx,
    input  wire [INDEX_W-1:0]     goal_idx,
    input  wire                   cursor_valid,
    input  wire [INDEX_W-1:0]     cursor_idx,
    // GAME
    input  wire [INDEX_W-1:0]     player_pos,
    input  wire [1:0]             player_dir,     // 0 up 1 down 2 left 3 right
    input  wire [4*INDEX_W-1:0]   ghost_pos,
    input  wire [3:0]             ghost_present,
    input  wire [COLS*ROWS-1:0]   pellet,
    output reg  [15:0]            pixel_data
);
    localparam CW = 6, CH = 5, Y0 = 2;                 // 16*6 = 96, 12*5 = 60
    localparam NUM_CELLS = COLS*ROWS;

    function [15:0] c565; input [11:0] c;              // RGB444 -> RGB565
        c565 = {c[11:8], c[11], c[7:4], c[7:6], c[3:0], c[3]};
    endfunction
    localparam [11:0] K_BLACK = 12'h000, K_WHITE = 12'hFFF, K_WALL = 12'h556, K_START = 12'hF0F,
                      K_GOAL = 12'hF00, K_WEIGHT = 12'h951, K_CURSOR = 12'hFF0,
                      P_WALL = 12'h24F, P_PELLET = 12'hFCA, P_PAC = 12'hFF0, P_WEIGHT = 12'h404,
                      P_G0 = 12'hF00, P_G1 = 12'hF8C, P_G2 = 12'h0FF, P_G3 = 12'hFA0;

    // ---------------------------------------------------------------- RUN
    wire [INDEX_W-1:0] run_addr;
    wire [15:0]        run_px;
    ast_oled_map4 u_map4 (
        .pixel_index(pixel_index), .cell_addr(run_addr), .quad(),
        .cell_cost(cell_cost), .cell_state(cell_state), .current(current),
        .busy(busy), .run_mask(run_mask), .place(place),
        .start_idx(start_idx), .goal_idx(goal_idx), .pixel_data(run_px)
    );

    // ---------------------------------------------------- single map (EDIT / GAME)
    wire [6:0] x  = pixel_index % OLED_W;
    wire [5:0] y  = pixel_index / OLED_W;
    wire       in_map = (y >= Y0) && (y < Y0 + ROWS*CH);
    wire [5:0] my = y - Y0;
    wire [3:0] col = x / CW;
    wire [3:0] row = my / CH;
    wire [2:0] lx  = x % CW;                           // 0..5 inside the cell
    wire [2:0] ly  = my % CH;                          // 0..4
    wire [INDEX_W-1:0] map_addr = in_map ? row * COLS + col : {INDEX_W{1'b0}};
    assign cell_addr = (view == 2'd1) ? run_addr : map_addr;

    // 6x5 sprites, bit 5 = left column.  Pac-Man facing right; other
    // directions are mirrored (left) or use the up sprite (flipped for down).
    function [5:0] pac_right; input [2:0] r;
        case (r) 0: pac_right = 6'b011110; 1: pac_right = 6'b111110; 2: pac_right = 6'b111000;
                 3: pac_right = 6'b111110; default: pac_right = 6'b011110; endcase
    endfunction
    function [5:0] pac_up; input [2:0] r;
        case (r) 0: pac_up = 6'b010010; 1: pac_up = 6'b110011; 2: pac_up = 6'b111111;
                 3: pac_up = 6'b111111; default: pac_up = 6'b011110; endcase
    endfunction
    function [5:0] ghost_body; input [2:0] r;
        case (r) 0: ghost_body = 6'b011110; 4: ghost_body = 6'b101101; default: ghost_body = 6'b111111; endcase
    endfunction
    function [5:0] mirror6; input [5:0] v; mirror6 = {v[0], v[1], v[2], v[3], v[4], v[5]}; endfunction

    wire [5:0] pac_row = (player_dir == 2'd3) ? pac_right(ly) :
                         (player_dir == 2'd2) ? mirror6(pac_right(ly)) :
                         (player_dir == 2'd0) ? pac_up(ly) : pac_up(3'd4 - ly);
    wire pac_px   = pac_row[5 - lx];
    wire [5:0] gb_row = ghost_body(ly);
    wire ghost_px = gb_row[5 - lx];
    wire eye_px   = (ly == 3'd1) && (lx == 3'd1 || lx == 3'd4);
    wire pel_px   = (ly == 3'd2) && (lx == 3'd2 || lx == 3'd3);
    wire edge_px  = (lx == 0) || (lx == CW-1) || (ly == 0) || (ly == CH-1);

    reg [3:0] g_here;
    integer g;
    always @* for (g = 0; g < 4; g = g + 1)
        g_here[g] = ghost_present[g] && (map_addr == ghost_pos[g*INDEX_W +: INDEX_W]);

    // ghosts sharing a cell: left half = lowest-numbered, right half = highest
    wire [11:0] g_lo = g_here[0] ? P_G0 : g_here[1] ? P_G1 : g_here[2] ? P_G2 : P_G3;
    wire [11:0] g_hi = g_here[3] ? P_G3 : g_here[2] ? P_G2 : g_here[1] ? P_G1 : P_G0;

    wire wall   = (cell_cost == `ARENA_COST_WALL);
    wire weight = (cell_cost > `ARENA_COST_OPEN);
    wire pel    = (map_addr < NUM_CELLS) && pellet[map_addr];

    reg [11:0] c;
    always @* begin
        c = K_BLACK;
        if (!in_map) c = K_BLACK;
        else if (view == 2'd2) begin                   // GAME
            if (wall)                         c = P_WALL;
            else if ((|g_here) && ghost_px)   c = eye_px ? K_WHITE : (lx >= 3'd3) ? g_hi : g_lo;
            else if (map_addr == player_pos && pac_px) c = P_PAC;
            else if (pel && pel_px)           c = P_PELLET;
            else if (weight)                  c = P_WEIGHT;
        end else begin                                 // EDIT
            if (cursor_valid && map_addr == cursor_idx && edge_px) c = K_CURSOR;
            else if (map_addr == start_idx)   c = K_START;
            else if (map_addr == goal_idx)    c = K_GOAL;
            else if (wall)                    c = K_WALL;
            else if (weight)                  c = K_WEIGHT;
        end
        pixel_data = (view == 2'd1) ? run_px : c565(c);
    end
endmodule
