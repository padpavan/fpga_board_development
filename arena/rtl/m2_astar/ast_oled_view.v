`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_oled_view : draw the maze + live A* search on the 96x64 Pmod OLED
//
// M2 TEST ONLY (used by m2_test_top).  The real arena graphics are M3's.
//
// Combinational pixel generator for Oled_Display: given the pixel being sent,
// it outputs which grid cell that pixel belongs to (cell_addr), receives that
// cell's cost / search state back, and returns the RGB565 colour.
//
// Layout (96 x 64):
//     each cell = CELL_W x CELL_H = 6 x 5 pixels  ->  16 x 12 cells = 96 x 60
//     the last pixel column / row of every cell is a dark grid line
//     rows 60..63 = status bar: yellow busy, green found, red no path
//
// Cell colours, highest priority first:
//     start  magenta   goal  red      current node  white (while busy)
//     PATH   green     wall  grey     VISITED       blue
//     FRONTIER yellow  weighted (cost > 1) brown    open  black
// When overlay = 0 (e.g. previewing a maze that hasn't been run yet), only
// walls / weights / start / goal are drawn.
//////////////////////////////////////////////////////////////////////////////
module ast_oled_view #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter COST_W  = `ARENA_COST_W,
    parameter OLED_W  = `ARENA_OLED_W,
    parameter OLED_H  = `ARENA_OLED_H,
    parameter CELL_W  = 6,                    // OLED_W / COLS
    parameter CELL_H  = 5                     // floor(OLED_H / ROWS)
) (
    input  wire [12:0]        pixel_index,    // from Oled_Display: y*96 + x
    output wire [INDEX_W-1:0] cell_addr,      // cell under this pixel
    input  wire [1:0]         cell_state,     // that cell's search state
    input  wire [COST_W-1:0]  cell_cost,      // that cell's cost (0 = wall)

    input  wire [INDEX_W-1:0] start_idx,
    input  wire [INDEX_W-1:0] goal_idx,
    input  wire [INDEX_W-1:0] current_idx,
    input  wire               overlay,        // draw the search state?
    input  wire               busy,
    input  wire               found,
    input  wire               no_path,

    output reg  [15:0]        pixel_data      // RGB565
);
    // ---- RGB565 palette -------------------------------------------------------
    localparam [15:0] C_BLACK   = 16'h0000,
                      C_GRID    = 16'h18E3,   // dark grey grid lines
                      C_WALL    = 16'h8410,   // grey
                      C_WEIGHT  = 16'h8A22,   // brown
                      C_FRONT   = 16'hFFE0,   // yellow
                      C_VISIT   = 16'h319F,   // blue
                      C_PATH    = 16'h07E0,   // green
                      C_START   = 16'hF81F,   // magenta
                      C_GOAL    = 16'hF800,   // red
                      C_CURRENT = 16'hFFFF,   // white
                      C_BUSY    = 16'hFFE0,
                      C_FOUND   = 16'h07E0,
                      C_NOPATH  = 16'hF800,
                      C_IDLE    = 16'h18E3;

    // Shared codes from arena_defs.vh, given plain names for the code below.
    localparam [COST_W-1:0] COST_WALL   = `ARENA_COST_WALL,
                            COST_OPEN   = `ARENA_COST_OPEN;
    localparam [1:0]        CS_FRONTIER = `ARENA_CS_FRONTIER,
                            CS_VISITED  = `ARENA_CS_VISITED,
                            CS_PATH     = `ARENA_CS_PATH;

    // ---- pixel -> (x, y) -> (col, row) ----------------------------------------
    // Divisions by constants; at the 6.25 MHz OLED clock there is ample time.
    wire [12:0] x   = pixel_index % OLED_W;
    wire [12:0] y   = pixel_index / OLED_W;
    wire [12:0] col = x / CELL_W;
    wire [12:0] row = y / CELL_H;

    wire in_grid  = (y < ROWS * CELL_H) && (col < COLS);
    wire on_line  = (x % CELL_W == CELL_W - 1) || (y % CELL_H == CELL_H - 1);

    wire [12:0] cell_full = row * COLS + col;
    assign cell_addr = in_grid ? cell_full[INDEX_W-1:0] : {INDEX_W{1'b0}};

    // ---- colour ---------------------------------------------------------------
    always @(*) begin
        if (!in_grid) begin
            // status bar under the grid
            if      (busy)    pixel_data = C_BUSY;
            else if (found)   pixel_data = C_FOUND;
            else if (no_path) pixel_data = C_NOPATH;
            else              pixel_data = C_IDLE;
        end else if (on_line) begin
            pixel_data = C_GRID;
        end else if (cell_addr == start_idx) begin
            pixel_data = C_START;
        end else if (cell_addr == goal_idx) begin
            pixel_data = C_GOAL;
        end else if (overlay && busy && cell_addr == current_idx) begin
            pixel_data = C_CURRENT;
        end else if (overlay && cell_state == CS_PATH) begin
            pixel_data = C_PATH;
        end else if (cell_cost == COST_WALL) begin
            pixel_data = C_WALL;
        end else if (overlay && cell_state == CS_VISITED) begin
            pixel_data = C_VISIT;
        end else if (overlay && cell_state == CS_FRONTIER) begin
            pixel_data = C_FRONT;
        end else if (cell_cost != COST_OPEN) begin
            pixel_data = C_WEIGHT;
        end else begin
            pixel_data = C_BLACK;
        end
    end
endmodule
