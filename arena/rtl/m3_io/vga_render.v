`timescale 1ns / 1ps
`include "arena_defs.vh"
// Full-grid EDIT renderer and four-panel RUN renderer for the 16x12 arena.
module vga_render #(
    parameter INDEX_W=`ARENA_INDEX_W, parameter COST_W=`ARENA_COST_W,
    parameter COLS=`ARENA_COLS, parameter ROWS=`ARENA_ROWS,
    parameter H_VISIBLE=`ARENA_VGA_H, parameter V_VISIBLE=`ARENA_VGA_V
) (
    input wire [9:0] x, input wire [9:0] y, input wire active_video,
    input wire edit_mode, input wire cursor_valid, input wire [INDEX_W-1:0] cursor_idx,
    input wire [INDEX_W-1:0] start_idx, input wire [INDEX_W-1:0] goal_idx,
    output wire [INDEX_W-1:0] cell_addr, input wire [COST_W-1:0] cell_cost,
    input wire [1:0] bfs_state, input wire [1:0] dij_state, input wire [1:0] ast_state,
    input wire [INDEX_W-1:0] bfs_current, input wire [INDEX_W-1:0] dij_current,
    input wire [INDEX_W-1:0] ast_current,
    output reg [3:0] red, output reg [3:0] green, output reg [3:0] blue
);
    localparam HALF_W=H_VISIBLE/2, HALF_H=V_VISIBLE/2;
    localparam EDIT_CELL_W=H_VISIBLE/COLS, EDIT_CELL_H=V_VISIBLE/ROWS;
    localparam RUN_CELL_W=HALF_W/COLS, RUN_CELL_H=HALF_H/ROWS;
    wire [9:0] gx = edit_mode ? x : (x % HALF_W);
    wire [9:0] gy = edit_mode ? y : (y % HALF_H);
    wire [9:0] col = edit_mode ? (x / EDIT_CELL_W) : ((x % HALF_W) / RUN_CELL_W);
    wire [9:0] row = edit_mode ? (y / EDIT_CELL_H) : ((y % HALF_H) / RUN_CELL_H);
    wire in_grid = (col < COLS) && (row < ROWS);
    wire [INDEX_W-1:0] addr_calc = row*COLS + col;
    assign cell_addr = in_grid ? addr_calc : {INDEX_W{1'b0}};
    wire [1:0] view_state = (y < HALF_H) ? ((x < HALF_W) ? bfs_state : dij_state) : ast_state;
    wire [INDEX_W-1:0] view_current = (y < HALF_H) ? ((x < HALF_W) ? bfs_current : dij_current) : ast_current;
    wire view_bfs = (y < HALF_H) && (x < HALF_W);
    wire view_dij = (y < HALF_H) && (x >= HALF_W);
    wire cursor_here = edit_mode && cursor_valid && (cell_addr == cursor_idx);
    function [4:0] small_glyph;
        input [7:0] c; input [2:0] r;
        begin
            case(c)
                "A": case(r) 0:small_glyph=14;1:small_glyph=17;2:small_glyph=17;3:small_glyph=31;4:small_glyph=17;5:small_glyph=17;6:small_glyph=17;default:small_glyph=0;endcase
                "B": case(r) 0:small_glyph=30;1:small_glyph=17;2:small_glyph=17;3:small_glyph=30;4:small_glyph=17;5:small_glyph=17;6:small_glyph=30;default:small_glyph=0;endcase
                "D": case(r) 0:small_glyph=30;1:small_glyph=17;2:small_glyph=17;3:small_glyph=17;4:small_glyph=17;5:small_glyph=17;6:small_glyph=30;default:small_glyph=0;endcase
                "F": case(r) 0:small_glyph=31;1:small_glyph=16;2:small_glyph=16;3:small_glyph=30;4:small_glyph=16;5:small_glyph=16;6:small_glyph=16;default:small_glyph=0;endcase
                "I": case(r) 0:small_glyph=14;1:small_glyph=4;2:small_glyph=4;3:small_glyph=4;4:small_glyph=4;5:small_glyph=4;6:small_glyph=14;default:small_glyph=0;endcase
                "J": case(r) 0:small_glyph=7;1:small_glyph=2;2:small_glyph=2;3:small_glyph=2;4:small_glyph=2;5:small_glyph=18;6:small_glyph=12;default:small_glyph=0;endcase
                "P": case(r) 0:small_glyph=30;1:small_glyph=17;2:small_glyph=17;3:small_glyph=30;4:small_glyph=16;5:small_glyph=16;6:small_glyph=16;default:small_glyph=0;endcase
                "S": case(r) 0:small_glyph=15;1:small_glyph=16;2:small_glyph=16;3:small_glyph=14;4:small_glyph=1;5:small_glyph=1;6:small_glyph=30;default:small_glyph=0;endcase
                "T": case(r) 0:small_glyph=31;1:small_glyph=4;2:small_glyph=4;3:small_glyph=4;4:small_glyph=4;5:small_glyph=4;6:small_glyph=4;default:small_glyph=0;endcase
                "V": case(r) 0:small_glyph=17;1:small_glyph=17;2:small_glyph=17;3:small_glyph=17;4:small_glyph=17;5:small_glyph=10;6:small_glyph=4;default:small_glyph=0;endcase
                "W": case(r) 0:small_glyph=17;1:small_glyph=17;2:small_glyph=17;3:small_glyph=21;4:small_glyph=21;5:small_glyph=21;6:small_glyph=10;default:small_glyph=0;endcase
                default: small_glyph=0;
            endcase
        end
    endfunction
    wire [9:0] panel_x = x % HALF_W;
    wire [9:0] panel_y = y % HALF_H;
    wire [2:0] header_x = panel_x % 8;
    wire [2:0] header_y = panel_y % 8;
    wire [1:0] header_slot = panel_x / 8;
    wire [2:0] legend_x = panel_x - 38;
    wire [7:0] header_char = (y < HALF_H) ?
        ((x < HALF_W) ? ((header_slot==0)?"B":(header_slot==1)?"F":"S") : ((header_slot==0)?"D":(header_slot==1)?"I":"J")) :
        ((header_slot==0)?"A":(header_slot==1)?"S":"T");
    wire [7:0] legend_char = (panel_y >= 16 && panel_y < 24) ? "W" :
        (panel_y >= 44 && panel_y < 52) ? "V" : (panel_y >= 72 && panel_y < 80) ? "F" :
        (panel_y >= 100 && panel_y < 108) ? "P" : (panel_y >= 128 && panel_y < 136) ? "T" : " ";
    wire [4:0] header_bits = small_glyph(header_char,header_y);
    wire [4:0] legend_bits = small_glyph(legend_char,panel_y[2:0]);
    always @* begin
        red=0; green=0; blue=0;
        if (active_video) begin
            if (!edit_mode && x >= HALF_W && y >= HALF_H) begin
                // panel separators and legend quadrant
                if (x == HALF_W || y == HALF_H) begin red=4'h4; green=4'h4; blue=4'h4; end
                else begin
                    red=1; green=1; blue=1;
                    if ((panel_x >= 12 && panel_x < 30) && (panel_y >= 16 && panel_y < 26)) begin red=5; green=5; blue=6; end
                    else if ((panel_x >= 12 && panel_x < 30) && (panel_y >= 44 && panel_y < 54)) begin red=2; green=5; blue=12; end
                    else if ((panel_x >= 12 && panel_x < 30) && (panel_y >= 72 && panel_y < 82)) begin red=15; green=12; blue=0; end
                    else if ((panel_x >= 12 && panel_x < 30) && (panel_y >= 100 && panel_y < 110)) begin red=0; green=15; blue=0; end
                    else if ((panel_x >= 12 && panel_x < 30) && (panel_y >= 128 && panel_y < 138)) begin red=9; green=5; blue=1; end
                    else if ((panel_x >= 38 && panel_x < 46) && legend_x < 5 && legend_bits[4-legend_x]) begin red=15; green=15; blue=15; end
                end
            end else if (!edit_mode && panel_y < 8 && panel_x < 24) begin
                if (header_x < 5 && header_bits[4-header_x]) begin
                    if (view_bfs) begin red=0; green=15; blue=15; end
                    else if (view_dij) begin red=15; green=15; blue=0; end
                    else begin red=0; green=15; blue=0; end
                end
            end else if (in_grid) begin
                if (edit_mode ? ((x % EDIT_CELL_W == EDIT_CELL_W-1) || (y % EDIT_CELL_H == EDIT_CELL_H-1)) :
                                ((gx % RUN_CELL_W == RUN_CELL_W-1) || (gy % RUN_CELL_H == RUN_CELL_H-1))) begin red=1; green=1; blue=2; end
                else if (cell_addr == start_idx) begin red=15; green=0; blue=15; end
                else if (cell_addr == goal_idx) begin red=15; green=0; blue=0; end
                else if (cursor_here) begin red=15; green=15; blue=0; end
                else if (!edit_mode && cell_addr == view_current) begin red=15; green=15; blue=15; end
                else if (cell_cost == `ARENA_COST_WALL) begin red=5; green=5; blue=6; end
                else if (!edit_mode && view_state == `ARENA_CS_PATH) begin
                    if(view_bfs) begin red=0; green=15; blue=15; end
                    else if(view_dij) begin red=15; green=15; blue=0; end
                    else begin red=0; green=15; blue=0; end
                end
                else if (!edit_mode && view_state == `ARENA_CS_VISITED) begin red=2; green=5; blue=12; end
                else if (!edit_mode && view_state == `ARENA_CS_FRONTIER) begin red=15; green=12; blue=0; end
                else if (cell_cost > `ARENA_COST_OPEN) begin red=9; green=5; blue=1; end
                else begin red=0; green=0; blue=0; end
            end
        end
    end
endmodule
