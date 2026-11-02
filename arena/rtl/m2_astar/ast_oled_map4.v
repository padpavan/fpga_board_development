`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_oled_map4 : all four engines' maps on one 96x64 OLED frame
//
// M2 TEST VIEW: tb_ast_oled_map4 and the board top m2_race_top.  Not part of
// arena_top, where every engine's single cell_state read port belongs to VGA.
//
// Combinational pixel generator, same idea as ast_oled_view: for the pixel
// being drawn it outputs cell_addr, gets that cell's cost and every engine's
// cell_state back, and returns RGB565.
//
// Layout: 2 x 2 quadrants of 48 x 32, same placement as the VGA RUN screen
//     BFS | DIJ          each quadrant: 8 px label row  "BFS #1"
//     ----+----                         24 px map: 16 x 12 cells of 3 x 2 px
//     A*  | DFS
//
// Cell colours (same as VGA), highest priority first:
//     start magenta, goal red, current white (while that engine is busy),
//     wall grey, PATH = engine colour, VISITED blue, FRONTIER orange,
//     weighted brown, open black.
// An engine not in run_mask shows only the maze, with a grey "OFF" label.
//////////////////////////////////////////////////////////////////////////////
module ast_oled_map4 #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter COST_W  = `ARENA_COST_W,
    parameter OLED_W  = `ARENA_OLED_W
) (
    input  wire [12:0]        pixel_index,     // y*96 + x
    output wire [INDEX_W-1:0] cell_addr,       // cell under this pixel
    output wire [1:0]         quad,            // 0 BFS, 1 DIJ, 2 A*, 3 DFS
    input  wire [COST_W-1:0]  cell_cost,
    input  wire [4*2-1:0]     cell_state,      // per engine, at cell_addr
    input  wire [4*INDEX_W-1:0] current,       // per engine
    input  wire [3:0]         busy,
    input  wire [3:0]         run_mask,
    input  wire [4*3-1:0]     place,           // per engine 0 = none, 1..4
    input  wire [INDEX_W-1:0] start_idx,
    input  wire [INDEX_W-1:0] goal_idx,
    output reg  [15:0]        pixel_data
);
    localparam QW = 48, QH = 32, LABEL_H = 8, CELL_W = 3, CELL_H = 2;
    localparam [COST_W-1:0] WALL = `ARENA_COST_WALL;
    localparam [COST_W-1:0] OPEN = `ARENA_COST_OPEN;
    localparam [1:0] CS_FRONT = `ARENA_CS_FRONTIER;
    localparam [1:0] CS_VISIT = `ARENA_CS_VISITED;
    localparam [1:0] CS_PATH  = `ARENA_CS_PATH;

    // RGB444 -> RGB565 (replicate the top bits)
    function [15:0] c565;
        input [3:0] r; input [3:0] g; input [3:0] b;
        c565 = {r, r[3], g, g[3:2], b, b[3]};
    endfunction

    wire [6:0] x  = pixel_index % OLED_W;
    wire [5:0] y  = pixel_index / OLED_W;
    assign quad   = {y >= QH, x >= QW};
    wire [5:0] qx = (x >= QW) ? x - QW : x;
    wire [4:0] qy = (y >= QH) ? y - QH : y;

    wire       in_map = (qy >= LABEL_H);
    wire [3:0] col    = qx / CELL_W;
    wire [3:0] row    = (qy - LABEL_H) / CELL_H;
    assign cell_addr  = in_map ? row * COLS + col : {INDEX_W{1'b0}};

    wire               on    = run_mask[quad];
    wire [1:0]         st    = cell_state[quad*2 +: 2];
    wire [INDEX_W-1:0] cur   = current[quad*INDEX_W +: INDEX_W];
    wire [2:0]         plc   = place[quad*3 +: 3];

    reg [15:0] eng_col;
    always @* begin
        case (quad)
            2'd0:    eng_col = c565(4'd0,  4'd15, 4'd15);   // BFS cyan
            2'd1:    eng_col = c565(4'd15, 4'd15, 4'd0);    // DIJ yellow
            2'd2:    eng_col = c565(4'd0,  4'd15, 4'd0);    // A* green
            default: eng_col = c565(4'd15, 4'd6,  4'd10);   // DFS pink
        endcase
    end

    // ---- label: "BFS #1" / "DIJ OFF", 6 px per char, glyph rows 0..6 ------
    function [4:0] glyph;
        input [7:0] c; input [2:0] r;
        begin
            case (c)
                "1": case(r) 0:glyph=4;1:glyph=12;2:glyph=4;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=14;default:glyph=0; endcase
                "2": case(r) 0:glyph=14;1:glyph=17;2:glyph=1;3:glyph=2;4:glyph=4;5:glyph=8;6:glyph=31;default:glyph=0; endcase
                "3": case(r) 0:glyph=30;1:glyph=1;2:glyph=1;3:glyph=14;4:glyph=1;5:glyph=1;6:glyph=30;default:glyph=0; endcase
                "4": case(r) 0:glyph=2;1:glyph=6;2:glyph=10;3:glyph=18;4:glyph=31;5:glyph=2;6:glyph=2;default:glyph=0; endcase
                "A": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=31;4:glyph=17;5:glyph=17;6:glyph=17;default:glyph=0; endcase
                "B": case(r) 0:glyph=30;1:glyph=17;2:glyph=17;3:glyph=30;4:glyph=17;5:glyph=17;6:glyph=30;default:glyph=0; endcase
                "D": case(r) 0:glyph=30;1:glyph=17;2:glyph=17;3:glyph=17;4:glyph=17;5:glyph=17;6:glyph=30;default:glyph=0; endcase
                "F": case(r) 0:glyph=31;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=16;5:glyph=16;6:glyph=16;default:glyph=0; endcase
                "I": case(r) 0:glyph=14;1:glyph=4;2:glyph=4;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=14;default:glyph=0; endcase
                "J": case(r) 0:glyph=7;1:glyph=2;2:glyph=2;3:glyph=2;4:glyph=2;5:glyph=18;6:glyph=12;default:glyph=0; endcase
                "O": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=17;4:glyph=17;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "S": case(r) 0:glyph=15;1:glyph=16;2:glyph=16;3:glyph=14;4:glyph=1;5:glyph=1;6:glyph=30;default:glyph=0; endcase
                "#": case(r) 0:glyph=10;1:glyph=10;2:glyph=31;3:glyph=10;4:glyph=31;5:glyph=10;6:glyph=10;default:glyph=0; endcase
                "*": case(r) 1:glyph=4;2:glyph=21;3:glyph=14;4:glyph=21;5:glyph=4;default:glyph=0; endcase
                "-": case(r) 3:glyph=31;default:glyph=0; endcase
                default: glyph=0;
            endcase
        end
    endfunction

    wire [2:0] lchr = qx / 6;           // label char 0..7
    wire [2:0] lgx  = qx % 6;
    reg  [7:0] lch;
    always @* begin
        lch = " ";
        case (lchr)
            3'd0: lch = (quad == 0) ? "B" : (quad == 1) ? "D" : (quad == 2) ? "A" : "D";
            3'd1: lch = (quad == 0) ? "F" : (quad == 1) ? "I" : (quad == 2) ? "*" : "F";
            3'd2: lch = (quad == 0) ? "S" : (quad == 1) ? "J" : (quad == 2) ? " " : "S";
            3'd4: lch = on ? "#" : "O";
            3'd5: lch = on ? ((plc == 0) ? "-" : "0" + plc) : "F";
            3'd6: lch = on ? " " : "F";
            default: lch = " ";
        endcase
    end
    wire [4:0] lbits = glyph(lch, qy[2:0]);

    always @* begin
        pixel_data = 16'h0000;
        if (!in_map) begin
            if (qy < 7 && lgx < 5 && lbits[4-lgx])
                pixel_data = !on ? c565(4'd6, 4'd6, 4'd6) :
                             (lchr == 3'd4 || lchr == 3'd5) ? 16'hFFFF : eng_col;
        end else if (cell_addr == start_idx)                    pixel_data = c565(4'd15, 4'd0, 4'd15);
        else if (cell_addr == goal_idx)                         pixel_data = c565(4'd15, 4'd0, 4'd0);
        else if (on && busy[quad] && cell_addr == cur)          pixel_data = 16'hFFFF;
        else if (cell_cost == WALL)                             pixel_data = c565(4'd5, 4'd5, 4'd6);
        else if (on && st == CS_PATH)                           pixel_data = eng_col;
        else if (on && st == CS_VISIT)                          pixel_data = c565(4'd2, 4'd5, 4'd12);
        else if (on && st == CS_FRONT)                          pixel_data = c565(4'd15, 4'd12, 4'd0);
        else if (cell_cost > OPEN)                              pixel_data = c565(4'd9, 4'd5, 4'd1);
    end
endmodule
