`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// oled_dashboard : 96x64 RGB565 results screen (CONTRACT 8).
//
// Combinational pixel generator for Oled_Display (6.25 MHz domain).  Every
// input is a snapshot register from oled_stats (100 MHz domain, changes at
// ~10 Hz), so a value changing mid-frame costs at most one glitched frame.
//
// 16 chars x 8 lines (6x8 px cells, 5x7 glyphs).  Two lines per engine, in
// engine order BFS, DIJ, A*, DFS:
//
//     BFS #1 CY 001204        name, finish place (#1..#4, #- while running),
//     E192 K0026 FOUND        cycles / expansions / path cost (decimal) and
//                             status: FOUND, NOPTH, BUSY, IDLE
//
//     DIJ OFF                 engine not selected for this run
//
// Colours match the VGA overlay: BFS cyan, DIJ yellow, A* green, DFS pink.
// The "#n" place is drawn white.
//////////////////////////////////////////////////////////////////////////////
module oled_dashboard (
    input  wire [12:0]     pixel_index,
    input  wire [3:0]      snap_mask,       // {dfs, ast, dij, bfs}
    input  wire [4*4-1:0]  snap_status,     // per engine {busy, done, found, no_path}
    input  wire [4*3-1:0]  snap_place,      // per engine 0 = none, 1..4
    input  wire [4*24-1:0] cyc_bcd,         // per engine 6 BCD digits
    input  wire [4*12-1:0] exp_bcd,         // per engine 3 BCD digits
    input  wire [4*16-1:0] cost_bcd,        // per engine 4 BCD digits
    output reg  [15:0]     pixel_data
);
    localparam OLED_W = `ARENA_OLED_W;
    localparam [15:0] WHITE = 16'hFFFF;

    wire [6:0] px     = pixel_index % OLED_W;
    wire [5:0] py     = pixel_index / OLED_W;
    wire [3:0] chr    = px / 6;            // text column 0..15
    wire [2:0] gx     = px % 6;
    wire [2:0] gy     = py % 8;
    wire [2:0] line   = py / 8;            // text line 0..7
    wire [1:0] eng    = line[2:1];         // engine shown on this line
    wire       second = line[0];           // 0 = name/cycles line, 1 = stats line

    wire        sel    = snap_mask[eng];
    wire [3:0]  status = snap_status[eng*4 +: 4];
    wire [2:0]  place  = snap_place[eng*3 +: 3];
    wire [23:0] cyc    = cyc_bcd[eng*24 +: 24];
    wire [11:0] expn   = exp_bcd[eng*12 +: 12];
    wire [15:0] cost   = cost_bcd[eng*16 +: 16];

    function [4:0] glyph;
        input [7:0] c; input [2:0] r;
        begin
            case (c)
                "0": case(r) 0:glyph=14;1:glyph=17;2:glyph=19;3:glyph=21;4:glyph=25;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "1": case(r) 0:glyph=4;1:glyph=12;2:glyph=4;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=14;default:glyph=0; endcase
                "2": case(r) 0:glyph=14;1:glyph=17;2:glyph=1;3:glyph=2;4:glyph=4;5:glyph=8;6:glyph=31;default:glyph=0; endcase
                "3": case(r) 0:glyph=30;1:glyph=1;2:glyph=1;3:glyph=14;4:glyph=1;5:glyph=1;6:glyph=30;default:glyph=0; endcase
                "4": case(r) 0:glyph=2;1:glyph=6;2:glyph=10;3:glyph=18;4:glyph=31;5:glyph=2;6:glyph=2;default:glyph=0; endcase
                "5": case(r) 0:glyph=31;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=1;5:glyph=1;6:glyph=30;default:glyph=0; endcase
                "6": case(r) 0:glyph=14;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=17;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "7": case(r) 0:glyph=31;1:glyph=1;2:glyph=2;3:glyph=4;4:glyph=8;5:glyph=8;6:glyph=8;default:glyph=0; endcase
                "8": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=14;4:glyph=17;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "9": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=15;4:glyph=1;5:glyph=1;6:glyph=14;default:glyph=0; endcase
                "A": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=31;4:glyph=17;5:glyph=17;6:glyph=17;default:glyph=0; endcase
                "B": case(r) 0:glyph=30;1:glyph=17;2:glyph=17;3:glyph=30;4:glyph=17;5:glyph=17;6:glyph=30;default:glyph=0; endcase
                "C": case(r) 0:glyph=14;1:glyph=17;2:glyph=16;3:glyph=16;4:glyph=16;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "D": case(r) 0:glyph=30;1:glyph=17;2:glyph=17;3:glyph=17;4:glyph=17;5:glyph=17;6:glyph=30;default:glyph=0; endcase
                "E": case(r) 0:glyph=31;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=16;5:glyph=16;6:glyph=31;default:glyph=0; endcase
                "F": case(r) 0:glyph=31;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=16;5:glyph=16;6:glyph=16;default:glyph=0; endcase
                "H": case(r) 0:glyph=17;1:glyph=17;2:glyph=17;3:glyph=31;4:glyph=17;5:glyph=17;6:glyph=17;default:glyph=0; endcase
                "I": case(r) 0:glyph=14;1:glyph=4;2:glyph=4;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=14;default:glyph=0; endcase
                "J": case(r) 0:glyph=7;1:glyph=2;2:glyph=2;3:glyph=2;4:glyph=2;5:glyph=18;6:glyph=12;default:glyph=0; endcase
                "K": case(r) 0:glyph=17;1:glyph=18;2:glyph=20;3:glyph=24;4:glyph=20;5:glyph=18;6:glyph=17;default:glyph=0; endcase
                "L": case(r) 0:glyph=16;1:glyph=16;2:glyph=16;3:glyph=16;4:glyph=16;5:glyph=16;6:glyph=31;default:glyph=0; endcase
                "N": case(r) 0:glyph=17;1:glyph=25;2:glyph=21;3:glyph=19;4:glyph=17;5:glyph=17;6:glyph=17;default:glyph=0; endcase
                "O": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=17;4:glyph=17;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "P": case(r) 0:glyph=30;1:glyph=17;2:glyph=17;3:glyph=30;4:glyph=16;5:glyph=16;6:glyph=16;default:glyph=0; endcase
                "S": case(r) 0:glyph=15;1:glyph=16;2:glyph=16;3:glyph=14;4:glyph=1;5:glyph=1;6:glyph=30;default:glyph=0; endcase
                "T": case(r) 0:glyph=31;1:glyph=4;2:glyph=4;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=4;default:glyph=0; endcase
                "U": case(r) 0:glyph=17;1:glyph=17;2:glyph=17;3:glyph=17;4:glyph=17;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "Y": case(r) 0:glyph=17;1:glyph=17;2:glyph=10;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=4;default:glyph=0; endcase
                "#": case(r) 0:glyph=10;1:glyph=10;2:glyph=31;3:glyph=10;4:glyph=31;5:glyph=10;6:glyph=10;default:glyph=0; endcase
                "*": case(r) 1:glyph=4;2:glyph=21;3:glyph=14;4:glyph=21;5:glyph=4;default:glyph=0; endcase
                "-": case(r) 3:glyph=31;default:glyph=0; endcase
                default: glyph=0;
            endcase
        end
    endfunction

    function [7:0] digit;                  // BCD nibble -> ASCII
        input [3:0] n;
        digit = "0" + n;
    endfunction

    function [7:0] name_ch;                // engine name, 3 chars
        input [1:0] e; input [1:0] i;
        case (e)
            2'd0:    name_ch = (i == 0) ? "B" : (i == 1) ? "F" : "S";
            2'd1:    name_ch = (i == 0) ? "D" : (i == 1) ? "I" : "J";
            2'd2:    name_ch = (i == 0) ? "A" : (i == 1) ? "*" : " ";
            default: name_ch = (i == 0) ? "D" : (i == 1) ? "F" : "S";
        endcase
    endfunction

    function [7:0] status_ch;              // status word, 5 chars
        input [3:0] st; input [2:0] i;     // st = {busy, done, found, no_path}
        reg [39:0] w;
        begin
            if (st[2] && st[1]) w = "FOUND";
            else if (st[2])     w = "NOPTH";
            else if (st[3])     w = "BUSY ";
            else                w = "IDLE ";
            status_ch = w[(4-i)*8 +: 8];
        end
    endfunction

    reg [7:0]  ch;
    reg [15:0] fg;
    reg [4:0]  bits;
    always @* begin
        case (eng)
            2'd0:    fg = 16'h07FF;        // BFS cyan
            2'd1:    fg = 16'hFFE0;        // DIJ yellow
            2'd2:    fg = 16'h07E0;        // A* green
            default: fg = 16'hFB35;        // DFS pink
        endcase
        ch = " ";
        if (!second) begin
            // BFS #1 CY 001204
            if (chr < 3)                       ch = name_ch(eng, chr[1:0]);
            else if (!sel) begin
                if (chr == 4)                  ch = "O";
                else if (chr == 5 || chr == 6) ch = "F";
            end else begin
                if (chr == 4)       begin ch = "#"; fg = WHITE; end
                else if (chr == 5)  begin ch = (place == 0) ? "-" : digit({1'b0, place}); fg = WHITE; end
                else if (chr == 7)  ch = "C";
                else if (chr == 8)  ch = "Y";
                else if (chr >= 10) ch = digit(cyc[(15-chr)*4 +: 4]);
            end
        end else if (sel) begin
            // E192 K0026 FOUND
            if (chr == 0)                      ch = "E";
            else if (chr <= 3)                 ch = digit(expn[(3-chr)*4 +: 4]);
            else if (chr == 5)                 ch = "K";
            else if (chr >= 6 && chr <= 9)     ch = digit(cost[(9-chr)*4 +: 4]);
            else if (chr >= 11)                ch = status_ch(status, chr - 4'd11);
        end
        bits = glyph(ch, gy);
        pixel_data = 16'h0000;
        if (gx < 5 && bits[4-gx]) pixel_data = fg;
    end
endmodule
