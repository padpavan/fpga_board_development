`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// oled_screen : arena_top's 96x64 RGB565 OLED screen (CONTRACT 8, v1.7).
//
// Combinational pixel generator for Oled_Display (6.25 MHz domain).  Every
// input is a register in the 100 MHz domain that changes at most ~10 Hz
// (oled_stats snapshot, or arena_top's view registers), so a change costs at
// most one glitched frame.  oled_dashboard (the older screen) stays for
// m2_race_top.
//
// 16 chars x 8 lines (6x8 px cells, 5x7 glyphs).  Three views:
//
//  view 0  SELECT (demo EDIT)        view 1  RESULTS (demo LOAD..DONE)
//    SELECT ENGINES                    DFS   6.88us 1ST     ranked fastest
//                                      FASTEST              first, 2 lines
//    BFS  ON                           BFS  12.04us 2ND     per engine
//    DIJ OFF                            +5.16us
//    A*   ON                           A*  RUN              still running
//    DFS  ON                           DIJ OFF              not selected
//
//    BTNC RUN
//
//  view 2  GAME: "SCORE" and the score in x3 digits.
//
// Time = cycles x 10 ns (100 MHz), shown in us: the 6 BCD cycle digits with
// a decimal point before the last two (001204 -> "12.04us"), leading zeros
// blanked.  cycles excludes animation pauses (CONTRACT 4.3), so this is the
// engine's compute time at full speed.
//////////////////////////////////////////////////////////////////////////////
module oled_screen (
    input  wire [12:0]     pixel_index,
    input  wire [1:0]      view,            // 0 select, 1 results, 2 game
    input  wire [3:0]      sel,             // live engine switches {dfs, ast, dij, bfs}
    input  wire [3:0]      snap_mask,       // engines in the run
    input  wire [4*4-1:0]  snap_status,     // per engine {busy, done, found, no_path}
    input  wire [4*3-1:0]  snap_place,      // per engine 0 = none, 1..4
    input  wire [4*2-1:0]  snap_order,      // engine id per results slot
    input  wire [4*24-1:0] cyc_bcd,         // per engine 6 BCD digits
    input  wire [4*24-1:0] gap_bcd,         // per engine 6 BCD digits
    input  wire [19:0]     score_bcd,       // 5 BCD digits
    output reg  [15:0]     pixel_data
);
    localparam OLED_W = `ARENA_OLED_W;
    localparam [15:0] BLACK = 16'h0000, WHITE = 16'hFFFF, GREY = 16'h7BEF, YELLOW = 16'hFFE0;
    localparam [1:0] V_SELECT = 2'd0, V_RESULTS = 2'd1, V_GAME = 2'd2;

    wire [6:0] px   = pixel_index % OLED_W;
    wire [5:0] py   = pixel_index / OLED_W;
    wire [3:0] chr  = px / 6;               // text column 0..15
    wire [2:0] gx   = px % 6;
    wire [2:0] gy   = py % 8;
    wire [2:0] line = py / 8;               // text line 0..7

    function [4:0] glyph;
        input [7:0] c; input [2:0] r;
        reg [34:0] g;
        begin
            case (c)
                "0": g = {5'd14,5'd17,5'd19,5'd21,5'd25,5'd17,5'd14};
                "1": g = {5'd4, 5'd12,5'd4, 5'd4, 5'd4, 5'd4, 5'd14};
                "2": g = {5'd14,5'd17,5'd1, 5'd2, 5'd4, 5'd8, 5'd31};
                "3": g = {5'd30,5'd1, 5'd1, 5'd14,5'd1, 5'd1, 5'd30};
                "4": g = {5'd2, 5'd6, 5'd10,5'd18,5'd31,5'd2, 5'd2};
                "5": g = {5'd31,5'd16,5'd16,5'd30,5'd1, 5'd1, 5'd30};
                "6": g = {5'd14,5'd16,5'd16,5'd30,5'd17,5'd17,5'd14};
                "7": g = {5'd31,5'd1, 5'd2, 5'd4, 5'd8, 5'd8, 5'd8};
                "8": g = {5'd14,5'd17,5'd17,5'd14,5'd17,5'd17,5'd14};
                "9": g = {5'd14,5'd17,5'd17,5'd15,5'd1, 5'd1, 5'd14};
                "A": g = {5'd14,5'd17,5'd17,5'd31,5'd17,5'd17,5'd17};
                "B": g = {5'd30,5'd17,5'd17,5'd30,5'd17,5'd17,5'd30};
                "C": g = {5'd14,5'd17,5'd16,5'd16,5'd16,5'd17,5'd14};
                "D": g = {5'd30,5'd17,5'd17,5'd17,5'd17,5'd17,5'd30};
                "E": g = {5'd31,5'd16,5'd16,5'd30,5'd16,5'd16,5'd31};
                "F": g = {5'd31,5'd16,5'd16,5'd30,5'd16,5'd16,5'd16};
                "G": g = {5'd14,5'd17,5'd16,5'd23,5'd17,5'd17,5'd15};
                "H": g = {5'd17,5'd17,5'd17,5'd31,5'd17,5'd17,5'd17};
                "I": g = {5'd14,5'd4, 5'd4, 5'd4, 5'd4, 5'd4, 5'd14};
                "J": g = {5'd7, 5'd2, 5'd2, 5'd2, 5'd2, 5'd18,5'd12};
                "L": g = {5'd16,5'd16,5'd16,5'd16,5'd16,5'd16,5'd31};
                "M": g = {5'd17,5'd27,5'd21,5'd21,5'd17,5'd17,5'd17};
                "N": g = {5'd17,5'd25,5'd21,5'd19,5'd17,5'd17,5'd17};
                "O": g = {5'd14,5'd17,5'd17,5'd17,5'd17,5'd17,5'd14};
                "P": g = {5'd30,5'd17,5'd17,5'd30,5'd16,5'd16,5'd16};
                "R": g = {5'd30,5'd17,5'd17,5'd30,5'd20,5'd18,5'd17};
                "S": g = {5'd15,5'd16,5'd16,5'd14,5'd1, 5'd1, 5'd30};
                "T": g = {5'd31,5'd4, 5'd4, 5'd4, 5'd4, 5'd4, 5'd4};
                "U": g = {5'd17,5'd17,5'd17,5'd17,5'd17,5'd17,5'd14};
                "u": g = {5'd0, 5'd0, 5'd17,5'd17,5'd17,5'd19,5'd13};
                "s": g = {5'd0, 5'd0, 5'd15,5'd16,5'd14,5'd1, 5'd30};
                "*": g = {5'd0, 5'd4, 5'd21,5'd14,5'd21,5'd4, 5'd0};
                "+": g = {5'd0, 5'd4, 5'd4, 5'd31,5'd4, 5'd4, 5'd0};
                ".": g = {5'd0, 5'd0, 5'd0, 5'd0, 5'd0, 5'd12,5'd12};
                default: g = 35'd0;
            endcase
            glyph = g[(6-r)*5 +: 5];
        end
    endfunction

    function [7:0] name_ch;                // engine name, 3 chars
        input [1:0] e; input [3:0] i;
        case (e)
            2'd0:    name_ch = (i == 0) ? "B" : (i == 1) ? "F" : "S";
            2'd1:    name_ch = (i == 0) ? "D" : (i == 1) ? "I" : "J";
            2'd2:    name_ch = (i == 0) ? "A" : (i == 1) ? "*" : " ";
            default: name_ch = (i == 0) ? "D" : (i == 1) ? "F" : "S";
        endcase
    endfunction

    function [15:0] eng_colour;
        input [1:0] e;
        case (e)
            2'd0:    eng_colour = 16'h07FF;        // BFS cyan
            2'd1:    eng_colour = 16'hFFE0;        // DIJ yellow
            2'd2:    eng_colour = 16'h07E0;        // A* green
            default: eng_colour = 16'hFB35;        // DFS pink
        endcase
    endfunction

    // index of the first digit shown (0..3) of a 6-digit value: d5..d3 are
    // blanked while they are leading zeros, d2 (units of us) always shows
    function [1:0] first_digit;
        input [23:0] v;
        first_digit = (v[23:20] != 0) ? 2'd0 : (v[19:16] != 0) ? 2'd1 :
                      (v[15:12] != 0) ? 2'd2 : 2'd3;
    endfunction

    // one char of a 7-char time field "dddd.dd" (col 0..6), leading zeros blank
    function [7:0] time_ch;
        input [23:0] v; input [2:0] col;
        reg [1:0] f;
        begin
            f = first_digit(v);
            case (col)
                3'd0: time_ch = (f <= 0) ? "0" + v[23:20] : " ";
                3'd1: time_ch = (f <= 1) ? "0" + v[19:16] : " ";
                3'd2: time_ch = (f <= 2) ? "0" + v[15:12] : " ";
                3'd3: time_ch = "0" + v[11:8];
                3'd4: time_ch = ".";
                3'd5: time_ch = "0" + v[7:4];
                default: time_ch = "0" + v[3:0];
            endcase
        end
    endfunction

    // ------------------------------------------------------------ text -----
    wire [1:0]  slot   = line[2:1];               // results: slot 0..3
    wire        second = line[0];
    wire [1:0]  re     = snap_order[slot*2 +: 2];  // engine in this slot
    wire [3:0]  st     = snap_status[re*4 +: 4];
    wire [2:0]  place  = snap_place[re*3 +: 3];
    wire [23:0] cyc    = cyc_bcd[re*24 +: 24];
    wire [23:0] gap    = gap_bcd[re*24 +: 24];
    wire        r_sel  = snap_mask[re];
    wire        r_done = st[2];
    wire [1:0]  gap_f  = first_digit(gap);
    wire [1:0]  se     = line[1:0] - 2'd2;         // select view: engine on lines 2..5

    reg [7:0]  ch;
    reg [15:0] fg;
    always @* begin
        ch = " "; fg = WHITE;
        case (view)
            V_SELECT: begin
                if (line == 0) begin
                    case (chr)          // "SELECT ENGINES"
                        0: ch="S"; 1: ch="E"; 2: ch="L"; 3: ch="E"; 4: ch="C"; 5: ch="T";
                        7: ch="E"; 8: ch="N"; 9: ch="G"; 10: ch="I"; 11: ch="N"; 12: ch="E"; 13: ch="S";
                        default: ch=" ";
                    endcase
                end else if (line >= 2 && line <= 5) begin
                    fg = sel[se] ? eng_colour(se) : GREY;
                    if (chr < 3)                        ch = name_ch(se, chr);
                    else if (sel[se] && chr == 5)       ch = "O";
                    else if (sel[se] && chr == 6)       ch = "N";
                    else if (!sel[se] && chr == 4)      ch = "O";
                    else if (!sel[se] && (chr == 5 || chr == 6)) ch = "F";
                end else if (line == 7) begin
                    case (chr)          // "BTNC RUN"
                        0: ch="B"; 1: ch="T"; 2: ch="N"; 3: ch="C"; 5: ch="R"; 6: ch="U"; 7: ch="N";
                        default: ch=" ";
                    endcase
                end
            end
            V_RESULTS: begin
                fg = r_sel ? eng_colour(re) : GREY;
                if (!second) begin
                    if (chr < 3)                          ch = name_ch(re, chr);
                    else if (!r_sel)                      ch = (chr == 4) ? "O" : (chr == 5 || chr == 6) ? "F" : " ";
                    else if (!r_done)                     ch = (chr == 4) ? "R" : (chr == 5) ? "U" : (chr == 6) ? "N" : " ";
                    else if (chr >= 3 && chr <= 9)        ch = time_ch(cyc, chr - 4'd3);
                    else if (chr == 10)                   ch = "u";
                    else if (chr == 11)                   ch = "s";
                    else if (chr >= 13) begin
                        fg = WHITE;
                        case (place)
                            3'd1: ch = (chr == 13) ? "1" : (chr == 14) ? "S" : "T";
                            3'd2: ch = (chr == 13) ? "2" : (chr == 14) ? "N" : "D";
                            3'd3: ch = (chr == 13) ? "3" : (chr == 14) ? "R" : "D";
                            3'd4: ch = (chr == 13) ? "4" : (chr == 14) ? "T" : "H";
                            default: ch = " ";
                        endcase
                    end
                end else if (r_sel && r_done) begin
                    if (st[0]) begin                      // no path
                        case (chr)
                            4: ch="N"; 5: ch="O"; 7: ch="P"; 8: ch="A"; 9: ch="T"; 10: ch="H";
                            default: ch=" ";
                        endcase
                    end else if (place == 3'd1) begin
                        fg = WHITE;
                        case (chr)
                            4: ch="F"; 5: ch="A"; 6: ch="S"; 7: ch="T"; 8: ch="E"; 9: ch="S"; 10: ch="T";
                            default: ch=" ";
                        endcase
                    end else begin                        // "+5.16us" under the time
                        if (chr == 4'd2 + gap_f)          ch = "+";
                        else if (chr >= 3 && chr <= 9)    ch = time_ch(gap, chr - 4'd3);
                        else if (chr == 10)               ch = "u";
                        else if (chr == 11)               ch = "s";
                    end
                end
            end
            default: begin                                 // game: "SCORE" on line 1
                fg = YELLOW;
                if (line == 1)
                    case (chr)
                        5: ch="S"; 6: ch="C"; 7: ch="O"; 8: ch="R"; 9: ch="E";
                        default: ch=" ";
                    endcase
            end
        endcase
    end

    // ------------------------------------------------------- big score -----
    // 5 digits, glyph x3 (15x21 px) on an 18 px pitch from x = 3, y = 24..44
    localparam BIG_X0 = 3, BIG_Y0 = 24;
    wire [6:0] bx  = px - BIG_X0;
    wire [5:0] by  = py - BIG_Y0;
    wire       big_area = (view == V_GAME) && (px >= BIG_X0) && (px < BIG_X0 + 90) &&
                          (py >= BIG_Y0) && (py < BIG_Y0 + 21);
    wire [2:0] bdig = bx / 18;                      // digit 0 (ten-thousands) .. 4
    wire [4:0] bcol = bx % 18;
    wire [3:0] bnib = score_bcd[(4-bdig)*4 +: 4];
    // leading zeros blanked; the last digit always shows
    wire       blead = (bdig == 0) ? 1'b1 :
                       (bdig == 1) ? (score_bcd[19:16] == 0) :
                       (bdig == 2) ? (score_bcd[19:12] == 0) :
                       (bdig == 3) ? (score_bcd[19:8]  == 0) : 1'b0;
    wire       bshow = !(blead && bnib == 0) || (bdig == 4);
    wire [4:0] bbits = glyph("0" + bnib, by / 3);

    reg [4:0] bits;
    always @* begin
        bits = glyph(ch, gy);
        pixel_data = BLACK;
        if (big_area) begin
            if (bshow && bcol < 15 && bbits[4 - bcol/3]) pixel_data = WHITE;
        end else if (gx < 5 && bits[4-gx])
            pixel_data = fg;
    end
endmodule
