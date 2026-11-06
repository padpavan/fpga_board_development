`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// gv_oled_menu : the OLED menu, points leaderboard and algorithm pages
// (CONTRACT 8b, v1.10).  arena_top picks the page with sw[11:9]:
//
//   000 automatic (oled_screen / gv_oled_plot, not drawn here)
//   001 MENU        lists the pages and their switch codes
//   010 graph       (gv_oled_plot)        011 race results (oled_screen view 1)
//   100 POINTS      leaderboard, ranked by total points
//   101 ALGORITHMS  one entry per engine
//
// Points (clk, 100 MHz): a demo race is scored when every engine selected for
// it is done.  Engines that found a path are ranked by cycles: place = 1 + the
// number of selected, found engines with fewer cycles (ties share a place),
// and places 1/2/3/4 earn 3/2/1/0 points.  No path or not selected: 0.  rst
// (BTNU) clears the totals; a race cut short by rst never counts.  Totals and
// the race count saturate at 999.
//
// Pixel side: combinational from Oled_Display's pixel_index (6.25 MHz), 16 x 8
// characters of 6 x 8 px.  It reads only registers of the 100 MHz domain, which
// change at most once per race (CONTRACT 8): at worst one glitched frame.
//////////////////////////////////////////////////////////////////////////////
module gv_oled_menu #(
    parameter CNT_W  = `ARENA_CNT_W,
    parameter OLED_W = `ARENA_OLED_W
) (
    input  wire               clk,
    input  wire               rst,
    // demo race (clk domain); buses packed {dfs, ast, dij, bfs}
    input  wire               start,          // demo start pulse (never a GAME search)
    input  wire [3:0]         run_mask,
    input  wire [3:0]         done,
    input  wire [3:0]         found,
    input  wire [4*CNT_W-1:0] cycles,
    // page to draw (registered on clk by arena_top): 1 menu, 4 points, 5 algorithms
    input  wire [2:0]         page,
    input  wire [12:0]        pixel_index,
    output reg  [15:0]        pixel_data
);
    localparam [15:0] BLACK = 16'h0000, WHITE = 16'hFFFF, GREY = 16'h7BEF, YELLOW = 16'hFFE0;
    localparam [9:0]  MAXV  = 10'd999;

    // ======================================================= points (clk) ===
    reg        armed = 1'b0;
    reg [3:0]  rmask = 4'd0;
    reg        s1 = 1'b0;
    reg [3:0]  s1_win;                       // engine scores this race
    reg [1:0]  s1_fewer [0:3];
    reg [9:0]  total [0:3];
    reg [9:0]  races = 10'd0;
    wire       race_done = armed && ((done & rmask) == rmask);

    function [CNT_W-1:0] cyc; input integer e; cyc = cycles[e*CNT_W +: CNT_W]; endfunction
    function [1:0] pts; input [1:0] fewer;
        pts = (fewer == 2'd0) ? 2'd3 : (fewer == 2'd1) ? 2'd2 : (fewer == 2'd2) ? 2'd1 : 2'd0;
    endfunction

    integer e, o;
    reg [1:0] cnt;
    always @(posedge clk) begin
        if (rst) begin
            armed <= 1'b0; s1 <= 1'b0; races <= 10'd0;
            for (e = 0; e < 4; e = e + 1) total[e] <= 10'd0;
        end else begin
            s1 <= 1'b0;
            if (start && run_mask != 4'd0) begin
                armed <= 1'b1;                    // done drops on the same edge
                rmask <= run_mask;
            end else if (race_done) begin
                armed <= 1'b0;
                s1    <= 1'b1;
                for (e = 0; e < 4; e = e + 1) begin
                    cnt = 2'd0;
                    for (o = 0; o < 4; o = o + 1)
                        if (o != e && rmask[o] && found[o] && (cyc(o) < cyc(e)))
                            cnt = cnt + 1'b1;
                    s1_fewer[e] <= cnt;
                    s1_win[e]   <= rmask[e] && found[e];
                end
            end
            if (s1) begin
                races <= (races == MAXV) ? MAXV : races + 1'b1;
                for (e = 0; e < 4; e = e + 1)
                    if (s1_win[e])
                        total[e] <= (total[e] + pts(s1_fewer[e]) > MAXV) ? MAXV
                                                                       : total[e] + pts(s1_fewer[e]);
            end
        end
    end

    // leaderboard order and decimal digits, registered for the pixel side
    function [11:0] bcd3; input [9:0] b;
        integer i; reg [21:0] t;
        begin
            t = {12'd0, b};
            for (i = 0; i < 10; i = i + 1) begin
                if (t[13:10] >= 5) t[13:10] = t[13:10] + 3;
                if (t[17:14] >= 5) t[17:14] = t[17:14] + 3;
                if (t[21:18] >= 5) t[21:18] = t[21:18] + 3;
                t = t << 1;
            end
            bcd3 = t[21:10];
        end
    endfunction
    reg [7:0]  order = 8'b11_10_01_00;        // engine in rank 0..3 (2 bits each)
    reg [47:0] tot_bcd = 48'd0;               // per engine, 3 BCD digits
    reg [11:0] races_bcd = 12'd0;
    reg [1:0]  rank [0:3];
    reg [1:0]  cnt2;
    integer r, e2, o2;
    always @(posedge clk) begin
        for (e2 = 0; e2 < 4; e2 = e2 + 1) begin
            cnt2 = 2'd0;
            for (o2 = 0; o2 < 4; o2 = o2 + 1)
                if ((total[o2] > total[e2]) || ((total[o2] == total[e2]) && (o2 < e2)))
                    cnt2 = cnt2 + 1'b1;
            rank[e2] <= cnt2;
            tot_bcd[e2*12 +: 12] <= bcd3(total[e2]);
        end
        for (r = 0; r < 4; r = r + 1)
            for (e2 = 0; e2 < 4; e2 = e2 + 1)
                if (rank[e2] == r) order[r*2 +: 2] <= e2;
        races_bcd <= bcd3(races);
    end
    integer ei;
    initial for (ei = 0; ei < 4; ei = ei + 1) begin total[ei] = 10'd0; rank[ei] = ei; end

    // ====================================================== pixel side ======
    wire [6:0] x    = pixel_index % OLED_W;
    wire [5:0] y    = pixel_index / OLED_W;
    wire [3:0] chr  = x / 6;
    wire [2:0] gx   = x % 6;
    wire [2:0] gy   = y[2:0];
    wire [2:0] line = y[5:3];

    // 5x7 font; constant slices only (see gv_oled_plot)
    function [4:0] glyph;
        input [7:0] c; input [2:0] row;
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
                "K": g = {5'd17,5'd18,5'd20,5'd24,5'd20,5'd18,5'd17};
                "L": g = {5'd16,5'd16,5'd16,5'd16,5'd16,5'd16,5'd31};
                "M": g = {5'd17,5'd27,5'd21,5'd21,5'd17,5'd17,5'd17};
                "N": g = {5'd17,5'd25,5'd21,5'd19,5'd17,5'd17,5'd17};
                "O": g = {5'd14,5'd17,5'd17,5'd17,5'd17,5'd17,5'd14};
                "P": g = {5'd30,5'd17,5'd17,5'd30,5'd16,5'd16,5'd16};
                "Q": g = {5'd14,5'd17,5'd17,5'd17,5'd21,5'd18,5'd13};
                "R": g = {5'd30,5'd17,5'd17,5'd30,5'd20,5'd18,5'd17};
                "S": g = {5'd15,5'd16,5'd16,5'd14,5'd1, 5'd1, 5'd30};
                "T": g = {5'd31,5'd4, 5'd4, 5'd4, 5'd4, 5'd4, 5'd4};
                "U": g = {5'd17,5'd17,5'd17,5'd17,5'd17,5'd17,5'd14};
                "V": g = {5'd17,5'd17,5'd17,5'd17,5'd17,5'd10,5'd4};
                "W": g = {5'd17,5'd17,5'd17,5'd21,5'd21,5'd21,5'd10};
                "X": g = {5'd17,5'd17,5'd10,5'd4, 5'd10,5'd17,5'd17};
                "Y": g = {5'd17,5'd17,5'd10,5'd4, 5'd4, 5'd4, 5'd4};
                "Z": g = {5'd31,5'd1, 5'd2, 5'd4, 5'd8, 5'd16,5'd31};
                "*": g = {5'd0, 5'd4, 5'd21,5'd14,5'd21,5'd4, 5'd0};
                "-": g = {5'd0, 5'd0, 5'd0, 5'd31,5'd0, 5'd0, 5'd0};
                ",": g = {5'd0, 5'd0, 5'd0, 5'd0, 5'd12,5'd4, 5'd8};
                ":": g = {5'd0, 5'd12,5'd12,5'd0, 5'd12,5'd12,5'd0};
                default: g = 35'd0;
            endcase
            case (row)
                3'd0: glyph = g[34:30];  3'd1: glyph = g[29:25];
                3'd2: glyph = g[24:20];  3'd3: glyph = g[19:15];
                3'd4: glyph = g[14:10];  3'd5: glyph = g[9:5];
                3'd6: glyph = g[4:0];    default: glyph = 5'd0;
            endcase
        end
    endfunction

    // one character of a 16-char string (constant slices)
    function [7:0] pick; input [127:0] s; input [3:0] i;
        case (i)
            4'd0:  pick = s[127:120]; 4'd1:  pick = s[119:112]; 4'd2:  pick = s[111:104]; 4'd3:  pick = s[103:96];
            4'd4:  pick = s[95:88];   4'd5:  pick = s[87:80];   4'd6:  pick = s[79:72];   4'd7:  pick = s[71:64];
            4'd8:  pick = s[63:56];   4'd9:  pick = s[55:48];   4'd10: pick = s[47:40];   4'd11: pick = s[39:32];
            4'd12: pick = s[31:24];   4'd13: pick = s[23:16];   4'd14: pick = s[15:8];    default: pick = s[7:0];
        endcase
    endfunction

    function [15:0] eng_colour; input [1:0] en;           // = oled_screen / VGA
        case (en)
            2'd0:    eng_colour = 16'h07FF;               // BFS cyan
            2'd1:    eng_colour = 16'hFFE0;               // DIJ yellow
            2'd2:    eng_colour = 16'h07E0;               // A* green
            default: eng_colour = 16'hFB35;               // DFS pink
        endcase
    endfunction
    function [23:0] name3; input [1:0] en;
        case (en) 2'd0: name3 = "BFS"; 2'd1: name3 = "DIJ"; 2'd2: name3 = "A* "; default: name3 = "DFS"; endcase
    endfunction

    // ---- MENU
    function [127:0] menu_line; input [2:0] l;
        case (l)
            3'd0: menu_line = "MENU     SW11-9 ";
            3'd1: menu_line = "000 AUTO        ";
            3'd2: menu_line = "001 MENU        ";
            3'd3: menu_line = "010 GRAPH       ";
            3'd4: menu_line = "011 RESULTS     ";
            3'd5: menu_line = "100 POINTS      ";
            3'd6: menu_line = "101 ALGORITHMS  ";
            default: menu_line = "                ";
        endcase
    endfunction

    // ---- ALGORITHMS: two lines per engine, name in its colour
    function [127:0] info_line; input [2:0] l;
        case (l)
            3'd0: info_line = "BFS FEWEST STEPS";
            3'd1: info_line = "    FIFO QUEUE  ";
            3'd2: info_line = "DIJ LEAST COST  ";
            3'd3: info_line = "    MIN-HEAP    ";
            3'd4: info_line = "A*  LEAST COST  ";
            3'd5: info_line = "    MANHATTAN H ";
            3'd6: info_line = "DFS ANY PATH    ";
            default: info_line = "    GOES DEEP   ";
        endcase
    endfunction

    // ---- POINTS: "POINTS 3-2-1-0", "RACES nnn", ranked lines 3..6, "BTNU RESETS"
    // engine on ranked line 3..6, and its total (constant slices)
    reg [1:0]  pe;
    reg [11:0] pv;
    always @* begin
        case (line)
            3'd3:    pe = order[1:0];
            3'd4:    pe = order[3:2];
            3'd5:    pe = order[5:4];
            default: pe = order[7:6];
        endcase
        case (pe)
            2'd0:    pv = tot_bcd[11:0];
            2'd1:    pv = tot_bcd[23:12];
            2'd2:    pv = tot_bcd[35:24];
            default: pv = tot_bcd[47:36];
        endcase
    end
    // digits with leading zeros blanked (the last digit always shows)
    function [7:0] d3; input [11:0] v; input [1:0] i;
        case (i)
            2'd0: d3 = (v[11:8] != 0) ? "0" + v[11:8] : " ";
            2'd1: d3 = (v[11:4] != 0) ? "0" + v[7:4]  : " ";
            default: d3 = "0" + v[3:0];
        endcase
    endfunction
    reg [7:0]  ch;
    reg [15:0] fg;
    always @* begin
        ch = " "; fg = WHITE;
        case (page)
            3'd1: begin
                ch = pick(menu_line(line), chr);
                fg = (line == 3'd0) ? YELLOW : (chr < 3) ? GREY : WHITE;
            end
            3'd5: begin
                ch = pick(info_line(line), chr);
                fg = (chr < 3 && !line[0]) ? eng_colour(line[2:1]) : (line[0] ? GREY : WHITE);
            end
            3'd4: begin
                if (line == 3'd0)      begin ch = pick("POINTS  3-2-1-0 ", chr); fg = YELLOW; end
                else if (line == 3'd1) begin
                    ch = (chr < 6) ? pick("RACES           ", chr) :
                         (chr < 9) ? d3(races_bcd, chr - 4'd6) : " ";
                end else if (line >= 3'd3 && line <= 3'd6) begin
                    fg = eng_colour(pe);
                    case (chr)
                        4'd0:  ch = "1" + (line - 3'd3);
                        4'd3:  ch = name3(pe) >> 16;
                        4'd4:  ch = name3(pe) >> 8;
                        4'd5:  ch = name3(pe);
                        4'd10: ch = d3(pv, 2'd0);
                        4'd11: ch = d3(pv, 2'd1);
                        4'd12: ch = d3(pv, 2'd2);
                        4'd14: ch = "P";
                        4'd15: ch = "T";
                        default: ch = " ";
                    endcase
                end else if (line == 3'd7) begin ch = pick("BTNU RESETS     ", chr); fg = GREY; end
            end
            default: ch = " ";
        endcase
    end
    wire [4:0] bits = glyph(ch, gy);
    always @*
        pixel_data = ((gx < 5) && ((gx == 0) ? bits[4] : (gx == 1) ? bits[3] : (gx == 2) ? bits[2] :
                                   (gx == 3) ? bits[1] : bits[0])) ? fg : BLACK;
endmodule
