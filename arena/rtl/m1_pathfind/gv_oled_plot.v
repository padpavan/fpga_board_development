`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// gv_oled_plot : the OLED results view with the performance graph
// (CONTRACT 8a, v1.9: M2 measures and stores, M1 draws).
//
// Shown on the demo results view (LOAD..DONE), so the curves grow live while
// the engines run and the VGA shows the race.
//
//   y 0..15   two 5x7 text lines, two engines each, fastest first
//             (oled_stats snap_order), in the engine colours:
//               fastest   name + time in us     "BFS12.04"
//               others    name + gap in us      "DIJ+5.16"
//               running   "A*  RUN"   no path "DFS NONE"   not run "DFS OFF" (grey)
//             Times come from oled_stats (cyc_bcd / gap_bcd), cycle-exact.
//   y 16..62  plot: level L (0..46 = 0..100 %) at y = 62 - L, sample i at
//             x = i + 1 (94 samples), axes at x = 0 and y = 63.  Dotted grid at
//             50 % and 100 %.  Each curve fills from rd_prev_level to rd_level
//             in its column (continuous), stops at its engine's finish
//             (end_idx), and a dotted line in its colour drops from there to
//             the x axis.  A* on top, then Dijkstra, BFS, DFS.
//   labels    3x5 font, dim, drawn behind everything else in the plot:
//             "100%" top left, "50" left of the 50 % line, "TIME>" bottom right.
//
// Combinational from Oled_Display's pixel_index (6.25 MHz).  Reads only
// registers / LUTRAM of the 100 MHz domain (CONTRACT 8 / 8a): rd_idx goes to
// ast_perf_hist, whose combinational levels come back the same pixel.
//////////////////////////////////////////////////////////////////////////////
module gv_oled_plot #(
    parameter AW        = `ARENA_PG_ADDR_W,
    parameter LW        = `ARENA_PG_LEVEL_W,
    parameter LEVEL_MAX = `ARENA_PG_LEVEL_MAX,
    parameter SAMPLES   = `ARENA_PG_SAMPLES,
    parameter OLED_W    = `ARENA_OLED_W
) (
    input  wire [12:0]      pixel_index,
    output reg  [15:0]      pixel_data,
    // ast_perf_hist / ast_perf_sampler (CONTRACT 8a)
    output wire [AW-1:0]    rd_idx,
    input  wire [4*LW-1:0]  rd_level,
    input  wire [4*LW-1:0]  rd_prev_level,
    input  wire [AW-1:0]    count,
    input  wire [3:0]       mask,
    input  wire [4*AW-1:0]  end_idx,
    input  wire [3:0]       end_valid,
    // oled_stats snapshot (CONTRACT 8)
    input  wire [3:0]       snap_mask,
    input  wire [4*4-1:0]   snap_status,      // per engine {busy, done, found, no_path}
    input  wire [4*3-1:0]   snap_place,
    input  wire [4*2-1:0]   snap_order,       // engine per slot, fastest first
    input  wire [4*24-1:0]  cyc_bcd,
    input  wire [4*24-1:0]  gap_bcd
);
    localparam AXIS_Y = 63;
    localparam TOP_Y  = AXIS_Y - 1 - LEVEL_MAX;          // 16: level 46
    localparam MID_Y  = AXIS_Y - 1 - LEVEL_MAX/2;        // 39: level 23 (50 %)
    localparam [15:0] BLACK = 16'h0000, GREY = 16'h7BEF, AXIS = 16'h8410,
                      GRID = 16'h31A6, LABEL = 16'h6B4D;

    wire [6:0] x = pixel_index % OLED_W;
    wire [5:0] y = pixel_index / OLED_W;

    function [15:0] eng_colour; input [1:0] en;          // = oled_screen / VGA
        case (en)
            2'd0:    eng_colour = 16'h07FF;               // BFS cyan
            2'd1:    eng_colour = 16'hFFE0;               // DIJ yellow
            2'd2:    eng_colour = 16'h07E0;               // A* green
            default: eng_colour = 16'hFB35;               // DFS pink
        endcase
    endfunction

    // ------------------------------------------------------------- plot ----
    wire [AW-1:0] idx = x - 1'b1;                        // sample in this column
    assign rd_idx = (x != 0 && idx < count) ? idx : {AW{1'b0}};
    wire          in_plot = (x != 0) && (y >= TOP_Y) && (y < AXIS_Y) && (idx < count) && (idx < SAMPLES);
    wire [LW-1:0] lv = AXIS_Y - 1 - y;                   // level of this row

    reg [3:0] curve, fin;
    reg [LW-1:0] c_lv, p_lv;
    reg [AW-1:0] e_i;
    integer e;
    always @* begin
        for (e = 0; e < 4; e = e + 1) begin
            c_lv = rd_level[e*LW +: LW];
            p_lv = rd_prev_level[e*LW +: LW];
            e_i  = end_idx[e*AW +: AW];
            // levels never fall, but take either order to be safe
            curve[e] = in_plot && mask[e] && (!end_valid[e] || idx <= e_i) &&
                       (((lv >= p_lv) && (lv <= c_lv)) || ((lv <= p_lv) && (lv >= c_lv)));
            fin[e]   = in_plot && mask[e] && end_valid[e] && (idx == e_i) &&
                       (lv < c_lv) && y[0];
        end
    end
    wire grid_on = (x != 0) && !x[0] && (y == TOP_Y || y == MID_Y);
    wire axis_on = (x == 0 && y >= TOP_Y) || (y == AXIS_Y);

    // ------------------------------------------------------- 5x7 header ----
    // constant slices only (a computed part-select into a glyph crashed
    // Vivado 2025.2 synthesis in an earlier draft)
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
                "D": g = {5'd30,5'd17,5'd17,5'd17,5'd17,5'd17,5'd30};
                "E": g = {5'd31,5'd16,5'd16,5'd30,5'd16,5'd16,5'd31};
                "F": g = {5'd31,5'd16,5'd16,5'd30,5'd16,5'd16,5'd16};
                "I": g = {5'd14,5'd4, 5'd4, 5'd4, 5'd4, 5'd4, 5'd14};
                "J": g = {5'd7, 5'd2, 5'd2, 5'd2, 5'd2, 5'd18,5'd12};
                "N": g = {5'd17,5'd25,5'd21,5'd19,5'd17,5'd17,5'd17};
                "O": g = {5'd14,5'd17,5'd17,5'd17,5'd17,5'd17,5'd14};
                "R": g = {5'd30,5'd17,5'd17,5'd30,5'd20,5'd18,5'd17};
                "S": g = {5'd15,5'd16,5'd16,5'd14,5'd1, 5'd1, 5'd30};
                "U": g = {5'd17,5'd17,5'd17,5'd17,5'd17,5'd17,5'd14};
                "*": g = {5'd0, 5'd4, 5'd21,5'd14,5'd21,5'd4, 5'd0};
                "+": g = {5'd0, 5'd4, 5'd4, 5'd31,5'd4, 5'd4, 5'd0};
                ".": g = {5'd0, 5'd0, 5'd0, 5'd0, 5'd0, 5'd12,5'd12};
                default: g = 35'd0;
            endcase
            case (r)
                3'd0: glyph = g[34:30];  3'd1: glyph = g[29:25];
                3'd2: glyph = g[24:20];  3'd3: glyph = g[19:15];
                3'd4: glyph = g[14:10];  3'd5: glyph = g[9:5];
                3'd6: glyph = g[4:0];    default: glyph = 5'd0;
            endcase
        end
    endfunction

    function [7:0] name_ch; input [1:0] en; input [2:0] i;
        case (en)
            2'd0:    name_ch = (i == 0) ? "B" : (i == 1) ? "F" : "S";
            2'd1:    name_ch = (i == 0) ? "D" : (i == 1) ? "I" : "J";
            2'd2:    name_ch = (i == 0) ? "A" : (i == 1) ? "*" : " ";
            default: name_ch = (i == 0) ? "D" : (i == 1) ? "F" : "S";
        endcase
    endfunction

    function [7:0] dig; input [3:0] d; dig = "0" + d; endfunction

    // 5-char time in us from 6 BCD digits d5..d0 (d2 = units of us):
    //   < 100 us "12.04" / " 6.88",  < 1000 us "123.4",  else "1234 "
    function [7:0] time5; input [23:0] v; input [2:0] col;
        begin
            if (v[23:20] != 0)
                case (col) 3'd0: time5 = dig(v[23:20]); 3'd1: time5 = dig(v[19:16]);
                           3'd2: time5 = dig(v[15:12]); 3'd3: time5 = dig(v[11:8]);
                           default: time5 = " "; endcase
            else if (v[19:16] != 0)
                case (col) 3'd0: time5 = dig(v[19:16]); 3'd1: time5 = dig(v[15:12]);
                           3'd2: time5 = dig(v[11:8]);  3'd3: time5 = ".";
                           default: time5 = dig(v[7:4]); endcase
            else
                case (col) 3'd0: time5 = (v[15:12] != 0) ? dig(v[15:12]) : " ";
                           3'd1: time5 = dig(v[11:8]);  3'd2: time5 = ".";
                           3'd3: time5 = dig(v[7:4]);   default: time5 = dig(v[3:0]); endcase
        end
    endfunction
    // "+" and a 4-char gap in us: "+5.16", "+12.3", "+123", "+1234"
    function [7:0] gap5; input [23:0] v; input [2:0] col;
        begin
            if (col == 3'd0) gap5 = "+";
            else if (v[23:20] != 0)
                case (col) 3'd1: gap5 = dig(v[23:20]); 3'd2: gap5 = dig(v[19:16]);
                           3'd3: gap5 = dig(v[15:12]); default: gap5 = dig(v[11:8]); endcase
            else if (v[19:16] != 0)
                case (col) 3'd1: gap5 = dig(v[19:16]); 3'd2: gap5 = dig(v[15:12]);
                           3'd3: gap5 = dig(v[11:8]);  default: gap5 = " "; endcase
            else if (v[15:12] != 0)
                case (col) 3'd1: gap5 = dig(v[15:12]); 3'd2: gap5 = dig(v[11:8]);
                           3'd3: gap5 = ".";           default: gap5 = dig(v[7:4]); endcase
            else
                case (col) 3'd1: gap5 = dig(v[11:8]);  3'd2: gap5 = ".";
                           3'd3: gap5 = dig(v[7:4]);   default: gap5 = dig(v[3:0]); endcase
        end
    endfunction

    wire [3:0]  chr  = x / 6;                            // 16 chars per line
    wire [2:0]  gx   = x % 6;
    wire [2:0]  gy   = y[2:0];
    wire [1:0]  slot = {y[3], chr[3]};                   // line 0: slots 0,1; line 1: 2,3
    wire [2:0]  sc   = chr[2:0];                         // char inside the 8-char slot
    wire [1:0]  re   = snap_order[slot*2 +: 2];
    wire [3:0]  st   = snap_status[re*4 +: 4];
    wire [2:0]  pl   = snap_place[re*3 +: 3];
    wire [23:0] cyc  = cyc_bcd[re*24 +: 24];
    wire [23:0] gap  = gap_bcd[re*24 +: 24];
    wire        r_sel  = snap_mask[re];
    wire        r_done = st[2];
    wire [2:0]  vc   = sc - 3'd3;                        // value column 0..4

    reg [7:0]  ch;
    reg [15:0] fg;
    always @* begin
        fg = r_sel ? eng_colour(re) : GREY;
        if (sc < 3)            ch = name_ch(re, sc);
        else if (!r_sel)       ch = (sc == 4) ? "O" : (sc == 5 || sc == 6) ? "F" : " ";
        else if (!r_done)      ch = (sc == 4) ? "R" : (sc == 5) ? "U" : (sc == 6) ? "N" : " ";
        else if (st[0])        ch = (sc == 4) ? "N" : (sc == 5) ? "O" : (sc == 6) ? "N" : (sc == 7) ? "E" : " ";
        else if (pl == 3'd1)   ch = time5(cyc, vc);
        else                   ch = gap5(gap, vc);
    end
    wire [4:0] tbits   = glyph(ch, gy);
    wire       text_on = (y < TOP_Y) && (gx < 5) &&
                         ((gx == 0) ? tbits[4] : (gx == 1) ? tbits[3] : (gx == 2) ? tbits[2] :
                          (gx == 3) ? tbits[1] : tbits[0]);

    // ------------------------------------------------------- 3x5 labels ----
    function [2:0] glyph3;
        input [7:0] c; input [2:0] r;
        reg [14:0] g;
        begin
            case (c)
                "0": g = {3'b111,3'b101,3'b101,3'b101,3'b111};
                "1": g = {3'b010,3'b110,3'b010,3'b010,3'b111};
                "5": g = {3'b111,3'b100,3'b111,3'b001,3'b111};
                "%": g = {3'b101,3'b001,3'b010,3'b100,3'b101};
                "T": g = {3'b111,3'b010,3'b010,3'b010,3'b010};
                "I": g = {3'b111,3'b010,3'b010,3'b010,3'b111};
                "M": g = {3'b101,3'b111,3'b111,3'b101,3'b101};
                "E": g = {3'b111,3'b100,3'b110,3'b100,3'b111};
                ">": g = {3'b100,3'b010,3'b001,3'b010,3'b100};
                default: g = 15'd0;
            endcase
            case (r)
                3'd0: glyph3 = g[14:12];  3'd1: glyph3 = g[11:9];
                3'd2: glyph3 = g[8:6];    3'd3: glyph3 = g[5:3];
                3'd4: glyph3 = g[2:0];    default: glyph3 = 3'd0;
            endcase
        end
    endfunction
    // Every label starts at x = 4 + 4k, so with lx = x - 4 the column inside a
    // glyph is lx[1:0] (3 = gap) and the character is lx[6:2].
    // "100%" at x 4..19, y 18..22 ; "50" at x 4..11, y 41..45 (under the 50 %
    // line) ; "TIME>" at x 76..95, y 57..61.  One small term per label.
    wire [6:0] lx  = x - 7'd4;                           // labels at x = 4 + 4k
    wire [5:0] y18 = y - 6'd18, y41 = y - 6'd41, y57 = y - 6'd57;
    wire in100 = (y >= 18) && (y < 23) && (x >= 4) && (x < 20);
    wire in50  = (y >= 41) && (y < 46) && (x >= 4) && (x < 12);
    wire intim = (y >= 57) && (y < 62) && (x >= 76);
    wire [2:0] b100 = glyph3((lx[6:2] == 5'd0) ? "1" : (lx[6:2] == 5'd3) ? "%" : "0", y18[2:0]);
    wire [2:0] b50  = glyph3((lx[6:2] == 5'd0) ? "5" : "0", y41[2:0]);
    wire [2:0] btim = glyph3((lx[6:2] == 5'd18) ? "T" : (lx[6:2] == 5'd19) ? "I" :
                             (lx[6:2] == 5'd20) ? "M" : (lx[6:2] == 5'd21) ? "E" : ">", y57[2:0]);
    wire [2:0] lbits = in100 ? b100 : in50 ? b50 : intim ? btim : 3'b000;
    wire label_on = (lx[1:0] == 2'd0) ? lbits[2] : (lx[1:0] == 2'd1) ? lbits[1] :
                    (lx[1:0] == 2'd2) ? lbits[0] : 1'b0;

    // ------------------------------------------------------------ output ---
    always @* begin
        if (y < TOP_Y)                          pixel_data = text_on ? fg : BLACK;
        else if (curve[`ARENA_ENG_AST])         pixel_data = eng_colour(`ARENA_ENG_AST);
        else if (curve[`ARENA_ENG_DIJ])         pixel_data = eng_colour(`ARENA_ENG_DIJ);
        else if (curve[`ARENA_ENG_BFS])         pixel_data = eng_colour(`ARENA_ENG_BFS);
        else if (curve[`ARENA_ENG_DFS])         pixel_data = eng_colour(`ARENA_ENG_DFS);
        else if (fin[`ARENA_ENG_AST])           pixel_data = eng_colour(`ARENA_ENG_AST);
        else if (fin[`ARENA_ENG_DIJ])           pixel_data = eng_colour(`ARENA_ENG_DIJ);
        else if (fin[`ARENA_ENG_BFS])           pixel_data = eng_colour(`ARENA_ENG_BFS);
        else if (fin[`ARENA_ENG_DFS])           pixel_data = eng_colour(`ARENA_ENG_DFS);
        else if (axis_on)                       pixel_data = AXIS;
        else if (label_on)                      pixel_data = LABEL;
        else if (grid_on)                       pixel_data = GRID;
        else                                    pixel_data = BLACK;
    end
endmodule
