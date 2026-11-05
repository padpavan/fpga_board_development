`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Member 1 - performance graph screen (CONTRACT 8a), 1920x1080 on clk_pix
//
// Plots each selected engine's samples (written by M2's ast_perf_sampler into
// four gv_hist_ram, one per engine) against time:
//
//   title    "PERFORMANCE: PROGRESS TO GOAL" / "... MAZE EXPLORED"
//   legend   # BFS  # DIJKSTRA  # A*  # DFS   (unselected engines grey)
//   y axis   0..100 %, labels and grid every 25 %
//   plot     1536 x 768 px at (192, 160): 3 px per sample (512 samples),
//            3 px per level (256 levels); vertical grid every 64 samples
//   curves   step lines in the VGA engine colours, A* on top; a white
//            marker where an engine finished
//   caption  cycles per sample, or NO DATA YET while hist_count = 0
//
// The sample and level under the beam come from counters stepped with the
// pixel counters (no divides).  Pipeline, the same latency as vga_render so
// both pictures share vga_timing:
//   S1  plot position, grid/axis flags, text position, sample valid / marker
//   S2  text character; the history RAMs' data arrive (2-cycle read)
//   S3  curve and marker tests, font lookup
//   out colour; RGB and syncs leave registered, LATENCY = 4 clk_pix cycles
//////////////////////////////////////////////////////////////////////////////
module gv_graph #(
    parameter H_TOTAL = 2200,
    parameter AW = `ARENA_GV_ADDR_W,
    parameter DW = `ARENA_GV_DATA_W,
    parameter PW = `ARENA_GV_PLOG_W
) (
    // 100 MHz: sample writes from the sampler
    input  wire              clk,
    input  wire              hist_we,
    input  wire [1:0]        hist_eng,
    input  wire [AW-1:0]     hist_addr,
    input  wire [DW-1:0]     hist_data,

    // clk_pix: beam position from vga_timing
    input  wire              clk_pix,
    input  wire              rst,
    input  wire [11:0]       x,
    input  wire [10:0]       y,
    input  wire              active_video,
    input  wire              hsync_in,
    input  wire              vsync_in,

    // frame-stable values (vga_cdc)
    input  wire [AW:0]       hist_count,
    input  wire [PW-1:0]     hist_plog,
    input  wire              hist_metric,
    input  wire [4*AW-1:0]   hist_end,
    input  wire [3:0]        hist_end_valid,
    input  wire [3:0]        run_mask,

    output reg  [3:0]        red,
    output reg  [3:0]        green,
    output reg  [3:0]        blue,
    output reg               hsync,
    output reg               vsync
);
    localparam NS  = 1 << AW;                 // samples per engine
    localparam NL  = 1 << DW;                 // levels
    localparam SPX = 3;                       // pixels per sample
    localparam LPX = 3;                       // pixels per level
    localparam X0  = 192, X1 = X0 + SPX*NS;   // plot x 192 .. 1727
    localparam Y0  = 160, Y1 = Y0 + LPX*NL;   // plot y 160 .. 927
    localparam LBL_X = 32;                    // y-axis labels: 4 chars from x = 32
    localparam TXT_C = 6;                     // title / legend / caption start at column 6 (x = 192)
    localparam SL  = 40;                      // characters per text line
    // y-axis label boxes, centred on the 100/75/50/25/0 % grid lines
    localparam G4 = Y0 + 1,                         G3 = Y0 + LPX*(NL-1-3*NL/4) + 1,
               G2 = Y0 + LPX*(NL-1-NL/2) + 1,       G1 = Y0 + LPX*(NL-1-NL/4) + 1,
               G0 = Y0 + LPX*(NL-1) + 1;

    localparam [11:0] C_BLACK=12'h000, C_WHITE=12'hFFF, C_GREY=12'h888, C_OFF=12'h666,
                      C_BG=12'h112, C_GRID=12'h333, C_AXIS=12'hAAA,
                      C_BFS=12'h0FF, C_DIJ=12'hFF0, C_AST=12'h0F0, C_DFS=12'hF6A;

    // ------------------------------------------------------------ RAMs -----
    reg  [AW:0]   sidx = 0;                   // sample under the beam
    reg  [1:0]    ssub = 0;                   // pixel inside the sample
    wire [DW-1:0] rd0, rd1, rd2, rd3;
    gv_hist_ram #(.AW(AW), .DW(DW)) u_bfs (.clk_w(clk), .we(hist_we && hist_eng == `ARENA_ENG_BFS),
        .waddr(hist_addr), .wdata(hist_data), .clk_r(clk_pix), .raddr(sidx[AW-1:0]), .rdata(rd0));
    gv_hist_ram #(.AW(AW), .DW(DW)) u_dij (.clk_w(clk), .we(hist_we && hist_eng == `ARENA_ENG_DIJ),
        .waddr(hist_addr), .wdata(hist_data), .clk_r(clk_pix), .raddr(sidx[AW-1:0]), .rdata(rd1));
    gv_hist_ram #(.AW(AW), .DW(DW)) u_ast (.clk_w(clk), .we(hist_we && hist_eng == `ARENA_ENG_AST),
        .waddr(hist_addr), .wdata(hist_data), .clk_r(clk_pix), .raddr(sidx[AW-1:0]), .rdata(rd2));
    gv_hist_ram #(.AW(AW), .DW(DW)) u_dfs (.clk_w(clk), .we(hist_we && hist_eng == `ARENA_ENG_DFS),
        .waddr(hist_addr), .wdata(hist_data), .clk_r(clk_pix), .raddr(sidx[AW-1:0]), .rdata(rd3));

    // -------------------------------------------------------- counters -----
    // sidx/ssub follow x and lvl/lsub follow y, so the sample and level under
    // the beam need no divide.  Values outside the plot are never used.
    reg  [DW:0]   lvl = 0;                    // level under the beam (counts down)
    reg  [1:0]    lsub = 0;                   // pixel row inside the level
    always @(posedge clk_pix) begin
        if (x == X0-1)               begin sidx <= 0; ssub <= 0; end
        else if (ssub == SPX-1)      begin sidx <= sidx + 1'b1; ssub <= 0; end
        else                               ssub <= ssub + 1'b1;
        if (x == H_TOTAL-1) begin
            if (y == Y0-1)           begin lvl <= NL-1; lsub <= 0; end
            else if (lsub == LPX-1)  begin lvl <= lvl - 1'b1; lsub <= 0; end
            else                           lsub <= lsub + 1'b1;
        end
    end

    // ------------------------------------------------------------ text -----
    function [4:0] font;                      // 5x7, bit 4 = leftmost column
        input [7:0] c; input [2:0] r;
        reg [34:0] g;
        begin
            case (c)
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
                "R": g = {5'd30,5'd17,5'd17,5'd30,5'd20,5'd18,5'd17};
                "S": g = {5'd15,5'd16,5'd16,5'd14,5'd1, 5'd1, 5'd30};
                "T": g = {5'd31,5'd4, 5'd4, 5'd4, 5'd4, 5'd4, 5'd4};
                "U": g = {5'd17,5'd17,5'd17,5'd17,5'd17,5'd17,5'd14};
                "W": g = {5'd17,5'd17,5'd17,5'd21,5'd21,5'd21,5'd10};
                "X": g = {5'd17,5'd17,5'd10,5'd4, 5'd10,5'd17,5'd17};
                "Y": g = {5'd17,5'd17,5'd10,5'd4, 5'd4, 5'd4, 5'd4};
                "Z": g = {5'd31,5'd1, 5'd2, 5'd4, 5'd8, 5'd16,5'd31};
                "0": g = {5'd14,5'd17,5'd19,5'd21,5'd25,5'd17,5'd14};
                "1": g = {5'd4, 5'd12,5'd4, 5'd4, 5'd4, 5'd4, 5'd14};
                "2": g = {5'd14,5'd17,5'd1, 5'd2, 5'd4, 5'd8, 5'd31};
                "3": g = {5'd30,5'd1, 5'd1, 5'd14,5'd1, 5'd1, 5'd30};
                "4": g = {5'd2, 5'd6, 5'd10,5'd18,5'd31,5'd2, 5'd2};
                "5": g = {5'd31,5'd16,5'd30,5'd1, 5'd1, 5'd17,5'd14};
                "6": g = {5'd6, 5'd8, 5'd16,5'd30,5'd17,5'd17,5'd14};
                "7": g = {5'd31,5'd1, 5'd2, 5'd4, 5'd8, 5'd8, 5'd8};
                "8": g = {5'd14,5'd17,5'd17,5'd14,5'd17,5'd17,5'd14};
                "9": g = {5'd14,5'd17,5'd17,5'd15,5'd1, 5'd1, 5'd14};
                ":": g = {5'd0, 5'd4, 5'd4, 5'd0, 5'd4, 5'd4, 5'd0};
                "%": g = {5'd24,5'd25,5'd2, 5'd4, 5'd8, 5'd19,5'd3};
                "-": g = {5'd0, 5'd0, 5'd0, 5'd31,5'd0, 5'd0, 5'd0};
                "+": g = {5'd0, 5'd4, 5'd4, 5'd31,5'd4, 5'd4, 5'd0};
                "*": g = {5'd0, 5'd21,5'd14,5'd31,5'd14,5'd21,5'd0};
                "(": g = {5'd2, 5'd4, 5'd8, 5'd8, 5'd8, 5'd4, 5'd2};
                ")": g = {5'd8, 5'd4, 5'd2, 5'd2, 5'd2, 5'd4, 5'd8};
                default: g = 35'd0;
            endcase
            font = g[(6-r)*5 +: 5];
        end
    endfunction

    // cycles per sample as 7 left-aligned decimal characters
    function [55:0] dec7;
        input [PW-1:0] p;
        case (p)
            5'd0:  dec7 = {"1",       {6{8'h20}}};  5'd1:  dec7 = {"2",       {6{8'h20}}};
            5'd2:  dec7 = {"4",       {6{8'h20}}};  5'd3:  dec7 = {"8",       {6{8'h20}}};
            5'd4:  dec7 = {"16",      {5{8'h20}}};  5'd5:  dec7 = {"32",      {5{8'h20}}};
            5'd6:  dec7 = {"64",      {5{8'h20}}};  5'd7:  dec7 = {"128",     {4{8'h20}}};
            5'd8:  dec7 = {"256",     {4{8'h20}}};  5'd9:  dec7 = {"512",     {4{8'h20}}};
            5'd10: dec7 = {"1024",    {3{8'h20}}};  5'd11: dec7 = {"2048",    {3{8'h20}}};
            5'd12: dec7 = {"4096",    {3{8'h20}}};  5'd13: dec7 = {"8192",    {3{8'h20}}};
            5'd14: dec7 = {"16384",   {2{8'h20}}};  5'd15: dec7 = {"32768",   {2{8'h20}}};
            5'd16: dec7 = {"65536",   {2{8'h20}}};  5'd17: dec7 = {"131072",  8'h20};
            5'd18: dec7 = {"262144",  8'h20};       5'd19: dec7 = {"524288",  8'h20};
            5'd20: dec7 = "1048576";
            default: dec7 = {"-",     {6{8'h20}}};
        endcase
    endfunction

    localparam [8*SL-1:0]
        T_TITLE0 = {"PERFORMANCE: PROGRESS TO GOAL",   {11{8'h20}}},
        T_TITLE1 = {"PERFORMANCE: MAZE EXPLORED",      {14{8'h20}}},
        T_LEGEND =  "# BFS     # DIJKSTRA     # A*      # DFS",
        T_NODATA = {"NO DATA YET - START A RUN (BTNC)", {8{8'h20}}},
        T_HELP   = {"SW9+SW10 GRAPH   SW11 METRIC",    {12{8'h20}}},
        T_NONE   = {SL{8'h20}};
    wire [8*SL-1:0] t_caption = {"TIME AXIS - CYCLES PER SAMPLE: ", dec7(hist_plog), {2{8'h20}}};
    localparam [31:0] L100 = "100%", L75 = " 75%", L50 = " 50%", L25 = " 25%", L0 = "  0%";

    // ------------------------------------------------------------ S1 -------
    wire in_plot = (x >= X0) && (x < X1) && (y >= Y0) && (y < Y1);
    wire [DW-1:0] lv = lvl[DW-1:0];
    wire grid_h  = (lsub == 2'd1) && ((lv[DW-3:0] == 0) || (lv == NL-1));   // 0/25/50/75/100 %
    wire grid_v  = (ssub == 2'd0) && (sidx[5:0] == 6'd0);                   // every 64 samples
    wire axis    = (((x == X0-1) || (x == X0-2)) && (y >= Y0) && (y < Y1+2)) ||
                   (((y == Y1) || (y == Y1+1)) && (x >= X0-2) && (x < X1));

    function near;                            // sample s within one of the finish sample e
        input [AW:0] s; input [AW-1:0] e; input v;
        reg [AW:0] ee;
        begin
            ee   = {1'b0, e};
            near = v && ((s == ee) || (s == ee + 1'b1) || (s + 1'b1 == ee));
        end
    endfunction

    wire [5:0] t_line = y[10:5];
    wire [6:0] t_col  = x[11:5];
    wire       t_main = (t_col >= TXT_C) && (t_col < TXT_C + SL);
    wire       lbl_x  = (x >= LBL_X) && (x < LBL_X + 4*32);
    wire [2:0] lbl    = !lbl_x ? 3'd0 :
                        (y >= G4-14 && y < G4+14) ? 3'd5 : (y >= G3-14 && y < G3+14) ? 3'd4 :
                        (y >= G2-14 && y < G2+14) ? 3'd3 : (y >= G1-14 && y < G1+14) ? 3'd2 :
                        (y >= G0-14 && y < G0+14) ? 3'd1 : 3'd0;
    wire [10:0] lbl_top = (lbl == 3'd5) ? G4-14 : (lbl == 3'd4) ? G3-14 : (lbl == 3'd3) ? G2-14 :
                          (lbl == 3'd2) ? G1-14 : G0-14;
    wire [10:0] lbl_ry  = y - lbl_top;

    reg              s1_active, s1_hs, s1_vs, s1_plot, s1_grid, s1_axis, s1_tmain;
    reg [1:0]        s1_ssub;
    reg [DW-1:0]     s1_lv;
    reg              s1_first;
    reg [3:0]        s1_valid, s1_near;
    reg [5:0]        s1_tline, s1_tc;
    reg [2:0]        s1_lbl, s1_gcol, s1_grow;
    reg [1:0]        s1_lci;
    always @(posedge clk_pix) begin
        s1_active <= active_video; s1_hs <= hsync_in; s1_vs <= vsync_in;
        s1_plot   <= in_plot;
        s1_grid   <= in_plot && (grid_h || grid_v);
        s1_axis   <= axis;
        s1_ssub   <= ssub;
        s1_lv     <= lv;
        s1_first  <= (sidx == 0);
        s1_valid  <= run_mask & {4{sidx < hist_count}};
        s1_near   <= {near(sidx, hist_end[3*AW +: AW], hist_end_valid[3]),
                      near(sidx, hist_end[2*AW +: AW], hist_end_valid[2]),
                      near(sidx, hist_end[1*AW +: AW], hist_end_valid[1]),
                      near(sidx, hist_end[0*AW +: AW], hist_end_valid[0])};
        s1_tmain  <= t_main;
        s1_tline  <= t_line;
        s1_tc     <= t_col - TXT_C;
        s1_lbl    <= lbl;
        s1_lci    <= t_col - LBL_X/32;
        s1_gcol   <= x[4:2];
        s1_grow   <= (lbl != 3'd0) ? lbl_ry[4:2] : y[4:2];
    end

    // ------------------------------------------------------------ S2 -------
    // text: which character, and in which colour (0 white, 1 grey, 2..5 engine 0..3)
    reg [8*SL-1:0] t_str;
    reg [31:0]     t_lbl;
    reg [2:0]      t_kind;
    always @* begin
        t_str = T_NONE; t_kind = 3'd0;
        case (s1_tline)
            6'd1:  t_str = hist_metric ? T_TITLE1 : T_TITLE0;
            6'd3:  begin
                       t_str  = T_LEGEND;
                       t_kind = (s1_tc < 10) ? 3'd2 : (s1_tc < 25) ? 3'd3 : (s1_tc < 35) ? 3'd4 : 3'd5;
                   end
            6'd30: t_str = (hist_count == 0) ? T_NODATA : t_caption;
            6'd32: begin t_str = T_HELP; t_kind = 3'd1; end
            default: t_str = T_NONE;
        endcase
        case (s1_lbl)
            3'd5: t_lbl = L100; 3'd4: t_lbl = L75; 3'd3: t_lbl = L50;
            3'd2: t_lbl = L25;  default: t_lbl = L0;
        endcase
    end
    wire [7:0] t_ch = (s1_lbl != 3'd0) ? t_lbl[(3-s1_lci)*8 +: 8] :
                      s1_tmain ? t_str[(SL-1-s1_tc)*8 +: 8] : 8'h20;

    reg              s2_active, s2_hs, s2_vs, s2_plot, s2_grid, s2_axis, s2_first;
    reg [1:0]        s2_ssub;
    reg [DW-1:0]     s2_lv;
    reg [3:0]        s2_valid, s2_near;
    // kept as a plain register: otherwise synthesis turns the font into a
    // block-RAM ROM with s2_ch as its address register, and the string select
    // has to meet the BRAM address setup (-0.22 ns at 148.5 MHz)
    (* keep = "true" *) reg [7:0] s2_ch;
    reg [2:0]        s2_kind, s2_gcol, s2_grow;
    always @(posedge clk_pix) begin
        s2_active <= s1_active; s2_hs <= s1_hs; s2_vs <= s1_vs;
        s2_plot <= s1_plot; s2_grid <= s1_grid; s2_axis <= s1_axis; s2_first <= s1_first;
        s2_ssub <= s1_ssub; s2_lv <= s1_lv; s2_valid <= s1_valid; s2_near <= s1_near;
        s2_ch   <= t_ch;
        s2_kind <= (s1_lbl != 3'd0) ? 3'd0 : t_kind;
        s2_gcol <= s1_gcol; s2_grow <= s1_grow;
    end

    // ------------------------------------------------------------ S3 -------
    // The history RAMs' data for this pixel arrive now (2-cycle read of sidx).
    // A curve is a step line: a horizontal run at the sample's level, joined
    // to the previous sample's level by a 2 px vertical in the sample's first
    // two pixel columns.
    reg [DW-1:0] prev0, prev1, prev2, prev3;  // previous sample of each engine
    function step_on;
        input [DW-1:0] v; input [DW-1:0] p; input [DW-1:0] l; input [1:0] sub;
        reg [DW-1:0] lo, hi;
        begin
            lo = (p < v) ? p : v;
            hi = (p < v) ? v : p;
            step_on = (l == v) || ((sub != 2'd2) && (l >= lo) && (l <= hi));
        end
    endfunction
    function mark_on;                         // within one level of the sample
        input [DW-1:0] v; input [DW-1:0] l;
        mark_on = ({1'b0, l} + 1'b1 >= {1'b0, v}) && ({1'b0, l} <= {1'b0, v} + 1'b1);
    endfunction
    wire [DW-1:0] p0 = s2_first ? rd0 : prev0, p1 = s2_first ? rd1 : prev1,
                  p2 = s2_first ? rd2 : prev2, p3 = s2_first ? rd3 : prev3;
    wire [3:0] on = s2_valid & {step_on(rd3, p3, s2_lv, s2_ssub), step_on(rd2, p2, s2_lv, s2_ssub),
                                step_on(rd1, p1, s2_lv, s2_ssub), step_on(rd0, p0, s2_lv, s2_ssub)};
    wire [3:0] mk = s2_near & run_mask & {mark_on(rd3, s2_lv), mark_on(rd2, s2_lv),
                                         mark_on(rd1, s2_lv), mark_on(rd0, s2_lv)};
    wire [4:0] glyph = font(s2_ch, s2_grow);

    (* rom_style = "distributed" *) reg [4:0] s3_glyph;
    reg              s3_active, s3_hs, s3_vs, s3_plot, s3_grid, s3_axis, s3_cell, s3_sw, s3_mark;
    reg [3:0]        s3_on;
    reg [2:0]        s3_kind, s3_gcol;
    always @(posedge clk_pix) begin
        if (s2_ssub == SPX-1) begin prev0 <= rd0; prev1 <= rd1; prev2 <= rd2; prev3 <= rd3; end
        s3_active <= s2_active; s3_hs <= s2_hs; s3_vs <= s2_vs;
        s3_plot <= s2_plot; s3_grid <= s2_grid; s3_axis <= s2_axis;
        s3_on   <= on & {4{s2_plot}};
        s3_mark <= s2_plot && (|mk);
        s3_glyph <= glyph;
        s3_gcol  <= s2_gcol;
        s3_cell  <= (s2_ch != "#") && (s2_gcol < 5) && (s2_grow < 7);
        s3_sw    <= (s2_ch == "#") && (s2_gcol < 5) && (s2_grow < 7);
        s3_kind  <= s2_kind;
    end

    // ------------------------------------------------------------ out ------
    wire        s3_text = s3_cell && s3_glyph[4 - s3_gcol];
    wire [1:0]  k_eng   = s3_kind[1:0] - 2'd2;          // kind 2..5 -> engine 0..3
    wire [11:0] c_eng   = !run_mask[k_eng] ? C_OFF :
                          (k_eng == 2'd0) ? C_BFS : (k_eng == 2'd1) ? C_DIJ :
                          (k_eng == 2'd2) ? C_AST : C_DFS;
    wire [11:0] c_text  = (s3_kind == 3'd0) ? C_WHITE : (s3_kind == 3'd1) ? C_GREY : c_eng;
    reg  [11:0] c;
    always @* begin
        if (!s3_active)        c = C_BLACK;
        else if (s3_text)      c = c_text;
        else if (s3_sw)        c = c_eng;
        else if (s3_mark)      c = C_WHITE;
        else if (s3_on[2])     c = C_AST;
        else if (s3_on[1])     c = C_DIJ;
        else if (s3_on[0])     c = C_BFS;
        else if (s3_on[3])     c = C_DFS;
        else if (s3_axis)      c = C_AXIS;
        else if (s3_grid)      c = C_GRID;
        else if (s3_plot)      c = C_BG;
        else                   c = C_BLACK;
    end
    always @(posedge clk_pix) begin
        {red, green, blue} <= c;
        hsync <= s3_hs;
        vsync <= s3_vs;
    end
endmodule
