`timescale 1ns / 1ps
`include "arena_defs.vh"
// 1920x1080 renderer on clk_pix (148.5 MHz).
//
// EDIT: one grid of square cells, full height, centred; help text in the left margin.
// RUN : 2x2 quadrants (BFS, Dijkstra, A*, DFS).  Each quadrant has its own
//       zoom/pan view (run_k, run_ox, run_oy from vga_zoom, one per quadrant);
//       the quadrant selected for zooming (left click) gets a yellow border.  A quadrant whose
//       engine is not in run_mask shows only the maze, with a grey header.
//       The legend is a half-size text strip in the DFS quadrant's right
//       margin (outside the grid at 1x; drawn over it when zoomed).
//
// Each pixel's grid position comes from a DDA instead of a divide: wx += k per
// pixel, wy += k per line, where k is cells per pixel in FRAC-bit fixed point.
// The integer part is the cell, the fraction is the position inside it.
//
// Themes (pacman input): classic colour fills, or Pac-Man (blue maze walls,
// pellets on the path, Pac-Man at each engine's current cell, ghost at goal).
//
// GAME (game input, CONTRACT v1.7): the EDIT layout in the Pac-Man look with
// the game's pellets, Pac-Man (mouth facing player_dir) and up to 4 ghosts
// (red, pink, cyan, orange).  Left margin: LEVEL / ALGO / LIVES / SCORE / PELLETS;
// right margin: the game state message.  No hover, crosshair or overlay.
//
// Pipeline: S1 position, addresses, text line/column; S2 RAM reads, squares,
// text character; S3 cell colour, font lookup; S4 text over cell.  RGB and
// syncs leave registered, LATENCY = 4 clk_pix cycles.
module vga_render #(
    parameter COLS=`ARENA_COLS, parameter ROWS=`ARENA_ROWS,
    parameter INDEX_W=`ARENA_INDEX_W, parameter COST_W=`ARENA_COST_W,
    parameter H_VISIBLE=`ARENA_VGA_H, parameter V_VISIBLE=`ARENA_VGA_V,
    parameter H_TOTAL=2200, parameter V_TOTAL=1125,
    parameter FRAC=20, parameter OW=28, parameter KW=16
) (
    input  wire clk, input wire rst,
    // from vga_timing
    input  wire [11:0] x, input wire [10:0] y, input wire active_video,
    input  wire hsync_in, input wire vsync_in,
    // view (from vga_cdc, changes only between frames)
    input  wire edit_mode, input wire pacman,
    // per quadrant q (0 BFS, 1 DIJ, 2 A*, 3 DFS) at [q*W +: W]; ox / oy are signed
    input  wire [4*OW-1:0] run_ox, input wire [4*OW-1:0] run_oy,
    input  wire [4*KW-1:0] run_k, input wire [4*3-1:0] zoom_level,
    input  wire [1:0] zoom_sel, input wire zoom_sel_valid,
    input  wire [11:0] mouse_x, input wire [11:0] mouse_y,
    input  wire cursor_valid, input wire [INDEX_W-1:0] cursor_idx,
    input  wire [1:0] edit_tool,
    input  wire [INDEX_W-1:0] start_idx, input wire [INDEX_W-1:0] goal_idx,
    input  wire [INDEX_W-1:0] bfs_current, input wire [INDEX_W-1:0] dij_current,
    input  wire [INDEX_W-1:0] ast_current, input wire [INDEX_W-1:0] dfs_current,
    input  wire [3:0] run_mask,           // {dfs, ast, dij, bfs}
    // maze_ram VGA ports (async): this cell and its four neighbours
    output reg  [INDEX_W-1:0] cell_addr, output reg [INDEX_W-1:0] up_addr,
    output reg  [INDEX_W-1:0] dn_addr,   output reg [INDEX_W-1:0] lf_addr,
    output reg  [INDEX_W-1:0] rt_addr,
    input  wire [COST_W-1:0] cell_cost, input wire [COST_W-1:0] up_cost,
    input  wire [COST_W-1:0] dn_cost,   input wire [COST_W-1:0] lf_cost,
    input  wire [COST_W-1:0] rt_cost,
    // engine cell_state at cell_addr (async)
    input  wire [1:0] bfs_state, input wire [1:0] dij_state, input wire [1:0] ast_state,
    input  wire [1:0] dfs_state,
    // GAME view (from vga_cdc)
    input  wire game, input wire [3:0] game_state,
    input  wire [INDEX_W-1:0] player_pos, input wire [1:0] player_dir,
    input  wire [4*INDEX_W-1:0] ghost_pos, input wire [3:0] ghost_present,
    input  wire [COLS*ROWS-1:0] pellet,
    input  wire [1:0] game_level, input wire [1:0] game_algo, input wire [1:0] game_lives,
    input  wire [19:0] game_score, input wire [11:0] game_pellets,   // BCD
    output reg  [3:0] red, output reg [3:0] green, output reg [3:0] blue,
    output reg  hsync, output reg vsync
);
    localparam [INDEX_W-1:0] NONE = `ARENA_IDX_NONE;
    localparam QUAD_W = H_VISIBLE/2, QUAD_H = V_VISIBLE/2;
    localparam EDIT_CELL_X = H_VISIBLE/COLS, EDIT_CELL_Y = V_VISIBLE/ROWS;
    localparam EDIT_CELL = (EDIT_CELL_X < EDIT_CELL_Y) ? EDIT_CELL_X : EDIT_CELL_Y;   // 90
    localparam EDIT_X0 = (H_VISIBLE - COLS*EDIT_CELL)/2;                             // 240
    localparam EDIT_Y0 = (V_VISIBLE - ROWS*EDIT_CELL)/2;                             // 0
    localparam [KW-1:0] EDIT_K = ((1 << FRAC) + EDIT_CELL/2)/EDIT_CELL;
    localparam signed [OW-1:0] EDIT_OX = -(EDIT_X0*EDIT_K);
    localparam signed [OW-1:0] EDIT_OY = -(EDIT_Y0*EDIT_K);
    localparam CW = OW - FRAC;            // width of the signed cell number
    // legend: 16x16 px characters (glyph x2), 7 columns, right edge of the DFS quadrant
    localparam LEG_COLS = 7;
    localparam LEG_X0   = QUAD_W - 16*LEG_COLS;                                  // 848
    localparam NUM_CELLS = COLS*ROWS;
    localparam GRID_X1  = EDIT_X0 + COLS*EDIT_CELL;                              // 1680
    // game message column starts past the maze border (9 px wide, BAND), so the
    // text never paints over it; 7 characters still fit before x = 1920
    localparam MSG_X0   = GRID_X1 + 16;                                          // 1696
    // game_ctrl states (game_ctrl.v)
    localparam [3:0] G_INIT = 4'd1, G_READY = 4'd2, G_PLAY = 4'd3, G_PAUSE = 4'd4, G_CAUGHT = 4'd5,
                     G_CLEAR = 4'd6, G_OVER = 4'd7, G_WIN = 4'd8, G_NOPATH = 4'd9, G_WAITENG = 4'd10;

    // Shapes, in 1/128ths of a cell (fx, fy below)
    localparam [6:0] BAND = 13;           // Pac-Man wall / border thickness, ~0.1 cell
    localparam R_PATH2 = 18*18, R_FRONT2 = 9*9, R_PAC2 = 54*54, R_GHOST2 = 51*51, R_EYE2 = 12*12;

    // Colours (RGB444)
    localparam [11:0] C_BLACK=12'h000, C_WHITE=12'hFFF, C_GRIDLINE=12'h112, C_SEP=12'h444,
                      C_LEGEND_BG=12'h111, C_START=12'hF0F, C_GOAL=12'hF00, C_HOVER=12'hFF0,
                      C_WALL=12'h556, C_VISITED=12'h25C, C_FRONTIER=12'hFC0, C_WEIGHT=12'h951,
                      C_BFS=12'h0FF, C_DIJ=12'hFF0, C_AST=12'h0F0, C_DFS=12'hF6A, C_OFF=12'h666,
                      P_WALL=12'h24F, P_FLOOR=12'h000, P_WALL_IN=12'h001, P_PELLET=12'hFCA,
                      P_FRONT=12'h666, P_VISITED=12'h003, P_WEIGHT=12'h404, P_PAC=12'hFF0,
                      P_GHOST=12'hF00, P_HOVER=12'h440, P_SWATCH_VIS=12'h006,
                      P_GHOST1=12'hF8C, P_GHOST2=12'h0FF, P_GHOST3=12'hFA0;

    // ------------------------------------------------------------ DDA ------
    // Each quadrant has its own view; EDIT / GAME use the fixed EDIT view everywhere.
    wire signed [OW-1:0] ox0 = edit_mode ? EDIT_OX : run_ox[0*OW +: OW];
    wire signed [OW-1:0] ox1 = edit_mode ? EDIT_OX : run_ox[1*OW +: OW];
    wire signed [OW-1:0] ox2 = edit_mode ? EDIT_OX : run_ox[2*OW +: OW];
    wire signed [OW-1:0] ox3 = edit_mode ? EDIT_OX : run_ox[3*OW +: OW];
    wire signed [OW-1:0] oy0 = edit_mode ? EDIT_OY : run_oy[0*OW +: OW];
    wire signed [OW-1:0] oy1 = edit_mode ? EDIT_OY : run_oy[1*OW +: OW];
    wire signed [OW-1:0] oy2 = edit_mode ? EDIT_OY : run_oy[2*OW +: OW];
    wire signed [OW-1:0] oy3 = edit_mode ? EDIT_OY : run_oy[3*OW +: OW];
    wire [KW-1:0]        k0  = edit_mode ? EDIT_K  : run_k[0*KW +: KW];
    wire [KW-1:0]        k1  = edit_mode ? EDIT_K  : run_k[1*KW +: KW];
    wire [KW-1:0]        k2  = edit_mode ? EDIT_K  : run_k[2*KW +: KW];
    wire [KW-1:0]        k3  = edit_mode ? EDIT_K  : run_k[3*KW +: KW];
    wire line_last   = (x == H_TOTAL-1);
    wire panel_x_end = !edit_mode && (x == QUAD_W-1);
    wire panel_y_end = !edit_mode && (y == QUAD_H-1);
    // The view values are registered ahead of use (y only changes at a line end),
    // so the DDA's load / step paths are as short as with a single view:
    //   ox_l : left quadrant of the NEXT line (loaded into wx at the line end)
    //   ox_r : right quadrant of this line   (loaded into wx at x = QUAD_W-1)
    //   kl/kr: k of this row's left / right quadrant; kcur: k of the next pixel's quadrant
    reg row_b = 1'b0, nrow_b = 1'b0;
    reg signed [OW-1:0] ox_l = 0, ox_r = 0;
    reg [KW-1:0] kl = 0, kr = 0, kcur = 0;
    always @(posedge clk) begin
        row_b  <= (y >= QUAD_H);
        nrow_b <= (y >= QUAD_H-1) && (y != V_TOTAL-1);
        ox_l   <= nrow_b ? ox2 : ox0;
        ox_r   <= row_b  ? ox3 : ox1;
        kl     <= row_b  ? k2  : k0;
        kr     <= row_b  ? k3  : k1;
        kcur   <= ((x >= QUAD_W-1) && (x != H_TOTAL-1)) ? kr : kl;
    end
    wire signed [OW-1:0] kx = {{(OW-KW){1'b0}}, kcur};
    reg  signed [OW-1:0] wx = 0, wy = 0;
    // one row position per quadrant column of the current quadrant row; wy is the
    // one for the pixel's quadrant (swapped in at the quadrant edges like wx)
    reg  signed [OW-1:0] wy_l = 0, wy_r = 0;
    wire signed [OW-1:0] wy_l_step = wy_l + {{(OW-KW){1'b0}}, kl};
    wire signed [OW-1:0] wy_r_step = wy_r + {{(OW-KW){1'b0}}, kr};
    wire signed [OW-1:0] wx_next = wx + kx;
    wire signed [OW-1:0] wy_next = wy + kx;
    always @(posedge clk) begin
        if (line_last)        wx <= ox_l;
        else if (panel_x_end) wx <= ox_r;
        else                  wx <= wx_next;
        if (line_last) begin
            if (y == V_TOTAL-1)    begin wy_l <= oy0; wy_r <= oy1; wy <= oy0; end
            else if (panel_y_end)  begin wy_l <= oy2; wy_r <= oy3; wy <= oy2; end
            else                   begin wy_l <= wy_l_step; wy_r <= wy_r_step; wy <= wy_l_step; end
        end else if (panel_x_end)
            wy <= wy_r;
    end

    reg [4:0] frame_cnt = 0;              // Pac-Man chomp animation
    always @(posedge clk)
        if (line_last && (y == V_TOTAL-1)) frame_cnt <= frame_cnt + 1'b1;

    // ------------------------------------------------------------ text -----
    function [4:0] font;                  // 5x7, bit 4 = leftmost column
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
                "V": g = {5'd17,5'd17,5'd17,5'd17,5'd17,5'd10,5'd4};
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
                "9": g = {5'd14,5'd17,5'd17,5'd15,5'd1, 5'd2, 5'd12};
                "*": g = {5'd0, 5'd21,5'd14,5'd31,5'd14,5'd21,5'd0};
                "-": g = {5'd0, 5'd0, 5'd0, 5'd31,5'd0, 5'd0, 5'd0};
                default: g = 35'd0;
            endcase
            font = g[(6-r)*5 +: 5];
        end
    endfunction

    // 24-character strings; text is drawn on a 32x32 px character grid
    // (5x7 glyph scaled x4).  '#' draws a colour swatch.
    localparam SL = 24;
    localparam [8*SL-1:0]
        T_BFS      = {"BFS",                 {21{8'h20}}},
        T_DIJ      = {"DIJKSTRA",            {16{8'h20}}},
        T_AST      = {"A*",                  {22{8'h20}}},
        T_DFS      = {"DFS",                 {21{8'h20}}},
        T_LEGEND   = {"LEGEND",              {18{8'h20}}},
        T_WALL     = {"# WALL",              {18{8'h20}}},
        T_VISITED  = {"# VISIT",             {17{8'h20}}},
        T_FRONTIER = {"# FRONT",             {17{8'h20}}},
        T_PATH     = {"# PATH",              {18{8'h20}}},
        T_WEIGHT   = {"# WGHT",              {18{8'h20}}},
        T_ZOOM     = {"ZOOM X",              {18{8'h20}}},
        T_CLICK    = {"CLICK +",             {17{8'h20}}},
        T_WHEEL    = {"WHEEL",               {19{8'h20}}},
        T_MIDDLE   = {"MIDDLE",              {18{8'h20}}},
        T_RESET    = {"RESETS",              {18{8'h20}}},
        T_EDIT     = {"EDIT",                {20{8'h20}}},
        T_TWALL    = {"WALL",                {20{8'h20}}},
        T_TSTART   = {"START",               {19{8'h20}}},
        T_TGOAL    = {"GOAL",                {20{8'h20}}},
        T_TWEIGHT  = {"WEIGHT",              {18{8'h20}}},
        T_SW6      = {"SW6",                 {21{8'h20}}},
        T_SW7      = {"SW7",                 {21{8'h20}}},
        T_GAME     = {"GAME",                {20{8'h20}}},
        T_LEVEL    = {"LEVEL",               {19{8'h20}}},
        T_ALGO     = {"ALGO",                {20{8'h20}}},
        T_LIVES    = {"LIVES",               {19{8'h20}}},
        T_SCORE    = {"SCORE",               {19{8'h20}}},
        T_PELLETS  = {"PELLETS",             {17{8'h20}}},
        T_SW6OFF   = {"SW6 OFF",             {17{8'h20}}},
        T_EXITS    = {"EXITS",               {19{8'h20}}},
        T_READY    = {"READY",               {19{8'h20}}},
        T_PRESS    = {"PRESS",               {19{8'h20}}},
        T_BTNC     = {"BTNC",                {20{8'h20}}},
        T_PAUSED   = {"PAUSED",              {18{8'h20}}},
        T_CAUGHT   = {"CAUGHT",              {18{8'h20}}},
        T_CLEAR    = {"CLEAR",               {19{8'h20}}},
        T_OVER     = {"OVER",                {20{8'h20}}},
        T_YOU      = {"YOU",                 {21{8'h20}}},
        T_WIN      = {"WIN",                 {21{8'h20}}},
        T_NO       = {"NO",                  {22{8'h20}}},
        T_PATHW    = {"PATH",                {20{8'h20}}},
        T_FIX      = {"FIX",                 {21{8'h20}}},
        T_MAZE     = {"MAZE",                {20{8'h20}}},
        T_LOADING  = {"LOADING",             {17{8'h20}}},
        T_SBFS     = {"BFS",                 {21{8'h20}}},
        T_SDIJ     = {"DIJ",                 {21{8'h20}}},
        T_PAC      = {"PAC-MAN",             {17{8'h20}}},
        T_CLS      = {"CLASSIC",             {17{8'h20}}},
        T_NONE     = {SL{8'h20}};

    // ------------------------------------------------------------ S1 -------
    wire signed [CW-1:0] col_i = wx[OW-1:FRAC];     // floor(wx)
    wire signed [CW-1:0] row_i = wy[OW-1:FRAC];
    wire [6:0] fx = wx[FRAC-1 -: 7];
    wire [6:0] fy = wy[FRAC-1 -: 7];
    wire col_ok = (col_i >= 0) && (col_i < COLS);
    wire row_ok = (row_i >= 0) && (row_i < ROWS);
    wire in_grid = col_ok && row_ok;
    wire [INDEX_W-1:0] idx = row_i[INDEX_W-1:0]*COLS + col_i[INDEX_W-1:0];
    wire ext_x = col_ok || ((col_i == -1) && (fx >= 128-BAND)) || ((col_i == COLS) && (fx < BAND));
    wire ext_y = row_ok || ((row_i == -1) && (fy >= 128-BAND)) || ((row_i == ROWS) && (fy < BAND));
    wire gridline = (wx_next[OW-1:FRAC] != wx[OW-1:FRAC]) || (wy_next[OW-1:FRAC] != wy[OW-1:FRAC]);

    // x-only region compares, registered from x+1 so they line up with x
    // (keeps the compare chain off the S1 text-select path)
    wire [11:0] xn = (x == H_TOTAL-1) ? 12'd0 : x + 1'b1;
    reg xr_right = 1'b0, xr_ledit = 1'b1, xr_rm = 1'b0, xr_leg = 1'b0;
    always @(posedge clk) begin
        xr_right <= (xn >= QUAD_W);
        xr_ledit <= (xn <  EDIT_X0);
        xr_rm    <= (xn >= MSG_X0);
        xr_leg   <= (xn >= QUAD_W + LEG_X0);
    end
    wire right  = xr_right;
    wire bottom = (y >= QUAD_H);
    // per-line flags of the current line (latched at line end with the line text below)
    reg  c_bot = 1'b0, c_hdr = 1'b0;     // RUN: bottom half / quadrant header line
    wire [1:0]  panel = edit_mode ? 2'd0 : {bottom, right};   // 0 BFS, 1 DIJ, 2 A*, 3 DFS
    wire [11:0] lx = (!edit_mode && right)  ? x - QUAD_W : x;
    wire [10:0] ly = (!edit_mode && bottom) ? y - QUAD_H : y;
    wire        legend = !edit_mode && c_bot && xr_leg;         // DFS quadrant, x >= 960+848
    wire [11:0] lgx = lx - LEG_X0;
    wire        rmargin = game && xr_rm;                    // game message column
    wire [11:0] rx = x - MSG_X0;
    // 32 px text grid (glyph x4), or 16 px in the legend (glyph x2)
    wire [5:0]  t_line = legend ? ly[9:4]   : ly[10:5];
    wire [6:0]  t_char = legend ? lgx[10:4] : rmargin ? rx[11:5] : lx[11:5];
    wire [2:0]  g_col  = legend ? lgx[3:1]  : rmargin ? rx[4:2]  : lx[4:2];
    wire [2:0]  g_row  = legend ? ly[3:1]   : ly[4:2];
    wire [INDEX_W-1:0] view_current = (panel == 2'd0) ? bfs_current :
                                      (panel == 2'd1) ? dij_current :
                                      (panel == 2'd2) ? ast_current : dfs_current;
    wire view_on = run_mask[panel];
    wire sep = !edit_mode && ((x == QUAD_W-1) || (x == QUAD_W) || (y == QUAD_H-1) || (y == QUAD_H));
    // 4 px yellow border inside the quadrant selected for zooming (RUN view only)
    wire selb = !edit_mode && !game && zoom_sel_valid && (panel == zoom_sel) &&
                ((lx < 4) || (lx >= QUAD_W-4) || (ly < 4) || (ly >= QUAD_H-4));

    wire [11:0] dxm = (x > mouse_x) ? x - mouse_x : mouse_x - x;
    wire [11:0] dym = ({1'b0, y} > mouse_y) ? {1'b0, y} - mouse_y : mouse_y - {1'b0, y};
    wire cross = !game && (((dxm < 14) && (dym < 2)) || ((dym < 14) && (dxm < 2)));

    // text region: 0 none, 1 EDIT margin / game HUD, 2 quadrant header /
    // game message, 3 legend
    wire [1:0] t_region = edit_mode ? (xr_ledit ? 2'd1 : rmargin ? 2'd2 : 2'd0) :
                          legend ? 2'd3 : c_hdr ? 2'd2 : 2'd0;

    reg [3:0] ghost_here;
    integer gi;
    always @* for (gi = 0; gi < 4; gi = gi + 1)
        ghost_here[gi] = ghost_present[gi] && (idx == ghost_pos[gi*INDEX_W +: INDEX_W]);

    // ---------------------------------------------------------- line text
    // The text on a line depends only on y (and on view inputs that change
    // between frames), so each region's string for the NEXT line is worked
    // out during the current line and latched at its end (line_last).  At
    // pixel time S1 only picks one of these ready strings by region; the
    // long string mux is off the pixel path (it missed 148.5 MHz there).
    //   m  : left margin (EDIT help / game HUD)   msg: game message (right margin)
    //   hl : header, left quadrant                 hr : header, right quadrant
    //   lg : legend (16 px lines)
    wire           pac_in = pacman || game;
    wire [10:0]    ny     = (y == V_TOTAL-1) ? 11'd0 : y + 1'b1;
    reg  [10:0]    p_y = 0;
    always @(posedge clk) p_y <= ny;
    // one more register stage: the next line's text has the whole line to settle
    wire           q_bottom = !edit_mode && (p_y >= QUAD_H);
    wire [10:0]    q_ly  = q_bottom ? p_y - QUAD_H : p_y;
    reg            p_bottom = 1'b0, p_hdr = 1'b0;
    reg  [5:0]     p_l32 = 0, p_l16 = 0;
    always @(posedge clk) begin
        p_bottom <= q_bottom; p_l32 <= q_ly[10:5]; p_l16 <= q_ly[9:4];
        p_hdr    <= (q_ly[10:5] == 6'd0);
    end
    wire [1:0]     p_pl  = {p_bottom, 1'b0};               // left quadrant on this line
    wire [1:0]     p_pr  = {p_bottom, 1'b1};               // right quadrant
    reg [8*SL-1:0] n_m, n_msg, n_hl, n_hr, n_lg;
    reg [2:0]      n_mk, n_hlk, n_hrk, n_lgsw;
    always @* begin
        n_m = T_NONE; n_mk = 3'd0;
        if (game) case (p_l32)                              // game HUD
            6'd1:  n_m = T_LEVEL;
            6'd2:  n_m = {8'd49 + game_level, {23{8'h20}}};
            6'd4:  n_m = T_ALGO;
            6'd5:  begin
                n_m  = (game_algo == 2'd0) ? T_SBFS : (game_algo == 2'd1) ? T_SDIJ :
                       (game_algo == 2'd2) ? T_AST : T_DFS;
                n_mk = {1'b0, game_algo} + 3'd1;
            end
            6'd7:  n_m = T_LIVES;
            6'd8:  n_m = {8'd48 + game_lives, {23{8'h20}}};
            6'd10: n_m = T_SCORE;
            6'd11: n_m = {8'd48 + game_score[19:16], 8'd48 + game_score[15:12],
                          8'd48 + game_score[11:8], 8'd48 + game_score[7:4],
                          8'd48 + game_score[3:0], {19{8'h20}}};
            6'd13: n_m = T_PELLETS;
            6'd14: n_m = {8'd48 + game_pellets[11:8], 8'd48 + game_pellets[7:4],
                          8'd48 + game_pellets[3:0], {21{8'h20}}};
            6'd30: n_m = T_SW6OFF;
            6'd31: n_m = T_EXITS;
            default: n_m = T_NONE;
        endcase else case (p_l32)                           // EDIT help
            6'd1: n_m = T_EDIT;
            6'd2: n_m = (edit_tool == 2'b00) ? T_TWALL : (edit_tool == 2'b01) ? T_TSTART :
                        (edit_tool == 2'b10) ? T_TGOAL : T_TWEIGHT;
            6'd4: n_m = T_SW7;
            6'd5: n_m = pac_in ? T_PAC : T_CLS;
            6'd7: n_m = T_SW6;
            6'd8: n_m = T_GAME;
            default: n_m = T_NONE;
        endcase

        case (game_state)                                   // game message
            G_READY:  n_msg = (p_l32 == 6'd2) ? T_READY : (p_l32 == 6'd4) ? T_PRESS :
                              (p_l32 == 6'd5) ? T_BTNC : T_NONE;
            G_PAUSE:  n_msg = (p_l32 == 6'd2) ? T_PAUSED : (p_l32 == 6'd4) ? T_BTNC : T_NONE;
            G_CAUGHT: n_msg = (p_l32 == 6'd2) ? T_CAUGHT : T_NONE;
            G_CLEAR:  n_msg = (p_l32 == 6'd2) ? T_LEVEL : (p_l32 == 6'd3) ? T_CLEAR :
                              (p_l32 == 6'd5) ? T_BTNC : T_NONE;
            G_OVER:   n_msg = (p_l32 == 6'd2) ? T_GAME : (p_l32 == 6'd3) ? T_OVER :
                              (p_l32 == 6'd5) ? T_BTNC : T_NONE;
            G_WIN:    n_msg = (p_l32 == 6'd2) ? T_YOU : (p_l32 == 6'd3) ? T_WIN :
                              (p_l32 == 6'd5) ? T_BTNC : T_NONE;
            G_NOPATH: n_msg = (p_l32 == 6'd2) ? T_NO : (p_l32 == 6'd3) ? T_PATHW :
                              (p_l32 == 6'd5) ? T_FIX : (p_l32 == 6'd6) ? T_MAZE : T_NONE;
            G_INIT, G_WAITENG: n_msg = (p_l32 == 6'd2) ? T_LOADING : T_NONE;
            default:  n_msg = T_NONE;
        endcase

        // quadrant headers (grey = engine not selected)
        n_hl  = p_bottom ? T_AST : T_BFS;
        n_hr  = p_bottom ? T_DFS : T_DIJ;
        n_hlk = run_mask[p_pl] ? {1'b0, p_pl} + 3'd1 : 3'd5;
        n_hrk = run_mask[p_pr] ? {1'b0, p_pr} + 3'd1 : 3'd5;

        n_lgsw = 3'd0;
        case (p_l16)                                        // legend, 16 px lines
            6'd2:  n_lg = T_LEGEND;
            6'd4:  begin n_lg = T_WALL;     n_lgsw = 3'd0; end
            6'd5:  begin n_lg = T_VISITED;  n_lgsw = 3'd1; end
            6'd6:  begin n_lg = T_FRONTIER; n_lgsw = 3'd2; end
            6'd7:  begin n_lg = T_PATH;     n_lgsw = 3'd3; end
            6'd8:  begin n_lg = T_WEIGHT;   n_lgsw = 3'd4; end
            6'd10: n_lg = T_ZOOM;
            6'd12: n_lg = T_CLICK;
            6'd13: n_lg = T_WHEEL;
            6'd15: n_lg = T_MIDDLE;
            6'd16: n_lg = T_RESET;
            6'd18: n_lg = T_SW7;
            6'd19: n_lg = pac_in ? T_PAC : T_CLS;
            default: n_lg = T_NONE;
        endcase
    end
    reg [8*SL-1:0] x_m, x_msg, x_hl, x_hr, x_lg;      // next line, registered
    reg [2:0]      x_mk, x_hlk, x_hrk, x_lgsw;
    reg            x_bot, x_hdr;
    reg [8*SL-1:0] c_m, c_msg, c_hl, c_hr, c_lg;      // current line
    reg [2:0]      c_mk, c_hlk, c_hrk, c_lgsw;
    always @(posedge clk) begin
        x_m <= n_m; x_msg <= n_msg; x_hl <= n_hl; x_hr <= n_hr; x_lg <= n_lg;
        x_mk <= n_mk; x_hlk <= n_hlk; x_hrk <= n_hrk; x_lgsw <= n_lgsw;
        x_bot <= p_bottom; x_hdr <= p_hdr;
        if (line_last) begin
            c_m <= x_m; c_msg <= x_msg; c_hl <= x_hl; c_hr <= x_hr; c_lg <= x_lg;
            c_mk <= x_mk; c_hlk <= x_hlk; c_hrk <= x_hrk; c_lgsw <= x_lgsw;
            c_bot <= x_bot; c_hdr <= x_hdr;
        end
    end

    // pick by region (S1)
    reg [8*SL-1:0] t_str;
    reg [2:0]      t_kind;      // 0 white, 1 BFS, 2 DIJ, 3 A*, 4 DFS, 5 grey
    reg [2:0]      t_swatch;    // legend swatch: 0 wall .. 4 weight
    always @* begin
        t_str = T_NONE; t_kind = 3'd0; t_swatch = 3'd0;
        case (t_region)
            2'd1: begin t_str = c_m; t_kind = c_mk; end
            2'd2: if (game) begin t_str = c_msg; t_kind = 3'd2; end   // yellow
                  else if (right) begin t_str = c_hr; t_kind = c_hrk; end
                  else begin t_str = c_hl; t_kind = c_hlk; end
            2'd3: begin t_str = c_lg; t_swatch = c_lgsw; end
            default: t_str = T_NONE;
        endcase
    end

    reg               s1_active, s1_hs, s1_vs, s1_edit, s1_pac, s1_in_grid, s1_border, s1_grid;
    reg               s1_sep, s1_cross, s1_start, s1_goal, s1_cur, s1_hover, s1_on, s1_selb;
    reg               s1_game, s1_player;
    reg [3:0]         s1_ghost;
    reg [1:0]         s1_panel, s1_tregion;
    reg [6:0]         s1_fx, s1_fy, s1_tchar;
    reg [5:0]         s1_tline;
    reg signed [7:0]  s1_dx, s1_dy;
    reg [6:0]         s1_adx, s1_ady, s1_aexl, s1_aexr, s1_aey;   // |offsets| for the squares
    reg [2:0]         s1_gcol, s1_grow;
    reg [8*SL-1:0]    s1_str;
    reg [2:0]         s1_kind, s1_swatch;
    always @(posedge clk) begin
        s1_active <= active_video; s1_hs <= hsync_in; s1_vs <= vsync_in;
        s1_edit <= edit_mode; s1_pac <= pacman || game; s1_panel <= panel;
        s1_game <= game;
        s1_player <= game && in_grid && (idx == player_pos);
        s1_ghost  <= (game && in_grid) ? ghost_here : 4'b0000;
        s1_in_grid <= in_grid; s1_border <= ext_x && ext_y && !in_grid; s1_grid <= gridline;
        s1_sep <= sep; s1_cross <= cross; s1_selb <= selb;
        s1_fx <= fx; s1_fy <= fy;
        s1_dx <= $signed({1'b0, fx}) - 8'sd64;
        s1_dy <= $signed({1'b0, fy}) - 8'sd64;
        // distances from the cell centre (64) and from the ghost's eyes (x 44 / 84, y 52)
        s1_adx  <= (fx >= 7'd64) ? fx - 7'd64 : 7'd64 - fx;
        s1_ady  <= (fy >= 7'd64) ? fy - 7'd64 : 7'd64 - fy;
        s1_aexl <= (fx >= 7'd44) ? fx - 7'd44 : 7'd44 - fx;
        s1_aexr <= (fx >= 7'd84) ? fx - 7'd84 : 7'd84 - fx;
        s1_aey  <= (fy >= 7'd52) ? fy - 7'd52 : 7'd52 - fy;
        s1_start <= in_grid && (idx == start_idx);
        s1_goal  <= in_grid && (idx == goal_idx);
        s1_cur   <= in_grid && (idx == view_current);
        s1_on    <= view_on;
        s1_hover <= in_grid && edit_mode && !game && cursor_valid && (idx == cursor_idx);
        s1_tline <= t_line; s1_tchar <= t_char; s1_tregion <= t_region;
        s1_gcol <= g_col; s1_grow <= g_row;
        s1_str <= t_str; s1_kind <= t_kind; s1_swatch <= t_swatch;
        cell_addr <= in_grid ? idx : NONE;
        up_addr   <= (in_grid && row_i > 0)        ? idx - COLS : NONE;
        dn_addr   <= (in_grid && row_i < ROWS-1)   ? idx + COLS : NONE;
        lf_addr   <= (in_grid && col_i > 0)        ? idx - 1'b1 : NONE;
        rt_addr   <= (in_grid && col_i < COLS-1)   ? idx + 1'b1 : NONE;
    end

    // ------------------------------------------------------------ S2 -------

    wire [2:0] zl_sel = zoom_level[zoom_sel*3 +: 3];
    wire [7:0] t_raw = (s1_tchar < SL) ? s1_str[(SL-1-s1_tchar)*8 +: 8] : 8'h20;
    wire [7:0] t_ch  = (s1_tregion == 2'd3 && s1_tline == 6'd10 && s1_tchar == 7'd6) ?
                       (8'd49 + zl_sel) : t_raw;         // selected quadrant's zoom '1'..'8'

    // Pac-Man's mouth axis: forward offset (signed) and |sideways offset|
    // direction decoded once into registers (it changes only between frames)
    reg  m_right = 1'b1, m_left = 1'b0, m_down = 1'b0, m_horiz = 1'b1;
    always @(posedge clk) begin
        m_right <= !game || (player_dir == 2'd3);           // 0 up 1 down 2 left 3 right
        m_left  <=  game && (player_dir == 2'd2);
        m_down  <=  game && (player_dir == 2'd1);
        m_horiz <= !game || player_dir[1];
    end
    wire signed [7:0] m_fwd  = m_right ? s1_dx : m_left ? -s1_dx : m_down ? s1_dy : -s1_dy;
    wire [6:0]        m_side = m_horiz ? s1_ady : s1_adx;

    reg               s2_active, s2_hs, s2_vs, s2_edit, s2_pac, s2_in_grid, s2_border, s2_grid;
    reg               s2_sep, s2_cross, s2_start, s2_goal, s2_cur, s2_hover, s2_header_box;
    reg               s2_on, s2_legend, s2_selb;
    reg               s2_game, s2_player, s2_pellet;
    reg               s2_ghost;           // any ghost on this cell
    reg [11:0]        s2_gcolour;         // its colour (split in half when ghosts share)
    reg               s2_wall, s2_band, s2_mouth, s2_gbody_low;
    reg [1:0]         s2_panel, s2_state;
    reg [COST_W-1:0]  s2_cost;
    reg signed [8:0]  s2_dy;
    // the two squares on DSP48s (the LUT multipliers were the last 148.5 MHz path)
    (* use_dsp = "yes" *) reg [13:0] s2_dx2, s2_dy2;
    reg               s2_eyel, s2_eyer;   // inside the ghost's left / right eye
    reg [2:0]         s2_kind, s2_swatch, s2_gcol, s2_grow;
    reg [7:0]         s2_ch;
    always @(posedge clk) begin
        s2_active <= s1_active; s2_hs <= s1_hs; s2_vs <= s1_vs;
        s2_edit <= s1_edit; s2_pac <= s1_pac; s2_panel <= s1_panel;
        s2_in_grid <= s1_in_grid; s2_border <= s1_border; s2_grid <= s1_grid;
        s2_sep <= s1_sep; s2_cross <= s1_cross; s2_start <= s1_start; s2_goal <= s1_goal;
        s2_selb <= s1_selb;
        s2_cur <= s1_cur; s2_hover <= s1_hover; s2_on <= s1_on;
        s2_game <= s1_game; s2_player <= s1_player; s2_ghost <= |s1_ghost;
        // several ghosts on one cell: left half = lowest-numbered, right half = highest
        s2_gcolour <= s1_fx[6] ? (s1_ghost[3] ? P_GHOST3 : s1_ghost[2] ? P_GHOST2 :
                                  s1_ghost[1] ? P_GHOST1 : P_GHOST)
                               : (s1_ghost[0] ? P_GHOST  : s1_ghost[1] ? P_GHOST1 :
                                  s1_ghost[2] ? P_GHOST2 : P_GHOST3);
        s2_pellet <= s1_game && (cell_addr < NUM_CELLS) && pellet[cell_addr];
        s2_legend <= (s1_tregion == 2'd3);
        s2_ch <= t_ch; s2_gcol <= s1_gcol; s2_grow <= s1_grow;
        s2_kind <= s1_kind; s2_swatch <= s1_swatch;
        s2_header_box <= (s1_tregion == 2'd2) && !s1_game && (t_ch != 8'h20);
        s2_cost  <= cell_cost;
        s2_wall  <= s1_in_grid && (cell_cost == `ARENA_COST_WALL);
        s2_band  <= ((lf_cost != `ARENA_COST_WALL) && (s1_fx < BAND)) ||
                    ((rt_cost != `ARENA_COST_WALL) && (s1_fx >= 128-BAND)) ||
                    ((up_cost != `ARENA_COST_WALL) && (s1_fy < BAND)) ||
                    ((dn_cost != `ARENA_COST_WALL) && (s1_fy >= 128-BAND));
        s2_state <= (s1_panel == 2'd0) ? bfs_state : (s1_panel == 2'd1) ? dij_state :
                    (s1_panel == 2'd2) ? ast_state : dfs_state;
        // squares only here; the sums are formed in S3
        s2_dx2 <= s1_adx * s1_adx;   s2_dy2 <= s1_ady * s1_ady;
        // eye radius 12: both offsets < 12, so the test is a small LUT function
        // of two 4-bit values (keeps an adder + compare off the S3 colour path)
        s2_eyel <= (s1_aexl < 7'd12) && (s1_aey < 7'd12) &&
                   ({4'd0, s1_aexl[3:0]} * s1_aexl[3:0] + {4'd0, s1_aey[3:0]} * s1_aey[3:0] < R_EYE2);
        s2_eyer <= (s1_aexr < 7'd12) && (s1_aey < 7'd12) &&
                   ({4'd0, s1_aexr[3:0]} * s1_aexr[3:0] + {4'd0, s1_aey[3:0]} * s1_aey[3:0] < R_EYE2);
        s2_dy <= s1_dy;
        // mouth: wedge opening along `fwd`, |side| < fwd * slope (slope 6/8 open,
        // 1/8 shut).  Facing right outside the game; player_dir in the game.
        s2_mouth <= !m_fwd[7] && (m_fwd != 0) &&
                    ({1'b0, m_side, 3'b000} < (frame_cnt[3] ? {1'b0, m_fwd[6:0], 2'b00} + {2'b00, m_fwd[6:0], 1'b0}
                                                   : {3'b000, m_fwd[6:0]}));
        // ghost lower body: rectangle under the round head
        s2_gbody_low <= !s1_dy[7] && (s1_dy < 8'sd45) && (s1_dx > -8'sd51) && (s1_dx < 8'sd51);
    end

    // ------------------------------------------------------------ S3 -------
    wire on_path  = (s2_state == `ARENA_CS_PATH);
    wire on_vis   = (s2_state == `ARENA_CS_VISITED);
    wire on_front = (s2_state == `ARENA_CS_FRONTIER);
    wire run      = !s2_edit && s2_on;     // search overlay only for selected engines
    wire [14:0] s2_d0 = s2_dx2 + s2_dy2;
    wire pac_body = (s2_d0 < R_PAC2) && !s2_mouth;
    wire g_eye    = s2_eyel || s2_eyer;
    wire g_body   = ((s2_d0 < R_GHOST2) && s2_dy[8]) || s2_gbody_low;

    wire [4:0] glyph = font(s2_ch, s2_grow);
    reg [11:0] c_algo, c_cls, c_pac, c_text, c_sw, c;
    always @* begin
        c_algo = (s2_panel == 2'd0) ? C_BFS : (s2_panel == 2'd1) ? C_DIJ :
                 (s2_panel == 2'd2) ? C_AST : C_DFS;

        // classic theme
        if (s2_grid)                     c_cls = C_GRIDLINE;
        else if (s2_start)               c_cls = C_START;
        else if (s2_goal)                c_cls = C_GOAL;
        else if (s2_hover)               c_cls = C_HOVER;
        else if (run && s2_cur)          c_cls = C_WHITE;
        else if (s2_wall)                c_cls = C_WALL;
        else if (run && on_path)         c_cls = c_algo;
        else if (run && on_vis)          c_cls = C_VISITED;
        else if (run && on_front)        c_cls = C_FRONTIER;
        else if (s2_cost > `ARENA_COST_OPEN) c_cls = C_WEIGHT;
        else                             c_cls = C_BLACK;

        // Pac-Man theme
        if (s2_wall)                                   c_pac = s2_band ? P_WALL : P_WALL_IN;
        else if (run && s2_cur && pac_body)            c_pac = P_PAC;
        else if (s2_edit && s2_start && pac_body)      c_pac = P_PAC;
        else if (s2_goal && g_body)                    c_pac = g_eye ? C_WHITE : P_GHOST;
        else if (s2_hover)                             c_pac = P_HOVER;
        else if (run && on_path && (s2_d0 < R_PATH2))  c_pac = P_PELLET;
        else if (run && on_front && (s2_d0 < R_FRONT2)) c_pac = P_FRONT;
        else if (run && on_vis)                        c_pac = P_VISITED;
        else if (s2_cost > `ARENA_COST_OPEN)           c_pac = P_WEIGHT;
        else                                           c_pac = P_FLOOR;

        c_text = (s2_kind == 3'd1) ? C_BFS : (s2_kind == 3'd2) ? C_DIJ :
                 (s2_kind == 3'd3) ? C_AST : (s2_kind == 3'd4) ? C_DFS :
                 (s2_kind == 3'd5) ? C_OFF : C_WHITE;
        case (s2_swatch)
            3'd0: c_sw = s2_pac ? P_WALL     : C_WALL;
            3'd1: c_sw = s2_pac ? P_SWATCH_VIS : C_VISITED;
            3'd2: c_sw = s2_pac ? P_FRONT    : C_FRONTIER;
            3'd3: c_sw = s2_pac ? P_PELLET   : C_AST;
            default: c_sw = s2_pac ? P_WEIGHT : C_WEIGHT;
        endcase

        // everything except text: background layers
        if (s2_header_box)                      c = C_BLACK;
        else if (s2_selb)                       c = C_HOVER;     // zoom-selected quadrant
        else if (s2_sep)                        c = C_SEP;
        else if (s2_legend)                     c = C_LEGEND_BG;
        else if (s2_in_grid)                    c = s2_pac ? c_pac : c_cls;   // GAME cells: S4
        else if (s2_pac && s2_border)           c = P_WALL;
        else                                    c = C_BLACK;
    end

    // glyph row kept in LUTs: a block-RAM ROM would absorb s2_ch as its address
    // register and put the string-select logic onto the BRAM address setup
    (* rom_style = "distributed" *) reg [4:0] s3_glyph;
    reg        s3_active, s3_hs, s3_vs, s3_cross, s3_cell, s3_sw;
    reg [2:0]  s3_gcol;
    reg [11:0] s3_bg, s3_ctext, s3_csw;
    // GAME cell (always the Pac-Man look): shape tests here, colour in S4
    reg        s3_gcell, s3_gwall, s3_gband, s3_gghost, s3_geye, s3_gpac, s3_gpel, s3_gweight;
    reg [11:0] s3_gcolour;
    always @(posedge clk) begin
        s3_active <= s2_active; s3_hs <= s2_hs; s3_vs <= s2_vs; s3_cross <= s2_cross;
        s3_glyph <= glyph;
        s3_gcol  <= s2_gcol;
        s3_cell  <= (s2_ch != "#") && (s2_gcol < 5) && (s2_grow < 7);
        s3_sw    <= (s2_ch == "#") && (s2_gcol < 5) && (s2_grow < 7);
        s3_bg    <= c;
        s3_gcell   <= s2_game && s2_in_grid;
        s3_gwall   <= s2_wall;   s3_gband <= s2_band;
        s3_gghost  <= s2_ghost && g_body;  s3_geye <= g_eye;
        s3_gpac    <= s2_player && pac_body;
        s3_gpel    <= s2_pellet && (s2_d0 < R_PATH2);
        s3_gweight <= (s2_cost > `ARENA_COST_OPEN);
        s3_gcolour <= s2_gcolour;
        s3_ctext <= c_text;
        s3_csw   <= c_sw;
    end

    // ------------------------------------------------------------ S4 -------
    wire s3_text = s3_cell && s3_glyph[4 - s3_gcol];
    // GAME cell colour; ghosts drawn over Pac-Man
    wire [11:0] c_game = s3_gwall   ? (s3_gband ? P_WALL : P_WALL_IN) :
                         s3_gghost  ? (s3_geye ? C_WHITE : s3_gcolour) :
                         s3_gpac    ? P_PAC :
                         s3_gpel    ? P_PELLET :
                         s3_gweight ? P_WEIGHT : P_FLOOR;
    always @(posedge clk) begin
        {red, green, blue} <= !s3_active ? C_BLACK : s3_cross ? C_WHITE :
                              s3_text ? s3_ctext : s3_sw ? s3_csw :
                              s3_gcell ? c_game : s3_bg;
        hsync <= s3_hs;
        vsync <= s3_vs;
    end
endmodule
