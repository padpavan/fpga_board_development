`timescale 1ns / 1ps
`include "arena_defs.vh"
// Scroll-wheel zoom for the RUN view, one view per quadrant (CONTRACT 9, v1.10).
//
// Each quadrant q (0 BFS, 1 Dijkstra, 2 A*, 3 DFS, i.e. {bottom, right}) has
// its own view (ox[q], oy[q], level[q]): the grid position (in cells, FRAC-bit
// fixed point) of the quadrant's top-left pixel, and the zoom level.  Level L
// magnifies by L+1, so k[q] = 1 / (CELL * (L+1)) cells per screen pixel.
//
// Left click (RUN view only) selects the quadrant under the cursor.  The wheel
// then zooms only the selected quadrant, one level per notch, keeping the grid
// point under the cursor still:
//     ox' = ox + mx * (k - k'),   mx = cursor x inside its quadrant.
// The result is clamped so the view never leaves the grid's level-0 framing;
// level 0 always returns to the default centred view.  Middle click resets the
// selected quadrant (all four if none is selected).  Before the first click no
// quadrant is selected and the wheel does nothing.  Clicks and the wheel are
// ignored in EDIT / GAME (enable = 0); rst clears every view and the selection.
module vga_zoom #(
    parameter COLS=`ARENA_COLS, parameter ROWS=`ARENA_ROWS,
    parameter H_VISIBLE=`ARENA_VGA_H, parameter V_VISIBLE=`ARENA_VGA_V,
    parameter FRAC=20, parameter OW=28, parameter KW=16,
    parameter ZOOM_IN_NEG=1      // 1: negative wheel delta (wheel pushed away) zooms in
) (
    input  wire clk, input wire rst, input wire enable,
    input  wire new_event, input wire [3:0] zpos, input wire middle, input wire left,
    input  wire [11:0] mouse_x, input wire [11:0] mouse_y,
    output wire [4*OW-1:0] ox,         // per quadrant, quadrant q at [q*OW +: OW] (signed)
    output wire [4*OW-1:0] oy,
    output wire [4*KW-1:0] k,
    output wire [4*3-1:0]  level,
    output reg  [1:0]      sel = 2'd0, // selected quadrant
    output reg             sel_valid = 1'b0
);
    localparam QUAD_W = H_VISIBLE/2, QUAD_H = V_VISIBLE/2;
    localparam CELL_X = QUAD_W/COLS, CELL_Y = QUAD_H/ROWS;
    localparam CELL   = (CELL_X < CELL_Y) ? CELL_X : CELL_Y;      // 45 px at level 0
    localparam MARGIN_X = (QUAD_W - COLS*CELL)/2;                 // 120
    localparam MARGIN_Y = (QUAD_H - ROWS*CELL)/2;                 // 0
    localparam ONE = 1 << FRAC;
    localparam K1 = (ONE + (CELL*1)/2)/(CELL*1), K2 = (ONE + (CELL*2)/2)/(CELL*2),
               K3 = (ONE + (CELL*3)/2)/(CELL*3), K4 = (ONE + (CELL*4)/2)/(CELL*4),
               K5 = (ONE + (CELL*5)/2)/(CELL*5), K6 = (ONE + (CELL*6)/2)/(CELL*6),
               K7 = (ONE + (CELL*7)/2)/(CELL*7), K8 = (ONE + (CELL*8)/2)/(CELL*8);
    localparam signed [OW-1:0] OX0 = -(MARGIN_X*K1);
    localparam signed [OW-1:0] OY0 = -(MARGIN_Y*K1);

    function [KW-1:0] k_of;
        input [2:0] l;
        case (l)
            3'd0: k_of = K1; 3'd1: k_of = K2; 3'd2: k_of = K3; 3'd3: k_of = K4;
            3'd4: k_of = K5; 3'd5: k_of = K6; 3'd6: k_of = K7; default: k_of = K8;
        endcase
    endfunction
    // largest allowed origin: the view's far edge stops at the level-0 margin
    function signed [OW-1:0] ox_max;
        input [2:0] l;
        case (l)
            3'd0: ox_max = OX0;
            3'd1: ox_max = (COLS*ONE) - OX0 - QUAD_W*K2; 3'd2: ox_max = (COLS*ONE) - OX0 - QUAD_W*K3;
            3'd3: ox_max = (COLS*ONE) - OX0 - QUAD_W*K4; 3'd4: ox_max = (COLS*ONE) - OX0 - QUAD_W*K5;
            3'd5: ox_max = (COLS*ONE) - OX0 - QUAD_W*K6; 3'd6: ox_max = (COLS*ONE) - OX0 - QUAD_W*K7;
            default: ox_max = (COLS*ONE) - OX0 - QUAD_W*K8;
        endcase
    endfunction
    function signed [OW-1:0] oy_max;
        input [2:0] l;
        case (l)
            3'd0: oy_max = OY0;
            3'd1: oy_max = (ROWS*ONE) - OY0 - QUAD_H*K2; 3'd2: oy_max = (ROWS*ONE) - OY0 - QUAD_H*K3;
            3'd3: oy_max = (ROWS*ONE) - OY0 - QUAD_H*K4; 3'd4: oy_max = (ROWS*ONE) - OY0 - QUAD_H*K5;
            3'd5: oy_max = (ROWS*ONE) - OY0 - QUAD_H*K6; 3'd6: oy_max = (ROWS*ONE) - OY0 - QUAD_H*K7;
            default: oy_max = (ROWS*ONE) - OY0 - QUAD_H*K8;
        endcase
    endfunction

    // the four views
    reg signed [OW-1:0] vox [0:3];
    reg signed [OW-1:0] voy [0:3];
    reg        [2:0]    vlev[0:3];
    genvar q;
    generate for (q = 0; q < 4; q = q + 1) begin : g_out
        assign ox[q*OW +: OW]  = vox[q];
        assign oy[q*OW +: OW]  = voy[q];
        assign level[q*3 +: 3] = vlev[q];
        assign k[q*KW +: KW]   = k_of(vlev[q]);
    end endgenerate

    wire [2:0] cur_level = vlev[sel];
    wire z_neg = zpos[3];
    wire z_pos = !zpos[3] && (zpos != 4'd0);
    wire want_in  = ZOOM_IN_NEG ? z_neg : z_pos;
    wire want_out = ZOOM_IN_NEG ? z_pos : z_neg;
    wire [2:0] up_level   = (cur_level == 3'd7) ? cur_level : cur_level + 1'b1;
    wire [2:0] down_level = (cur_level == 3'd0) ? cur_level : cur_level - 1'b1;
    wire [2:0] req_level  = want_in ? up_level : down_level;
    wire [1:0] click_q    = {(mouse_y >= QUAD_H), (mouse_x >= QUAD_W)};

    reg        middle_d = 1'b0, left_d = 1'b0;
    reg [1:0]  st = 2'd0;             // 0 idle, 1 multiply, 2 apply
    reg [2:0]  nlev;
    reg signed [12:0]   mx, my;       // cursor inside its quadrant
    reg signed [KW:0]   dk;           // k(level) - k(nlev)
    reg signed [OW-1:0] px, py;
    wire signed [OW-1:0] nx = vox[sel] + px;
    wire signed [OW-1:0] ny = voy[sel] + py;
    wire signed [OW-1:0] nx_max = ox_max(nlev);
    wire signed [OW-1:0] ny_max = oy_max(nlev);

    integer i;
    initial for (i = 0; i < 4; i = i + 1) begin vox[i] = OX0; voy[i] = OY0; vlev[i] = 3'd0; end

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 4; i = i + 1) begin vox[i] <= OX0; voy[i] <= OY0; vlev[i] <= 3'd0; end
            sel <= 2'd0; sel_valid <= 1'b0; st <= 2'd0; middle_d <= 1'b0; left_d <= 1'b0;
        end else begin
            middle_d <= middle;
            left_d   <= left;
            case (st)
                2'd0: begin
                    if (middle && !middle_d) begin
                        for (i = 0; i < 4; i = i + 1)
                            if (!sel_valid || sel == i) begin
                                vox[i] <= OX0; voy[i] <= OY0; vlev[i] <= 3'd0;
                            end
                    end else if (enable && left && !left_d) begin
                        sel <= click_q; sel_valid <= 1'b1;
                    end else if (enable && sel_valid && new_event && (want_in || want_out) &&
                                 (req_level != cur_level)) begin
                        nlev <= req_level;
                        mx <= (mouse_x >= QUAD_W) ? mouse_x - QUAD_W : mouse_x;
                        my <= (mouse_y >= QUAD_H) ? mouse_y - QUAD_H : mouse_y;
                        dk <= $signed({1'b0, k_of(cur_level)}) - $signed({1'b0, k_of(req_level)});
                        st <= 2'd1;
                    end
                end
                2'd1: begin
                    px <= mx * dk;
                    py <= my * dk;
                    st <= 2'd2;
                end
                default: begin
                    if (nlev == 3'd0) begin vox[sel] <= OX0; voy[sel] <= OY0; end
                    else begin
                        vox[sel] <= (nx < OX0) ? OX0 : (nx > nx_max) ? nx_max : nx;
                        voy[sel] <= (ny < OY0) ? OY0 : (ny > ny_max) ? ny_max : ny;
                    end
                    vlev[sel] <= nlev;
                    st <= 2'd0;
                end
            endcase
        end
    end
endmodule
