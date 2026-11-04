`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// oled_stats : 100 MHz snapshot + finish order + decimal conversion for the
// OLED dashboard (CONTRACT 8).
//
// Engine order everywhere: 0 = BFS, 1 = Dijkstra, 2 = A*, 3 = DFS.
// Packed buses: engine e uses bits [e*W +: W].
//
// Finish order ("place")
//   All engines start on the same edge and share step_en, so the engine with
//   fewer `cycles` finished earlier.  For a done, selected engine e:
//       place[e] = 1 + #{ selected j != e : done[j] && cycles[j] < cycles[e] }
//   Equal cycles share a place.  place = 0 means "no place yet" (not
//   selected, or still running).
//
// Snapshot (CONTRACT 8 clock-domain rule)
//   Every SNAP_DIV cycles (default 10 Hz) the live values are copied into
//   registers.  Then a small double-dabble FSM converts them, one value at a
//   time, into BCD registers (12 values x 21 cycles; a tick during a
//   conversion is skipped).  The 6.25 MHz OLED
//   logic reads only these registers; they change rarely, so at worst one
//   frame shows a half-updated number.
//
//   cyc_bcd  : 6 digits, cycles clamped to 999999
//   exp_bcd  : 3 digits, expansions clamped to 999
//   cost_bcd : 4 digits, path_cost clamped to 9999
//   status   : {busy, done, found, no_path}
//////////////////////////////////////////////////////////////////////////////
module oled_stats #(
    parameter CNT_W    = `ARENA_CNT_W,
    parameter DIST_W   = `ARENA_DIST_W,
    parameter SNAP_DIV = `ARENA_CLK_HZ / 10
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire [3:0]           run_mask,
    input  wire [4*CNT_W-1:0]   cycles,
    input  wire [4*DIST_W-1:0]  expansions,
    input  wire [4*DIST_W-1:0]  path_cost,
    input  wire [3:0]           busy,
    input  wire [3:0]           done,
    input  wire [3:0]           found,
    input  wire [3:0]           no_path,

    output reg  [3:0]           snap_mask,
    output reg  [4*4-1:0]       snap_status,
    output reg  [4*3-1:0]       snap_place,
    output reg  [4*24-1:0]      cyc_bcd,
    output reg  [4*12-1:0]      exp_bcd,
    output reg  [4*16-1:0]      cost_bcd
);
    localparam BIN_W = 20;                       // 999999 < 2^20
    localparam [BIN_W-1:0] CYC_MAX  = 20'd999999;
    localparam [BIN_W-1:0] EXP_MAX  = 20'd999;
    localparam [BIN_W-1:0] COST_MAX = 20'd9999;

    // ---------------- finish order (combinational, live values) ----------
    function [CNT_W-1:0] cyc_of;
        input [4*CNT_W-1:0] v; input [1:0] e;
        cyc_of = v[e*CNT_W +: CNT_W];
    endfunction

    reg [2:0] place_live [0:3];
    integer e, j;      // combinational loop variables
    integer k;         // clocked-block loop variable
    always @* begin
        for (e = 0; e < 4; e = e + 1) begin
            place_live[e] = 3'd0;
            if (run_mask[e] && done[e]) begin
                place_live[e] = 3'd1;
                for (j = 0; j < 4; j = j + 1)
                    if (j != e && run_mask[j] && done[j] &&
                        cyc_of(cycles, j) < cyc_of(cycles, e))
                        place_live[e] = place_live[e] + 3'd1;
            end
        end
    end

    // ---------------- 10 Hz snapshot ------------------------------------
    reg [$clog2(SNAP_DIV+1)-1:0] div;
    wire tick = (div == SNAP_DIV - 1);

    // raw (clamped) binary values waiting for conversion: 0..3 cycles,
    // 4..7 expansions, 8..11 cost
    reg [BIN_W-1:0] raw [0:11];

    function [BIN_W-1:0] clamp;
        input [CNT_W-1:0] v; input [BIN_W-1:0] max;
        clamp = (v > max) ? max : v[BIN_W-1:0];
    endfunction

    // ---------------- double dabble, one value at a time ------------------
    reg        conv_busy;
    reg [3:0]  conv_idx;            // 0..11
    reg [4:0]  conv_bit;            // BIN_W..1 shifts left
    reg [BIN_W-1:0] sh_bin;
    reg [23:0] sh_bcd;              // 6 BCD digits

    // add 3 to every BCD digit >= 5, before the shift
    wire [23:0] bcd_adj;
    genvar d;
    generate
        for (d = 0; d < 6; d = d + 1) begin : g_adj
            assign bcd_adj[d*4 +: 4] = (sh_bcd[d*4 +: 4] >= 4'd5) ? sh_bcd[d*4 +: 4] + 4'd3
                                                                  : sh_bcd[d*4 +: 4];
        end
    endgenerate
    wire [23:0] bcd_next = {bcd_adj[22:0], sh_bin[BIN_W-1]};

    always @(posedge clk) begin
        if (rst) begin
            div         <= 0;
            snap_mask   <= 4'b0000;
            snap_status <= 0;
            snap_place  <= 0;
            cyc_bcd     <= 0;
            exp_bcd     <= 0;
            cost_bcd    <= 0;
            conv_busy   <= 1'b0;
            conv_idx    <= 0;
            conv_bit    <= 0;
            sh_bin      <= 0;
            sh_bcd      <= 0;
        end else begin
            div <= tick ? 0 : div + 1'b1;

            // a tick that lands mid-conversion is skipped, so a conversion
            // is never cut short (matters only if SNAP_DIV < 12*21)
            if (tick && !conv_busy) begin
                snap_mask <= run_mask;
                for (k = 0; k < 4; k = k + 1) begin
                    snap_status[k*4 +: 4] <= {busy[k], done[k], found[k], no_path[k]};
                    snap_place[k*3 +: 3]  <= place_live[k];
                    raw[k]     <= clamp(cycles[k*CNT_W +: CNT_W], CYC_MAX);
                    raw[4 + k] <= clamp({{(CNT_W-DIST_W){1'b0}}, expansions[k*DIST_W +: DIST_W]}, EXP_MAX);
                    raw[8 + k] <= clamp({{(CNT_W-DIST_W){1'b0}}, path_cost[k*DIST_W +: DIST_W]}, COST_MAX);
                end
                conv_busy <= 1'b1;
                conv_idx  <= 0;
                conv_bit  <= 0;           // 0 = load the next value
            end else if (conv_busy) begin
                if (conv_bit == 0) begin
                    sh_bin   <= raw[conv_idx];
                    sh_bcd   <= 0;
                    conv_bit <= BIN_W;
                end else begin
                    sh_bin   <= {sh_bin[BIN_W-2:0], 1'b0};
                    sh_bcd   <= bcd_next;
                    conv_bit <= conv_bit - 1'b1;
                    if (conv_bit == 1) begin
                        // bcd_next is the finished value
                        if (conv_idx < 4)
                            cyc_bcd[conv_idx*24 +: 24] <= bcd_next;
                        else if (conv_idx < 8)
                            exp_bcd[(conv_idx-4)*12 +: 12] <= bcd_next[11:0];
                        else
                            cost_bcd[(conv_idx-8)*16 +: 16] <= bcd_next[15:0];
                        if (conv_idx == 11) conv_busy <= 1'b0;
                        else                conv_idx  <= conv_idx + 1'b1;
                    end
                end
            end
        end
    end
endmodule
