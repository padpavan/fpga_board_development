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
//   time, into BCD registers (16 values x 21 cycles; a tick during a
//   conversion is skipped).  The 6.25 MHz OLED
//   logic reads only these registers; they change rarely, so at worst one
//   frame shows a half-updated number.
//
//   cyc_bcd  : 6 digits, cycles clamped to 999999
//   exp_bcd  : 3 digits, expansions clamped to 999
//   cost_bcd : 4 digits, path_cost clamped to 9999
//   gap_bcd  : 6 digits, cycles - fastest selected done engine (0 if not done)
//   status   : {busy, done, found, no_path}
//
// Results order (snap_order, 4 x 2-bit engine ids, entry 0 first)
//   Selected + done engines by cycles (ties by engine id), then selected
//   engines still running, then unselected ones.  Used by arena_top's
//   OLED results screen; m2_race_top leaves it unconnected.
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
    output reg  [4*16-1:0]      cost_bcd,
    output reg  [4*24-1:0]      gap_bcd,
    output reg  [4*2-1:0]       snap_order
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
    // 4..7 expansions, 8..11 cost, 12..15 gap
    localparam NVAL = 16;
    reg [BIN_W-1:0] raw [0:NVAL-1];

    function [BIN_W-1:0] clamp;
        input [CNT_W-1:0] v; input [BIN_W-1:0] max;
        clamp = (v > max) ? max : v[BIN_W-1:0];
    endfunction

    // ---------------- results order and gap to the fastest -----------------
    // Computed from the snapshot (clamped cycles in raw[0..3], snap_mask,
    // snap_status) in the two cycles after a tick, not from the live 32-bit
    // counters (that was one 31-level path at 100 MHz).  raw[12..15] are
    // converted last, long after they are written.
    wire [3:0] sfin = snap_mask & {snap_status[14], snap_status[10], snap_status[6], snap_status[2]};
    // (every value is passed in: a function reading module variables is not
    // re-evaluated by a continuous assignment when those variables change)
    wire [BIN_W-1:0] fv0 = sfin[0] ? raw[0] : {BIN_W{1'b1}};   // cycles, or max if not finished
    wire [BIN_W-1:0] fv1 = sfin[1] ? raw[1] : {BIN_W{1'b1}};
    wire [BIN_W-1:0] fv2 = sfin[2] ? raw[2] : {BIN_W{1'b1}};
    wire [BIN_W-1:0] fv3 = sfin[3] ? raw[3] : {BIN_W{1'b1}};
    wire [BIN_W-1:0] m01 = (fv0 < fv1) ? fv0 : fv1;
    wire [BIN_W-1:0] m23 = (fv2 < fv3) ? fv2 : fv3;
    // sort key {group, cycles, id}: smaller = earlier
    function [BIN_W+3:0] skey; input fin; input sel; input [BIN_W-1:0] cyc; input [1:0] id;
        skey = {fin ? 2'd0 : sel ? 2'd1 : 2'd2, fin ? cyc : {BIN_W{1'b0}}, id};
    endfunction
    wire [BIN_W+3:0] key0 = skey(sfin[0], snap_mask[0], raw[0], 2'd0);
    wire [BIN_W+3:0] key1 = skey(sfin[1], snap_mask[1], raw[1], 2'd1);
    wire [BIN_W+3:0] key2 = skey(sfin[2], snap_mask[2], raw[2], 2'd2);
    wire [BIN_W+3:0] key3 = skey(sfin[3], snap_mask[3], raw[3], 2'd3);
    wire [4*(BIN_W+4)-1:0] keys = {key3, key2, key1, key0};
    reg  [BIN_W-1:0] fastest_r;
    reg  [15:0]      lt_r;                 // lt_r[e*4+j] = key j sorts before key e
    reg  [1:0]       rank [0:3];
    reg  [7:0]       order_next;
    integer          e2, j2;
    always @* begin
        for (e2 = 0; e2 < 4; e2 = e2 + 1) begin
            rank[e2] = 2'd0;
            for (j2 = 0; j2 < 4; j2 = j2 + 1)
                if (lt_r[e2*4 + j2]) rank[e2] = rank[e2] + 2'd1;
        end
        order_next = 8'd0;
        for (e2 = 0; e2 < 4; e2 = e2 + 1)
            order_next[rank[e2]*2 +: 2] = e2[1:0];
    end

    // ---------------- double dabble, one value at a time ------------------
    reg        conv_busy;
    reg [3:0]  conv_idx;            // 0..NVAL-1
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
            gap_bcd     <= 0;
            snap_order  <= 8'b11_10_01_00;
            conv_busy   <= 1'b0;
            conv_idx    <= 0;
            conv_bit    <= 0;
            sh_bin      <= 0;
            sh_bcd      <= 0;
            fastest_r   <= 0;
            lt_r        <= 0;
        end else begin
            div <= tick ? 0 : div + 1'b1;

            // a tick that lands mid-conversion is skipped, so a conversion
            // is never cut short (matters only if SNAP_DIV < NVAL*21)
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
                // order / gap: stage A on the first conversion cycle, stage B on the second
                if (conv_idx == 0 && conv_bit == 0) begin
                    fastest_r <= (m01 < m23) ? m01 : m23;
                    for (k = 0; k < 16; k = k + 1)
                        lt_r[k] <= (keys[(k % 4)*(BIN_W+4) +: BIN_W+4] < keys[(k / 4)*(BIN_W+4) +: BIN_W+4]);
                end
                if (conv_idx == 0 && conv_bit == BIN_W) begin
                    snap_order <= order_next;
                    for (k = 0; k < 4; k = k + 1)
                        raw[12 + k] <= sfin[k] ? raw[k] - fastest_r : {BIN_W{1'b0}};
                end
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
                        else if (conv_idx < 12)
                            cost_bcd[(conv_idx-8)*16 +: 16] <= bcd_next[15:0];
                        else
                            gap_bcd[(conv_idx-12)*24 +: 24] <= bcd_next;
                        if (conv_idx == NVAL-1) conv_busy <= 1'b0;
                        else                conv_idx  <= conv_idx + 1'b1;
                    end
                end
            end
        end
    end
endmodule
