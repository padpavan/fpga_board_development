`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_perf_sampler : measures a demo run for the OLED graph (CONTRACT 8a)
//
// Every 2^plog step_en cycles it stores one word holding each engine's plot
//   level = floor(LEVEL_MAX * (d0 - best) / d0)      d0 = distance start..goal
// (best from ast_perf_probe).  The divide is a multiply by a reciprocal:
//   ((d0 - best) * ceil(LEVEL_MAX * 512 / d0)) >> 9, exact for every d0 <= 26.
// Special cases: 0 until the first expansion; LEVEL_MAX once expanded if d0 = 0.
//
// Auto-scaling: logical sample i lives at address rotl(i, rot).  When sample
// SAMPLES is due, in one clock count SAMPLES -> SAMPLES/2, plog + 1, rot + 1.
// Even samples keep their address, because rotl(2j, rot) = rotl(j, rot + 1),
// and new samples land in the slots of the dropped odd ones, so no data moves.
//
// One multiplier serves the four engines in turn: best changes rarely and a
// level lags it by at most 8 clocks.  done is delayed by more than that, so an
// engine's end sample and the flush sample always hold its final level.
//
// When every selected engine is done (even with step_en = 0: an abandoned run)
// a last sample is written and the sampler ignores the engines until the next
// start, so GAME searches never reach the plot.
//
// The tick counter is 16 bits: enough for 94 * 2^16 cycles, far beyond any run.
//////////////////////////////////////////////////////////////////////////////
module ast_perf_sampler #(
    parameter INDEX_W   = `ARENA_INDEX_W,
    parameter DIST_W    = `ARENA_DIST_W,
    parameter AW        = `ARENA_PG_ADDR_W,
    parameter RW        = `ARENA_PG_ROT_W,
    parameter LW        = `ARENA_PG_LEVEL_W,
    parameter BW        = `ARENA_PG_BEST_W,
    parameter SAMPLES   = `ARENA_PG_SAMPLES,
    parameter LEVEL_MAX = `ARENA_PG_LEVEL_MAX
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 step_en,      // demo step enable (pauses excluded)
    input  wire                 start,        // 1-cycle: a demo run starts
    input  wire [3:0]           run_mask,     // {dfs, ast, dij, bfs}
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,
    input  wire [4*INDEX_W-1:0] current,      // {dfs, ast, dij, bfs}
    input  wire [4*DIST_W-1:0]  expansions,
    input  wire [3:0]           done,

    // history RAM write port (to ast_perf_hist)
    output reg                  we = 1'b0,
    output reg  [AW-1:0]        waddr = 0,
    output reg  [4*LW-1:0]      wdata = 0,    // one level per engine

    // what the plot shows; these change on the clock edge that lands the write
    output reg  [AW-1:0]        count = 0,    // samples valid, 0 = empty plot
    output reg  [RW-1:0]        rot = 0,
    output reg  [4*AW-1:0]      end_idx = 0,  // sample at which each engine finished
    output reg  [3:0]           end_valid = 0,
    output reg  [3:0]           mask = 0,     // engines in this run
    output reg  [4:0]           plog = 0,     // cycles per sample = 2^plog

    output wire [4*BW-1:0]      dbg_best
);
    localparam TICK_W   = 16;
    localparam DONE_DLY = 10;                 // > the level lag above
    localparam SHIFT    = 9;
    localparam KW       = 15;                 // K[1] = 46 * 512 = 23552
    localparam [BW-1:0] NO_BEST = {BW{1'b1}};

    // ---- input registers (all together, so they stay aligned) --------------
    reg               start_q = 1'b0, step_q = 1'b0;
    reg [3:0]         done_q = 0, nz_q = 0, run_mask_q = 0;
    reg [INDEX_W-1:0] s_q = 0, g_q = 0;
    reg [4*INDEX_W-1:0] cur_q = 0;
    integer i;
    always @(posedge clk) begin
        start_q    <= start && !rst;
        step_q     <= step_en;
        done_q     <= done;
        run_mask_q <= run_mask;
        s_q        <= start_idx;
        g_q        <= goal_idx;
        cur_q      <= current;
        for (i = 0; i < 4; i = i + 1)
            nz_q[i] <= |expansions[i*DIST_W +: DIST_W];
    end

    // ---- run state ---------------------------------------------------------
    reg              running = 1'b0;
    reg [INDEX_W-1:0] goal_r = 0;
    reg [BW-1:0]     d0 = 0;
    wire [DIST_W-1:0] d0_full;
    manhattan u_d0 (.a_idx(s_q), .b_idx(g_q), .h(d0_full));

    // ---- probes ------------------------------------------------------------
    wire [4*BW-1:0] best;
    assign dbg_best = best;
    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : g_probe
            ast_perf_probe u_probe (
                .clk(clk), .rst(rst), .clr(start_q),
                .en(running && mask[g] && nz_q[g]),
                .current(cur_q[g*INDEX_W +: INDEX_W]), .goal_idx(goal_r),
                .best(best[g*BW +: BW])
            );
        end
    endgenerate

    // ---- best -> level, shared multiplier, one engine per clock -------------
    // K[d0] = ceil(LEVEL_MAX * 512 / d0); d0 = 0 uses K[1] with n = 1
    function [KW-1:0] k_of;
        input [BW-1:0] d;
        integer n;
        begin
            k_of = LEVEL_MAX << SHIFT;
            for (n = 2; n <= `ARENA_COLS + `ARENA_ROWS - 2; n = n + 1)
                if (d == n) k_of = ((LEVEL_MAX << SHIFT) + n - 1) / n;
        end
    endfunction

    reg [1:0]        rr = 0, rr1 = 0, rr2 = 0;
    reg              v1 = 1'b0, v2 = 1'b0;
    reg [KW-1:0]     k_r = 0;
    reg [BW-1:0]     n_r = 0;
    reg [BW+KW-1:0]  prod = 0;
    reg [4*LW-1:0]   levels = 0;
    wire [BW-1:0]    b_sel = best[rr*BW +: BW];
    wire [BW-1:0]    n_sel = (b_sel == NO_BEST) ? {BW{1'b0}} :
                             (d0 == 0)          ? {{(BW-1){1'b0}}, 1'b1} : d0 - b_sel;
    always @(posedge clk) begin
        rr   <= rr + 1'b1;
        k_r  <= k_of(d0);
        n_r  <= n_sel;
        rr1  <= rr;
        prod <= n_r * k_r;
        rr2  <= rr1;
        if (rst || start_q) begin
            v1 <= 1'b0; v2 <= 1'b0; levels <= 0;
        end else begin
            v1 <= running;
            v2 <= v1;
            if (v2) levels[rr2*LW +: LW] <= prod[SHIFT +: LW];
        end
    end

    // ---- done, delayed so the levels have settled -----------------------------
    reg  [DONE_DLY-1:0] done_sh [0:3];
    wire [3:0] done_d = {done_sh[3][DONE_DLY-1], done_sh[2][DONE_DLY-1],
                         done_sh[1][DONE_DLY-1], done_sh[0][DONE_DLY-1]};
    always @(posedge clk)
        for (i = 0; i < 4; i = i + 1)
            done_sh[i] <= (rst || start_q || !running) ? {DONE_DLY{1'b0}}
                                                       : {done_sh[i][DONE_DLY-2:0], done_q[i]};

    // ---- sampling ----------------------------------------------------------
    reg [4:0]        plog_i = 0;
    reg [RW-1:0]     rot_i = 0;
    reg [AW-1:0]     count_i = 0;
    reg [4*AW-1:0]   end_i = 0;
    reg [3:0]        end_v = 0;
    reg [TICK_W-1:0] tick = 0;

    wire all_done = running && (&(done_d | ~mask));
    wire due      = running && step_q && (tick == 0);
    wire take     = due || all_done;
    wire halve    = take && (count_i == SAMPLES);

    wire [AW-1:0] count_n = halve ? (SAMPLES / 2) : count_i;
    wire [RW-1:0] rot_n   = !halve ? rot_i : (rot_i == AW - 1) ? {RW{1'b0}} : rot_i + 1'b1;
    wire [4:0]    plog_n  = halve ? plog_i + 1'b1 : plog_i;
    wire [2*AW-1:0] rot_dbl = {count_n, count_n};
    wire [2*AW-1:0] rot_sh  = rot_dbl >> (AW - rot_n);

    always @(posedge clk) begin
        if (rst) begin
            running <= 1'b0;
            we <= 1'b0; waddr <= 0; wdata <= 0;
            count <= 0; rot <= 0; end_idx <= 0; end_valid <= 0; mask <= 0; plog <= 0;
            plog_i <= 0; rot_i <= 0; count_i <= 0; end_i <= 0; end_v <= 0; tick <= 0;
            goal_r <= 0; d0 <= 0;
        end else if (start_q) begin
            running <= 1'b1;
            we <= 1'b0;
            count <= 0; rot <= 0; end_valid <= 0; mask <= run_mask_q; plog <= 0;
            plog_i <= 0; rot_i <= 0; count_i <= 0; end_v <= 0; tick <= 0;
            goal_r <= g_q; d0 <= d0_full[BW-1:0];
        end else begin
            we <= take;
            if (take) begin
                waddr   <= rot_sh[AW-1:0];
                wdata   <= levels;
                count_i <= count_n + 1'b1;
                rot_i   <= rot_n;
                plog_i  <= plog_n;
                for (i = 0; i < 4; i = i + 1) begin
                    if (end_v[i]) begin
                        if (halve) end_i[i*AW +: AW] <= (end_i[i*AW +: AW] + 1'b1) >> 1;
                    end else if (mask[i] && done_d[i]) begin
                        end_i[i*AW +: AW] <= count_n;
                        end_v[i]          <= 1'b1;
                    end
                end
            end
            if (running && step_q)
                tick <= (tick != 0) ? tick - 1'b1
                                    : ({{(TICK_W-1){1'b0}}, 1'b1} << plog_n) - 1'b1;
            if (all_done) running <= 1'b0;
            // published as the RAM write lands
            count     <= count_i;
            rot       <= rot_i;
            plog      <= plog_i;
            end_idx   <= end_i;
            end_valid <= end_v;
        end
    end
endmodule
