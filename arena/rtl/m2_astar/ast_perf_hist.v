`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_perf_hist : the performance graph's sample history (CONTRACT 8a)
//
// ast_perf_sampler writes here.  A plot asks for logical sample rd_idx and gets
// every engine's level for that sample and for the one before it, so it can fill
// a column between the two and keep the curve continuous.  At rd_idx = 0 the two
// are the same.  Packed {dfs, ast, dij, bfs}, engine e at [e*LW +: LW].
//
// The sampler keeps logical sample i at address rotl(i, rot); the rotation is
// undone here, so a reader never sees it.  Only ask for rd_idx < count.
//
// Two LUTRAM copies give the two reads.  Written on clk (100 MHz) and read
// combinationally, so a 6.25 MHz reader can use it directly: a read during a
// write can at worst mis-colour one pixel for one frame.
//////////////////////////////////////////////////////////////////////////////
module ast_perf_hist #(
    parameter AW = `ARENA_PG_ADDR_W,
    parameter RW = `ARENA_PG_ROT_W,
    parameter LW = `ARENA_PG_LEVEL_W
) (
    input  wire            clk,
    input  wire            we,
    input  wire [AW-1:0]   waddr,
    input  wire [4*LW-1:0] wdata,
    input  wire [RW-1:0]   rot,

    input  wire [AW-1:0]   rd_idx,
    output wire [4*LW-1:0] rd_level,
    output wire [4*LW-1:0] rd_prev_level
);
    wire [AW-1:0]   prev_idx = (rd_idx == 0) ? {AW{1'b0}} : rd_idx - 1'b1;
    wire [2*AW-1:0] cur_sh   = {rd_idx, rd_idx} >> (AW - rot);      // rotl(rd_idx, rot)
    wire [2*AW-1:0] prev_sh  = {prev_idx, prev_idx} >> (AW - rot);

    grid_mem #(.ADDR_W(AW), .DATA_W(4*LW)) u_cur (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdata),
        .raddr(cur_sh[AW-1:0]), .rdata(rd_level)
    );
    grid_mem #(.ADDR_W(AW), .DATA_W(4*LW)) u_prev (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdata),
        .raddr(prev_sh[AW-1:0]), .rdata(rd_prev_level)
    );
endmodule
