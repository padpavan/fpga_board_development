`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Member 2 - performance graph sampler (CONTRACT 8a)
//
// *** STUB: agreed port list only, outputs tied off (the graph shows
// "NO DATA YET").  M2 replaces the body with the real measurement. ***
//
// Job: during a demo run, measure each selected engine every 2^hist_plog
// step_en cycles and write the samples into M1's gv_hist_ram through
// hist_we / hist_eng / hist_addr / hist_data, then publish hist_count,
// hist_plog, hist_metric and the finish markers.  Rules in CONTRACT 8a.
//////////////////////////////////////////////////////////////////////////////
module ast_perf_sampler #(
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter DIST_W  = `ARENA_DIST_W,
    parameter AW      = `ARENA_GV_ADDR_W,
    parameter DW      = `ARENA_GV_DATA_W,
    parameter PW      = `ARENA_GV_PLOG_W
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 step_en,       // demo step enable (pauses excluded)
    input  wire                 start,         // 1-cycle: a demo run starts
    input  wire [3:0]           run_mask,      // {dfs, ast, dij, bfs}
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,
    input  wire [4*INDEX_W-1:0] current,       // {dfs, ast, dij, bfs}
    input  wire [4*DIST_W-1:0]  expansions,
    input  wire [3:0]           busy,
    input  wire [3:0]           done,
    input  wire [3:0]           found,
    input  wire                 metric_sel,    // sw[11]: 0 progress, 1 explored

    output wire                 hist_we,
    output wire [1:0]           hist_eng,
    output wire [AW-1:0]        hist_addr,
    output wire [DW-1:0]        hist_data,
    output wire [AW:0]          hist_count,
    output wire [PW-1:0]        hist_plog,
    output wire                 hist_metric,
    output wire [4*AW-1:0]      hist_end,
    output wire [3:0]           hist_end_valid
);
    assign hist_we        = 1'b0;
    assign hist_eng       = 2'd0;
    assign hist_addr      = {AW{1'b0}};
    assign hist_data      = {DW{1'b0}};
    assign hist_count     = {(AW+1){1'b0}};
    assign hist_plog      = {PW{1'b0}};
    assign hist_metric    = metric_sel;
    assign hist_end       = {(4*AW){1'b0}};
    assign hist_end_valid = 4'b0000;
endmodule
