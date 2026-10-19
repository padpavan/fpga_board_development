`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Member 2 - A* engine  *** STUB ***
//
// Placeholder with the exact CONTRACT.md section 4 port list, so that
// pathfind_top and arena_top compile before the real engine exists.
// Behaviour: every accepted `start` finishes one cycle later with
// done = 1, no_path = 1 and all statistics 0.
//
// Replace the body with the real A* engine; do NOT change the port list.
//////////////////////////////////////////////////////////////////////////////
module astar_engine #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter COST_W  = `ARENA_COST_W,
    parameter DIST_W  = `ARENA_DIST_W,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter CNT_W   = `ARENA_CNT_W
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 step_en,

    // maze load (only while not busy)
    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    // run control
    input  wire                 start,
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    // status
    output reg                  busy,
    output reg                  done,
    output reg                  found,
    output reg                  no_path,

    // path readback
    input  wire [INDEX_W-1:0]   path_rd_addr,
    output wire [INDEX_W-1:0]   path_rd_data,
    output wire [DIST_W-1:0]    path_cells,
    output wire [DIST_W-1:0]    path_cost,

    // live view
    output wire [INDEX_W-1:0]   current,
    input  wire [INDEX_W-1:0]   cell_rd_addr,
    output wire [1:0]           cell_state,

    // statistics
    output wire [CNT_W-1:0]     cycles,
    output wire [DIST_W-1:0]    expansions
);
    assign path_rd_data = `ARENA_IDX_NONE;
    assign path_cells   = 0;
    assign path_cost    = 0;
    assign current      = `ARENA_IDX_NONE;
    assign cell_state   = `ARENA_CS_UNSEEN;
    assign cycles       = 0;
    assign expansions   = 0;

    always @(posedge clk) begin
        if (rst) begin
            busy    <= 1'b0;
            done    <= 1'b0;
            found   <= 1'b0;
            no_path <= 1'b0;
        end else if (start && !busy) begin
            busy    <= 1'b1;
            done    <= 1'b0;
            found   <= 1'b0;
            no_path <= 1'b0;
        end else if (busy) begin
            busy    <= 1'b0;
            done    <= 1'b1;
            no_path <= 1'b1;
        end
    end
endmodule
