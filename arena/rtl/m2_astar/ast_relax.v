`timescale 1ns / 1ps
`include "arena_defs.vh"

// A* engine
module ast_relax #(
    parameter COST_W = `ARENA_COST_W,
    parameter DIST_W = `ARENA_DIST_W
) (
    input  wire [DIST_W-1:0]   g_u,
    input  wire [COST_W-1:0]   cost_v,
    input  wire                v_valid,
    input  wire [1:0]          cs_v,
    input  wire [DIST_W-1:0]   g_v_old,
    input  wire [DIST_W-1:0]   h_v,

    output wire [DIST_W-1:0]   g_new,
    output wire [DIST_W-1:0]   f_new,
    output wire                improve
);
    localparam [COST_W-1:0] COST_WALL  = `ARENA_COST_WALL;
    localparam [1:0]        CS_UNSEEN  = `ARENA_CS_UNSEEN,
                            CS_VISITED = `ARENA_CS_VISITED,
                            CS_PATH    = `ARENA_CS_PATH;

    wire v_seen   = (cs_v != CS_UNSEEN);
    wire v_closed = (cs_v == CS_VISITED) || (cs_v == CS_PATH);
    wire v_wall   = (cost_v == COST_WALL);

    assign g_new = g_u + {{(DIST_W-COST_W){1'b0}}, cost_v};
    assign f_new = g_new + h_v;

    assign improve = v_valid && !v_wall && !v_closed &&
                     (!v_seen || (g_new < g_v_old));
endmodule
