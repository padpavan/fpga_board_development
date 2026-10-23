`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_relax : combinational A* node-evaluation datapath  (f = g + h)
//
// While the engine expands node u, it looks at each neighbour v in turn.
// This block answers, in the same cycle:
//
//     g_new   = g_u + cost_v        cost of reaching v through u
//     f_new   = g_new + h_v         A* priority of v via this route
//     improve = should the engine record this route?
//
//     improve = v_valid                  v is inside the grid
//             & cost_v != WALL           v is passable
//             & !closed(v)               v not already expanded
//             & ( !seen(v)               first route found to v, or
//               | g_new < g_v_old )      strictly cheaper than the old one
//
// seen / closed are decoded from v's 2-bit search state (CONTRACT 4.2):
//     UNSEEN   -> not seen
//     FRONTIER -> seen, open
//     VISITED  -> seen, closed
//     PATH     -> seen, closed (only appears after the search ends)
// g_v_old is only meaningful when v has been seen; it is ignored otherwise,
// which is why the g RAM never needs clearing.
//
// "Strictly cheaper" (<, not <=) keeps the FIRST equal-cost route, matching
// ref/arena_ref.py, so the hardware path is predictable.
//
// Width: the largest g is (NUM_CELLS-1) * 15 = 2865 on a 16x12 grid, plus
// h <= 26, so DIST_W = 16 bits can never overflow.
//////////////////////////////////////////////////////////////////////////////
module ast_relax #(
    parameter COST_W = `ARENA_COST_W,
    parameter DIST_W = `ARENA_DIST_W
) (
    input  wire [DIST_W-1:0]   g_u,       // g of the node being expanded
    input  wire [COST_W-1:0]   cost_v,    // cost of entering v (0 = wall)
    input  wire                v_valid,   // v is inside the grid
    input  wire [1:0]          cs_v,      // v's search state (ARENA_CS_*)
    input  wire [DIST_W-1:0]   g_v_old,   // best g recorded for v so far
    input  wire [DIST_W-1:0]   h_v,       // Manhattan(v, goal)

    output wire [DIST_W-1:0]   g_new,
    output wire [DIST_W-1:0]   f_new,
    output wire                improve
);
    // Shared codes from arena_defs.vh, given plain names for the code below.
    localparam [COST_W-1:0] COST_WALL  = `ARENA_COST_WALL;
    localparam [1:0]        CS_UNSEEN  = `ARENA_CS_UNSEEN,
                            CS_VISITED = `ARENA_CS_VISITED,
                            CS_PATH    = `ARENA_CS_PATH;

    // Decode the search state.
    wire v_seen   = (cs_v != CS_UNSEEN);
    wire v_closed = (cs_v == CS_VISITED) || (cs_v == CS_PATH);
    wire v_wall   = (cost_v == COST_WALL);

    // Arithmetic: zero-extend the 4-bit cost before adding.
    assign g_new = g_u + {{(DIST_W-COST_W){1'b0}}, cost_v};
    assign f_new = g_new + h_v;

    // The relaxation decision.
    assign improve = v_valid && !v_wall && !v_closed &&
                     (!v_seen || (g_new < g_v_old));
endmodule
