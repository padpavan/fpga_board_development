`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Member 1 - top module (graph storage + BFS + Dijkstra engines)
//
// Instantiates the BFS and Dijkstra engines and fans out the shared maze-load
// and start/goal control so both engines receive identical frozen-maze data
// and launch concurrently on one `start` pulse.
//
// Member 3 integrates this module into the system top: it drives
// load_we/load_addr/load_data (maze snapshot), then start_idx/goal_idx and a
// single `start` pulse, and reads the per-engine status / path / stats ports.
//////////////////////////////////////////////////////////////////////////////
module pathfind_top #(
    parameter COLS    = 16,
    parameter ROWS    = 12,
    parameter COST_W  = 4,
    parameter DIST_W  = 16,
    parameter INDEX_W = 8
) (
    input  wire                 clk,
    input  wire                 rst,

    // shared maze load interface
    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    // shared run control
    input  wire                 start,
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    // BFS status
    output wire                 bfs_busy,
    output wire                 bfs_done,
    output wire                 bfs_found,
    output wire                 bfs_no_path,

    // BFS path readback
    input  wire [INDEX_W-1:0]   bfs_path_rd_addr,
    output wire [INDEX_W-1:0]   bfs_path_rd_data,
    output wire [DIST_W-1:0]    bfs_path_cells,
    output wire [DIST_W-1:0]    bfs_path_cost,

    // BFS live view + stats
    output wire [INDEX_W-1:0]   bfs_current,
    output wire [DIST_W-1:0]    bfs_cycles,
    output wire [DIST_W-1:0]    bfs_expansions,

    // Dijkstra status
    output wire                 dij_busy,
    output wire                 dij_done,
    output wire                 dij_found,
    output wire                 dij_no_path,

    // Dijkstra path readback
    input  wire [INDEX_W-1:0]   dij_path_rd_addr,
    output wire [INDEX_W-1:0]   dij_path_rd_data,
    output wire [DIST_W-1:0]    dij_path_cells,
    output wire [DIST_W-1:0]    dij_path_cost,

    // Dijkstra live view + stats
    output wire [INDEX_W-1:0]   dij_current,
    output wire [DIST_W-1:0]    dij_cycles,
    output wire [DIST_W-1:0]    dij_expansions
);
    bfs_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W)
    ) u_bfs (
        .clk(clk), .rst(rst),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(bfs_busy), .done(bfs_done), .found(bfs_found), .no_path(bfs_no_path),
        .path_rd_addr(bfs_path_rd_addr), .path_rd_data(bfs_path_rd_data),
        .path_cells(bfs_path_cells), .path_cost(bfs_path_cost),
        .current(bfs_current),
        .cycles(bfs_cycles), .expansions(bfs_expansions)
    );

    dijkstra_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W)
    ) u_dijkstra (
        .clk(clk), .rst(rst),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(dij_busy), .done(dij_done), .found(dij_found), .no_path(dij_no_path),
        .path_rd_addr(dij_path_rd_addr), .path_rd_data(dij_path_rd_data),
        .path_cells(dij_path_cells), .path_cost(dij_path_cost),
        .current(dij_current),
        .cycles(dij_cycles), .expansions(dij_expansions)
    );
endmodule
