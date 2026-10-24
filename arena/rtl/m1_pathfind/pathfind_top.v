`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Member 1 - engine cluster (CONTRACT.md section 5)
//
// Instantiates the BFS, Dijkstra and A* engines and fans out the shared
// maze-load, run control and cell_rd_addr so all three engines receive
// identical frozen-maze data and launch concurrently on one `start` pulse.
//
// Every other section 4 port is per engine, prefixed bfs_ / dij_ / ast_.
// M3 integrates only this module.  M2 owns the u_astar instance.
//////////////////////////////////////////////////////////////////////////////
module pathfind_top #(
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

    // shared maze load interface
    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    // shared run control
    input  wire                 start,
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    // shared live-view address
    input  wire [INDEX_W-1:0]   cell_rd_addr,

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
    output wire [1:0]           bfs_cell_state,
    output wire [CNT_W-1:0]     bfs_cycles,
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
    output wire [1:0]           dij_cell_state,
    output wire [CNT_W-1:0]     dij_cycles,
    output wire [DIST_W-1:0]    dij_expansions,

    // A* status
    output wire                 ast_busy,
    output wire                 ast_done,
    output wire                 ast_found,
    output wire                 ast_no_path,

    // A* path readback
    input  wire [INDEX_W-1:0]   ast_path_rd_addr,
    output wire [INDEX_W-1:0]   ast_path_rd_data,
    output wire [DIST_W-1:0]    ast_path_cells,
    output wire [DIST_W-1:0]    ast_path_cost,

    // A* live view + stats
    output wire [INDEX_W-1:0]   ast_current,
    output wire [1:0]           ast_cell_state,
    output wire [CNT_W-1:0]     ast_cycles,
    output wire [DIST_W-1:0]    ast_expansions,

    // A* debug (CONTRACT 0: optional dbg_ output, may be left unconnected)
    output wire [3:0]           ast_dbg_state
);
    bfs_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) u_bfs (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(bfs_busy), .done(bfs_done), .found(bfs_found), .no_path(bfs_no_path),
        .path_rd_addr(bfs_path_rd_addr), .path_rd_data(bfs_path_rd_data),
        .path_cells(bfs_path_cells), .path_cost(bfs_path_cost),
        .current(bfs_current),
        .cell_rd_addr(cell_rd_addr), .cell_state(bfs_cell_state),
        .cycles(bfs_cycles), .expansions(bfs_expansions)
    );

    dijkstra_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) u_dijkstra (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(dij_busy), .done(dij_done), .found(dij_found), .no_path(dij_no_path),
        .path_rd_addr(dij_path_rd_addr), .path_rd_data(dij_path_rd_data),
        .path_cells(dij_path_cells), .path_cost(dij_path_cost),
        .current(dij_current),
        .cell_rd_addr(cell_rd_addr), .cell_state(dij_cell_state),
        .cycles(dij_cycles), .expansions(dij_expansions)
    );

    astar_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) u_astar (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(ast_busy), .done(ast_done), .found(ast_found), .no_path(ast_no_path),
        .path_rd_addr(ast_path_rd_addr), .path_rd_data(ast_path_rd_data),
        .path_cells(ast_path_cells), .path_cost(ast_path_cost),
        .current(ast_current),
        .cell_rd_addr(cell_rd_addr), .cell_state(ast_cell_state),
        .cycles(ast_cycles), .expansions(ast_expansions),
        .dbg_state(ast_dbg_state)
    );
endmodule
