`timescale 1ns / 1ps
`include "arena_defs.vh"

// Search engines
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

    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    input  wire                 start,
    input  wire [3:0]           run_mask,
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    input  wire [INDEX_W-1:0]   cell_rd_addr,

    output wire                 bfs_busy,
    output wire                 bfs_done,
    output wire                 bfs_found,
    output wire                 bfs_no_path,

    input  wire [INDEX_W-1:0]   bfs_path_rd_addr,
    output wire [INDEX_W-1:0]   bfs_path_rd_data,
    output wire [DIST_W-1:0]    bfs_path_cells,
    output wire [DIST_W-1:0]    bfs_path_cost,

    output wire [INDEX_W-1:0]   bfs_current,
    output wire [1:0]           bfs_cell_state,
    output wire [CNT_W-1:0]     bfs_cycles,
    output wire [DIST_W-1:0]    bfs_expansions,

    output wire                 dij_busy,
    output wire                 dij_done,
    output wire                 dij_found,
    output wire                 dij_no_path,

    input  wire [INDEX_W-1:0]   dij_path_rd_addr,
    output wire [INDEX_W-1:0]   dij_path_rd_data,
    output wire [DIST_W-1:0]    dij_path_cells,
    output wire [DIST_W-1:0]    dij_path_cost,

    output wire [INDEX_W-1:0]   dij_current,
    output wire [1:0]           dij_cell_state,
    output wire [CNT_W-1:0]     dij_cycles,
    output wire [DIST_W-1:0]    dij_expansions,

    output wire                 ast_busy,
    output wire                 ast_done,
    output wire                 ast_found,
    output wire                 ast_no_path,

    input  wire [INDEX_W-1:0]   ast_path_rd_addr,
    output wire [INDEX_W-1:0]   ast_path_rd_data,
    output wire [DIST_W-1:0]    ast_path_cells,
    output wire [DIST_W-1:0]    ast_path_cost,

    output wire [INDEX_W-1:0]   ast_current,
    output wire [1:0]           ast_cell_state,
    output wire [CNT_W-1:0]     ast_cycles,
    output wire [DIST_W-1:0]    ast_expansions,

    output wire [3:0]           ast_dbg_state,

    output wire                 dfs_busy,
    output wire                 dfs_done,
    output wire                 dfs_found,
    output wire                 dfs_no_path,

    input  wire [INDEX_W-1:0]   dfs_path_rd_addr,
    output wire [INDEX_W-1:0]   dfs_path_rd_data,
    output wire [DIST_W-1:0]    dfs_path_cells,
    output wire [DIST_W-1:0]    dfs_path_cost,

    output wire [INDEX_W-1:0]   dfs_current,
    output wire [1:0]           dfs_cell_state,
    output wire [CNT_W-1:0]     dfs_cycles,
    output wire [DIST_W-1:0]    dfs_expansions
);
    bfs_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) u_bfs (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start && run_mask[0]), .start_idx(start_idx), .goal_idx(goal_idx),
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
        .start(start && run_mask[1]), .start_idx(start_idx), .goal_idx(goal_idx),
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
        .start(start && run_mask[2]), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(ast_busy), .done(ast_done), .found(ast_found), .no_path(ast_no_path),
        .path_rd_addr(ast_path_rd_addr), .path_rd_data(ast_path_rd_data),
        .path_cells(ast_path_cells), .path_cost(ast_path_cost),
        .current(ast_current),
        .cell_rd_addr(cell_rd_addr), .cell_state(ast_cell_state),
        .cycles(ast_cycles), .expansions(ast_expansions),
        .dbg_state(ast_dbg_state)
    );

    dfs_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) u_dfs (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start && run_mask[3]), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(dfs_busy), .done(dfs_done), .found(dfs_found), .no_path(dfs_no_path),
        .path_rd_addr(dfs_path_rd_addr), .path_rd_data(dfs_path_rd_data),
        .path_cells(dfs_path_cells), .path_cost(dfs_path_cost),
        .current(dfs_current),
        .cell_rd_addr(cell_rd_addr), .cell_state(dfs_cell_state),
        .cycles(dfs_cycles), .expansions(dfs_expansions)
    );
endmodule
