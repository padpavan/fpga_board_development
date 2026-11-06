`timescale 1ns / 1ps
`include "arena_defs.vh"
// EDIT -> LOAD -> START -> RUN/PAUSE -> DONE controller and maze edit bus.
// run_sel (sw[5:2]) picks the engines; it is latched into run_mask when BTNC
// starts a run (a run with nothing selected is ignored). DONE = every
// selected engine is done.
// wipe_pulse (short BTNU): from EDIT, RUN, PAUSE or DONE, clears the maze and
// puts start and goal back at their defaults.  back_pulse (BTNR): from RUN,
// PAUSE or DONE, returns to EDIT with the maze and markers untouched.
// wipe_accept = a wipe was taken; abort_pulse = a run was stopped by either
// button.  arena_top uses both to reset the engines, which would otherwise stay
// busy.
// hold = 1 (GAME mode owns the board, CONTRACT v1.7): a demo run is abandoned
// (LOAD..DONE -> EDIT), and EDIT ignores buttons and mouse edits, so the maze
// and the markers stay frozen.  A maze generation in progress still finishes.
module sys_ctrl #(
    parameter NUM_CELLS=`ARENA_NUM_CELLS, parameter INDEX_W=`ARENA_INDEX_W,
    parameter COST_W=`ARENA_COST_W, parameter SPEED_W=23
) (
    input wire clk, input wire rst,
    input wire run_pulse, input wire pause_pulse, input wire wipe_pulse, input wire back_pulse, input wire random_pulse,
    input wire [2:0] speed_sel,
    input wire mouse_left, input wire mouse_right, input wire [INDEX_W-1:0] mouse_idx,
    input wire [1:0] edit_tool,
    input wire gen_busy, input wire gen_done,
    input wire [3:0] engines_done,   // {dfs, ast, dij, bfs}
    input wire [3:0] run_sel,        // {dfs, ast, dij, bfs} switches
    input wire hold,                 // GAME mode: freeze the demo
    output reg gen_start, output reg gen_random,
    output reg maze_we, output reg [INDEX_W-1:0] maze_addr, output reg [COST_W-1:0] maze_data,
    output reg load_we, output reg [INDEX_W-1:0] load_addr,
    output reg start_pulse, output reg step_en, output reg edit_mode,
    output reg [INDEX_W-1:0] start_idx, output reg [INDEX_W-1:0] goal_idx,
    output reg [2:0] mode, output reg paused,
    output reg [3:0] run_mask,       // engines in the current / last run
    output wire wipe_accept,         // wipe_pulse taken this cycle
    output wire abort_pulse          // wipe or back stopped a run this cycle
);
    localparam [2:0] M_EDIT=3'd0, M_GEN=3'd1, M_LOAD=3'd2, M_START=3'd3,
                     M_RUN=3'd4, M_PAUSE=3'd5, M_DONE=3'd6, M_GEN_REQ=3'd7;
    localparam [INDEX_W-1:0] START_DEFAULT = 0, GOAL_DEFAULT = NUM_CELLS-1;
    reg [INDEX_W-1:0] load_ptr;
    reg [SPEED_W-1:0] speed_count;
    wire run_ok = run_pulse && (run_sel != 4'b0000);
    wire all_sel_done = &(engines_done | ~run_mask);
    wire in_run = (mode == M_RUN) || (mode == M_PAUSE);
    wire wipe_ok = wipe_pulse && (in_run || mode == M_EDIT || mode == M_DONE);
    wire back_ok = back_pulse && (in_run || mode == M_DONE);
    assign wipe_accept = wipe_ok && !hold;
    assign abort_pulse = (wipe_pulse || back_pulse) && in_run && !hold;
    wire [SPEED_W-1:0] speed_limit = (speed_sel == 0) ? 0 : ((1 << (speed_sel + 10))-1);
    always @* begin
        edit_mode = (mode == M_EDIT);
        load_we = (mode == M_LOAD);
        load_addr = load_ptr;
        start_pulse = (mode == M_START);
        step_en = ((mode == M_RUN) && !paused && ((speed_sel == 0) || (speed_count == speed_limit))) ||
                  ((mode == M_PAUSE) && 1'b0);
        maze_we = 1'b0; maze_addr = mouse_idx; maze_data = `ARENA_COST_OPEN;
        gen_start = (mode == M_GEN_REQ);
        if (mode == M_EDIT && !hold && (mouse_left || mouse_right) &&
            (mouse_idx != start_idx) && (mouse_idx != goal_idx)) begin
            maze_we = 1'b1;
            if (mouse_right) maze_data = `ARENA_COST_OPEN;
            else case (edit_tool)
                2'b00: maze_data = `ARENA_COST_WALL;
                2'b11: maze_data = {{(COST_W-2){1'b0}},2'b11};
                default: maze_data = `ARENA_COST_OPEN;
            endcase
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            mode <= M_GEN_REQ; gen_random <= 1'b0; paused <= 1'b0;
            load_ptr <= 0; start_idx <= START_DEFAULT; goal_idx <= GOAL_DEFAULT;
            speed_count <= 0; run_mask <= 4'b0000;
        end else begin
            speed_count <= speed_count + 1'b1;
            if (mode != M_RUN || paused || speed_sel == 0 || speed_count == speed_limit) speed_count <= 0;
            if (mode == M_EDIT && !hold && mouse_left) begin
                case (edit_tool)
                    2'b01: start_idx <= mouse_idx;
                    2'b10: goal_idx <= mouse_idx;
                    default: begin end
                endcase
            end
            if (hold && mode != M_GEN_REQ && mode != M_GEN) begin
                mode <= M_EDIT; paused <= 1'b0;
            end else if (wipe_ok) begin
                start_idx <= START_DEFAULT; goal_idx <= GOAL_DEFAULT;
                gen_random <= 1'b0; paused <= 1'b0; mode <= M_GEN_REQ;
            end else if (back_ok) begin
                paused <= 1'b0; mode <= M_EDIT;
            end else case (mode)
                M_GEN_REQ: mode <= M_GEN;
                M_GEN: if (!gen_busy && gen_done) mode <= M_EDIT;
                M_EDIT: begin
                    if (random_pulse) begin
                        gen_random <= 1'b1; mode <= M_GEN_REQ;
                    end else if (run_ok) begin load_ptr <= 0; run_mask <= run_sel; mode <= M_LOAD; end
                end
                M_LOAD: if (load_ptr == NUM_CELLS-1) mode <= M_START;
                        else load_ptr <= load_ptr + 1'b1;
                M_START: begin paused <= 1'b0; mode <= M_RUN; end
                M_RUN: begin
                    if (pause_pulse) begin paused <= 1'b1; mode <= M_PAUSE; end
                    else if (all_sel_done) mode <= M_DONE;
                end
                M_PAUSE: if (pause_pulse) begin paused <= 1'b0; mode <= M_RUN; end
                M_DONE: if (random_pulse) begin gen_random <= 1'b1; mode <= M_GEN_REQ; end
                        else if (run_ok) begin load_ptr <= 0; run_mask <= run_sel; mode <= M_LOAD; end
                default: mode <= M_EDIT;
            endcase
        end
    end
endmodule
