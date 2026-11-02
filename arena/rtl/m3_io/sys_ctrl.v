`timescale 1ns / 1ps
`include "arena_defs.vh"
// EDIT -> LOAD -> START -> RUN/PAUSE -> DONE controller and maze edit bus.
module sys_ctrl #(
    parameter NUM_CELLS=`ARENA_NUM_CELLS, parameter INDEX_W=`ARENA_INDEX_W,
    parameter COST_W=`ARENA_COST_W, parameter SPEED_W=23
) (
    input wire clk, input wire rst,
    input wire run_pulse, input wire pause_pulse, input wire clear_pulse, input wire random_pulse,
    input wire [2:0] speed_sel,
    input wire mouse_left, input wire mouse_right, input wire [INDEX_W-1:0] mouse_idx,
    input wire [1:0] edit_tool,
    input wire gen_busy, input wire gen_done,
    input wire [2:0] engines_done,
    output reg gen_start, output reg gen_random,
    output reg maze_we, output reg [INDEX_W-1:0] maze_addr, output reg [COST_W-1:0] maze_data,
    output reg load_we, output reg [INDEX_W-1:0] load_addr,
    output reg start_pulse, output reg step_en, output reg edit_mode,
    output reg [INDEX_W-1:0] start_idx, output reg [INDEX_W-1:0] goal_idx,
    output reg [2:0] mode, output reg paused
);
    localparam [2:0] M_EDIT=3'd0, M_GEN=3'd1, M_LOAD=3'd2, M_START=3'd3,
                     M_RUN=3'd4, M_PAUSE=3'd5, M_DONE=3'd6, M_GEN_REQ=3'd7;
    reg [INDEX_W-1:0] load_ptr;
    reg [SPEED_W-1:0] speed_count;
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
        if (mode == M_EDIT && (mouse_left || mouse_right) &&
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
            load_ptr <= 0; start_idx <= 0; goal_idx <= NUM_CELLS-1;
            speed_count <= 0;
        end else begin
            speed_count <= speed_count + 1'b1;
            if (mode != M_RUN || paused || speed_sel == 0 || speed_count == speed_limit) speed_count <= 0;
            if (mode == M_EDIT && mouse_left) begin
                case (edit_tool)
                    2'b01: start_idx <= mouse_idx;
                    2'b10: goal_idx <= mouse_idx;
                    default: begin end
                endcase
            end
            case (mode)
                M_GEN_REQ: mode <= M_GEN;
                M_GEN: if (!gen_busy && gen_done) mode <= M_EDIT;
                M_EDIT: begin
                    if (clear_pulse || random_pulse) begin
                        gen_random <= random_pulse; mode <= M_GEN_REQ;
                    end else if (run_pulse) begin load_ptr <= 0; mode <= M_LOAD; end
                end
                M_LOAD: if (load_ptr == NUM_CELLS-1) mode <= M_START;
                        else load_ptr <= load_ptr + 1'b1;
                M_START: begin paused <= 1'b0; mode <= M_RUN; end
                M_RUN: begin
                    if (pause_pulse) begin paused <= 1'b1; mode <= M_PAUSE; end
                    else if (&engines_done) mode <= M_DONE;
                end
                M_PAUSE: if (pause_pulse) begin paused <= 1'b0; mode <= M_RUN; end
                M_DONE: if (clear_pulse || random_pulse) begin gen_random <= random_pulse; mode <= M_GEN_REQ; end
                        else if (run_pulse) begin load_ptr <= 0; mode <= M_LOAD; end
                default: mode <= M_EDIT;
            endcase
        end
    end
endmodule
