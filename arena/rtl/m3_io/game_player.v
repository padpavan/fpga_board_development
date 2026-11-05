`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// game_player : Pac-Man style grid movement.
//
//   btn_*  : 1-cycle press pulses (debounce_btn).  A press sets the wanted
//            direction; it stays queued until it can be taken (or another
//            press replaces it).
//   tick   : 1-cycle move enable (one cell per tick at most).  On a tick:
//              1. if a turn is queued and that neighbour is open, turn and
//                 step (the queue clears);
//              2. else if moving and the cell ahead is open, step;
//              3. else stop (at a wall).
//   spawn  : puts the player on spawn_pos, stopped, facing left, nothing
//            queued (level start / respawn).
//   moved  : 1-cycle pulse after every step; prev_pos is the cell left.
// The player never enters a wall or leaves the grid.
//////////////////////////////////////////////////////////////////////////////
module game_player #(
    parameter NUM_CELLS = `ARENA_NUM_CELLS, parameter INDEX_W = `ARENA_INDEX_W
) (
    input  wire                 clk, input wire rst,
    input  wire [NUM_CELLS-1:0] open,
    input  wire                 spawn,
    input  wire [INDEX_W-1:0]   spawn_pos,
    input  wire                 btn_up, input wire btn_down, input wire btn_left, input wire btn_right,
    input  wire                 tick,
    output reg  [INDEX_W-1:0]   pos,
    output reg  [INDEX_W-1:0]   prev_pos,
    output reg  [1:0]           dir,      // facing: 0 up, 1 down, 2 left, 3 right
    output reg                  moving,
    output reg                  moved
);
    reg       want_v;
    reg [1:0] want;

    wire [INDEX_W-1:0] nb_want, nb_dir;
    wire               in_want, in_dir;
    game_step u_want (.pos(pos), .dir(want), .nb(nb_want), .ok(in_want));
    game_step u_dir  (.pos(pos), .dir(dir),  .nb(nb_dir),  .ok(in_dir));
    wire can_want = want_v && in_want && open[nb_want];
    wire can_dir  = moving && in_dir  && open[nb_dir];

    always @(posedge clk) begin
        moved <= 1'b0;
        if (rst) begin
            pos <= 0; prev_pos <= 0; dir <= 2'd2; moving <= 1'b0; want_v <= 1'b0; want <= 2'd2;
        end else if (spawn) begin
            pos <= spawn_pos; prev_pos <= spawn_pos; dir <= 2'd2; moving <= 1'b0; want_v <= 1'b0;
        end else begin
            if (btn_up)         begin want <= 2'd0; want_v <= 1'b1; end
            else if (btn_down)  begin want <= 2'd1; want_v <= 1'b1; end
            else if (btn_left)  begin want <= 2'd2; want_v <= 1'b1; end
            else if (btn_right) begin want <= 2'd3; want_v <= 1'b1; end
            if (tick) begin
                if (can_want) begin
                    dir <= want; prev_pos <= pos; pos <= nb_want; moving <= 1'b1; moved <= 1'b1;
                    // a press in this same cycle stays queued
                    if (!(btn_up || btn_down || btn_left || btn_right)) want_v <= 1'b0;
                end else if (can_dir) begin
                    prev_pos <= pos; pos <= nb_dir; moved <= 1'b1;
                end else
                    moving <= 1'b0;
            end
        end
    end
endmodule
