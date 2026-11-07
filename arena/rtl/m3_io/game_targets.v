`timescale 1ns / 1ps
`include "arena_defs.vh"

// Game mode
module game_targets #(
    parameter COLS = `ARENA_COLS, parameter ROWS = `ARENA_ROWS,
    parameter NUM_CELLS = `ARENA_NUM_CELLS, parameter INDEX_W = `ARENA_INDEX_W,
    parameter NEAR = 2
) (
    input  wire                   clk, input wire rst,
    input  wire [NUM_CELLS-1:0]   open,
    input  wire [NUM_CELLS-1:0]   reach,
    input  wire [INDEX_W-1:0]     player_pos,
    input  wire [1:0]             player_dir,
    input  wire [4*INDEX_W-1:0]   ghost_pos,
    output reg  [4*INDEX_W-1:0]   target
);
    reg  [INDEX_W-1:0] a1, a2, a3, b1, b2, b3;
    reg  [1:0]         dir1, dir2;
    wire [1:0]         back0 = player_dir ^ 2'd1, back1 = dir1 ^ 2'd1, back2 = dir2 ^ 2'd1;
    wire [INDEX_W-1:0] na1, na2, na3, nb1, nb2, nb3;
    wire               oa1, oa2, oa3, ob1, ob2, ob3;
    game_step sa1 (.pos(player_pos), .dir(player_dir), .nb(na1), .ok(oa1));
    game_step sa2 (.pos(a1), .dir(dir1), .nb(na2), .ok(oa2));
    game_step sa3 (.pos(a2), .dir(dir2), .nb(na3), .ok(oa3));
    game_step sb1 (.pos(player_pos), .dir(back0), .nb(nb1), .ok(ob1));
    game_step sb2 (.pos(b1), .dir(back1), .nb(nb2), .ok(ob2));
    game_step sb3 (.pos(b2), .dir(back2), .nb(nb3), .ok(ob3));

    wire [INDEX_W-1:0] prow = player_pos / COLS, pcol = player_pos % COLS;
    wire [INDEX_W-1:0] g0   = ghost_pos[0 +: INDEX_W];
    wire [INDEX_W-1:0] grow = g0 / COLS, gcol = g0 % COLS;
    wire signed [INDEX_W+1:0] vr = $signed({2'b00, prow, 1'b0} - {3'b000, grow});
    wire signed [INDEX_W+1:0] vc = $signed({2'b00, pcol, 1'b0} - {3'b000, gcol});
    wire [INDEX_W-1:0] mr = vr[INDEX_W+1] ? 0 : (vr > ROWS-1) ? ROWS-1 : vr[INDEX_W-1:0];
    wire [INDEX_W-1:0] mc = vc[INDEX_W+1] ? 0 : (vc > COLS-1) ? COLS-1 : vc[INDEX_W-1:0];
    reg  [INDEX_W-1:0] m1;

    function near; input [INDEX_W-1:0] a, b;
        reg [INDEX_W-1:0] ar, ac, br, bc, dr, dc;
        begin
            ar = a / COLS; ac = a % COLS; br = b / COLS; bc = b % COLS;
            dr = (ar > br) ? ar - br : br - ar;
            dc = (ac > bc) ? ac - bc : bc - ac;
            near = (dr + dc) <= NEAR;
        end
    endfunction
    function [INDEX_W-1:0] pick; input [INDEX_W-1:0] want, ghost, player;
        input [NUM_CELLS-1:0] o, r;
        pick = (want < NUM_CELLS && o[want] && r[want] && want != ghost && !near(ghost, player))
               ? want : player;
    endfunction

    integer g;
    reg [INDEX_W-1:0] want [0:3];
    always @* begin
        want[0] = player_pos; want[1] = a3; want[2] = b3; want[3] = m1;
    end

    always @(posedge clk) begin
        if (rst) begin
            a1 <= 0; a2 <= 0; a3 <= 0; b1 <= 0; b2 <= 0; b3 <= 0;
            dir1 <= 0; dir2 <= 0; m1 <= 0; target <= 0;
        end else begin
            dir1 <= player_dir; dir2 <= dir1;
            a1 <= (oa1 && open[na1]) ? na1 : player_pos;
            a2 <= (oa2 && open[na2]) ? na2 : a1;
            a3 <= (oa3 && open[na3]) ? na3 : a2;
            b1 <= (ob1 && open[nb1]) ? nb1 : player_pos;
            b2 <= (ob2 && open[nb2]) ? nb2 : b1;
            b3 <= (ob3 && open[nb3]) ? nb3 : b2;
            m1 <= mr * COLS + mc;
            for (g = 0; g < 4; g = g + 1)
                target[g*INDEX_W +: INDEX_W] <= pick(want[g], ghost_pos[g*INDEX_W +: INDEX_W],
                                                     player_pos, open, reach);
        end
    end
endmodule
