`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// game_board : the game's copy of the maze, reachability and pellets.
//
// On `init` (1-cycle pulse; player_spawn / ghost_spawn sampled then):
//   LOAD  : streams maze_ram cells 0..NUM_CELLS-1 (1 per clk) over rd_addr.
//           The same address/strobe drive the engines' load bus (load_we),
//           so the engines get the maze in the same pass.  open[i] = cost != wall.
//   FLOOD : reach = cells reachable from player_spawn, one BFS layer per clk
//           (192-bit neighbour shifts), until nothing changes.
//   COUNT : pellet = reach minus both spawn cells; pellets_left counted by a
//           1-cell-per-clk scan.
//   READY : `ready` = 1.  no_path = 1 if the player spawn is a wall or the
//           ghost spawn is not reachable (the level can't be played).
// While READY, `eat` (1-cycle pulse) clears the pellet under eat_pos and
// raises `ate` for one cycle if there was one.
// keep_pellets = 1 on init keeps the current pellet vector and count (used
// for a respawn); the maze is still reloaded and re-flooded.
//////////////////////////////////////////////////////////////////////////////
module game_board #(
    parameter COLS = `ARENA_COLS, parameter ROWS = `ARENA_ROWS,
    parameter NUM_CELLS = `ARENA_NUM_CELLS,
    parameter INDEX_W = `ARENA_INDEX_W, parameter COST_W = `ARENA_COST_W
) (
    input  wire                 clk, input wire rst,
    input  wire                 init,
    input  wire                 keep_pellets,
    input  wire [INDEX_W-1:0]   player_spawn,
    input  wire [INDEX_W-1:0]   ghost_spawn,
    // maze_ram read (async) + engine load strobe
    output wire [INDEX_W-1:0]   rd_addr,
    input  wire [COST_W-1:0]    rd_data,
    output wire                 load_we,
    // results
    output reg  [NUM_CELLS-1:0] open,
    output reg  [NUM_CELLS-1:0] pellet,
    output reg  [NUM_CELLS-1:0] reach,          // reachable from player_spawn (valid when ready)
    output reg  [INDEX_W-1:0]   pellets_left,
    output wire                 ready,
    output reg                  no_path,
    // eating
    input  wire                 eat,
    input  wire [INDEX_W-1:0]   eat_pos,
    output reg                  ate
);
    localparam [2:0] B_IDLE = 3'd0, B_LOAD = 3'd1, B_FLOOD = 3'd2, B_COUNT = 3'd3, B_READY = 3'd4;
    reg [2:0]           st;
    reg [INDEX_W-1:0]   ptr;
    reg [INDEX_W-1:0]   p_spawn, g_spawn;
    reg                 keep;

    // cells in column 0 / column COLS-1
    function [NUM_CELLS-1:0] col_mask;
        input integer c;
        integer i;
        begin
            col_mask = {NUM_CELLS{1'b0}};
            for (i = 0; i < NUM_CELLS; i = i + 1)
                if (i % COLS == c) col_mask[i] = 1'b1;
        end
    endfunction
    localparam [NUM_CELLS-1:0] FIRST_COL = col_mask(0);
    localparam [NUM_CELLS-1:0] LAST_COL  = col_mask(COLS-1);

    // cell i is reached from i-1 (left), i+1 (right), i-COLS (up), i+COLS (down)
    wire [NUM_CELLS-1:0] from_left  = (reach << 1)    & ~FIRST_COL;
    wire [NUM_CELLS-1:0] from_right = (reach >> 1)    & ~LAST_COL;
    wire [NUM_CELLS-1:0] from_up    =  reach << COLS;
    wire [NUM_CELLS-1:0] from_down  =  reach >> COLS;
    wire [NUM_CELLS-1:0] reach_next = reach | (open & (from_left | from_right | from_up | from_down));

    assign rd_addr = ptr;
    assign load_we = (st == B_LOAD);
    assign ready   = (st == B_READY);

    always @(posedge clk) begin
        ate <= 1'b0;
        if (rst) begin
            st <= B_IDLE; ptr <= 0; no_path <= 1'b0;
            open <= 0; reach <= 0; pellet <= 0; pellets_left <= 0;
            p_spawn <= 0; g_spawn <= 0; keep <= 1'b0;
        end else if (init) begin
            st <= B_LOAD; ptr <= 0; no_path <= 1'b0;
            p_spawn <= player_spawn; g_spawn <= ghost_spawn; keep <= keep_pellets;
        end else begin
            case (st)
                B_LOAD: begin
                    open[ptr] <= (rd_data != `ARENA_COST_WALL);
                    if (ptr == NUM_CELLS-1) begin
                        st <= B_FLOOD;
                        reach <= {NUM_CELLS{1'b0}};
                        ptr <= 0;
                    end else ptr <= ptr + 1'b1;
                end
                B_FLOOD: begin
                    // first cycle seeds the player spawn (if it is open)
                    if (reach == 0 && ptr == 0) begin
                        reach[p_spawn] <= open[p_spawn];
                        ptr <= 1;
                        if (!open[p_spawn]) begin no_path <= 1'b1; st <= B_READY; end
                    end else if (reach_next == reach) begin
                        no_path <= !reach[g_spawn];
                        ptr <= 0;
                        if (keep) st <= B_READY;
                        else begin
                            pellet <= reach & ~({{(NUM_CELLS-1){1'b0}}, 1'b1} << p_spawn)
                                            & ~({{(NUM_CELLS-1){1'b0}}, 1'b1} << g_spawn);
                            pellets_left <= 0;
                            st <= B_COUNT;
                        end
                    end else reach <= reach_next;
                end
                B_COUNT: begin
                    if (pellet[ptr]) pellets_left <= pellets_left + 1'b1;
                    if (ptr == NUM_CELLS-1) st <= B_READY;
                    else ptr <= ptr + 1'b1;
                end
                B_READY: begin
                    if (eat && pellet[eat_pos]) begin
                        pellet[eat_pos] <= 1'b0;
                        pellets_left <= pellets_left - 1'b1;
                        ate <= 1'b1;
                    end
                end
                default: st <= B_IDLE;
            endcase
        end
    end
endmodule
