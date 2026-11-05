`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// game_ctrl : Pac-Man chase game (GAME mode, sw[6] = 1).  CONTRACT 9, v1.7.
//
// The player (Pac-Man) is steered with BTNU/D/L/R; ghosts chase him using one
// of the four pathfinding engines.  Eat every pellet to clear a level.
//
// States
//   G_OFF    game_mode = 0.  Nothing driven; leaving GAME always lands here.
//   G_INIT   wait for the ghost engine to be idle, then game_board loads the
//            maze (also into every engine over the load bus), floods and
//            places pellets.  -> G_NOPATH if the level can't be played.
//   G_READY  everyone on the spawn cells, waiting for BTNC.
//   G_PLAY   ticks run; BTNC -> G_PAUSE (BTNC again resumes).
//   G_CAUGHT a ghost touched Pac-Man: freeze CATCH_CYCLES, lose a life;
//            -> G_READY (respawn, pellets kept) or G_OVER.
//   G_CLEAR  last pellet eaten: +500 x level; BTNC -> next level (G_INIT,
//            pellets refilled) or, after level 4, G_WIN.
//   G_OVER / G_WIN : BTNC -> new game (level 1, 3 lives, score 0).
//   G_NOPATH player spawn is a wall, equals the ghost spawn, or the ghost
//            spawn can't be reached;
//            BTNC retries (after the user fixes the maze in demo mode).
//
// Levels (L1..L4)   algorithm  ghosts  ghost period
//                   DFS        2        750 ms
//                   Dijkstra   3        600 ms
//                   BFS        3        475 ms
//                   A*         4        350 ms
// Player period 200 ms.  If any of eng_sel (sw[5:2]) is on when a level
// starts, the levels cycle through the selected engines instead, in the
// order BFS, Dijkstra, A*, DFS: level L uses selected engine number
// L mod (number selected).  E.g. DIJ + A* + DFS on: L1 DIJ, L2 A*, L3 DFS,
// L4 DIJ; one engine on: every level uses it.  The choice is made when a
// level starts and held for the whole level.
// Ghost targets (game_targets): red chases Pac-Man, pink aims 3 cells ahead
// of him, cyan 3 cells behind, orange at his cell mirrored away from red;
// any ghost within 2 cells of him goes straight for him.  Different targets
// stop ghosts that meet from merging into one.
// Ghost routes: a DFS ghost follows its whole path before replanning (see
// game_ghosts); BFS / Dijkstra / A* ghosts replan before every step.
// Catch rule: a ghost on Pac-Man's cell, or Pac-Man and a ghost swapping
// cells in the same update.
// Score: 10 per pellet, +500 x level on a clear; kept as binary and 5 BCD
// digits (updated by small adders, no divider).
//////////////////////////////////////////////////////////////////////////////
module game_ctrl #(
    parameter COLS = `ARENA_COLS, parameter NUM_CELLS = `ARENA_NUM_CELLS,
    parameter INDEX_W = `ARENA_INDEX_W, parameter COST_W = `ARENA_COST_W,
    parameter DIST_W = `ARENA_DIST_W,
    parameter CLK_HZ = `ARENA_CLK_HZ,
    parameter MS = CLK_HZ / 1000,                  // cycles per "millisecond" (small in sim)
    parameter PLAYER_MS = 200,
    parameter RELEASE_MS = 3000,
    parameter CATCH_MS = 1000
) (
    input  wire                 clk, input wire rst,
    input  wire                 game_mode,
    input  wire                 btn_c, input wire btn_u, input wire btn_d,
    input  wire                 btn_l, input wire btn_r,
    input  wire [3:0]           eng_sel,            // sw[5:2] {dfs, ast, dij, bfs}
    input  wire [INDEX_W-1:0]   player_spawn,       // demo start marker
    input  wire [INDEX_W-1:0]   ghost_spawn,        // demo goal marker
    // maze_ram read + engine load bus (address/strobe; data comes from maze_ram)
    output wire [INDEX_W-1:0]   maze_addr,
    input  wire [COST_W-1:0]    maze_data,
    output wire                 load_we,
    // engine control
    output wire                 eng_start,
    output wire [INDEX_W-1:0]   eng_start_idx,
    output wire [INDEX_W-1:0]   eng_goal_idx,
    output wire [INDEX_W-1:0]   eng_path_addr,      // path RAM read address (all engines)
    output wire [3:0]           run_mask,
    input  wire [3:0]           eng_done,           // {dfs, ast, dij, bfs}
    input  wire [3:0]           eng_found,
    input  wire [4*DIST_W-1:0]  eng_path_cells,
    input  wire [4*INDEX_W-1:0] eng_path_data,      // path RAM at eng_path_addr
    // view
    output reg  [3:0]           state,
    output wire [INDEX_W-1:0]   player_pos,
    output wire [1:0]           player_dir,
    output wire [4*INDEX_W-1:0] ghost_pos,
    output wire [3:0]           ghost_present,
    output wire [NUM_CELLS-1:0] pellet,
    output wire [INDEX_W-1:0]   pellets_left,
    output reg  [1:0]           level,              // 0..3 = L1..L4
    output reg  [1:0]           algo,               // engine id: 0 BFS 1 DIJ 2 A* 3 DFS
    output reg  [1:0]           lives,
    output reg  [16:0]          score,
    output reg  [19:0]          score_bcd,
    output wire                 eng_busy            // a ghost search is still in flight
);
    localparam [3:0] G_OFF = 4'd0, G_INIT = 4'd1, G_READY = 4'd2, G_PLAY = 4'd3, G_PAUSE = 4'd4,
                     G_CAUGHT = 4'd5, G_CLEAR = 4'd6, G_OVER = 4'd7, G_WIN = 4'd8, G_NOPATH = 4'd9,
                     G_WAITENG = 4'd10;

    // ---------------------------------------------------------- level table
    function [1:0] lvl_algo;   input [1:0] l;
        lvl_algo = (l == 0) ? 2'd3 : (l == 1) ? 2'd1 : (l == 2) ? 2'd0 : 2'd2;
    endfunction
    function [2:0] lvl_ghosts; input [1:0] l;
        lvl_ghosts = (l == 0) ? 3'd2 : (l == 3) ? 3'd4 : 3'd3;
    endfunction
    function [10:0] lvl_ghost_ms; input [1:0] l;
        lvl_ghost_ms = (l == 0) ? 11'd750 : (l == 1) ? 11'd600 : (l == 2) ? 11'd475 : 11'd350;
    endfunction
    // switch override: the (level mod n)-th selected engine, n = number selected
    function [1:0] sel_algo; input [3:0] sel; input [1:0] l;
        integer i; reg [2:0] n, k, seen;
        begin
            n = sel[0] + sel[1] + sel[2] + sel[3];
            k = (n == 0) ? 3'd0 : l % n;
            sel_algo = 2'd0; seen = 3'd0;
            for (i = 0; i < 4; i = i + 1)
                if (sel[i]) begin
                    if (seen == k) sel_algo = i;
                    seen = seen + 1'b1;
                end
        end
    endfunction

    // ------------------------------------------------------------- ticks ---
    localparam TW = $clog2(1000 * MS + 1);
    reg [TW-1:0]  ptick_cnt, gtick_cnt, wait_cnt;
    reg [TW-1:0]  ghost_period;
    wire playing = (state == G_PLAY);
    wire ptick = playing && (ptick_cnt == PLAYER_MS * MS - 1);
    wire gtick = playing && (gtick_cnt == ghost_period - 1);

    // ---------------------------------------------------------- submodules
    reg  board_init, board_keep, p_spawn, g_spawn;
    wire board_ready, board_nopath, ate;
    wire [NUM_CELLS-1:0] open, reach;
    wire [4*INDEX_W-1:0] g_target;
    wire p_moved, g_moved, g_idle;
    wire [INDEX_W-1:0] p_prev;
    wire [4*INDEX_W-1:0] g_prev;
    wire [3:0] g_released;

    game_board u_board (
        .clk(clk), .rst(rst), .init(board_init), .keep_pellets(board_keep),
        .player_spawn(player_spawn), .ghost_spawn(ghost_spawn),
        .rd_addr(maze_addr), .rd_data(maze_data), .load_we(load_we),
        .open(open), .pellet(pellet), .reach(reach), .pellets_left(pellets_left),
        .ready(board_ready), .no_path(board_nopath),
        .eat(p_moved), .eat_pos(player_pos), .ate(ate)
    );
    game_player u_player (
        .clk(clk), .rst(rst), .open(open), .spawn(p_spawn), .spawn_pos(player_spawn),
        .btn_up(btn_u && playing), .btn_down(btn_d && playing),
        .btn_left(btn_l && playing), .btn_right(btn_r && playing),
        .tick(ptick), .pos(player_pos), .prev_pos(p_prev), .dir(player_dir),
        .moving(), .moved(p_moved)
    );
    game_targets u_targets (
        .clk(clk), .rst(rst), .open(open), .reach(reach), .player_pos(player_pos),
        .player_dir(player_dir), .ghost_pos(ghost_pos), .target(g_target)
    );
    game_ghosts #(.RELEASE_CYCLES(RELEASE_MS * MS)) u_ghosts (
        .clk(clk), .rst(rst), .spawn(g_spawn), .spawn_pos(ghost_spawn),
        .nghost(lvl_ghosts(level)), .commit_all(algo == 2'd3), .enable(playing), .tick(gtick),
        .target(g_target), .open(open),
        .eng_start(eng_start), .eng_start_idx(eng_start_idx), .eng_goal_idx(eng_goal_idx),
        .eng_path_addr(eng_path_addr),
        .eng_done(eng_done[algo]), .eng_found(eng_found[algo]),
        .eng_path_cells(eng_path_cells[algo*DIST_W +: DIST_W]),
        .eng_path_data(eng_path_data[algo*INDEX_W +: INDEX_W]),
        .pos(ghost_pos), .prev_pos(g_prev), .present(ghost_present), .released(g_released),
        .moved(g_moved), .idle(g_idle)
    );
    assign run_mask = 4'b0001 << algo;
    assign eng_busy = !g_idle;

    // ---------------------------------------------------------- collision
    // checked the cycle after any move (positions and prev_* are then stable)
    reg p_moved_d, g_moved_d;
    reg caught;
    integer g;
    always @* begin
        caught = 1'b0;
        for (g = 0; g < 4; g = g + 1)
            if (ghost_present[g]) begin
                if (ghost_pos[g*INDEX_W +: INDEX_W] == player_pos) caught = 1'b1;
                // swap: the ghost left Pac-Man's new cell for Pac-Man's old one
                if (p_moved_d && g_moved_d &&
                    ghost_pos[g*INDEX_W +: INDEX_W] == p_prev &&
                    g_prev[g*INDEX_W +: INDEX_W] == player_pos) caught = 1'b1;
            end
    end

    // --------------------------------------------------------- score (BCD)
    // add a BCD value (5 digits) to score_bcd with per-digit carry
    function [19:0] bcd_add; input [19:0] a, b;
        integer i; reg [4:0] s; reg c;
        begin
            c = 1'b0;
            for (i = 0; i < 5; i = i + 1) begin
                s = a[i*4 +: 4] + b[i*4 +: 4] + c;
                if (s > 9) begin s = s + 5'd6; c = 1'b1; end else c = 1'b0;
                bcd_add[i*4 +: 4] = s[3:0];
            end
        end
    endfunction
    wire [19:0] clear_bonus_bcd = (level == 0) ? 20'h00500 : (level == 1) ? 20'h01000 :
                                  (level == 2) ? 20'h01500 : 20'h02000;

    // ------------------------------------------------------------ the FSM
    always @(posedge clk) begin
        board_init <= 1'b0; p_spawn <= 1'b0; g_spawn <= 1'b0;
        p_moved_d <= p_moved; g_moved_d <= g_moved;
        if (rst) begin
            state <= G_OFF; level <= 0; algo <= 2'd3; lives <= 2'd3; score <= 0; score_bcd <= 0;
            board_keep <= 1'b0; ptick_cnt <= 0; gtick_cnt <= 0; wait_cnt <= 0; ghost_period <= 1000 * MS;
        end else if (!game_mode) begin
            state <= G_OFF;
        end else begin
            // tick counters run only while playing
            if (playing) begin
                ptick_cnt <= ptick ? 0 : ptick_cnt + 1'b1;
                gtick_cnt <= gtick ? 0 : gtick_cnt + 1'b1;
            end
            if (ate) begin
                score     <= score + 17'd10;
                score_bcd <= bcd_add(score_bcd, 20'h00010);
            end
            case (state)
                G_OFF: begin                       // entering GAME: new game
                    level <= 0; lives <= 2'd3; score <= 0; score_bcd <= 0;
                    board_keep <= 1'b0; state <= G_WAITENG;
                end
                G_WAITENG: if (g_idle) begin       // engines idle before reloading them
                    algo <= |eng_sel ? sel_algo(eng_sel, level) : lvl_algo(level);
                    ghost_period <= lvl_ghost_ms(level) * MS;
                    board_init <= 1'b1; state <= G_INIT;
                end
                G_INIT: if (board_ready && !board_init) begin
                    p_spawn <= 1'b1; g_spawn <= 1'b1;
                    ptick_cnt <= 0; gtick_cnt <= 0;
                    state <= (board_nopath || player_spawn == ghost_spawn) ? G_NOPATH : G_READY;
                end
                G_READY:  if (btn_c) state <= G_PLAY;
                G_PLAY: begin
                    if (btn_c) state <= G_PAUSE;
                    else if (caught) begin state <= G_CAUGHT; wait_cnt <= 0; end
                    else if (pellets_left == 0 && !ate) begin
                        score     <= score + 17'd500 * (level + 1'b1);
                        score_bcd <= bcd_add(score_bcd, clear_bonus_bcd);
                        state <= G_CLEAR;
                    end
                end
                G_PAUSE:  if (btn_c) state <= G_PLAY;
                G_CAUGHT: begin
                    if (wait_cnt == CATCH_MS * MS - 1) begin
                        if (lives == 2'd1) begin lives <= 2'd0; state <= G_OVER; end
                        else begin
                            lives <= lives - 1'b1;
                            p_spawn <= 1'b1; g_spawn <= 1'b1;
                            ptick_cnt <= 0; gtick_cnt <= 0;
                            state <= G_READY;
                        end
                    end else wait_cnt <= wait_cnt + 1'b1;
                end
                G_CLEAR: if (btn_c) begin
                    if (level == 2'd3) state <= G_WIN;
                    else begin level <= level + 1'b1; board_keep <= 1'b0; state <= G_WAITENG; end
                end
                G_OVER, G_WIN: if (btn_c) begin
                    level <= 0; lives <= 2'd3; score <= 0; score_bcd <= 0;
                    board_keep <= 1'b0; state <= G_WAITENG;
                end
                G_NOPATH: if (btn_c) state <= G_WAITENG;
                default: state <= G_OFF;
            endcase
        end
    end
endmodule
