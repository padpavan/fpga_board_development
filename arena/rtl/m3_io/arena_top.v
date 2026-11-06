`timescale 1ns / 1ps
`include "arena_defs.vh"
// Integrated Basys 3 top for the FPGA Pathfinding Arena.
//
// Two modes (CONTRACT 9, v1.7):
//   demo (sw[6] = 0): edit a maze, race the selected engines (sw[5:2]), OLED
//         shows the engines' compute times.  BTNU short press = wipe the map,
//         BTNU held 3 s = full reset, BTNR = back to EDIT with the maze intact.
//   GAME (sw[6] = 1): game_ctrl takes the engines, the maze and the buttons
//         (BTNU/D/L/R steer Pac-Man, BTNC start/pause); BTNU never wipes or resets.
//         sys_ctrl is held in EDIT.  The game starts once the demo is in EDIT
//         and no engine is busy, and it keeps the engines until its last
//         search has finished after sw[6] goes off.
//   sw[8] = 1: OLED mirror (testing without a monitor): the OLED shows the
//         VGA picture as small maps (EDIT map / 4-engine race / game) via
//         oled_map, which then owns the engines' cell_rd_addr and maze_ram's
//         first VGA read port (the VGA overlay is wrong meanwhile).
//   RUN view zoom (CONTRACT 9, v1.10): left click selects a quadrant (yellow
//         border); the wheel zooms only that quadrant, middle click resets it.
//   sw[11:9] (CONTRACT 8b, v1.10): OLED page.  000 automatic (as before),
//         001 menu, 010 graph, 011 race results, 100 points, 101 algorithms.
//         GAME always shows the score; the sw[8] mirror wins over every page.
module arena_top #(
    parameter DEBOUNCE_LOCKOUT=20_000_000,
    parameter RESET_HOLD_CYCLES=3*`ARENA_CLK_HZ, // BTNU hold for a full reset (3 s); small in simulation
    parameter GEN_CELLS=`ARENA_NUM_CELLS,
    parameter OLED_SNAP_DIV=`ARENA_CLK_HZ/10,    // OLED stats refresh (10 Hz); small in simulation
    parameter GAME_MS=`ARENA_CLK_HZ/1000         // game timing unit (1 ms); small in simulation
) (
    input wire clk,
    input wire btnU, input wire btnC, input wire btnL, input wire btnR, input wire btnD,
    input wire [15:0] sw,
    output wire [15:0] led,
    output wire [6:0] seg, output wire dp, output wire [3:0] an,
    output wire [3:0] vga_r, output wire [3:0] vga_g, output wire [3:0] vga_b,
    output wire vga_hs, output wire vga_vs,
    inout wire ps2_clk, inout wire ps2_data,
    output wire oled_cs, output wire oled_sdin, output wire oled_sclk,
    output wire oled_d_cn, output wire oled_resn, output wire oled_vccen, output wire oled_pmoden
);
    localparam INDEX_W=`ARENA_INDEX_W, COST_W=`ARENA_COST_W,
               DIST_W=`ARENA_DIST_W, CNT_W=`ARENA_CNT_W;
    wire rst, up_pulse, run_pulse, pause_pulse, back_pulse, random_pulse;
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_up (.CLOCK(clk),.btn(btnU),.press_pulse(up_pulse));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_run (.CLOCK(clk),.btn(btnC),.press_pulse(run_pulse));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_pause (.CLOCK(clk),.btn(btnL),.press_pulse(pause_pulse));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_back (.CLOCK(clk),.btn(btnR),.press_pulse(back_pulse));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_random (.CLOCK(clk),.btn(btnD),.press_pulse(random_pulse));

    reg [15:0] sw_meta = 16'h0000, sw_sync = 16'h0000;
    always @(posedge clk) begin
        sw_meta <= sw;
        sw_sync <= sw_meta;
    end
    wire game_sw = sw_sync[6];
    wire oled_mirror = sw_sync[8];
    wire [INDEX_W-1:0] oled_cell;               // oled_map's cell (mirror mode)
    wire [INDEX_W-1:0] view_cell;               // cell read by maze_ram port 0 + engines' cell_state
    // rst: once at power-on (nothing else initialises the markers, the mouse
    // limits or the maze), then only from a long BTNU hold.  In GAME, BTNU is "up".
    reg [3:0] por_cnt = 4'hF;
    wire por_rst = (por_cnt != 0);
    always @(posedge clk) if (por_rst) por_cnt <= por_cnt - 1'b1;
    wire wipe_pulse, hold_rst_pulse;
    btn_hold #(.HOLD_CYCLES(RESET_HOLD_CYCLES),.LOCKOUT(DEBOUNCE_LOCKOUT)) u_btnu_hold (
        .clk(clk),.rst(por_rst),.enable(!game_sw),.btn(btnU),.press_pulse(up_pulse),
        .short_pulse(wipe_pulse),.long_pulse(hold_rst_pulse)
    );
    assign rst = por_rst || hold_rst_pulse;

    // Configure Digilent MouseCtl to the VGA canvas coordinate limits.
    reg [1:0] mouse_cfg;
    reg [11:0] mouse_value;
    wire [11:0] mouse_x, mouse_y;
    wire [3:0] mouse_z;
    wire mouse_left, mouse_middle, mouse_right, mouse_event;
    wire setmax_x = (mouse_cfg == 2'd1);
    wire setmax_y = (mouse_cfg == 2'd2);
    always @(posedge clk) begin
        if (rst) mouse_cfg <= 2'd1;
        else if (mouse_cfg != 2'd0) mouse_cfg <= mouse_cfg + 1'b1;
    end
    always @* mouse_value = (mouse_cfg == 2'd1) ? (`ARENA_VGA_H-1) : (`ARENA_VGA_V-1);
    MouseCtl #(.SYSCLK_FREQUENCY_HZ(`ARENA_CLK_HZ)) u_mouse (
        .clk(clk), .rst(rst), .xpos(mouse_x), .ypos(mouse_y), .zpos(mouse_z),
        .left(mouse_left), .middle(mouse_middle), .right(mouse_right), .new_event(mouse_event),
        .value(mouse_value), .setx(1'b0), .sety(1'b0), .setmax_x(setmax_x), .setmax_y(setmax_y),
        .ps2_clk(ps2_clk), .ps2_data(ps2_data)
    );
    wire mouse_left_pulse, mouse_right_pulse, cursor_valid;
    wire [INDEX_W-1:0] mouse_idx;
    mouse_if u_mouse_if (
        .clk(clk),.rst(rst),.xpos(mouse_x),.ypos(mouse_y),
        .left_down(mouse_left),.right_down(mouse_right),.new_event(mouse_event),
        .left_pulse(mouse_left_pulse),.right_pulse(mouse_right_pulse),
        .cell_idx(mouse_idx),.cursor_valid(cursor_valid)
    );

    wire gen_start, gen_random, gen_busy, gen_done, gen_we;
    wire [INDEX_W-1:0] gen_addr;
    wire [COST_W-1:0] gen_data;
    wire maze_we, edit_mode, load_we, start_search, step_en, paused;
    wire [INDEX_W-1:0] maze_waddr, load_addr, start_idx, goal_idx, maze_rd_addr;
    wire [COST_W-1:0] maze_wdata, maze_cost, maze_load_cost;
    wire [2:0] mode;
    wire [3:0] eng_done;
    wire [3:0] run_mask;                 // {dfs, ast, dij, bfs}, latched from sw[5:2] on BTNC
    wire       game_hold;                // GAME owns the board: sys_ctrl frozen in EDIT
    wire       wipe_accept, abort_pulse; // sys_ctrl took a wipe / stopped a run
    wire                pf_load_we;          // engine load bus source (demo or game)
    wire [INDEX_W-1:0]  pf_load_addr;
    maze_gen #(.NUM_CELLS(GEN_CELLS)) u_maze_gen (
        .clk(clk),.rst(rst),.start(gen_start),.randomize(gen_random),
        .start_idx(start_idx),.goal_idx(goal_idx),
        .busy(gen_busy),.done(gen_done),.we(gen_we),.addr(gen_addr),.data(gen_data)
    );

    sys_ctrl u_sys_ctrl (
        .clk(clk),.rst(rst),.run_pulse(run_pulse),.pause_pulse(pause_pulse),
        .wipe_pulse(wipe_pulse),.back_pulse(back_pulse),.random_pulse(random_pulse),.speed_sel(sw_sync[15:13]),
        .mouse_left(mouse_left_pulse && cursor_valid),.mouse_right(mouse_right_pulse && cursor_valid),
        .mouse_idx(mouse_idx),.edit_tool(sw_sync[1:0]),.gen_busy(gen_busy),.gen_done(gen_done),.engines_done(eng_done),.run_sel(sw_sync[5:2]),.hold(game_hold),
        .gen_start(gen_start),.gen_random(gen_random),.maze_we(maze_we),.maze_addr(maze_waddr),
        .maze_data(maze_wdata),.load_we(load_we),.load_addr(load_addr),.start_pulse(start_search),
        .step_en(step_en),.edit_mode(edit_mode),.start_idx(start_idx),.goal_idx(goal_idx),
        .mode(mode),.paused(paused),.run_mask(run_mask),
        .wipe_accept(wipe_accept),.abort_pulse(abort_pulse)
    );
    assign view_cell = oled_mirror ? oled_cell : maze_rd_addr;
    // VGA read addresses / data (clk_pix domain): cell, up, down, left, right
    wire [INDEX_W-1:0] vga_up_addr, vga_dn_addr, vga_lf_addr, vga_rt_addr;
    wire [COST_W-1:0] vga_up_cost, vga_dn_cost, vga_lf_cost, vga_rt_cost;
    maze_ram u_maze_ram (
        .clk(clk),.rst(rst),.we(gen_we || maze_we),
        .waddr(gen_we ? gen_addr : maze_waddr),.wdata(gen_we ? gen_data : maze_wdata),
        .raddr(pf_load_addr),.rdata(maze_load_cost),
        .vga_addr0(view_cell),.vga_addr1(vga_up_addr),.vga_addr2(vga_dn_addr),
        .vga_addr3(vga_lf_addr),.vga_addr4(vga_rt_addr),
        .vga_data0(maze_cost),.vga_data1(vga_up_cost),.vga_data2(vga_dn_cost),
        .vga_data3(vga_lf_cost),.vga_data4(vga_rt_cost)
    );
    // The load bus is registered one cycle before it fans out to the four
    // engines' cost RAMs (100 MHz timing).  The last cell (191) is then
    // written on the same edge the engines sample `start`, while busy is
    // still 0 (allowed, CONTRACT 4.1.1), and CLEAR reads the cost RAM only
    // after that.
    reg                load_we_q;
    reg [INDEX_W-1:0]  load_addr_q;
    reg [COST_W-1:0]   load_data_q;
    always @(posedge clk) begin
        load_we_q   <= pf_load_we && !rst;
        load_addr_q <= pf_load_addr;
        load_data_q <= maze_load_cost;
    end

    wire bfs_busy,bfs_done,bfs_found,bfs_no_path;
    wire dij_busy,dij_done,dij_found,dij_no_path;
    wire ast_busy,ast_done,ast_found,ast_no_path;
    wire dfs_busy,dfs_done,dfs_found,dfs_no_path;
    wire [DIST_W-1:0] bfs_cells,bfs_cost,bfs_exp;
    wire [DIST_W-1:0] dij_cells,dij_cost,dij_exp;
    wire [DIST_W-1:0] ast_cells,ast_cost,ast_exp;
    wire [DIST_W-1:0] dfs_cells,dfs_cost,dfs_exp;
    wire [CNT_W-1:0] bfs_cycles,dij_cycles,ast_cycles,dfs_cycles;
    wire [INDEX_W-1:0] bfs_current,dij_current,ast_current,dfs_current;
    wire [1:0] bfs_state,dij_state,ast_state,dfs_state;
    wire [INDEX_W-1:0] bfs_path, dij_path, ast_path, dfs_path;
    assign eng_done = {dfs_done,ast_done,dij_done,bfs_done};
    wire eng_any_busy = bfs_busy || dij_busy || ast_busy || dfs_busy;

    // ---------------- GAME mode ----------------
    wire g_load_we, g_start, g_eng_busy;
    wire [INDEX_W-1:0] g_maze_addr, g_start_idx, g_goal_idx, g_path_addr;
    wire [3:0] g_run_mask, g_state;
    wire [INDEX_W-1:0] g_player, g_pellets;
    wire [1:0] g_dir, g_level, g_algo, g_lives;
    wire [4*INDEX_W-1:0] g_ghosts;
    wire [3:0] g_ghost_present;
    wire [`ARENA_NUM_CELLS-1:0] g_pellet;
    wire [19:0] g_score_bcd;
    // the game starts once the demo is back in EDIT and the engines are idle
    reg game_go = 1'b0;
    always @(posedge clk) begin
        if (rst || !game_sw) game_go <= 1'b0;
        else if (mode == 3'd0 && !eng_any_busy && !gen_busy) game_go <= 1'b1;
    end
    wire game_owns = game_go || g_eng_busy;     // the game drives the engines
    assign game_hold = game_sw || game_owns;
    game_ctrl #(.MS(GAME_MS)) u_game (
        .clk(clk),.rst(rst),.game_mode(game_go),
        .btn_c(run_pulse),.btn_u(up_pulse),.btn_d(random_pulse),.btn_l(pause_pulse),.btn_r(back_pulse),
        .eng_sel(sw_sync[5:2]),.player_spawn(start_idx),.ghost_spawn(goal_idx),
        .maze_addr(g_maze_addr),.maze_data(maze_load_cost),.load_we(g_load_we),
        .eng_start(g_start),.eng_start_idx(g_start_idx),.eng_goal_idx(g_goal_idx),
        .eng_path_addr(g_path_addr),.run_mask(g_run_mask),
        .eng_done(eng_done),.eng_found({dfs_found,ast_found,dij_found,bfs_found}),
        .eng_path_cells({dfs_cells,ast_cells,dij_cells,bfs_cells}),
        .eng_path_data({dfs_path,ast_path,dij_path,bfs_path}),
        .state(g_state),.player_pos(g_player),.player_dir(g_dir),
        .ghost_pos(g_ghosts),.ghost_present(g_ghost_present),
        .pellet(g_pellet),.pellets_left(g_pellets),
        .level(g_level),.algo(g_algo),.lives(g_lives),.score(),.score_bcd(g_score_bcd),
        .eng_busy(g_eng_busy)
    );
    // engine inputs: the game's while it owns them, else sys_ctrl's.  In GAME
    // the engines always run at full speed (a demo run still in flight when
    // sw[6] goes on finishes in well under a millisecond).
    assign              pf_load_we   = game_owns ? g_load_we   : load_we;
    assign              pf_load_addr = game_owns ? g_maze_addr : load_addr;
    wire                pf_start     = game_owns ? g_start     : start_search;
    wire [3:0]          pf_run_mask  = game_owns ? g_run_mask  : run_mask;
    wire [INDEX_W-1:0]  pf_start_idx = game_owns ? g_start_idx : start_idx;
    wire [INDEX_W-1:0]  pf_goal_idx  = game_owns ? g_goal_idx  : goal_idx;
    wire                pf_step_en   = (game_owns || game_sw) ? 1'b1 : step_en;

    // A wipe or a stopped run also resets the engines: they only accept start
    // when idle or done (CONTRACT 4.1), so one left busy would carry on with its
    // old search at the next BTNC.  sys_ctrl never reports either in GAME.
    reg pf_rst = 1'b0;
    always @(posedge clk) pf_rst <= rst || wipe_accept || abort_pulse;
    pathfind_top u_pathfind (
        .clk(clk),.rst(pf_rst),.step_en(pf_step_en),.load_we(load_we_q),.load_addr(load_addr_q),.load_data(load_data_q),
        .start(pf_start),.run_mask(pf_run_mask),.start_idx(pf_start_idx),.goal_idx(pf_goal_idx),.cell_rd_addr(view_cell),
        .bfs_busy(bfs_busy),.bfs_done(bfs_done),.bfs_found(bfs_found),.bfs_no_path(bfs_no_path),
        .bfs_path_rd_addr(g_path_addr),.bfs_path_rd_data(bfs_path),.bfs_path_cells(bfs_cells),
        .bfs_path_cost(bfs_cost),.bfs_current(bfs_current),.bfs_cell_state(bfs_state),
        .bfs_cycles(bfs_cycles),.bfs_expansions(bfs_exp),
        .dij_busy(dij_busy),.dij_done(dij_done),.dij_found(dij_found),.dij_no_path(dij_no_path),
        .dij_path_rd_addr(g_path_addr),.dij_path_rd_data(dij_path),.dij_path_cells(dij_cells),
        .dij_path_cost(dij_cost),.dij_current(dij_current),.dij_cell_state(dij_state),
        .dij_cycles(dij_cycles),.dij_expansions(dij_exp),
        .ast_busy(ast_busy),.ast_done(ast_done),.ast_found(ast_found),.ast_no_path(ast_no_path),
        .ast_path_rd_addr(g_path_addr),.ast_path_rd_data(ast_path),.ast_path_cells(ast_cells),
        .ast_path_cost(ast_cost),.ast_current(ast_current),.ast_cell_state(ast_state),
        .ast_cycles(ast_cycles),.ast_expansions(ast_exp),.ast_dbg_state(),
        .dfs_busy(dfs_busy),.dfs_done(dfs_done),.dfs_found(dfs_found),.dfs_no_path(dfs_no_path),
        .dfs_path_rd_addr(g_path_addr),.dfs_path_rd_data(dfs_path),.dfs_path_cells(dfs_cells),
        .dfs_path_cost(dfs_cost),.dfs_current(dfs_current),.dfs_cell_state(dfs_state),
        .dfs_cycles(dfs_cycles),.dfs_expansions(dfs_exp)
    );

    // ---------------- VGA: 1920x1080 @ 60 Hz on clk_pix (148.5 MHz) ----------------
    localparam FRAC=20, OW=28, KW=16;
    wire clk_pix, pix_locked;
    vga_clkgen u_vga_clk (.clk(clk),.clk_pix(clk_pix),.locked(pix_locked));

    // stretch the 1-cycle rst so the pixel domain samples it, then synchronise
    reg [3:0] rst_hold = 4'hF;
    always @(posedge clk)
        if (rst) rst_hold <= 4'hF; else if (rst_hold != 0) rst_hold <= rst_hold - 1'b1;
    (* ASYNC_REG = "TRUE" *) reg rst_pix_m = 1'b1, rst_pix = 1'b1;
    always @(posedge clk_pix) begin
        rst_pix_m <= (|rst_hold) || !pix_locked;
        rst_pix   <= rst_pix_m;
    end

    // scroll-wheel zoom of the RUN view, one view per quadrant (100 MHz domain,
    // next to the mouse); left click selects the quadrant the wheel zooms
    wire [4*OW-1:0] zoom_ox, zoom_oy;
    wire [4*KW-1:0] zoom_k;
    wire [4*3-1:0]  zoom_level;
    wire [1:0]      zoom_sel;
    wire            zoom_sel_valid;
    vga_zoom #(.FRAC(FRAC),.OW(OW),.KW(KW)) u_vga_zoom (
        .clk(clk),.rst(rst || wipe_accept),.enable(!edit_mode && !game_go),
        .new_event(mouse_event),.zpos(mouse_z),.middle(mouse_middle),.left(mouse_left),
        .mouse_x(mouse_x),.mouse_y(mouse_y),
        .ox(zoom_ox),.oy(zoom_oy),.k(zoom_k),.level(zoom_level),
        .sel(zoom_sel),.sel_valid(zoom_sel_valid)
    );

    // everything the renderer needs from the 100 MHz side, moved across once per frame
    // pellets left (0..192) as 3 BCD digits for the HUD
    function [11:0] bcd3; input [7:0] b;
        integer i; reg [19:0] t;
        begin
            t = {12'd0, b};
            for (i = 0; i < 8; i = i + 1) begin
                if (t[11:8]  >= 5) t[11:8]  = t[11:8]  + 3;
                if (t[15:12] >= 5) t[15:12] = t[15:12] + 3;
                if (t[19:16] >= 5) t[19:16] = t[19:16] + 3;
                t = t << 1;
            end
            bcd3 = t[19:8];
        end
    endfunction
    localparam GAME_W = 1 + 4 + INDEX_W + 2 + 4*INDEX_W + 4 + `ARENA_NUM_CELLS + 2 + 2 + 2 + 20 + 12;
    wire [GAME_W-1:0] game_view = {game_go, g_state, g_player, g_dir, g_ghosts, g_ghost_present,
        g_pellet, g_level, g_algo, g_lives, g_score_bcd, bcd3(g_pellets)};
    localparam VIEW_W = 4*(2*OW + KW + 3) + 3 + 12 + 12 + 1 + INDEX_W + 1 + 1 + 2 + 6*INDEX_W + 4 + GAME_W;
    wire [VIEW_W-1:0] view_a = {zoom_ox, zoom_oy, zoom_k, zoom_level, zoom_sel, zoom_sel_valid, mouse_x, mouse_y,
        cursor_valid && !game_go, mouse_idx, edit_mode, sw_sync[7] || game_go, sw_sync[1:0],
        start_idx, goal_idx, bfs_current, dij_current, ast_current, dfs_current, run_mask, game_view};
    wire [VIEW_W-1:0] view_p;
    wire view_tick;
    vga_cdc #(.W(VIEW_W)) u_vga_cdc (
        .clk_a(clk),.rst_a(rst),.din(view_a),
        .clk_b(clk_pix),.rst_b(rst_pix),.apply_b(view_tick),.dout(view_p)
    );
    wire [4*OW-1:0] v_ox, v_oy;
    wire [4*KW-1:0] v_k;
    wire [4*3-1:0] v_level;
    wire [1:0] v_zsel;
    wire v_zsel_valid;
    wire [11:0] v_mouse_x, v_mouse_y;
    wire v_cursor_valid, v_edit, v_pacman;
    wire [1:0] v_tool;
    wire [INDEX_W-1:0] v_cursor_idx, v_start, v_goal, v_bfs_cur, v_dij_cur, v_ast_cur, v_dfs_cur;
    wire [3:0] v_run_mask;
    wire v_game; wire [3:0] v_gstate; wire [INDEX_W-1:0] v_player; wire [1:0] v_dir;
    wire [4*INDEX_W-1:0] v_ghosts; wire [3:0] v_gpresent; wire [`ARENA_NUM_CELLS-1:0] v_pellet;
    wire [1:0] v_glevel, v_galgo, v_glives; wire [19:0] v_gscore; wire [11:0] v_gpellets;
    assign {v_ox, v_oy, v_k, v_level, v_zsel, v_zsel_valid, v_mouse_x, v_mouse_y, v_cursor_valid, v_cursor_idx,
            v_edit, v_pacman, v_tool, v_start, v_goal, v_bfs_cur, v_dij_cur, v_ast_cur,
            v_dfs_cur, v_run_mask,
            v_game, v_gstate, v_player, v_dir, v_ghosts, v_gpresent, v_pellet,
            v_glevel, v_galgo, v_glives, v_gscore, v_gpellets} = view_p;

    wire active_video, hs, vs;
    wire [11:0] px;
    wire [10:0] py;
    vga_timing u_vga_timing (.clk(clk_pix),.rst(rst_pix),.active_video(active_video),
        .x(px),.y(py),.hsync(hs),.vsync(vs),.view_tick(view_tick));
    // The engines' cell_state ports and maze_ram's VGA ports are async LUTRAM
    // reads addressed from clk_pix; all four engines share one cell address.
    vga_render #(.FRAC(FRAC),.OW(OW),.KW(KW)) u_vga_render (
        .clk(clk_pix),.rst(rst_pix),
        .x(px),.y(py),.active_video(active_video),.hsync_in(hs),.vsync_in(vs),
        .edit_mode(v_edit),.pacman(v_pacman),
        .run_ox(v_ox),.run_oy(v_oy),.run_k(v_k),.zoom_level(v_level),
        .zoom_sel(v_zsel),.zoom_sel_valid(v_zsel_valid),
        .mouse_x(v_mouse_x),.mouse_y(v_mouse_y),
        .cursor_valid(v_cursor_valid),.cursor_idx(v_cursor_idx),.edit_tool(v_tool),
        .start_idx(v_start),.goal_idx(v_goal),
        .bfs_current(v_bfs_cur),.dij_current(v_dij_cur),.ast_current(v_ast_cur),.dfs_current(v_dfs_cur),
        .run_mask(v_run_mask),
        .cell_addr(maze_rd_addr),.up_addr(vga_up_addr),.dn_addr(vga_dn_addr),
        .lf_addr(vga_lf_addr),.rt_addr(vga_rt_addr),
        .cell_cost(maze_cost),.up_cost(vga_up_cost),.dn_cost(vga_dn_cost),
        .lf_cost(vga_lf_cost),.rt_cost(vga_rt_cost),
        .bfs_state(bfs_state),.dij_state(dij_state),.ast_state(ast_state),.dfs_state(dfs_state),
        .game(v_game),.game_state(v_gstate),.player_pos(v_player),.player_dir(v_dir),
        .ghost_pos(v_ghosts),.ghost_present(v_gpresent),.pellet(v_pellet),
        .game_level(v_glevel),.game_algo(v_galgo),.game_lives(v_glives),
        .game_score(v_gscore),.game_pellets(v_gpellets),
        .red(vga_r),.green(vga_g),.blue(vga_b),.hsync(vga_hs),.vsync(vga_vs)
    );

    // Snapshot + finish order + decimal conversion for the OLED (CONTRACT 8):
    // oled_stats registers everything in the 100 MHz domain; the 6.25 MHz
    // screen reads only those registers and the view registers below.
    wire [3:0]  snap_mask;
    wire [15:0] snap_status;
    wire [11:0] snap_place;
    wire [95:0] cyc_bcd;
    wire [47:0] exp_bcd;
    wire [63:0] cost_bcd;
    wire [95:0] gap_bcd;
    wire [7:0]  snap_order;
    oled_stats #(.SNAP_DIV(OLED_SNAP_DIV)) u_oled_stats (
        .clk(clk),.rst(rst),.run_mask(run_mask),
        .cycles({dfs_cycles,ast_cycles,dij_cycles,bfs_cycles}),
        .expansions({dfs_exp,ast_exp,dij_exp,bfs_exp}),
        .path_cost({dfs_cost,ast_cost,dij_cost,bfs_cost}),
        .busy({dfs_busy,ast_busy,dij_busy,bfs_busy}),
        .done({dfs_done,ast_done,dij_done,bfs_done}),
        .found({dfs_found,ast_found,dij_found,bfs_found}),
        .no_path({dfs_no_path,ast_no_path,dij_no_path,bfs_no_path}),
        .snap_mask(snap_mask),.snap_status(snap_status),.snap_place(snap_place),
        .cyc_bcd(cyc_bcd),.exp_bcd(exp_bcd),.cost_bcd(cost_bcd),
        .gap_bcd(gap_bcd),.snap_order(snap_order)
    );
    // OLED view: GAME score / demo engine selection (EDIT, maze generation) /
    // demo results (LOAD..DONE).  Registered here: they change only on mode
    // changes, switch flips or a pellet, so the 6.25 MHz side sees clean values.
    reg [1:0]  oled_view = 2'd0;
    reg [3:0]  oled_sel = 4'd0;
    reg [19:0] oled_score = 20'd0;
    reg [2:0]  oled_page = 3'd0;     // sw[11:9]; 0 (automatic) in GAME
    always @(posedge clk) begin
        oled_view  <= game_go ? 2'd2 : (mode == 3'd0 || mode == 3'd1 || mode == 3'd7) ? 2'd0 : 2'd1;
        oled_sel   <= sw_sync[5:2];
        oled_score <= g_score_bcd;
        oled_page  <= game_go ? 3'd0 : sw_sync[11:9];
    end
    // page 011 shows oled_screen's ranked race results whatever the mode
    wire [1:0] screen_view = (oled_page == 3'd3) ? 2'd1 : oled_view;
    wire clk6p25m;
    wire [12:0] oled_pixel_index;
    wire [15:0] oled_pixel_data;
    clk_div #(.DIV(`ARENA_OLED_CLK_DIV)) u_oled_clk (.clk(clk),.clk_out(clk6p25m));
    reg [7:0] oled_rst_stretch;
    reg oled_rst_meta, oled_rst_sync;
    always @(posedge clk) begin
        if (rst) oled_rst_stretch <= 8'hFF;
        else if (oled_rst_stretch != 0) oled_rst_stretch <= oled_rst_stretch - 1'b1;
    end
    always @(posedge clk6p25m) begin
        oled_rst_meta <= |oled_rst_stretch;
        oled_rst_sync <= oled_rst_meta;
    end
    // Performance graph (CONTRACT 8a): M2's sampler measures the demo run (never a
    // GAME search) and ast_perf_hist stores it; M1's gv_oled_plot draws it on the
    // OLED results view (below), reading the history through pg_rd_idx.  The
    // timing labels come from oled_stats above (snap_order, cyc_bcd, gap_bcd).
    localparam PG_AW = `ARENA_PG_ADDR_W, PG_RW = `ARENA_PG_ROT_W, PG_LW = `ARENA_PG_LEVEL_W;
    wire               pg_we;
    wire [PG_AW-1:0]   pg_waddr, pg_count;
    wire [4*PG_LW-1:0] pg_wdata;
    wire [PG_RW-1:0]   pg_rot;
    wire [4*PG_AW-1:0] pg_end;
    wire [3:0]         pg_end_valid, pg_mask;
    wire [4:0]         pg_plog;
    ast_perf_sampler u_perf (
        .clk(clk),.rst(rst),.step_en(step_en),.start(start_search && !game_owns),
        .run_mask(run_mask),.start_idx(start_idx),.goal_idx(goal_idx),
        .current({dfs_current,ast_current,dij_current,bfs_current}),
        .expansions({dfs_exp,ast_exp,dij_exp,bfs_exp}),
        .done({dfs_done,ast_done,dij_done,bfs_done}),
        .we(pg_we),.waddr(pg_waddr),.wdata(pg_wdata),.count(pg_count),.rot(pg_rot),
        .end_idx(pg_end),.end_valid(pg_end_valid),.mask(pg_mask),.plog(pg_plog),
        .dbg_best()
    );
    wire [PG_AW-1:0]   pg_rd_idx;                   // from gv_oled_plot: sample under the pixel
    wire [4*PG_LW-1:0] pg_rd_level, pg_rd_prev;
    ast_perf_hist u_perf_hist (
        .clk(clk),.we(pg_we),.waddr(pg_waddr),.wdata(pg_wdata),.rot(pg_rot),
        .rd_idx(pg_rd_idx),.rd_level(pg_rd_level),.rd_prev_level(pg_rd_prev)
    );
    wire [15:0] screen_px, map_px;
    oled_screen u_oled_screen (
        .pixel_index(oled_pixel_index),.view(screen_view),.sel(oled_sel),
        .snap_mask(snap_mask),.snap_status(snap_status),.snap_place(snap_place),
        .snap_order(snap_order),.cyc_bcd(cyc_bcd),.gap_bcd(gap_bcd),.score_bcd(oled_score),
        .pixel_data(screen_px)
    );
    // sw[8]: the VGA picture on the OLED (same view selection: 0 EDIT, 1 RUN, 2 GAME)
    oled_map u_oled_map (
        .pixel_index(oled_pixel_index),.view(oled_view),.cell_addr(oled_cell),.cell_cost(maze_cost),
        .cell_state({dfs_state,ast_state,dij_state,bfs_state}),
        .current({dfs_current,ast_current,dij_current,bfs_current}),
        .busy({dfs_busy,ast_busy,dij_busy,bfs_busy}),.run_mask(run_mask),.place(snap_place),
        .start_idx(start_idx),.goal_idx(goal_idx),.cursor_valid(cursor_valid),.cursor_idx(mouse_idx),
        .player_pos(g_player),.player_dir(g_dir),.ghost_pos(g_ghosts),.ghost_present(g_ghost_present),
        .pellet(g_pellet),.pixel_data(map_px)
    );
    // demo results view (LOAD..DONE): M1's timing header + live performance graph
    wire [15:0] plot_px;
    gv_oled_plot u_gv_plot (
        .pixel_index(oled_pixel_index),.pixel_data(plot_px),
        .rd_idx(pg_rd_idx),.rd_level(pg_rd_level),.rd_prev_level(pg_rd_prev),
        .count(pg_count),.mask(pg_mask),.end_idx(pg_end),.end_valid(pg_end_valid),
        .snap_mask(snap_mask),.snap_status(snap_status),.snap_place(snap_place),
        .snap_order(snap_order),.cyc_bcd(cyc_bcd),.gap_bcd(gap_bcd)
    );
    // menu, points leaderboard and algorithm pages (M1); the points count demo races
    wire [15:0] menu_px;
    gv_oled_menu u_gv_menu (
        .clk(clk),.rst(rst),.start(start_search && !game_owns),.run_mask(run_mask),
        .done({dfs_done,ast_done,dij_done,bfs_done}),.found({dfs_found,ast_found,dij_found,bfs_found}),
        .cycles({dfs_cycles,ast_cycles,dij_cycles,bfs_cycles}),
        .page(oled_page),.pixel_index(oled_pixel_index),.pixel_data(menu_px)
    );
    reg [15:0] page_px;
    always @* begin
        case (oled_page)
            3'd1, 3'd4, 3'd5: page_px = menu_px;                 // menu / points / algorithms
            3'd2:             page_px = plot_px;                 // graph
            3'd3:             page_px = screen_px;               // race results (view forced to 1)
            default:          page_px = (oled_view == 2'd1) ? plot_px : screen_px;   // automatic
        endcase
    end
    assign oled_pixel_data = oled_mirror ? map_px : page_px;
    Oled_Display u_oled (
        .clk(clk6p25m),.reset(oled_rst_sync),.frame_begin(),.sending_pixels(),.sample_pixel(),
        .pixel_index(oled_pixel_index),.pixel_data(oled_pixel_data),
        .cs(oled_cs),.sdin(oled_sdin),.sclk(oled_sclk),.d_cn(oled_d_cn),
        .resn(oled_resn),.vccen(oled_vccen),.pmoden(oled_pmoden)
    );

    // LEDs summarize each selected solver (off when not in the run, and dark
    // in GAME mode, where the ghosts' searches would only flicker them);
    // unused 7-segment display is dark (active-low).
    wire led_on = !game_go;
    assign led[2:0] = {bfs_found,bfs_done,bfs_busy} & {3{run_mask[0] && led_on}};
    assign led[5:3] = {dij_found,dij_done,dij_busy} & {3{run_mask[1] && led_on}};
    assign led[8:6] = {ast_found,ast_done,ast_busy} & {3{run_mask[2] && led_on}};
    assign led[11:9] = 3'b000;
    assign led[14:12] = {dfs_found,dfs_done,dfs_busy} & {3{run_mask[3] && led_on}};   // M2's LEDs: DFS status
    assign led[15] = mode[0];
    assign seg = 7'b1111111;
    assign dp = 1'b1;
    assign an = 4'b1111;
endmodule
