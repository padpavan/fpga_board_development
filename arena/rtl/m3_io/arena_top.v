`timescale 1ns / 1ps
`include "arena_defs.vh"
// Integrated Basys 3 top for the FPGA Pathfinding Arena.
module arena_top #(
    parameter DEBOUNCE_LOCKOUT=20_000_000,
    parameter GEN_CELLS=`ARENA_NUM_CELLS,
    parameter OLED_SNAP_DIV=`ARENA_CLK_HZ/10     // OLED stats refresh (10 Hz); small in simulation
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
    wire rst, run_pulse, pause_pulse, clear_pulse, random_pulse;
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_rst (.CLOCK(clk),.btn(btnU),.press_pulse(rst));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_run (.CLOCK(clk),.btn(btnC),.press_pulse(run_pulse));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_pause (.CLOCK(clk),.btn(btnL),.press_pulse(pause_pulse));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_clear (.CLOCK(clk),.btn(btnR),.press_pulse(clear_pulse));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_random (.CLOCK(clk),.btn(btnD),.press_pulse(random_pulse));

    reg [15:0] sw_meta, sw_sync;
    always @(posedge clk) begin
        sw_meta <= sw;
        sw_sync <= sw_meta;
    end

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
    maze_gen #(.NUM_CELLS(GEN_CELLS)) u_maze_gen (
        .clk(clk),.rst(rst),.start(gen_start),.randomize(gen_random),
        .start_idx(start_idx),.goal_idx(goal_idx),
        .busy(gen_busy),.done(gen_done),.we(gen_we),.addr(gen_addr),.data(gen_data)
    );

    sys_ctrl u_sys_ctrl (
        .clk(clk),.rst(rst),.run_pulse(run_pulse),.pause_pulse(pause_pulse),
        .clear_pulse(clear_pulse),.random_pulse(random_pulse),.speed_sel(sw_sync[15:13]),
        .mouse_left(mouse_left_pulse && cursor_valid),.mouse_right(mouse_right_pulse && cursor_valid),
        .mouse_idx(mouse_idx),.edit_tool(sw_sync[1:0]),.gen_busy(gen_busy),.gen_done(gen_done),.engines_done(eng_done),.run_sel(sw_sync[5:2]),
        .gen_start(gen_start),.gen_random(gen_random),.maze_we(maze_we),.maze_addr(maze_waddr),
        .maze_data(maze_wdata),.load_we(load_we),.load_addr(load_addr),.start_pulse(start_search),
        .step_en(step_en),.edit_mode(edit_mode),.start_idx(start_idx),.goal_idx(goal_idx),
        .mode(mode),.paused(paused),.run_mask(run_mask)
    );
    // VGA read addresses / data (clk_pix domain): cell, up, down, left, right
    wire [INDEX_W-1:0] vga_up_addr, vga_dn_addr, vga_lf_addr, vga_rt_addr;
    wire [COST_W-1:0] vga_up_cost, vga_dn_cost, vga_lf_cost, vga_rt_cost;
    maze_ram u_maze_ram (
        .clk(clk),.rst(rst),.we(gen_we || maze_we),
        .waddr(gen_we ? gen_addr : maze_waddr),.wdata(gen_we ? gen_data : maze_wdata),
        .raddr(load_addr),.rdata(maze_load_cost),
        .vga_addr0(maze_rd_addr),.vga_addr1(vga_up_addr),.vga_addr2(vga_dn_addr),
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
        load_we_q   <= load_we && !rst;
        load_addr_q <= load_addr;
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
    wire [INDEX_W-1:0] ignored_bfs_path, ignored_dij_path, ignored_ast_path, ignored_dfs_path;
    assign eng_done = {dfs_done,ast_done,dij_done,bfs_done};
    pathfind_top u_pathfind (
        .clk(clk),.rst(rst),.step_en(step_en),.load_we(load_we_q),.load_addr(load_addr_q),.load_data(load_data_q),
        .start(start_search),.run_mask(run_mask),.start_idx(start_idx),.goal_idx(goal_idx),.cell_rd_addr(maze_rd_addr),
        .bfs_busy(bfs_busy),.bfs_done(bfs_done),.bfs_found(bfs_found),.bfs_no_path(bfs_no_path),
        .bfs_path_rd_addr({INDEX_W{1'b0}}),.bfs_path_rd_data(ignored_bfs_path),.bfs_path_cells(bfs_cells),
        .bfs_path_cost(bfs_cost),.bfs_current(bfs_current),.bfs_cell_state(bfs_state),
        .bfs_cycles(bfs_cycles),.bfs_expansions(bfs_exp),
        .dij_busy(dij_busy),.dij_done(dij_done),.dij_found(dij_found),.dij_no_path(dij_no_path),
        .dij_path_rd_addr({INDEX_W{1'b0}}),.dij_path_rd_data(ignored_dij_path),.dij_path_cells(dij_cells),
        .dij_path_cost(dij_cost),.dij_current(dij_current),.dij_cell_state(dij_state),
        .dij_cycles(dij_cycles),.dij_expansions(dij_exp),
        .ast_busy(ast_busy),.ast_done(ast_done),.ast_found(ast_found),.ast_no_path(ast_no_path),
        .ast_path_rd_addr({INDEX_W{1'b0}}),.ast_path_rd_data(ignored_ast_path),.ast_path_cells(ast_cells),
        .ast_path_cost(ast_cost),.ast_current(ast_current),.ast_cell_state(ast_state),
        .ast_cycles(ast_cycles),.ast_expansions(ast_exp),.ast_dbg_state(),
        .dfs_busy(dfs_busy),.dfs_done(dfs_done),.dfs_found(dfs_found),.dfs_no_path(dfs_no_path),
        .dfs_path_rd_addr({INDEX_W{1'b0}}),.dfs_path_rd_data(ignored_dfs_path),.dfs_path_cells(dfs_cells),
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

    // scroll-wheel zoom of the RUN view (100 MHz domain, next to the mouse)
    wire signed [OW-1:0] zoom_ox, zoom_oy;
    wire [KW-1:0] zoom_k;
    wire [2:0] zoom_level;
    vga_zoom #(.FRAC(FRAC),.OW(OW),.KW(KW)) u_vga_zoom (
        .clk(clk),.rst(rst),.enable(!edit_mode),
        .new_event(mouse_event),.zpos(mouse_z),.middle(mouse_middle),
        .mouse_x(mouse_x),.mouse_y(mouse_y),
        .ox(zoom_ox),.oy(zoom_oy),.k(zoom_k),.level(zoom_level)
    );

    // everything the renderer needs from the 100 MHz side, moved across once per frame
    localparam VIEW_W = 2*OW + KW + 3 + 12 + 12 + 1 + INDEX_W + 1 + 1 + 2 + 6*INDEX_W + 4;
    wire [VIEW_W-1:0] view_a = {zoom_ox, zoom_oy, zoom_k, zoom_level, mouse_x, mouse_y,
        cursor_valid, mouse_idx, edit_mode, sw_sync[6], sw_sync[1:0],
        start_idx, goal_idx, bfs_current, dij_current, ast_current, dfs_current, run_mask};
    wire [VIEW_W-1:0] view_p;
    wire view_tick;
    vga_cdc #(.W(VIEW_W)) u_vga_cdc (
        .clk_a(clk),.rst_a(rst),.din(view_a),
        .clk_b(clk_pix),.rst_b(rst_pix),.apply_b(view_tick),.dout(view_p)
    );
    wire signed [OW-1:0] v_ox, v_oy;
    wire [KW-1:0] v_k;
    wire [2:0] v_level;
    wire [11:0] v_mouse_x, v_mouse_y;
    wire v_cursor_valid, v_edit, v_pacman;
    wire [1:0] v_tool;
    wire [INDEX_W-1:0] v_cursor_idx, v_start, v_goal, v_bfs_cur, v_dij_cur, v_ast_cur, v_dfs_cur;
    wire [3:0] v_run_mask;
    assign {v_ox, v_oy, v_k, v_level, v_mouse_x, v_mouse_y, v_cursor_valid, v_cursor_idx,
            v_edit, v_pacman, v_tool, v_start, v_goal, v_bfs_cur, v_dij_cur, v_ast_cur,
            v_dfs_cur, v_run_mask} = view_p;

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
        .red(vga_r),.green(vga_g),.blue(vga_b),.hsync(vga_hs),.vsync(vga_vs)
    );

    // Snapshot + finish order + decimal conversion for the OLED (CONTRACT 8):
    // oled_stats registers everything in the 100 MHz domain; the 6.25 MHz
    // dashboard reads only those registers.
    wire [3:0]  snap_mask;
    wire [15:0] snap_status;
    wire [11:0] snap_place;
    wire [95:0] cyc_bcd;
    wire [47:0] exp_bcd;
    wire [63:0] cost_bcd;
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
        .cyc_bcd(cyc_bcd),.exp_bcd(exp_bcd),.cost_bcd(cost_bcd)
    );
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
    oled_dashboard u_oled_dashboard (
        .pixel_index(oled_pixel_index),
        .snap_mask(snap_mask),.snap_status(snap_status),.snap_place(snap_place),
        .cyc_bcd(cyc_bcd),.exp_bcd(exp_bcd),.cost_bcd(cost_bcd),
        .pixel_data(oled_pixel_data)
    );
    Oled_Display u_oled (
        .clk(clk6p25m),.reset(oled_rst_sync),.frame_begin(),.sending_pixels(),.sample_pixel(),
        .pixel_index(oled_pixel_index),.pixel_data(oled_pixel_data),
        .cs(oled_cs),.sdin(oled_sdin),.sclk(oled_sclk),.d_cn(oled_d_cn),
        .resn(oled_resn),.vccen(oled_vccen),.pmoden(oled_pmoden)
    );

    // LEDs summarize each selected solver (off when not in the run);
    // unused 7-segment display is dark (active-low).
    assign led[2:0] = {bfs_found,bfs_done,bfs_busy} & {3{run_mask[0]}};
    assign led[5:3] = {dij_found,dij_done,dij_busy} & {3{run_mask[1]}};
    assign led[8:6] = {ast_found,ast_done,ast_busy} & {3{run_mask[2]}};
    assign led[11:9] = 3'b000;
    assign led[14:12] = {dfs_found,dfs_done,dfs_busy} & {3{run_mask[3]}};   // M2's LEDs: DFS status
    assign led[15] = mode[0];
    assign seg = 7'b1111111;
    assign dp = 1'b1;
    assign an = 4'b1111;
endmodule
