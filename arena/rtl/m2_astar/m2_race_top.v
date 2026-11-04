`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// m2_race_top : M2's board top for the 4-engine race, viewed on the Pmod OLED
//
// NOT part of the arena build (arena_top is M3's).  For testing without a VGA
// monitor or mouse: set m2_race_top as the top module and use the MASTER
// constraints file constraints/arena_basys3.xdc.  Like arena_top (CONTRACT 9)
// it has every port that XDC constrains; unused ones are tied off.
//
// Here the OLED owns pathfind_top's shared cell_rd_addr port (in arena_top it
// belongs to VGA), so the OLED can draw all four engines' maps live.
//
// Controls
//   btnU          reset
//   btnC          load the maze into the selected engines and start them all
//                 on the same clock (ignored while running or if sw[5:2] = 0)
//   btnL          new random maze (when sw[6] = 1)
//   sw[1:0]       built-in maze: 0 open, 1 weighted, 2 unreachable, 3 corridor
//                 (same mazes + start/goal as sim/mazes/*.mem, ast_test_maze)
//   sw[5:2]       engines to race: sw[2] BFS, sw[3] DIJ, sw[4] A*, sw[5] DFS
//   sw[6]         1 = random maze (maze_gen, start 0 -> goal 191)
//   sw[7]         OLED page: 0 = 4 maps (race view), 1 = stats dashboard
//   sw[12]        pause
//   sw[15:13]     speed: 0 full (whole race takes < 60 us), 1..7 one engine
//                 step every 2^(speed+SPEED_SHIFT) clocks; 7 = ~190 steps/s
//                 (DFS ~4 s, BFS ~6 s, DIJ ~12 s, A* ~30 s on the open maze)
// Outputs
//   OLED map page: BFS | DIJ / A* | DFS, each with its finish place "#1".."#4"
//                  ("#-" running, "OFF" not selected), see ast_oled_map4.
//   OLED dashboard: oled_stats + oled_dashboard (same screen as arena_top).
//   7-seg: finish place of each engine, digits left->right DFS A* DIJ BFS
//          (lined up with sw[5:2]); 0 = not run / still running.
//   led[0] maze loading   led[1] race finished   led[5:2] busy
//   led[9:6] found        led[13:10] no path     led[15:14] maze on sw[1:0]
// Changing the maze (sw[1:0], sw[6], btnL) after a race shows the new maze
// clean, with the selected engines labelled "#-", until btnC is pressed.
//////////////////////////////////////////////////////////////////////////////
module m2_race_top #(
    parameter DEBOUNCE_LOCKOUT = 20_000_000,   // 200 ms (tb uses a tiny value)
    parameter SPEED_SHIFT      = 12,           // speed s -> step every 2^(s+SHIFT) clocks
    parameter SNAP_DIV         = `ARENA_CLK_HZ / 10,
    parameter REFRESH_W        = 18
) (
    input  wire        clk,
    input  wire        btnU,
    input  wire        btnC,
    input  wire        btnL,
    input  wire        btnR,
    input  wire        btnD,
    input  wire [15:0] sw,
    output wire [15:0] led,
    output wire [6:0]  seg,
    output wire        dp,
    output wire [3:0]  an,
    output wire [3:0]  vga_r,
    output wire [3:0]  vga_g,
    output wire [3:0]  vga_b,
    output wire        vga_hs,
    output wire        vga_vs,
    inout  wire        ps2_clk,
    inout  wire        ps2_data,
    output wire        oled_cs,
    output wire        oled_sdin,
    output wire        oled_sclk,
    output wire        oled_d_cn,
    output wire        oled_resn,
    output wire        oled_vccen,
    output wire        oled_pmoden
);
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam COST_W    = `ARENA_COST_W;
    localparam DIST_W    = `ARENA_DIST_W;
    localparam CNT_W     = `ARENA_CNT_W;
    localparam [INDEX_W-1:0] LAST = NUM_CELLS - 1;

    // ---- buttons and switches ------------------------------------------------
    wire rst, run_pulse, rand_pulse;
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_rst  (.CLOCK(clk), .btn(btnU), .press_pulse(rst));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_run  (.CLOCK(clk), .btn(btnC), .press_pulse(run_pulse));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_rand (.CLOCK(clk), .btn(btnL), .press_pulse(rand_pulse));

    reg [15:0] sw_s1 = 16'd0, sw_s2 = 16'd0;
    always @(posedge clk) begin
        sw_s1 <= sw;
        sw_s2 <= sw_s1;
    end
    wire [1:0] sw_maze  = sw_s2[1:0];
    wire [3:0] sw_mask  = sw_s2[5:2];
    wire       sw_rand  = sw_s2[6];
    wire       sw_page  = sw_s2[7];
    wire       pause    = sw_s2[12];
    wire [2:0] speed    = sw_s2[15:13];

    // ---- step_en ----------------------------------------------------------------
    localparam SPD_W = SPEED_SHIFT + 8;
    reg  [SPD_W-1:0] spd_cnt = {SPD_W{1'b0}};
    wire [SPD_W-1:0] spd_lim = ({{(SPD_W-1){1'b0}}, 1'b1} << (speed + SPEED_SHIFT)) - 1'b1;
    wire             spd_hit = (speed == 3'd0) || (spd_cnt >= spd_lim);
    always @(posedge clk)
        spd_cnt <= (pause || spd_hit) ? {SPD_W{1'b0}} : spd_cnt + 1'b1;
    wire step_en = !pause && spd_hit;

    // ---- engines -------------------------------------------------------------------
    wire [3:0] busy, done, found, no_path;     // {dfs, ast, dij, bfs}
    wire [4*CNT_W-1:0]   cycles;
    wire [4*DIST_W-1:0]  expansions, path_cost, path_cells;
    wire [4*INDEX_W-1:0] current;
    wire [4*2-1:0]       cell_state;
    wire [INDEX_W-1:0]   view_cell;            // OLED map: cell being drawn
    wire [INDEX_W-1:0]   unused_path [0:3];
    wire                 any_busy = |busy;

    reg                 eng_we   = 1'b0;
    reg [INDEX_W-1:0]   eng_addr = {INDEX_W{1'b0}};
    reg [COST_W-1:0]    eng_data = {COST_W{1'b0}};
    reg [3:0]           run_mask = 4'b0000;
    wire                eng_start;
    reg [INDEX_W-1:0]   maze_start = {INDEX_W{1'b0}}, maze_goal = {INDEX_W{1'b0}};

    pathfind_top u_pathfind (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(eng_we), .load_addr(eng_addr), .load_data(eng_data),
        .start(eng_start), .run_mask(run_mask), .start_idx(maze_start), .goal_idx(maze_goal),
        .cell_rd_addr(view_cell),
        .bfs_busy(busy[0]), .bfs_done(done[0]), .bfs_found(found[0]), .bfs_no_path(no_path[0]),
        .bfs_path_rd_addr({INDEX_W{1'b0}}), .bfs_path_rd_data(unused_path[0]),
        .bfs_path_cells(path_cells[0*DIST_W +: DIST_W]), .bfs_path_cost(path_cost[0*DIST_W +: DIST_W]),
        .bfs_current(current[0*INDEX_W +: INDEX_W]), .bfs_cell_state(cell_state[1:0]),
        .bfs_cycles(cycles[0*CNT_W +: CNT_W]), .bfs_expansions(expansions[0*DIST_W +: DIST_W]),
        .dij_busy(busy[1]), .dij_done(done[1]), .dij_found(found[1]), .dij_no_path(no_path[1]),
        .dij_path_rd_addr({INDEX_W{1'b0}}), .dij_path_rd_data(unused_path[1]),
        .dij_path_cells(path_cells[1*DIST_W +: DIST_W]), .dij_path_cost(path_cost[1*DIST_W +: DIST_W]),
        .dij_current(current[1*INDEX_W +: INDEX_W]), .dij_cell_state(cell_state[3:2]),
        .dij_cycles(cycles[1*CNT_W +: CNT_W]), .dij_expansions(expansions[1*DIST_W +: DIST_W]),
        .ast_busy(busy[2]), .ast_done(done[2]), .ast_found(found[2]), .ast_no_path(no_path[2]),
        .ast_path_rd_addr({INDEX_W{1'b0}}), .ast_path_rd_data(unused_path[2]),
        .ast_path_cells(path_cells[2*DIST_W +: DIST_W]), .ast_path_cost(path_cost[2*DIST_W +: DIST_W]),
        .ast_current(current[2*INDEX_W +: INDEX_W]), .ast_cell_state(cell_state[5:4]),
        .ast_cycles(cycles[2*CNT_W +: CNT_W]), .ast_expansions(expansions[2*DIST_W +: DIST_W]),
        .ast_dbg_state(),
        .dfs_busy(busy[3]), .dfs_done(done[3]), .dfs_found(found[3]), .dfs_no_path(no_path[3]),
        .dfs_path_rd_addr({INDEX_W{1'b0}}), .dfs_path_rd_data(unused_path[3]),
        .dfs_path_cells(path_cells[3*DIST_W +: DIST_W]), .dfs_path_cost(path_cost[3*DIST_W +: DIST_W]),
        .dfs_current(current[3*INDEX_W +: INDEX_W]), .dfs_cell_state(cell_state[7:6]),
        .dfs_cycles(cycles[3*CNT_W +: CNT_W]), .dfs_expansions(expansions[3*DIST_W +: DIST_W])
    );

    // ---- maze store: two copies, one read by the engine loader, one by the OLED
    // Filled from the built-in ROM (ast_test_maze) or from maze_gen (random).
    reg                maze_valid = 1'b0;      // a fill has completed
    reg  [2:0]         maze_id    = 3'd0;      // {random, sw[1:0]} of the stored maze
    reg                rand_req   = 1'b0;
    reg                stale      = 1'b1;      // stored maze not raced yet
    wire [2:0]         want_id    = {sw_rand, sw_maze};
    wire               need_fill  = !maze_valid || (want_id != maze_id) || (rand_req && sw_rand);

    localparam [1:0] F_IDLE = 2'd0, F_ROM = 2'd1, F_GEN = 2'd2;
    localparam [2:0] L_IDLE = 3'd0, L_LOAD = 3'd1, L_WAIT = 3'd2, L_START = 3'd3;
    reg [1:0]          f_state = F_IDLE;
    reg [2:0]          l_state = L_IDLE;
    reg [INDEX_W-1:0]  f_ptr = {INDEX_W{1'b0}}, l_ptr = {INDEX_W{1'b0}};

    wire [COST_W-1:0]  rom_cost;
    wire [INDEX_W-1:0] rom_start, rom_goal;
    ast_test_maze u_rom (
        .sel(maze_id[1:0]), .idx(f_ptr),
        .cost(rom_cost), .start_idx(rom_start), .goal_idx(rom_goal)
    );

    wire               gen_go = (f_state == F_IDLE) && need_fill && !any_busy &&
                                (l_state == L_IDLE) && sw_rand;
    wire               gen_busy, gen_done, gen_we;
    wire [INDEX_W-1:0] gen_addr;
    wire [COST_W-1:0]  gen_data;
    maze_gen u_gen (
        .clk(clk), .rst(rst), .start(gen_go), .randomize(1'b1),
        .start_idx({INDEX_W{1'b0}}), .goal_idx(LAST),
        .busy(gen_busy), .done(gen_done),
        .we(gen_we), .addr(gen_addr), .data(gen_data)
    );

    wire               mz_we   = (f_state == F_ROM) || gen_we;
    wire [INDEX_W-1:0] mz_addr = (f_state == F_ROM) ? f_ptr    : gen_addr;
    wire [COST_W-1:0]  mz_data = (f_state == F_ROM) ? rom_cost : gen_data;
    wire [COST_W-1:0]  eng_src, view_cost;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_mz_eng (
        .clk(clk), .we(mz_we), .waddr(mz_addr), .wdata(mz_data), .raddr(l_ptr), .rdata(eng_src));
    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_mz_view (
        .clk(clk), .we(mz_we), .waddr(mz_addr), .wdata(mz_data), .raddr(view_cell), .rdata(view_cost));

    always @(posedge clk) begin
        if (rst) begin
            f_state    <= F_IDLE;
            maze_valid <= 1'b0;
            rand_req   <= 1'b0;
            stale      <= 1'b1;
        end else begin
            if (rand_pulse && sw_rand) rand_req <= 1'b1;
            case (f_state)
                F_IDLE: if (need_fill && !any_busy && l_state == L_IDLE) begin
                    maze_id  <= want_id;
                    rand_req <= 1'b0;
                    stale    <= 1'b1;
                    f_ptr    <= {INDEX_W{1'b0}};
                    f_state  <= sw_rand ? F_GEN : F_ROM;
                end
                F_ROM: begin
                    f_ptr <= f_ptr + 1'b1;
                    if (f_ptr == LAST) begin
                        maze_valid <= 1'b1;
                        f_state    <= F_IDLE;
                    end
                end
                default: if (gen_done && !gen_busy && !gen_we) begin   // last write landed
                    maze_valid <= 1'b1;
                    f_state    <= F_IDLE;
                end
            endcase
            if (l_state == L_START) stale <= 1'b0;
        end
    end

    // ---- loader: maze -> engines' load bus (registered), then one start -----
    assign eng_start = (l_state == L_START);
    always @(posedge clk) begin
        eng_we <= 1'b0;
        if (rst) begin
            l_state  <= L_IDLE;
            run_mask <= 4'b0000;
        end else begin
            case (l_state)
                L_IDLE: if (run_pulse && !any_busy && sw_mask != 4'b0000 &&
                            f_state == F_IDLE && !need_fill) begin
                    run_mask   <= sw_mask;
                    maze_start <= maze_id[2] ? {INDEX_W{1'b0}} : rom_start;
                    maze_goal  <= maze_id[2] ? LAST            : rom_goal;
                    l_ptr      <= {INDEX_W{1'b0}};
                    l_state    <= L_LOAD;
                end
                L_LOAD: begin
                    eng_we   <= 1'b1;
                    eng_addr <= l_ptr;
                    eng_data <= eng_src;
                    l_ptr    <= l_ptr + 1'b1;
                    if (l_ptr == LAST) l_state <= L_WAIT;
                end
                L_WAIT:  l_state <= L_START;   // last registered write lands now
                default: l_state <= L_IDLE;    // L_START: start pulses this cycle
            endcase
        end
    end

    // ---- stats + finish places (100 MHz snapshot, CONTRACT 8) ---------------
    wire [3:0]  snap_mask;
    wire [15:0] snap_status;
    wire [11:0] snap_place;
    wire [95:0] cyc_bcd;
    wire [47:0] exp_bcd;
    wire [63:0] cost_bcd;
    oled_stats #(.SNAP_DIV(SNAP_DIV)) u_stats (
        .clk(clk), .rst(rst), .run_mask(run_mask),
        .cycles(cycles), .expansions(expansions), .path_cost(path_cost),
        .busy(busy), .done(done), .found(found), .no_path(no_path),
        .snap_mask(snap_mask), .snap_status(snap_status), .snap_place(snap_place),
        .cyc_bcd(cyc_bcd), .exp_bcd(exp_bcd), .cost_bcd(cost_bcd)
    );

    // What the map page shows: the race, or (stale) the clean new maze with the
    // currently selected engines labelled "#-".
    reg [3:0]           view_mask  = 4'b0000;
    reg [3:0]           view_busy  = 4'b0000;
    reg [11:0]          view_place = 12'd0;
    reg                 view_live  = 1'b0;
    reg [INDEX_W-1:0]   view_start = {INDEX_W{1'b0}}, view_goal = {INDEX_W{1'b0}};
    always @(posedge clk) begin
        view_live  <= !stale;
        view_mask  <= stale ? sw_mask : run_mask;
        view_busy  <= stale ? 4'b0000 : busy;
        view_place <= stale ? 12'd0   : snap_place;
        view_start <= maze_id[2] ? {INDEX_W{1'b0}} : rom_start;
        view_goal  <= maze_id[2] ? LAST            : rom_goal;
    end

    // ---- Pmod OLED (6.25 MHz domain) ----------------------------------------
    // The 100 MHz view_* registers and engine outputs are read directly: they
    // change at most once per engine step, so a sampling glitch can only
    // mis-colour one pixel for one frame (same test shortcut as m2_test_top).
    wire        clk6p25m;
    wire [12:0] pixel_index;
    wire [15:0] map_pixel, dash_pixel;
    clk_div #(.DIV(`ARENA_OLED_CLK_DIV)) u_oled_clk (.clk(clk), .clk_out(clk6p25m));

    reg page_s1 = 1'b0, page_s2 = 1'b0;
    always @(posedge clk6p25m) begin
        page_s1 <= sw_page;
        page_s2 <= page_s1;
    end

    ast_oled_map4 u_map (
        .pixel_index(pixel_index), .cell_addr(view_cell), .quad(),
        .cell_cost(view_cost), .cell_state(view_live ? cell_state : 8'd0),
        .current(current), .busy(view_busy), .run_mask(view_mask), .place(view_place),
        .start_idx(view_start), .goal_idx(view_goal),
        .pixel_data(map_pixel)
    );

    oled_dashboard u_dash (
        .pixel_index(pixel_index),
        .snap_mask(snap_mask), .snap_status(snap_status), .snap_place(snap_place),
        .cyc_bcd(cyc_bcd), .exp_bcd(exp_bcd), .cost_bcd(cost_bcd),
        .pixel_data(dash_pixel)
    );

    Oled_Display u_oled (
        .clk(clk6p25m), .reset(1'b0),
        .frame_begin(), .sending_pixels(), .sample_pixel(),
        .pixel_index(pixel_index), .pixel_data(page_s2 ? dash_pixel : map_pixel),
        .cs(oled_cs), .sdin(oled_sdin), .sclk(oled_sclk), .d_cn(oled_d_cn),
        .resn(oled_resn), .vccen(oled_vccen), .pmoden(oled_pmoden)
    );

    // ---- 7-seg: finish places, LEDs ---------------------------------------------
    wire [15:0] seg_value = {1'b0, view_place[11:9], 1'b0, view_place[8:6],
                             1'b0, view_place[5:3],  1'b0, view_place[2:0]};
    ast_sevenseg_hex #(.REFRESH_W(REFRESH_W)) u_7seg (
        .clk(clk), .rst(rst), .value(seg_value), .seg(seg), .an(an), .dp(dp)
    );

    wire race_done = !stale && run_mask != 4'b0000 && &(done | ~run_mask);
    assign led[0]     = (f_state != F_IDLE) || (l_state != L_IDLE);
    assign led[1]     = race_done;
    assign led[5:2]   = busy & run_mask;
    assign led[9:6]   = done & found & run_mask;
    assign led[13:10] = done & no_path & run_mask;
    assign led[15:14] = maze_id[1:0];

    // ---- unused board ports (CONTRACT 9: tie off, don't delete) ---------------
    // btnR / btnD: inputs, not used.
    assign vga_r    = 4'd0;
    assign vga_g    = 4'd0;
    assign vga_b    = 4'd0;
    assign vga_hs   = 1'b0;
    assign vga_vs   = 1'b0;
    assign ps2_clk  = 1'bz;
    assign ps2_data = 1'bz;
endmodule
