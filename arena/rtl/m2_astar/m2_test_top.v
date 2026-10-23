`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// m2_test_top : M2's private board-bring-up top for the A* engine
//
// NOT part of the arena build (arena_top is M3's).  Set m2_test_top as the
// top module and use the MASTER constraints file constraints/arena_basys3.xdc.
// Like arena_top (CONTRACT 9), it has every port that XDC constrains, so the
// build has no "no ports matched" critical warnings.  Ports this test does
// not use are tied off safely (see "unused board ports" at the bottom).
//
// Controls
//   btnU          reset (debounced, CONTRACT 2)
//   btnC          load the selected maze into the engine, then start A*
//   sw[1:0]       maze: 0 open, 1 weighted, 2 unreachable, 3 corridor
//                 (same mazes + start/goal as sim/mazes/*.mem, see ast_test_maze)
//   sw[3:2]       7-seg shows (hex): 0 path_cost, 1 expansions,
//                                    2 path_cells, 3 cycles[15:0]
//   sw[14]        slow motion: one engine step every 2^SLOW_W clocks (~1.5 kHz)
//   sw[13]        with sw[14]: extra slow, every 2^SLOWER_W clocks (~95 Hz),
//                 slow enough to watch single expansions on the OLED
//   sw[15]        pause (step_en = 0)
// LEDs
//   led[0] busy  led[1] done  led[2] found  led[3] no_path
//   led[5:4] maze that was run   led[6] loading   led[15:12] engine FSM state
// Pmod OLED (plug into JB)
//   live view of the maze selected on sw[1:0] (see ast_oled_view for colours):
//   grey walls, brown weights, magenta start, red goal; once that maze has
//   been run: yellow frontier, blue visited, white current node, green path.
//   Bottom bar: yellow busy, green found, red no path.
//   Flip sw[1:0] to preview another maze before pressing btnC.
//
// Expected on the board (hex on the 7-seg), from sim/mazes/*.ast.expect:
//   open        cost 001A (26)  expansions 001B (27)
//               cycles 1624 (5668) with USE_HEAP=0, 028D (653) with USE_HEAP=1
//   weighted    cost 0017 (23)  expansions 005F (95)
//   unreachable no_path,        expansions 00B7 (183)
//   corridor    cost 0064 (100) expansions 0065 (101)
// cycles must be identical in slow motion and after pausing.
//////////////////////////////////////////////////////////////////////////////
module m2_test_top #(
    parameter DEBOUNCE_LOCKOUT = 20_000_000,   // 200 ms (tb uses a tiny value)
    parameter SLOW_W           = 16,           // slow motion: 100 MHz / 2^16 = 1.5 kHz steps
    parameter SLOWER_W         = 20,           // extra slow:  100 MHz / 2^20 = 95 Hz steps
    parameter REFRESH_W        = 18,           // 7-seg multiplex rate
    parameter USE_HEAP         = 0             // astar_engine open set: 0 scan, 1 heap
) (
    // ---- used by this test ----
    input  wire        clk,
    input  wire        btnU,
    input  wire        btnC,
    input  wire [15:0] sw,
    output wire [15:0] led,
    output wire [6:0]  seg,
    output wire        dp,
    output wire [3:0]  an,

    // ---- in the master XDC but unused here (tied off below) ----
    input  wire        btnL,
    input  wire        btnR,
    input  wire        btnD,
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

    // ---- buttons (debounced -> 1-cycle pulses) and switches (2-FF sync) ----
    wire rst, run_pulse;

    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_rst (.CLOCK(clk), .btn(btnU), .press_pulse(rst));
    debounce_btn #(.LOCKOUT(DEBOUNCE_LOCKOUT)) u_db_run (.CLOCK(clk), .btn(btnC), .press_pulse(run_pulse));

    reg [15:0] sw_s1 = 16'd0, sw_s2 = 16'd0;
    always @(posedge clk) begin
        sw_s1 <= sw;
        sw_s2 <= sw_s1;
    end

    wire [1:0] maze_sel = sw_s2[1:0];
    wire [1:0] disp_sel = sw_s2[3:2];
    wire       slower   = sw_s2[13];
    wire       slow     = sw_s2[14];
    wire       pause    = sw_s2[15];

    // ---- step_en: full speed, slow motion, extra slow, or paused ------------
    reg [SLOWER_W-1:0] slow_cnt = {SLOWER_W{1'b0}};
    always @(posedge clk)
        slow_cnt <= slow_cnt + 1'b1;

    wire slow_tick   = (slow_cnt[SLOW_W-1:0] == {SLOW_W{1'b1}});
    wire slower_tick = (slow_cnt == {SLOWER_W{1'b1}});
    wire step_en     = !pause && (!slow || (slower ? slower_tick : slow_tick));

    // ---- load-and-start sequencer ------------------------------------------
    localparam [1:0] T_IDLE = 2'd0, T_LOAD = 2'd1, T_START = 2'd2;

    reg [1:0]         t_state = T_IDLE;
    reg [INDEX_W-1:0] ld_ptr  = {INDEX_W{1'b0}};
    reg [1:0]         run_sel = 2'd0;          // maze latched at button press

    wire               busy, done, found, no_path;
    wire [COST_W-1:0]  maze_cost;
    wire [INDEX_W-1:0] maze_start, maze_goal;

    ast_test_maze u_maze (
        .sel(run_sel), .idx(ld_ptr),
        .cost(maze_cost), .start_idx(maze_start), .goal_idx(maze_goal)
    );

    always @(posedge clk) begin
        if (rst) begin
            t_state <= T_IDLE;
            ld_ptr  <= {INDEX_W{1'b0}};
        end else begin
            case (t_state)
                T_IDLE: if (run_pulse && !busy) begin  // loads only while not busy (4.1.1)
                    run_sel <= maze_sel;
                    ld_ptr  <= {INDEX_W{1'b0}};
                    t_state <= T_LOAD;
                end
                T_LOAD: begin                      // one cell per clock
                    ld_ptr <= ld_ptr + 1'b1;
                    if (ld_ptr == LAST)
                        t_state <= T_START;
                end
                default: t_state <= T_IDLE;        // T_START: start pulses this cycle
            endcase
        end
    end

    // ---- the A* engine --------------------------------------------------------
    wire [DIST_W-1:0]  path_cells, path_cost, expansions;
    wire [CNT_W-1:0]   cycles;
    wire [INDEX_W-1:0] path_rd_data, current;
    wire [1:0]         cell_state;
    wire [3:0]         dbg_state;
    wire [INDEX_W-1:0] view_cell;              // cell the OLED is drawing (see below)

    astar_engine #(.USE_HEAP(USE_HEAP)) u_ast (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(t_state == T_LOAD), .load_addr(ld_ptr), .load_data(maze_cost),
        .start(t_state == T_START), .start_idx(maze_start), .goal_idx(maze_goal),
        .busy(busy), .done(done), .found(found), .no_path(no_path),
        .path_rd_addr({INDEX_W{1'b0}}), .path_rd_data(path_rd_data),
        .path_cells(path_cells), .path_cost(path_cost),
        .current(current), .cell_rd_addr(view_cell), .cell_state(cell_state),
        .cycles(cycles), .expansions(expansions),
        .dbg_state(dbg_state)
    );

    // ---- outputs ----------------------------------------------------------------
    reg [15:0] disp_value;
    always @(*) begin
        case (disp_sel)
            2'd0:    disp_value = path_cost;
            2'd1:    disp_value = expansions;
            2'd2:    disp_value = path_cells;
            default: disp_value = cycles[15:0];
        endcase
    end

    ast_sevenseg_hex #(.REFRESH_W(REFRESH_W)) u_7seg (
        .clk(clk), .rst(rst), .value(disp_value), .seg(seg), .an(an), .dp(dp)
    );

    assign led[0]     = busy;
    assign led[1]     = done;
    assign led[2]     = found;
    assign led[3]     = no_path;
    assign led[5:4]   = run_sel;
    assign led[6]     = (t_state == T_LOAD);
    assign led[11:7]  = 5'd0;
    assign led[15:12] = dbg_state;

    // ---- Pmod OLED live view -------------------------------------------------------
    // CONTRACT 2: only Oled_Display and the logic making its pixel_data run on
    // the 6.25 MHz clock (from common/clk_div, instance named u_oled_clk to
    // match the commented create_generated_clock line in the master XDC).
    wire        clk6p25m;
    wire [12:0] pixel_index;
    wire [15:0] pixel_data;

    clk_div #(.DIV(`ARENA_OLED_CLK_DIV)) u_oled_clk (.clk(clk), .clk_out(clk6p25m));

    // Search overlay only when the OLED shows the maze that was actually run.
    wire overlay_fast = (maze_sel == run_sel) && (busy || done);

    // Status bits cross into the OLED clock domain through 2-FF synchronisers.
    // The multi-bit values (cell_state, current, start/goal, maze_sel) are
    // read directly: they change at most once per engine step and a wrong
    // value can only mis-colour one pixel for one frame, which is invisible.
    // (A test-only shortcut; M3's dashboard uses the CONTRACT 8 snapshot rule.)
    reg [3:0] st_s1 = 4'd0, st_s2 = 4'd0;
    always @(posedge clk6p25m) begin
        st_s1 <= {overlay_fast, busy, found, no_path};
        st_s2 <= st_s1;
    end

    wire [COST_W-1:0]  view_cost;
    wire [INDEX_W-1:0] view_start, view_goal;

    ast_test_maze u_view_maze (                // second, read-only copy of the ROM
        .sel(maze_sel), .idx(view_cell),
        .cost(view_cost), .start_idx(view_start), .goal_idx(view_goal)
    );

    ast_oled_view u_view (
        .pixel_index(pixel_index), .cell_addr(view_cell),
        .cell_state(cell_state), .cell_cost(view_cost),
        .start_idx(view_start), .goal_idx(view_goal), .current_idx(current),
        .overlay(st_s2[3]), .busy(st_s2[2]), .found(st_s2[1]), .no_path(st_s2[0]),
        .pixel_data(pixel_data)
    );

    Oled_Display u_oled (
        .clk(clk6p25m), .reset(1'b0),
        .frame_begin(), .sending_pixels(), .sample_pixel(),
        .pixel_index(pixel_index), .pixel_data(pixel_data),
        .cs(oled_cs), .sdin(oled_sdin), .sclk(oled_sclk), .d_cn(oled_d_cn),
        .resn(oled_resn), .vccen(oled_vccen), .pmoden(oled_pmoden)
    );

    // ---- unused board ports (CONTRACT 9: tie off, don't delete) ---------------
    // btnL / btnR / btnD: inputs, simply not used.
    assign vga_r  = 4'd0;                       // VGA: black, no sync pulses
    assign vga_g  = 4'd0;
    assign vga_b  = 4'd0;
    assign vga_hs = 1'b0;
    assign vga_vs = 1'b0;
    assign ps2_clk  = 1'bz;                     // PS/2: released (XDC pull-ups hold it idle)
    assign ps2_data = 1'bz;
endmodule
