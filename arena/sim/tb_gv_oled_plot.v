`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_gv_oled_plot : self-checking bench for M1's OLED results view with the
// performance graph (CONTRACT 8a).
//
// Real data: pathfind_top + M2's ast_perf_sampler + ast_perf_hist + oled_stats
// feed gv_oled_plot, exactly as in arena_top.  Scenarios (plusarg maze=, default
// rand3; the run used is the found run with the longest start-goal distance):
//   A  all four engines: frames mid-run (paused) and at the end
//   B  run_mask = 0101 (BFS + A*): unselected engines draw nothing, grey OFF
//   C  unreachable.mem: no path, curves stay below 100 %, NONE
// Every frame is checked pixel by pixel in the plot against a model of the
// drawing rules (curve fill between prev and current level, end at end_idx,
// dotted finish line, priority A* > DIJ > BFS > DFS).  Also: empty plot,
// axes, grid, axis labels, header colours per slot.
// Frames go to sim/build/oled_plot_<tag>.hex (python3 ref/oled_frames_to_png.py).
//////////////////////////////////////////////////////////////////////////////
module tb_gv_oled_plot;
    localparam COLS = `ARENA_COLS, NUM_CELLS = `ARENA_NUM_CELLS, INDEX_W = `ARENA_INDEX_W;
    localparam COST_W = `ARENA_COST_W, DIST_W = `ARENA_DIST_W, CNT_W = `ARENA_CNT_W;
    localparam AW = `ARENA_PG_ADDR_W, RW = `ARENA_PG_ROT_W, LW = `ARENA_PG_LEVEL_W;
    localparam [15:0] BLACK = 16'h0000, GREY = 16'h7BEF, AXIS = 16'h8410, GRID = 16'h31A6,
                      LABEL = 16'h6B4D;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst = 1'b1, step_en = 1'b1, load_we = 1'b0, start = 1'b0;
    reg [INDEX_W-1:0] load_addr = 0, start_idx = 0, goal_idx = 0;
    reg [COST_W-1:0]  load_data = 0;
    reg [3:0] run_mask = 4'hF;

    wire [3:0] busy, done, found, no_path;
    wire [INDEX_W-1:0] cur0, cur1, cur2, cur3;
    wire [CNT_W-1:0]   cyc0, cyc1, cyc2, cyc3;
    wire [DIST_W-1:0]  exp0, exp1, exp2, exp3, cost0, cost1, cost2, cost3;
    pathfind_top u_pf (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .run_mask(run_mask), .start_idx(start_idx), .goal_idx(goal_idx),
        .cell_rd_addr({INDEX_W{1'b0}}),
        .bfs_busy(busy[0]), .bfs_done(done[0]), .bfs_found(found[0]), .bfs_no_path(no_path[0]),
        .bfs_path_rd_addr({INDEX_W{1'b0}}), .bfs_path_rd_data(), .bfs_path_cells(), .bfs_path_cost(cost0),
        .bfs_current(cur0), .bfs_cell_state(), .bfs_cycles(cyc0), .bfs_expansions(exp0),
        .dij_busy(busy[1]), .dij_done(done[1]), .dij_found(found[1]), .dij_no_path(no_path[1]),
        .dij_path_rd_addr({INDEX_W{1'b0}}), .dij_path_rd_data(), .dij_path_cells(), .dij_path_cost(cost1),
        .dij_current(cur1), .dij_cell_state(), .dij_cycles(cyc1), .dij_expansions(exp1),
        .ast_busy(busy[2]), .ast_done(done[2]), .ast_found(found[2]), .ast_no_path(no_path[2]),
        .ast_path_rd_addr({INDEX_W{1'b0}}), .ast_path_rd_data(), .ast_path_cells(), .ast_path_cost(cost2),
        .ast_current(cur2), .ast_cell_state(), .ast_cycles(cyc2), .ast_expansions(exp2), .ast_dbg_state(),
        .dfs_busy(busy[3]), .dfs_done(done[3]), .dfs_found(found[3]), .dfs_no_path(no_path[3]),
        .dfs_path_rd_addr({INDEX_W{1'b0}}), .dfs_path_rd_data(), .dfs_path_cells(), .dfs_path_cost(cost3),
        .dfs_current(cur3), .dfs_cell_state(), .dfs_cycles(cyc3), .dfs_expansions(exp3)
    );

    wire            we;
    wire [AW-1:0]   waddr, count, rd_idx;
    wire [4*LW-1:0] wdata, rd_level, rd_prev;
    wire [RW-1:0]   rot;
    wire [4*AW-1:0] end_idx;
    wire [3:0]      end_valid, mask;
    wire [4:0]      plog;
    ast_perf_sampler u_s (
        .clk(clk), .rst(rst), .step_en(step_en), .start(start), .run_mask(run_mask),
        .start_idx(start_idx), .goal_idx(goal_idx),
        .current({cur3, cur2, cur1, cur0}), .expansions({exp3, exp2, exp1, exp0}), .done(done),
        .we(we), .waddr(waddr), .wdata(wdata), .count(count), .rot(rot),
        .end_idx(end_idx), .end_valid(end_valid), .mask(mask), .plog(plog), .dbg_best()
    );
    ast_perf_hist u_h (.clk(clk), .we(we), .waddr(waddr), .wdata(wdata), .rot(rot),
                       .rd_idx(rd_idx), .rd_level(rd_level), .rd_prev_level(rd_prev));

    wire [3:0]      snap_mask;
    wire [4*4-1:0]  snap_status;
    wire [4*3-1:0]  snap_place;
    wire [4*2-1:0]  snap_order;
    wire [4*24-1:0] cyc_bcd, gap_bcd;
    oled_stats #(.SNAP_DIV(400)) u_st (
        .clk(clk), .rst(rst), .run_mask(run_mask),
        .cycles({cyc3, cyc2, cyc1, cyc0}), .expansions({exp3, exp2, exp1, exp0}),
        .path_cost({cost3, cost2, cost1, cost0}),
        .busy(busy), .done(done), .found(found), .no_path(no_path),
        .snap_mask(snap_mask), .snap_status(snap_status), .snap_place(snap_place), .cyc_bcd(cyc_bcd),
        .exp_bcd(), .cost_bcd(), .gap_bcd(gap_bcd), .snap_order(snap_order)
    );

    reg  [12:0] pixel_index = 0;
    wire [15:0] pixel_data;
    gv_oled_plot dut (
        .pixel_index(pixel_index), .pixel_data(pixel_data),
        .rd_idx(rd_idx), .rd_level(rd_level), .rd_prev_level(rd_prev),
        .count(count), .mask(mask), .end_idx(end_idx), .end_valid(end_valid),
        .snap_mask(snap_mask), .snap_status(snap_status), .snap_place(snap_place),
        .snap_order(snap_order), .cyc_bcd(cyc_bcd), .gap_bcd(gap_bcd)
    );

    // ---------------------------------------------------------------- helpers
    integer errors = 0, checks = 0;
    task fail(input [8*60-1:0] msg); begin
        errors = errors + 1;
        if (errors <= 30) $display("FAIL %0s", msg);
    end endtask

    function [15:0] eng_colour; input [1:0] en;
        case (en) 2'd0: eng_colour = 16'h07FF; 2'd1: eng_colour = 16'hFFE0;
                  2'd2: eng_colour = 16'h07E0; default: eng_colour = 16'hFB35; endcase
    endfunction
    function is_eng; input [15:0] c;
        is_eng = (c == 16'h07FF) || (c == 16'hFFE0) || (c == 16'h07E0) || (c == 16'hFB35);
    endfunction

    task probe(input [6:0] px, input [5:0] py, input [15:0] exp, input [8*28-1:0] what); begin
        pixel_index = py * 96 + px; #1;
        checks = checks + 1;
        if (pixel_data !== exp) begin
            errors = errors + 1;
            $display("FAIL %0s: (%0d,%0d) = %h, expected %h", what, px, py, pixel_data, exp);
        end
    end endtask

    // whole plot vs a model of the drawing rules (uses the hist read that the
    // DUT itself drives through rd_idx for the same pixel)
    integer px_, py_, en, lvl, cl, pl, ei, bad_frame;
    reg [15:0] want;
    reg        want_eng;
    task check_plot(input [8*16-1:0] tag); begin
        bad_frame = 0;
        for (py_ = 16; py_ < 63; py_ = py_ + 1)
            for (px_ = 1; px_ < 96; px_ = px_ + 1) begin
                pixel_index = py_ * 96 + px_; #1;
                lvl = 62 - py_;
                want_eng = 1'b0; want = BLACK;
                // curves, priority A* (2) > DIJ (1) > BFS (0) > DFS (3)
                if (!want_eng) model_engine(2);
                if (!want_eng) model_engine(1);
                if (!want_eng) model_engine(0);
                if (!want_eng) model_engine(3);
                if (!want_eng) model_fin(2);
                if (!want_eng) model_fin(1);
                if (!want_eng) model_fin(0);
                if (!want_eng) model_fin(3);
                checks = checks + 1;
                if (want_eng ? (pixel_data !== want) : is_eng(pixel_data)) begin
                    bad_frame = bad_frame + 1;
                    if (bad_frame <= 5)
                        $display("FAIL %0s plot pixel (%0d,%0d) = %h, model %h (count=%0d)",
                                 tag, px_, py_, pixel_data, want_eng ? want : 16'hXXXX, count);
                end
            end
        errors = errors + bad_frame;
    end endtask
    task model_engine(input integer e); begin
        cl = rd_level[e*LW +: LW]; pl = rd_prev[e*LW +: LW]; ei = end_idx[e*AW +: AW];
        if (px_ - 1 < count && px_ - 1 < 94 && mask[e] && (!end_valid[e] || px_ - 1 <= ei) &&
            ((lvl >= pl && lvl <= cl) || (lvl <= pl && lvl >= cl))) begin
            want_eng = 1'b1; want = eng_colour(e);
        end
    end endtask
    task model_fin(input integer e); begin
        cl = rd_level[e*LW +: LW]; ei = end_idx[e*AW +: AW];
        if (px_ - 1 < count && px_ - 1 < 94 && mask[e] && end_valid[e] && px_ - 1 == ei &&
            lvl < cl && py_[0]) begin
            want_eng = 1'b1; want = eng_colour(e);
        end
    end endtask

    // header slot s (0..3): every lit pixel has the slot engine's colour (grey if not run)
    integer sx, sy, lit;
    reg [1:0] se;
    task check_header(input [8*16-1:0] tag); begin
        for (sx = 0; sx < 4; sx = sx + 1) begin
            se = snap_order[sx*2 +: 2];
            lit = 0;
            for (sy = 0; sy < 8; sy = sy + 1)
                for (px_ = 0; px_ < 48; px_ = px_ + 1) begin
                    pixel_index = ((sx >> 1) * 8 + sy) * 96 + (sx & 1) * 48 + px_; #1;
                    if (pixel_data != BLACK) begin
                        lit = lit + 1;
                        if (pixel_data !== (snap_mask[se] ? eng_colour(se) : GREY)) begin
                            errors = errors + 1;
                            if (errors <= 30)
                                $display("FAIL %0s header slot %0d pixel %h (engine %0d)", tag, sx, pixel_data, se);
                        end
                    end
                end
            checks = checks + 1;
            if (lit < 20) begin
                errors = errors + 1;
                $display("FAIL %0s header slot %0d nearly empty (%0d px)", tag, sx, lit);
            end
        end
    end endtask

    integer fd, p;
    task dump(input [8*48-1:0] name); begin
        fd = $fopen(name, "w");
        for (p = 0; p < 96*64; p = p + 1) begin
            pixel_index = p; #1;
            $fdisplay(fd, "%04h", pixel_data);
        end
        $fclose(fd);
    end endtask

    // ------------------------------------------------------------- maze runs
    reg [COST_W-1:0] cost [0:NUM_CELLS-1];
    reg [8*64-1:0] path;
    reg [8*16-1:0] maze;
    task load_maze(input [8*16-1:0] name); integer i; begin
        $sformat(path, "sim/mazes/%0s.mem", name);
        $readmemh(path, cost);
        for (i = 0; i < NUM_CELLS; i = i + 1) begin
            @(negedge clk); load_we = 1'b1; load_addr = i; load_data = cost[i];
        end
        @(negedge clk); load_we = 1'b0;
    end endtask
    // the found (or, with want_found = 0, unfound) run with the longest distance
    integer best_d, rs, rg, rf, r1, r2, r3, r4, rr, d;
    task pick_run(input [8*16-1:0] name, input want_found); begin
        $sformat(path, "sim/mazes/%0s.expect", name);
        fd = $fopen(path, "r");
        best_d = -1;
        rr = $fscanf(fd, "%d %d %d %d %d %d %d\n", rs, rg, rf, r1, r2, r3, r4);
        while (rr == 7) begin
            d = ((rs / COLS > rg / COLS) ? rs / COLS - rg / COLS : rg / COLS - rs / COLS) +
                ((rs % COLS > rg % COLS) ? rs % COLS - rg % COLS : rg % COLS - rs % COLS);
            if (rf == want_found && d > best_d) begin best_d = d; start_idx = rs; goal_idx = rg; end
            rr = $fscanf(fd, "%d %d %d %d %d %d %d\n", rs, rg, rf, r1, r2, r3, r4);
        end
        $fclose(fd);
        if (best_d < 0) fail("no suitable run in expect file");
    end endtask
    task launch; begin
        @(negedge clk); start = 1'b1; @(negedge clk); start = 1'b0;
    end endtask
    task wait_done; integer t; begin
        t = 0;
        while (((done & run_mask) != run_mask || count == 0 || (end_valid & mask) != mask) && t < 400000) begin
            @(negedge clk); t = t + 1;
        end
        if (t >= 400000) fail("timeout waiting for the run");
        repeat (1200) @(negedge clk);          // final sample + oled_stats snapshot (SNAP_DIV 400)
    end endtask

    integer k;
    initial begin
        if (!$value$plusargs("maze=%s", maze)) maze = "rand3";
        repeat (4) @(negedge clk);
        rst = 1'b0;
        repeat (500) @(negedge clk);           // first snapshot

        // ---- empty plot: only axes, grid and labels
        probe(0, 40,  AXIS,  "y axis");
        probe(50, 63, AXIS,  "x axis");
        probe(2, 39,  GRID,  "50 % grid dot");
        probe(3, 39,  BLACK, "50 % grid gap");
        probe(2, 16,  GRID,  "100 % grid dot");
        probe(5, 18,  LABEL, "label 1 of 100%");
        probe(4, 18,  BLACK, "label 1 left");
        probe(7, 18,  BLACK, "label gap");
        probe(8, 18,  LABEL, "label 0 of 100%");
        probe(16, 18, LABEL, "label %");
        probe(4, 41,  LABEL, "label 5 of 50");
        probe(76, 57, LABEL, "label T of TIME");
        probe(92, 57, LABEL, "label arrow");
        probe(93, 57, BLACK, "label arrow off");
        check_plot("empty");

        // ---- A: all four engines
        load_maze(maze);
        pick_run(maze, 1);
        $display("maze %0s run %0d -> %0d (distance %0d)", maze, start_idx, goal_idx, best_d);
        run_mask = 4'b1111;
        launch;
        k = 0;
        while (done == 4'b0000 && k < 400000) begin @(negedge clk); k = k + 1; end
        repeat (40) @(negedge clk);
        step_en = 1'b0;                        // pause: frame mid-run
        repeat (900) @(negedge clk);           // snapshot shows the first finisher
        check_plot("A mid");
        dump("sim/build/oled_plot_mid.hex");
        step_en = 1'b1;
        wait_done;
        $display("A: count=%0d plog=%0d end_valid=%b order=%h cycles bfs=%0d dij=%0d ast=%0d dfs=%0d",
                 count, plog, end_valid, snap_order, cyc0, cyc1, cyc2, cyc3);
        check_plot("A end");
        check_header("A end");
        // every finished engine's curve reaches 100 % (found)
        for (k = 0; k < 4; k = k + 1) begin
            pixel_index = 16 * 96 + 1 + end_idx[k*AW +: AW]; #1;
            checks = checks + 1;
            if (!is_eng(pixel_data)) begin
                errors = errors + 1;
                $display("FAIL A: engine %0d has no curve pixel at 100 %% at its end column", k);
            end
        end
        dump("sim/build/oled_plot_all.hex");

        // ---- B: BFS + A* only
        run_mask = 4'b0101;
        launch;
        wait_done;
        check_plot("B end");
        check_header("B end");
        for (py_ = 16; py_ < 63; py_ = py_ + 1)
            for (px_ = 1; px_ < 96; px_ = px_ + 1) begin
                pixel_index = py_ * 96 + px_; #1;
                if (pixel_data == eng_colour(1) || pixel_data == eng_colour(3)) begin
                    errors = errors + 1;
                    $display("FAIL B: unselected engine drawn at (%0d,%0d)", px_, py_);
                end
            end
        dump("sim/build/oled_plot_bfs_ast.hex");

        // ---- C: no path
        load_maze("unreachable");
        pick_run("unreachable", 0);
        run_mask = 4'b1111;
        launch;
        wait_done;
        check_plot("C end");
        check_header("C end");
        for (px_ = 1; px_ < 96; px_ = px_ + 1) begin
            pixel_index = 16 * 96 + px_; #1;
            if (is_eng(pixel_data)) begin
                errors = errors + 1;
                $display("FAIL C: a no-path curve reached 100 %% at x=%0d", px_);
            end
        end
        // "BFS NONE": the char between name and NONE (slot 0, char 3) is blank
        lit = 0;
        for (sy = 0; sy < 8; sy = sy + 1)
            for (px_ = 18; px_ < 24; px_ = px_ + 1) begin
                pixel_index = sy * 96 + px_; #1;
                if (pixel_data != BLACK) lit = lit + 1;
            end
        checks = checks + 1;
        if (lit != 0) begin errors = errors + 1; $display("FAIL C: char before NONE is not blank"); end
        dump("sim/build/oled_plot_nopath.hex");

        if (errors == 0) $display("PASS: tb_gv_oled_plot errors=0 checks=%0d", checks);
        else             $display("FAIL: tb_gv_oled_plot errors=%0d checks=%0d", errors, checks);
        $finish;
    end
endmodule
