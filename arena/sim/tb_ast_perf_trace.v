`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_perf_trace : the graph's numbers from the real engines (CONTRACT 8a)
//
// Runs pathfind_top + ast_perf_sampler + ast_perf_hist + oled_stats on every
// shared maze and run, three times: all engines at full speed, all engines with
// random step_en, and with varying engine subsets.  Checks:
//   * every engine's expansions match <name>.perf.total
//   * after every expansion, best matches <name>.perf.expect (exact trace)
//   * final levels match; found -> 100 %, no path -> below 100 %
//   * the curve never falls, and its marker sample holds the final level
//   * 47 <= count <= 94: a run always fills 50-100 % of the plot's width
//   * timing:
//       - end markers keep the engines' finishing order
//       - end_idx * 2^plog is within the documented bound of cycles
//       - snap_order, the leader's cyc_bcd and every gap_bcd are the real cycles
//   * pausing changes neither final levels nor cycles
//   * unselected engines stay empty
//////////////////////////////////////////////////////////////////////////////
module tb_ast_perf_trace;
    localparam COLS      = `ARENA_COLS;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam COST_W    = `ARENA_COST_W;
    localparam DIST_W    = `ARENA_DIST_W;
    localparam CNT_W     = `ARENA_CNT_W;
    localparam AW        = `ARENA_PG_ADDR_W;
    localparam RW        = `ARENA_PG_ROT_W;
    localparam LW        = `ARENA_PG_LEVEL_W;
    localparam BW        = `ARENA_PG_BEST_W;
    localparam SAMPLES   = `ARENA_PG_SAMPLES;
    localparam LEVEL_MAX = `ARENA_PG_LEVEL_MAX;
    localparam NUM_MAZES = 12;
    localparam MAX_RUNS  = 128;
    localparam MAX_LINES = 4000;
    localparam SNAP_DIV  = 400;         // conversions take 336 clocks
    localparam DONE_SLOP = 14;          // done -> end sample, clocks
    localparam [BW-1:0] NO_BEST = {BW{1'b1}};

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

    wire           we;
    wire [AW-1:0]  waddr, count;
    wire [4*LW-1:0] wdata, rd_level, rd_prev;
    wire [RW-1:0]  rot;
    wire [4*AW-1:0] end_idx;
    wire [3:0]     end_valid, mask;
    wire [4:0]     plog;
    wire [4*BW-1:0] best;
    ast_perf_sampler u_s (
        .clk(clk), .rst(rst), .step_en(step_en), .start(start), .run_mask(run_mask),
        .start_idx(start_idx), .goal_idx(goal_idx),
        .current({cur3, cur2, cur1, cur0}), .expansions({exp3, exp2, exp1, exp0}), .done(done),
        .we(we), .waddr(waddr), .wdata(wdata), .count(count), .rot(rot),
        .end_idx(end_idx), .end_valid(end_valid), .mask(mask), .plog(plog), .dbg_best(best)
    );
    reg [AW-1:0] rd_idx = 0;
    ast_perf_hist u_h (.clk(clk), .we(we), .waddr(waddr), .wdata(wdata), .rot(rot),
                       .rd_idx(rd_idx), .rd_level(rd_level), .rd_prev_level(rd_prev));

    wire [3:0]      snap_mask;
    wire [4*2-1:0]  snap_order;
    wire [4*24-1:0] cyc_bcd, gap_bcd;
    oled_stats #(.SNAP_DIV(SNAP_DIV)) u_st (
        .clk(clk), .rst(rst), .run_mask(run_mask),
        .cycles({cyc3, cyc2, cyc1, cyc0}), .expansions({exp3, exp2, exp1, exp0}),
        .path_cost({cost3, cost2, cost1, cost0}),
        .busy(busy), .done(done), .found(found), .no_path(no_path),
        .snap_mask(snap_mask), .snap_status(), .snap_place(), .cyc_bcd(cyc_bcd),
        .exp_bcd(), .cost_bcd(), .gap_bcd(gap_bcd), .snap_order(snap_order)
    );

    // ---- bookkeeping --------------------------------------------------------------
    reg [8*12-1:0] names [0:NUM_MAZES-1];
    integer maze = 0;
    integer errors = 0;
    integer n_runs = 0, n_compared = 0;
    integer worst_late = 0;
    task fail(input [8*50-1:0] msg);
        begin
            errors = errors + 1;
            if (errors <= 25)
                $display("  t=%0t FAIL %0s (%s %0d->%0d mask=%b)", $time, msg, names[maze], start_idx, goal_idx, run_mask);
        end
    endtask

    reg [8*64-1:0] path;
    reg [COST_W-1:0] cost [0:NUM_CELLS-1];

    // <name>.perf.expect, whole file
    integer pf_s [0:MAX_LINES-1];
    integer pf_g [0:MAX_LINES-1];
    integer pf_e [0:MAX_LINES-1];
    integer pf_k [0:MAX_LINES-1];
    integer pf_b [0:MAX_LINES-1];
    integer pf_l [0:MAX_LINES-1];
    integer pf_n = 0;

    // this run: expected best and level after k expansions of engine e, at [e*256+k]
    reg [BW-1:0] tbl_best [0:1023];
    reg [LW-1:0] tbl_lvl  [0:1023];

    // full-speed results of each run, to compare with the paused pass
    integer ref_cyc [0:4*MAX_RUNS-1];
    reg [4*LW-1:0] ref_lvl [0:MAX_RUNS-1];

    function integer cyc_of(input integer e);
        cyc_of = (e == 0) ? cyc0 : (e == 1) ? cyc1 : (e == 2) ? cyc2 : cyc3;
    endfunction
    function integer exp_of(input integer e);
        exp_of = (e == 0) ? exp0 : (e == 1) ? exp1 : (e == 2) ? exp2 : exp3;
    endfunction
    function integer bcd_int(input [23:0] b);
        bcd_int = b[23:20]*100000 + b[19:16]*10000 + b[15:12]*1000 + b[11:8]*100 + b[7:4]*10 + b[3:0];
    endfunction

    task read_perf;
        integer fd, n, s, g, e, k, b, l;
        begin
            $sformat(path, "sim/mazes/%0s.perf.expect", names[maze]);
            fd = $fopen(path, "r");
            if (fd == 0) begin fail("cannot open perf.expect"); $finish; end
            pf_n = 0;
            n = $fscanf(fd, "%d %d %d %d %d %d", s, g, e, k, b, l);
            while (n == 6) begin
                pf_s[pf_n] = s; pf_g[pf_n] = g; pf_e[pf_n] = e;
                pf_k[pf_n] = k; pf_b[pf_n] = b; pf_l[pf_n] = l;
                pf_n = pf_n + 1;
                n = $fscanf(fd, "%d %d %d %d %d %d", s, g, e, k, b, l);
            end
            $fclose(fd);
        end
    endtask

    task build_table(input integer s, input integer g);
        integer i, k;
        begin
            for (i = 0; i < 1024; i = i + 1) begin tbl_best[i] = NO_BEST; tbl_lvl[i] = 0; end
            for (i = 0; i < pf_n; i = i + 1)
                if (pf_s[i] == s && pf_g[i] == g)
                    for (k = pf_k[i]; k < 256; k = k + 1) begin
                        tbl_best[pf_e[i]*256 + k] = pf_b[i];
                        tbl_lvl [pf_e[i]*256 + k] = pf_l[i];
                    end
        end
    endtask

    task load_maze;
        integer i;
        begin
            $sformat(path, "sim/mazes/%0s.mem", names[maze]);
            $readmemh(path, cost);
            for (i = 0; i < NUM_CELLS; i = i + 1) begin
                @(negedge clk);
                load_we = 1'b1; load_addr = i; load_data = cost[i];
            end
            @(negedge clk) load_we = 1'b0;
        end
    endtask

    // ---- live trace check ------------------------------------------------------------
    // expansions at time t reaches best two clocks later (input register + probe).
    reg check_on = 1'b0;
    integer h1 [0:3];
    integer h2 [0:3];
    integer t_e;
    always @(negedge clk) begin
        if (check_on && u_s.running) begin
            for (t_e = 0; t_e < 4; t_e = t_e + 1)
                if (mask[t_e]) begin
                    if (best[t_e*BW +: BW] !== tbl_best[t_e*256 + h2[t_e]]) begin
                        fail("best does not match the reference trace");
                        if (errors <= 25)
                            $display("    engine %0d expansions %0d best %0d want %0d", t_e, h2[t_e],
                                     best[t_e*BW +: BW], tbl_best[t_e*256 + h2[t_e]]);
                    end
                    n_compared = n_compared + 1;
                end
        end
        for (t_e = 0; t_e < 4; t_e = t_e + 1) begin
            h2[t_e] = h1[t_e];
            h1[t_e] = exp_of(t_e);
        end
    end

    // ---- step_en driver ---------------------------------------------------------------
    reg rand_step = 1'b0;
    always @(negedge clk) step_en <= rand_step ? ($unsigned($random) % 2) : 1'b1;

    // ---- one run --------------------------------------------------------------------
    task run_one(input integer s, input integer g, input [3:0] m, input integer pass,
                 input integer run_no, input integer exp_b, input integer exp_d,
                 input integer exp_a, input integer exp_f);
        integer e, j, sm, lv, prev, timeout, t, fin_c, min_c, want, lead;
        reg [4*LW-1:0] final_lv;
        begin
            build_table(s, g);
            @(negedge clk);
            start_idx = s; goal_idx = g; run_mask = m; start = 1'b1;
            @(negedge clk) start = 1'b0;
            repeat (3) @(negedge clk);
            check_on = 1'b1;
            timeout = 0;
            while (((done & m) != m || u_s.running) && timeout < 400000) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 400000) fail("timeout");
            check_on = 1'b0;
            repeat (3) @(negedge clk);              // the last sample and markers publish

            // ---- results and final values
            for (e = 0; e < 4; e = e + 1) if (m[e]) begin
                want = (e == 0) ? exp_b : (e == 1) ? exp_d : (e == 2) ? exp_a : exp_f;
                if (exp_of(e) !== want) begin
                    fail("expansions differ from the reference");
                    $display("    engine %0d expansions %0d want %0d", e, exp_of(e), want);
                end
                if (best[e*BW +: BW] !== tbl_best[e*256 + want]) fail("final best wrong");
                if (u_s.levels[e*LW +: LW] !== tbl_lvl[e*256 + want]) begin
                    fail("final level wrong");
                    $display("    engine %0d level %0d want %0d", e, u_s.levels[e*LW +: LW], tbl_lvl[e*256 + want]);
                end
                if (found[e] && u_s.levels[e*LW +: LW] !== LEVEL_MAX[LW-1:0]) fail("found but not 100 %");
                if (no_path[e] && u_s.levels[e*LW +: LW] === LEVEL_MAX[LW-1:0]) fail("no path but 100 %");
            end else begin
                if (best[e*BW +: BW] !== NO_BEST) fail("unselected probe moved");
                if (u_s.levels[e*LW +: LW] !== 0) fail("unselected level not 0");
            end
            final_lv = u_s.levels;

            // ---- the curves
            if (mask !== m || end_valid !== m) fail("mask / end_valid wrong");
            if (count < SAMPLES / 2 || count > SAMPLES) fail("count outside 50-100 % of the width");
            rd_idx = count - 1; #0.01;
            if (rd_level !== final_lv) fail("last sample is not the final levels");
            for (e = 0; e < 4; e = e + 1) begin
                prev = 0;
                for (sm = 0; sm < count; sm = sm + 1) begin
                    rd_idx = sm; #0.01;
                    lv = rd_level[e*LW +: LW];
                    if (lv < prev) fail("curve fell");
                    prev = lv;
                end
                if (m[e]) begin
                    rd_idx = end_idx[e*AW +: AW]; #0.01;
                    if (rd_level[e*LW +: LW] !== final_lv[e*LW +: LW]) fail("not final at its marker");
                    if (end_idx[e*AW +: AW] >= count) fail("marker beyond the plot");
                end
            end
            rd_idx = 0;

            // ---- timing: markers against cycles
            for (e = 0; e < 4; e = e + 1) if (m[e]) begin
                fin_c = cyc_of(e);
                t = end_idx[e*AW +: AW] << plog;
                if (t < fin_c - 2) begin
                    fail("end marker earlier than the engine finished");
                    $display("    engine %0d cycles %0d marker %0d plog %0d", e, fin_c, t, plog);
                end
                if (t > fin_c + DONE_SLOP + (2 << plog)) begin
                    fail("end marker too late");
                    $display("    engine %0d cycles %0d marker %0d plog %0d", e, fin_c, t, plog);
                end
                if (t - fin_c - (1 << plog) > worst_late) worst_late = t - fin_c - (1 << plog);
                for (j = 0; j < 4; j = j + 1)
                    if (m[j] && cyc_of(j) < fin_c && end_idx[j*AW +: AW] > end_idx[e*AW +: AW])
                        fail("end markers out of finishing order");
            end

            // ---- timing: what the labels print, from oled_stats
            repeat (2 * SNAP_DIV + 400) @(negedge clk);
            min_c = 2147483647;
            for (e = 0; e < 4; e = e + 1) if (m[e] && cyc_of(e) < min_c) min_c = cyc_of(e);
            if (snap_mask !== m) fail("snap_mask wrong");
            lead = snap_order[1:0];
            if (!m[lead] || cyc_of(lead) != min_c) fail("snap_order[0] is not the first finisher");
            for (e = 0; e < 4; e = e + 1) if (m[e]) begin
                if (bcd_int(cyc_bcd[e*24 +: 24]) != cyc_of(e)) fail("cyc_bcd != cycles");
                if (bcd_int(gap_bcd[e*24 +: 24]) != cyc_of(e) - min_c) fail("gap_bcd != cycles - fastest");
            end
            for (j = 1; j < 4; j = j + 1)
                if (m[snap_order[j*2 +: 2]] && !m[snap_order[(j-1)*2 +: 2]])
                    fail("snap_order: unselected before selected");
                else if (m[snap_order[j*2 +: 2]] &&
                         cyc_of(snap_order[j*2 +: 2]) < cyc_of(snap_order[(j-1)*2 +: 2]))
                    fail("snap_order not fastest first");

            // ---- pausing changes nothing that lasts
            if (pass == 0) begin
                for (e = 0; e < 4; e = e + 1) ref_cyc[run_no*4 + e] = cyc_of(e);
                ref_lvl[run_no] = final_lv;
            end else if (pass == 1) begin
                for (e = 0; e < 4; e = e + 1)
                    if (ref_cyc[run_no*4 + e] != cyc_of(e)) fail("cycles changed by pausing");
                if (ref_lvl[run_no] !== final_lv) fail("final levels changed by pausing");
            end
            n_runs = n_runs + 1;
        end
    endtask

    // ---- one pass over every maze and run ------------------------------------------------
    task run_pass(input integer pass);
        integer fd, n, s, g, b, d, a, f, run_no;
        reg [3:0] m;
        begin
            run_no = 0;
            for (maze = 0; maze < NUM_MAZES; maze = maze + 1) begin
                load_maze;
                read_perf;
                $sformat(path, "sim/mazes/%0s.perf.total", names[maze]);
                fd = $fopen(path, "r");
                if (fd == 0) begin fail("cannot open perf.total"); $finish; end
                n = $fscanf(fd, "%d %d %d %d %d %d", s, g, b, d, a, f);
                while (n == 6) begin
                    // pass 2 walks through the 15 possible subsets
                    m = (pass == 2) ? (run_no % 15) + 1 : 4'hF;
                    run_one(s, g, m, pass, run_no, b, d, a, f);
                    run_no = run_no + 1;
                    n = $fscanf(fd, "%d %d %d %d %d %d", s, g, b, d, a, f);
                end
                $fclose(fd);
            end
            $display("  pass %0d done, %0d runs so far, errors %0d", pass, n_runs, errors);
        end
    endtask

    integer init_i;
    initial begin
        for (init_i = 0; init_i < 4; init_i = init_i + 1) begin h1[init_i] = 0; h2[init_i] = 0; end
        names[0] = "open";     names[1] = "weighted"; names[2] = "unreachable"; names[3] = "corridor";
        names[4] = "rand0";    names[5] = "rand1";    names[6] = "rand2";       names[7] = "rand3";
        names[8] = "rand4";    names[9] = "rand5";    names[10] = "rand6";      names[11] = "rand7";
        repeat (3) @(negedge clk);
        rst = 1'b0;
        repeat (3) @(negedge clk);

        rand_step = 1'b0; run_pass(0);
        rand_step = 1'b1; run_pass(1);
        rand_step = 1'b0; run_pass(2);

        $display("  %0d trace comparisons; worst marker delay past one column = %0d clocks",
                 n_compared, worst_late);
        if (n_compared < 10000) begin errors = errors + 1; $display("FAIL: trace check barely ran"); end
        if (errors == 0) $display("PASS: tb_ast_perf_trace");
        else             $display("FAIL: tb_ast_perf_trace errors=%0d", errors);
        $finish;
    end
endmodule
