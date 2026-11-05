`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_perf_sampler : the sampler against fake engines (CONTRACT 8a)
//
//   A. level is exact for every (d0, best); 0 before the first expansion, even
//      with the previous run's goal still on `current`; d0 = 0 jumps to the top
//   B. a run long enough for more than 7 halvings (the rotation wraps), every
//      level word unique: after each halving and at random times, every sample
//      read through ast_perf_hist is the word from step i * 2^plog
//   C. finish markers across a sweep of run lengths, full speed and random
//      step_en: end_idx within the documented bound, order kept, the curve is
//      final at its marker and has not started before it, the flush sample
//      holds the final values, count in range
//   D. unselected engines stay empty, even with stale done and expansions
//   E. rst mid-run, restart while running, restart from done
//   F. abandoned run (step_en = 0 throughout), then GAME-style searches: the
//      plot does not change
//////////////////////////////////////////////////////////////////////////////
module tb_ast_perf_sampler;
    localparam COLS      = `ARENA_COLS;
    localparam ROWS      = `ARENA_ROWS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam DIST_W    = `ARENA_DIST_W;
    localparam AW        = `ARENA_PG_ADDR_W;
    localparam RW        = `ARENA_PG_ROT_W;
    localparam LW        = `ARENA_PG_LEVEL_W;
    localparam BW        = `ARENA_PG_BEST_W;
    localparam SAMPLES   = `ARENA_PG_SAMPLES;
    localparam LEVEL_MAX = `ARENA_PG_LEVEL_MAX;
    localparam DONE_SLOP = 14;          // done -> end sample, clocks (see check_ends)
    localparam HIST_MAX  = 400000;
    localparam [BW-1:0] NO_BEST = {BW{1'b1}};

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst = 1'b1, step_en = 1'b1, start = 1'b0;
    reg [3:0] run_mask = 4'hF, done = 0;
    reg [INDEX_W-1:0] start_idx = 0, goal_idx = 0;
    reg [INDEX_W-1:0] cur [0:3];
    reg [DIST_W-1:0]  expn [0:3];

    wire           we;
    wire [AW-1:0]  waddr, count;
    wire [4*LW-1:0] wdata, rd_level, rd_prev;
    wire [RW-1:0]  rot;
    wire [4*AW-1:0] end_idx;
    wire [3:0]     end_valid, mask;
    wire [4:0]     plog;
    wire [4*BW-1:0] best;

    ast_perf_sampler dut (
        .clk(clk), .rst(rst), .step_en(step_en), .start(start), .run_mask(run_mask),
        .start_idx(start_idx), .goal_idx(goal_idx),
        .current({cur[3], cur[2], cur[1], cur[0]}),
        .expansions({expn[3], expn[2], expn[1], expn[0]}),
        .done(done),
        .we(we), .waddr(waddr), .wdata(wdata), .count(count), .rot(rot),
        .end_idx(end_idx), .end_valid(end_valid), .mask(mask), .plog(plog),
        .dbg_best(best)
    );

    reg [AW-1:0] rd_idx = 0;
    ast_perf_hist u_hist (.clk(clk), .we(we), .waddr(waddr), .wdata(wdata), .rot(rot),
                          .rd_idx(rd_idx), .rd_level(rd_level), .rd_prev_level(rd_prev));

    integer errors = 0;
    task fail(input [8*44-1:0] msg);
        begin
            errors = errors + 1;
            if (errors <= 25)
                $display("  t=%0t FAIL %0s (count=%0d plog=%0d rot=%0d)", $time, msg, count, plog, rot);
        end
    endtask

    // ---- steps the sampler has counted in this run -----------------------------
    // Sample i is taken at step i * 2^plog.  Read at negedge: these are the values
    // the next clock edge acts on.
    integer steps = 0;
    reg [4*LW-1:0] word_at [0:HIST_MAX];
    always @(negedge clk) begin
        if (dut.start_q) steps = 0;
        else if (dut.running && dut.step_q) begin
            if (steps <= HIST_MAX) word_at[steps] = dut.levels;
            steps = steps + 1;
        end
    end

    // ---- fake engine helpers ----------------------------------------------------
    integer e_i;
    task set_all(input [INDEX_W-1:0] c, input [DIST_W-1:0] x);
        begin
            for (e_i = 0; e_i < 4; e_i = e_i + 1) begin cur[e_i] = c; expn[e_i] = x; end
        end
    endtask

    // Like a real engine: current and expansions still hold the previous run on the
    // start cycle, and expansions and done clear on the edge that accepts it.
    task go(input [3:0] m, input [INDEX_W-1:0] s, input [INDEX_W-1:0] g);
        begin
            @(negedge clk);
            run_mask = m; start_idx = s; goal_idx = g; start = 1'b1;
            @(negedge clk);
            start = 1'b0; done = done & ~m;
            for (e_i = 0; e_i < 4; e_i = e_i + 1) if (m[e_i]) expn[e_i] = 0;
            @(negedge clk);                         // the sampler has cleared by now
        end
    endtask

    function [LW-1:0] level_of(input [4*LW-1:0] w, input integer e);
        level_of = w[e*LW +: LW];
    endfunction

    function integer end_of(input integer e);
        end_of = end_idx[e*AW +: AW];
    endfunction

    // ---- A : the metric ---------------------------------------------------------
    integer d0, b, gr, gc, walk, wr, wc, want_lv, ei;
    task test_levels;
        begin
            for (d0 = 0; d0 <= COLS + ROWS - 2; d0 = d0 + 1) begin
                gr = (d0 < ROWS - 1) ? d0 : ROWS - 1;
                gc = d0 - gr;
                set_all(gr * COLS + gc, 16'd5);       // stale: the old goal, old count
                go(4'hF, 0, gr * COLS + gc);
                repeat (14) @(negedge clk);
                if (dut.levels !== 0) fail("level not 0 before the first expansion");
                if (best !== {4{NO_BEST}}) fail("best not empty before the first expansion");
                for (b = d0; b >= 0; b = b - 1) begin
                    walk = d0 - b;                    // cell `walk` steps from the start
                    wr = (walk < gr) ? walk : gr;
                    wc = walk - wr;
                    set_all(wr * COLS + wc, 16'd1);
                    repeat (14) @(negedge clk);
                    want_lv = (d0 == 0) ? LEVEL_MAX : (LEVEL_MAX * (d0 - b)) / d0;
                    for (ei = 0; ei < 4; ei = ei + 1) begin
                        if (level_of(dut.levels, ei) !== want_lv[LW-1:0]) begin
                            fail("level wrong");
                            $display("    d0=%0d best=%0d engine=%0d level=%0d want=%0d",
                                     d0, b, ei, level_of(dut.levels, ei), want_lv);
                        end
                        if (best[ei*BW +: BW] !== b[BW-1:0]) fail("best wrong");
                    end
                end
                done = 4'hF;                          // end the run
                repeat (20) @(negedge clk);
                if (dut.running) fail("still running after every engine is done");
            end
            set_all(0, 0);
        end
    endtask

    // ---- read every sample and compare with the word at its step ------------------
    integer ri;
    task check_samples(input [8*20-1:0] where);
        integer c, p;
        begin
            c = count; p = plog;
            for (ri = 0; ri < c; ri = ri + 1) begin
                rd_idx = ri; #0.01;
                if (rd_level !== word_at[ri << p]) begin
                    fail("sample read back wrong");
                    if (errors <= 25)
                        $display("    %0s: sample %0d step %0d got %h want %h",
                                 where, ri, ri << p, rd_level, word_at[ri << p]);
                end
                if (rd_prev !== word_at[((ri == 0) ? 0 : ri - 1) << p]) fail("previous sample wrong");
            end
            rd_idx = 0;
        end
    endtask

    // ---- B : halving --------------------------------------------------------------
    reg [4*LW-1:0] forced = 0;
    integer last_plog, halvings;
    task test_halving(input rand_step, input integer target_plog);
        begin
            set_all(0, 0);
            go(4'hF, 0, 8'd191);
            force dut.levels = forced;
            last_plog = 0; halvings = 0;
            while (plog < target_plog && errors < 25) begin
                @(negedge clk);
                step_en = rand_step ? ($unsigned($random) % 10 < 8) : 1'b1;
                if (count > SAMPLES) fail("count above SAMPLES");
                if (rot !== plog % AW) fail("rot out of step with plog");
                if (plog != last_plog) begin
                    if (plog != last_plog + 1) fail("plog jumped");
                    if (count != SAMPLES / 2 + 1 && count != SAMPLES / 2)
                        fail("count wrong just after halving");
                    check_samples("after halving");
                    last_plog = plog; halvings = halvings + 1;
                end else if ($unsigned($random) % 400 == 0) begin
                    check_samples("mid run");
                end
            end
            check_samples("end of run");
            if (halvings < AW + 1) fail("fewer halvings than a full rotation");
            step_en = 1'b1;
            release dut.levels;
            done = 4'hF;
            repeat (20) @(negedge clk);
            $display("  halving: %0d halvings, %0d steps, rot=%0d, rand_step=%0d",
                     halvings, steps, rot, rand_step);
        end
    endtask
    always @(posedge clk) forced <= forced * 5 + 24'd1234567;   // a new word every clock

    // ---- C : finish markers ---------------------------------------------------------
    integer fin [0:3];
    integer slack_max = 0;
    task check_ends(input [3:0] m);
        integer e, j, t, s, lv, prev_lv, top;
        begin
            if (dut.running) fail("still running after the last engine");
            if (end_valid !== m) fail("end_valid != the selected engines");
            if (mask !== m) fail("mask != the selected engines");
            if (count == 0 || count > SAMPLES) fail("count out of range");
            // the last sample holds the final values
            rd_idx = count - 1; #0.01;
            for (e = 0; e < 4; e = e + 1) begin
                top = m[e] ? LEVEL_MAX : 0;
                if (level_of(rd_level, e) !== top[LW-1:0]) fail("flush sample not final");
            end
            for (e = 0; e < 4; e = e + 1) if (m[e]) begin
                // the marker: first sample taken after done has settled, then rounded
                // up by each later halving, so within 2 columns + the done delay
                t = end_of(e) << plog;
                if (t < fin[e]) fail("end marker before the engine finished");
                if (t > fin[e] + DONE_SLOP + (2 << plog)) begin
                    fail("end marker too late");
                    $display("    e=%0d fin=%0d end=%0d t=%0d plog=%0d", e, fin[e], end_of(e), t, plog);
                end
                if (t - fin[e] - (1 << plog) > slack_max) slack_max = t - fin[e] - (1 << plog);
                if (end_of(e) >= count) fail("end marker beyond the plot");
                // the curve is final at the marker
                rd_idx = end_of(e); #0.01;
                if (level_of(rd_level, e) !== LEVEL_MAX[LW-1:0]) fail("curve not final at its marker");
                // the engine jumps to 100 % as it finishes: well before that it is 0
                prev_lv = 0;
                for (s = 0; s < count; s = s + 1) begin
                    rd_idx = s; #0.01;
                    lv = level_of(rd_level, e);
                    if (lv < prev_lv) fail("curve fell");
                    prev_lv = lv;
                    if ((s << plog) + (1 << plog) < fin[e] - 12 && lv != 0) fail("curve rose too early");
                end
                // finishing order is kept
                for (j = 0; j < 4; j = j + 1)
                    if (m[j] && fin[j] < fin[e] && end_of(j) > end_of(e)) fail("end markers out of order");
            end
            rd_idx = 0;
        end
    endtask

    // Every engine sits at the start, then expands the goal and is done at step fin[e].
    // (steps is the sampler's own count, so a pause moves nothing here.)
    task run_fake(input [3:0] m, input rand_step);
        integer e, left;
        begin
            set_all(0, 0);
            go(m, 0, 8'd191);
            for (e = 0; e < 4; e = e + 1) if (m[e]) expn[e] = 1;
            left = 1;
            while (left != 0) begin
                @(negedge clk);
                step_en = rand_step ? ($unsigned($random) % 2) : 1'b1;
                left = 0;
                for (e = 0; e < 4; e = e + 1) if (m[e]) begin
                    if (steps >= fin[e] && !done[e]) begin
                        cur[e] = 8'd191; done[e] = 1'b1;
                    end
                    if (!done[e]) left = 1;
                end
            end
            step_en = 1'b1;
            repeat (20) @(negedge clk);
        end
    endtask

    integer T;
    task test_ends;
        begin
            for (T = 40; T <= 420; T = T + 1) begin
                fin[0] = T; fin[1] = T + 3; fin[2] = 2 * T; fin[3] = 3 * T;
                run_fake(4'hF, 1'b0);
                check_ends(4'hF);
            end
            for (T = 40; T <= 420; T = T + 7) begin
                fin[0] = 3 * T; fin[1] = T; fin[2] = T + 1; fin[3] = 2 * T + 5;
                run_fake(4'hF, 1'b1);
                check_ends(4'hF);
            end
            fin[0] = 50000; fin[1] = 20000; fin[2] = 7; fin[3] = 12345;
            run_fake(4'hF, 1'b0);
            check_ends(4'hF);
            $display("  ends: worst marker delay past one column = %0d clocks", slack_max);
        end
    endtask

    // ---- D : unselected engines -------------------------------------------------------
    task test_mask;
        integer e;
        begin
            set_all(8'd191, 16'd9);
            done = 4'b1010;                         // stale, from an earlier run
            go(4'b0101, 0, 8'd191);
            // unselected engines keep their status, as they would in pathfind_top
            set_all(8'd191, 16'd9);
            expn[0] = 0; expn[2] = 0; cur[0] = 0; cur[2] = 0;
            repeat (300) @(negedge clk);
            expn[0] = 1; expn[2] = 1;
            repeat (20) @(negedge clk);
            cur[0] = 8'd191; cur[2] = 8'd191;
            repeat (20) @(negedge clk);
            done[0] = 1'b1; done[2] = 1'b1;
            repeat (30) @(negedge clk);
            if (end_valid !== 4'b0101) fail("unselected engine got an end marker");
            if (mask !== 4'b0101) fail("mask wrong");
            if (best[1*BW +: BW] !== NO_BEST || best[3*BW +: BW] !== NO_BEST) fail("unselected probe moved");
            for (e = 0; e < count; e = e + 1) begin
                rd_idx = e; #0.01;
                if (level_of(rd_level, 1) !== 0 || level_of(rd_level, 3) !== 0) fail("unselected curve not 0");
            end
            rd_idx = 0;
        end
    endtask

    // ---- E : reset and restart --------------------------------------------------------
    task test_restart;
        begin
            done = 0;
            set_all(0, 1);
            go(4'hF, 0, 8'd191);
            set_all(0, 1);
            repeat (500) @(negedge clk);
            if (count == 0 || plog == 0) fail("run did not get going");

            @(negedge clk) rst = 1'b1;
            @(negedge clk) rst = 1'b0;
            @(posedge clk); #1;
            if (count !== 0 || plog !== 0 || rot !== 0 || end_valid !== 0 || mask !== 0 || dut.running !== 1'b0)
                fail("rst did not clear the sampler");
            if (best !== {4{NO_BEST}}) fail("rst did not clear best");
            repeat (40) @(negedge clk);
            if (count !== 0 || we !== 1'b0) fail("sampling after rst");

            // restart while running
            go(4'hF, 0, 8'd191);
            set_all(0, 1);
            repeat (500) @(negedge clk);
            go(4'h3, 0, 8'd191);                    // clears what the 2 selected use
            @(posedge clk); #1;
            if (plog !== 0 && count > 2) fail("restart did not clear the plot");
            fin[0] = 200; fin[1] = 300; fin[2] = 0; fin[3] = 0;
            done[3:2] = 2'b11;
            run_fake(4'h3, 1'b0);
            check_ends(4'h3);

            // restart from done
            fin[0] = 90; fin[1] = 80; fin[2] = 70; fin[3] = 60;
            run_fake(4'hF, 1'b0);
            check_ends(4'hF);
        end
    endtask

    // ---- F : abandoned run, then GAME searches -----------------------------------------
    task test_abandon;
        integer n, c0;
        reg [RW-1:0] r0;
        reg [4*BW-1:0] b0;
        reg [4*LW-1:0] l0;
        begin
            set_all(0, 0);
            done = 0;
            @(negedge clk) step_en = 1'b0;
            go(4'hF, 0, 8'd191);
            repeat (50) @(negedge clk);
            if (count !== 0) fail("sampled during a pause");
            // the engines finish at full speed while the demo step_en stays 0
            set_all(8'd100, 1);
            repeat (5) @(negedge clk);
            done = 4'hF;
            repeat (25) @(negedge clk);
            if (dut.running) fail("abandoned run did not finalise with step_en = 0");
            if (count !== 1) fail("abandoned run: want exactly the flush sample");
            if (end_valid !== 4'hF) fail("abandoned run: end markers");

            c0 = count; r0 = rot; b0 = best; l0 = dut.levels;
            step_en = 1'b1;
            for (n = 0; n < 2000; n = n + 1) begin
                @(negedge clk);
                done = $random; set_all($random, n + 1);
                if (we !== 1'b0) fail("written after the run ended");
                if (count !== c0 || rot !== r0 || best !== b0) fail("GAME search reached the plot");
            end
            rd_idx = 0; #0.01;
            if (rd_level !== l0) fail("GAME search changed the stored sample");
            done = 0; set_all(0, 0);
        end
    endtask

    initial begin
        set_all(0, 0);
        repeat (3) @(negedge clk);
        rst = 1'b0;
        repeat (3) @(negedge clk);

        test_levels;
        if (errors == 0) $display("  A levels ok");
        test_halving(1'b0, AW + 3);
        test_halving(1'b1, AW + 2);
        if (errors == 0) $display("  B halving ok");
        test_ends;
        if (errors == 0) $display("  C finish markers ok");
        test_mask;
        if (errors == 0) $display("  D unselected ok");
        test_restart;
        if (errors == 0) $display("  E restart ok");
        test_abandon;
        if (errors == 0) $display("  F abandoned run ok");

        if (errors == 0) $display("PASS: tb_ast_perf_sampler");
        else             $display("FAIL: tb_ast_perf_sampler errors=%0d", errors);
        $finish;
    end
endmodule
