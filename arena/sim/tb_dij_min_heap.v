`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_dij_min_heap : self-checking testbench for M1's dij_min_heap
//
// A TB-side model (in_q[cell], key[cell]) tracks what the heap should hold.
//   1. fill to NUM_CELLS entries, then drain: pops come out in key order
//   2. random mix of insert / decrease-key / pop, checked on every pop
//      (the popped key must be the model's minimum, and that cell must be
//      queued with that key)
//   3. step_en randomly low: the heap must still give the same results
//   4. clr in the middle of a sift empties the heap
// Ends with PASS or FAIL (error count).
//////////////////////////////////////////////////////////////////////////////
module tb_dij_min_heap;
    parameter NUM_CELLS = `ARENA_NUM_CELLS;
    parameter INDEX_W   = `ARENA_INDEX_W;
    parameter DIST_W    = `ARENA_DIST_W;

    reg clk = 0;
    always #5 clk = ~clk;

    reg                rst = 1;
    reg                step_en = 1;
    reg                clr = 0, push = 0, push_upd = 0, pop = 0;
    reg  [INDEX_W-1:0] push_idx = 0;
    reg  [DIST_W-1:0]  push_key = 0;
    wire               ready, empty;
    wire [INDEX_W-1:0] min_idx;
    wire [DIST_W-1:0]  min_key;

    dij_min_heap #(.NUM_CELLS(NUM_CELLS), .INDEX_W(INDEX_W), .DIST_W(DIST_W)) dut (
        .clk(clk), .rst(rst), .step_en(step_en),
        .clr(clr), .push(push), .push_upd(push_upd),
        .push_idx(push_idx), .push_key(push_key), .pop(pop),
        .ready(ready), .empty(empty), .min_idx(min_idx), .min_key(min_key)
    );

    // model
    reg              in_q [0:NUM_CELLS-1];
    reg [DIST_W-1:0] key  [0:NUM_CELLS-1];
    integer          count;

    integer errors = 0;
    integer pauses = 0;     // 1 = randomly drop step_en while waiting
    integer i, n, c, op, best, max_wait;
    integer e0;             // error count at the start of a section
    reg [DIST_W-1:0] last_key, k;

    // wait (in step_en cycles) until the heap is ready, with random pauses
    task wait_ready;
        integer w;
        begin
            w = 0;
            while (!ready && w < 100) begin
                step_en = pauses ? ($random & 1) : 1;
                @(posedge clk); #1;
                if (step_en) w = w + 1;
            end
            step_en = 1;
            if (w > max_wait) max_wait = w;
            if (!ready) begin
                $display("FAIL: heap not ready after 100 step cycles");
                errors = errors + 1;
            end
        end
    endtask

    // issue one command for exactly one step_en = 1 cycle
    task issue;
        begin
            step_en = 1;
            @(posedge clk); #1;
            clr = 0; push = 0; pop = 0; push_upd = 0;
        end
    endtask

    task do_insert;
        input integer          cidx;
        input [DIST_W-1:0]     kk;
        begin
            wait_ready;
            push = 1; push_upd = 0; push_idx = cidx; push_key = kk;
            issue;
            in_q[cidx] = 1; key[cidx] = kk; count = count + 1;
        end
    endtask

    task do_decrease;
        input integer          cidx;
        input [DIST_W-1:0]     kk;
        begin
            wait_ready;
            push = 1; push_upd = 1; push_idx = cidx; push_key = kk;
            issue;
            key[cidx] = kk;
        end
    endtask

    // pop and check against the model; returns the popped key in last_key
    task do_pop;
        begin
            wait_ready;
            best = -1;
            for (i = 0; i < NUM_CELLS; i = i + 1)
                if (in_q[i] && (best < 0 || key[i] < key[best])) best = i;
            if (empty != (count == 0)) begin
                $display("FAIL: empty=%b but model count=%0d", empty, count);
                errors = errors + 1;
            end
            if (count > 0) begin
                if (!in_q[min_idx] || key[min_idx] != min_key || min_key != key[best]) begin
                    $display("FAIL pop: got cell %0d key %0d, model min key %0d (cell %0d in_q=%b key=%0d)",
                             min_idx, min_key, key[best], min_idx, in_q[min_idx], key[min_idx]);
                    errors = errors + 1;
                end
                last_key = min_key;
                in_q[min_idx] = 0; count = count - 1;
                pop = 1;
                issue;
            end
        end
    endtask

    task model_clear;
        begin
            for (i = 0; i < NUM_CELLS; i = i + 1) in_q[i] = 0;
            count = 0;
        end
    endtask

    // random mix of operations
    task random_mix;
        input integer ops;
        begin
            for (n = 0; n < ops; n = n + 1) begin
                op = $random & 3;
                c  = ({$random} % NUM_CELLS);
                if (op <= 1 && !in_q[c] && count < NUM_CELLS) begin
                    do_insert(c, {$random} % 3000);
                end else if (op == 2 && in_q[c] && key[c] > 0) begin
                    k = {$random} % key[c];          // strictly smaller
                    do_decrease(c, k);
                end else begin
                    do_pop;
                end
            end
            while (count > 0) do_pop;
        end
    endtask

    initial begin
        model_clear;
        max_wait = 0;
        repeat (3) @(posedge clk);
        #1 rst = 0;
        clr = 1; issue;

        // 1. fill completely, then drain in key order
        for (c = 0; c < NUM_CELLS; c = c + 1)
            do_insert(c, {$random} % 500);
        last_key = 0;
        while (count > 0) begin
            k = last_key;
            do_pop;
            if (last_key < k) begin
                $display("FAIL drain: key %0d after %0d (not sorted)", last_key, k);
                errors = errors + 1;
            end
        end
        if (!empty) begin
            $display("FAIL drain: heap not empty at the end");
            errors = errors + 1;
        end
        $display("%s fill/drain %0d entries", (errors == 0) ? "PASS" : "FAIL", NUM_CELLS);

        // 2. random insert / decrease-key / pop
        e0 = errors;
        random_mix(3000);
        $display("%s random mix", (errors == e0) ? "PASS" : "FAIL");

        // 3. same with random step_en pauses
        e0 = errors;
        pauses = 1;
        random_mix(1500);
        pauses = 0;
        $display("%s random mix with step_en pauses", (errors == e0) ? "PASS" : "FAIL");

        // 4. clr in the middle of a sift-down
        e0 = errors;
        for (c = 0; c < 40; c = c + 1) do_insert(c, 100 - c);
        wait_ready;
        pop = 1; issue;               // starts a sift-down
        clr = 1; issue;               // abort it
        model_clear;
        if (!ready || !empty) begin
            $display("FAIL clr: ready=%b empty=%b", ready, empty);
            errors = errors + 1;
        end
        do_insert(7, 5);
        do_insert(9, 3);
        do_pop;
        if (last_key != 3) begin
            $display("FAIL after clr: popped key %0d, expected 3", last_key);
            errors = errors + 1;
        end
        do_pop;
        $display("%s clr mid-sift", (errors == e0) ? "PASS" : "FAIL");

        $display("longest wait for ready: %0d step cycles", max_wait);
        if (errors == 0)
            $display("TB_DIJ_MIN_HEAP PASS");
        else
            $display("TB_DIJ_MIN_HEAP FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
