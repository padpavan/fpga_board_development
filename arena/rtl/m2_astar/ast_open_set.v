`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_open_set : A* open set (priority queue), v1 = LINEAR SCAN
//
// Holds the cells A* has discovered but not yet expanded, each with its
// priority (f) and tie-break value (g), and hands back the best one on demand.
//
// Storage is one slot per CELL (not per entry), in three LUTRAMs:
//     open[idx]  1 bit       is this cell currently in the set?
//     fg[idx]    {f, g}      its priority and g
// so pushing a cell that is already open simply overwrites its f/g: that is
// decrease-key for free, and the set can never hold duplicates.
//
// ---------------------------------------------------------------------------
// Interface (used by astar_engine)
//
//   ready      1 = idle, a command may be issued this cycle
//   clr        empty the set.              ready again NUM_CELLS cycles later
//   push       insert / decrease-key       (push_idx, push_f, push_g), 1 cycle
//   pop_req    remove the minimum.         pop_valid NUM_CELLS+1 cycles later
//   pop_valid  1-cycle result pulse with   pop_idx, pop_empty (1 = was empty)
//
//   * Commands are only accepted when ready = 1 AND step_en = 1.  If both
//     clr and push/pop_req are high, clr wins and the others are ignored.
//   * push and pop_req in the same cycle are allowed: the push is written
//     first, so the scan that follows sees it.
//   * step_en = 0 freezes everything (all latencies above are counted in
//     step_en = 1 cycles).  pop_valid is high for exactly one step_en cycle.
//   * After rst, issue clr before first use (the RAMs are not reset).
//
// Pop priority (must match ref/arena_ref.py astar()):
//     lowest f  ->  then highest g  ->  then lowest index
// The scan walks idx 0,1,2,... and only replaces the current best on a
// STRICTLY better (f, g), so on a full tie the lowest index wins by itself.
//////////////////////////////////////////////////////////////////////////////
module ast_open_set #(
    parameter NUM_CELLS = `ARENA_NUM_CELLS,
    parameter INDEX_W   = `ARENA_INDEX_W,
    parameter DIST_W    = `ARENA_DIST_W
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               step_en,

    input  wire               clr,
    input  wire               push,
    input  wire [INDEX_W-1:0] push_idx,
    input  wire [DIST_W-1:0]  push_f,
    input  wire [DIST_W-1:0]  push_g,
    input  wire               pop_req,

    output wire               ready,
    output wire               pop_valid,
    output wire [INDEX_W-1:0] pop_idx,
    output wire               pop_empty
);
    localparam [INDEX_W-1:0] LAST = NUM_CELLS - 1;

    // ---- FSM -------------------------------------------------------------
    localparam [1:0] S_IDLE  = 2'd0,    // waiting for a command
                     S_CLEAR = 2'd1,    // writing open[ptr] = 0 for every cell
                     S_SCAN  = 2'd2,    // reading every cell, tracking the best
                     S_TAKE  = 2'd3;    // remove the winner, report it

    reg [1:0]         state = S_IDLE;
    reg [INDEX_W-1:0] ptr   = {INDEX_W{1'b0}};   // clear / scan address

    // Best candidate found so far during a scan.
    reg               best_valid = 1'b0;
    reg [DIST_W-1:0]  best_f     = {DIST_W{1'b0}};
    reg [DIST_W-1:0]  best_g     = {DIST_W{1'b0}};
    reg [INDEX_W-1:0] best_idx   = {INDEX_W{1'b0}};

    // ---- command decode (accepted only when idle and stepping) -----------
    assign ready = (state == S_IDLE);

    wire do_clr  = step_en && ready && clr;
    wire do_push = step_en && ready && push    && !clr;
    wire do_pop  = step_en && ready && pop_req && !clr;
    wire do_take = step_en && (state == S_TAKE);

    // ---- memories ----------------------------------------------------------
    // open[] has three writers, never active in the same cycle:
    //   CLEAR : open[ptr]      <= 0
    //   push  : open[push_idx] <= 1      (only in IDLE)
    //   TAKE  : open[best_idx] <= 0      (only if something was found)
    wire               open_we    = (step_en && state == S_CLEAR) || do_push ||
                                    (do_take && best_valid);
    wire [INDEX_W-1:0] open_waddr = (state == S_CLEAR) ? ptr      :
                                    (state == S_TAKE)  ? best_idx : push_idx;
    wire               open_wdata = do_push;
    wire               open_rdata;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(1)) u_open (
        .clk(clk), .we(open_we), .waddr(open_waddr), .wdata(open_wdata),
        .raddr(ptr), .rdata(open_rdata)
    );

    // fg[] is written only by push; read by the scan.
    wire [2*DIST_W-1:0] fg_rdata;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(2*DIST_W)) u_fg (
        .clk(clk), .we(do_push), .waddr(push_idx), .wdata({push_f, push_g}),
        .raddr(ptr), .rdata(fg_rdata)
    );

    // ---- scan comparator ---------------------------------------------------
    // The cell at `ptr` replaces the best if it is open and strictly better.
    wire [DIST_W-1:0] cand_f = fg_rdata[2*DIST_W-1:DIST_W];
    wire [DIST_W-1:0] cand_g = fg_rdata[DIST_W-1:0];

    wire cand_better = open_rdata &&
                       (!best_valid ||
                        (cand_f <  best_f) ||
                        (cand_f == best_f && cand_g > best_g));

    // ---- state machine -----------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            ptr        <= {INDEX_W{1'b0}};
            best_valid <= 1'b0;
        end else if (step_en) begin
            case (state)
                S_IDLE: begin
                    if (clr) begin
                        ptr   <= {INDEX_W{1'b0}};
                        state <= S_CLEAR;
                    end else if (pop_req) begin
                        ptr        <= {INDEX_W{1'b0}};
                        best_valid <= 1'b0;
                        state      <= S_SCAN;
                    end
                end

                S_CLEAR: begin
                    ptr <= ptr + 1'b1;
                    if (ptr == LAST)
                        state <= S_IDLE;
                end

                S_SCAN: begin
                    if (cand_better) begin
                        best_valid <= 1'b1;
                        best_f     <= cand_f;
                        best_g     <= cand_g;
                        best_idx   <= ptr;
                    end
                    ptr <= ptr + 1'b1;
                    if (ptr == LAST)
                        state <= S_TAKE;
                end

                S_TAKE: begin
                    state <= S_IDLE;            // open[best] cleared by open_we
                end
            endcase
        end
    end

    // ---- result ------------------------------------------------------------
    assign pop_valid = do_take;
    assign pop_idx   = best_idx;
    assign pop_empty = !best_valid;
endmodule
