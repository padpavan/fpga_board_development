`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Member 1 - indexed binary min-heap (Dijkstra priority queue)
//
//   - heap array in LUTRAM: position p holds {cell index, key}.  The minimum
//     is always at position 0; children of p are 2p+1 and 2p+2.
//   - pos[cell] remembers where each cell sits, so decrease-key moves the
//     existing entry instead of adding a duplicate.  A cell is in the heap at
//     most once, so it never holds more than NUM_CELLS entries.
//   - the heap array is kept in two copies (port A / port B, same writes) so
//     the sift-down can read both children in the same cycle.
//
// Commands (accepted only when ready = 1 and step_en = 1):
//   clr            empty the heap.  Accepted in ANY state (aborts a sift).
//   push           push_upd = 0: insert push_idx with push_key
//                  push_upd = 1: decrease push_idx's key to push_key
//                  (the caller knows whether the cell is already queued;
//                  pos[] is never cleared, it is written on every insert)
//   pop            remove the minimum (min_idx / min_key)
//
//   min_idx / min_key show the minimum whenever ready = 1 and empty = 0.
//   push: 1 + up to log2(N) cycles (sift-up, 1 cycle per level)
//   pop : 1 + up to 2*log2(N) cycles (sift-down, 2 cycles per level)
//   step_en = 0 freezes the FSM and all writes.
//////////////////////////////////////////////////////////////////////////////
module dij_min_heap #(
    parameter NUM_CELLS = `ARENA_NUM_CELLS,
    parameter INDEX_W   = `ARENA_INDEX_W,
    parameter DIST_W    = `ARENA_DIST_W
) (
    input  wire                clk,
    input  wire                rst,
    input  wire                step_en,

    input  wire                clr,
    input  wire                push,
    input  wire                push_upd,
    input  wire [INDEX_W-1:0]  push_idx,
    input  wire [DIST_W-1:0]   push_key,
    input  wire                pop,

    output wire                ready,
    output wire                empty,
    output wire [INDEX_W-1:0]  min_idx,
    output wire [DIST_W-1:0]   min_key
);
    // FSM states
    localparam [1:0] S_IDLE  = 2'd0,
                     S_UP    = 2'd1,   // compare with parent, move parent down or place
                     S_DN_RD = 2'd2,   // register both children
                     S_DN_WR = 2'd3;   // compare, move smaller child up or place

    reg [1:0]         state;
    reg [INDEX_W:0]   size;            // entries in the heap (0..NUM_CELLS)
    reg [INDEX_W-1:0] hole;            // position being filled by the sift
    reg [INDEX_W-1:0] parent;          // (hole-1)/2, precomputed for S_UP
    reg [INDEX_W-1:0] hold_idx;        // entry being sifted
    reg [DIST_W-1:0]  hold_key;

    // children registered in S_DN_RD
    reg               l_ok, r_ok;
    reg [INDEX_W-1:0] l_idx, r_idx;
    reg [DIST_W-1:0]  l_key, r_key;

    // memory read data
    wire [INDEX_W-1:0] a_idx, b_idx, pos_ram, pos_rdata;
    wire [DIST_W-1:0]  a_key, b_key;

    // -------- heap arithmetic (one bit wider so 2p+2 can't overflow) -------
    wire [INDEX_W:0]   hole_w  = {1'b0, hole};
    wire [INDEX_W:0]   left_w  = {hole_w[INDEX_W-1:0], 1'b1};   // 2p+1
    wire [INDEX_W:0]   right_w = left_w + 1'b1;                 // 2p+2
    // parent of a push's start position (registered into `parent`, so the
    // S_UP read -> compare -> write path doesn't include this subtraction)
    wire [INDEX_W-1:0] up_start = push_upd ? pos_rdata : size[INDEX_W-1:0];

    // -------- read-address muxes --------------------------------------------
    // port A: parent (sift-up), left child (sift-down), else the root (min)
    // port B: right child (sift-down), else the last entry (for pop)
    wire [INDEX_W:0]   last_w = size - 1'b1;
    wire [INDEX_W-1:0] a_raddr = (state == S_UP)    ? parent :
                                 (state == S_DN_RD) ? left_w[INDEX_W-1:0] : {INDEX_W{1'b0}};
    wire [INDEX_W-1:0] b_raddr = (state == S_DN_RD) ? right_w[INDEX_W-1:0] : last_w[INDEX_W-1:0];

    // -------- sift decisions ------------------------------------------------
    // S_UP: move the parent down while the held key is strictly smaller
    wire up_move  = (hole != 0) && (hold_key < a_key);
    // S_DN_WR: pick the smaller child, move it up while strictly smaller
    wire pick_r   = r_ok && (r_key < l_key);
    wire [INDEX_W-1:0] c_idx = pick_r ? r_idx : l_idx;
    wire [DIST_W-1:0]  c_key = pick_r ? r_key : l_key;
    wire [INDEX_W-1:0] c_pos = pick_r ? right_w[INDEX_W-1:0] : left_w[INDEX_W-1:0];
    wire dn_move  = l_ok && (c_key < hold_key);

    // -------- memory writes (combinational: land on this clock edge) --------
    wire up_wr   = (state == S_UP);
    wire dn_wr   = (state == S_DN_WR);
    wire move    = (up_wr && up_move) || (dn_wr && dn_move);
    wire heap_we = step_en && !clr && (up_wr || dn_wr);

    wire [INDEX_W-1:0] w_idx = !move ? hold_idx :
                               up_wr ? a_idx    : c_idx;
    wire [DIST_W-1:0]  w_key = !move ? hold_key :
                               up_wr ? a_key    : c_key;

    // -------- memories -------------------------------------------------------
    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_idx_a (
        .clk(clk), .we(heap_we), .waddr(hole), .wdata(w_idx),
        .raddr(a_raddr), .rdata(a_idx)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_idx_b (
        .clk(clk), .we(heap_we), .waddr(hole), .wdata(w_idx),
        .raddr(b_raddr), .rdata(b_idx)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(DIST_W)) u_key_a (
        .clk(clk), .we(heap_we), .waddr(hole), .wdata(w_key),
        .raddr(a_raddr), .rdata(a_key)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(DIST_W)) u_key_b (
        .clk(clk), .we(heap_we), .waddr(hole), .wdata(w_key),
        .raddr(b_raddr), .rdata(b_key)
    );

    // pos[cell] = heap position of that cell (written for every entry placed).
    // The write is registered one cycle so the sift compare (which picks w_idx)
    // stays off the LUTRAM write-enable decode; a read of the cell whose write
    // is still pending is forwarded.
    reg               pos_we_d = 1'b0;
    reg [INDEX_W-1:0] pos_waddr_d, pos_wdata_d;
    always @(posedge clk) begin
        pos_we_d    <= heap_we;
        pos_waddr_d <= w_idx;
        pos_wdata_d <= hole;
    end

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_pos (
        .clk(clk), .we(pos_we_d), .waddr(pos_waddr_d), .wdata(pos_wdata_d),
        .raddr(push_idx), .rdata(pos_ram)
    );
    assign pos_rdata = (pos_we_d && pos_waddr_d == push_idx) ? pos_wdata_d : pos_ram;

    assign ready   = (state == S_IDLE);
    assign empty   = (size == 0);
    assign min_idx = a_idx;
    assign min_key = a_key;

    // -------- FSM -----------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            size  <= 0;
            hole  <= 0;
        end else if (step_en) begin
            if (clr) begin
                state <= S_IDLE;
                size  <= 0;
            end else begin
                case (state)
                    S_IDLE: begin
                        if (push) begin
                            hold_idx <= push_idx;
                            hold_key <= push_key;
                            hole     <= up_start;
                            parent   <= (up_start - 1'b1) >> 1;
                            if (!push_upd)
                                size <= size + 1'b1;
                            state    <= S_UP;
                        end else if (pop && !empty) begin
                            // refill the root with the last entry, then sift it down
                            hold_idx <= b_idx;
                            hold_key <= b_key;
                            hole     <= 0;
                            size     <= size - 1'b1;
                            if (size != 1)
                                state <= S_DN_RD;
                        end
                    end

                    S_UP: begin
                        if (up_move) begin
                            hole   <= parent;
                            parent <= (parent - 1'b1) >> 1;
                        end else
                            state <= S_IDLE;
                    end

                    S_DN_RD: begin
                        l_ok  <= (left_w  < size);
                        r_ok  <= (right_w < size);
                        l_idx <= a_idx;
                        l_key <= a_key;
                        r_idx <= b_idx;
                        r_key <= b_key;
                        state <= S_DN_WR;
                    end

                    S_DN_WR: begin
                        if (dn_move) begin
                            hole  <= c_pos;
                            state <= S_DN_RD;
                        end else begin
                            state <= S_IDLE;
                        end
                    end
                endcase
            end
        end
    end
endmodule
