`timescale 1ns / 1ps
`include "arena_defs.vh"
// Splits one button into a short press and a long hold (CONTRACT v1.11).
//
// press_pulse comes from the button's common/debounce_btn, which starts the
// timer.  debounce_btn has no level output, so the release is read from this
// module's own 2-FF copy of btn.  Anything low inside the first LOCKOUT cycles
// is treated as contact bounce, so a short_pulse never comes earlier than that.
//
//   short_pulse: released before HOLD_CYCLES (one cycle, on release)
//   long_pulse : still held at HOLD_CYCLES (one cycle, while held; the release
//                that follows gives no short_pulse)
//   enable = 0 : ignores presses and cancels a hold in progress, with no pulse
module btn_hold #(
    parameter HOLD_CYCLES = 3 * `ARENA_CLK_HZ,
    parameter LOCKOUT     = 20_000_000       // same value as the debounce_btn
) (
    input  wire clk, input wire rst, input wire enable,
    input  wire btn, input wire press_pulse,
    output reg  short_pulse, output reg long_pulse
);
    function integer bits_for;
        input integer value;
        integer v;
        begin
            bits_for = 1;
            for (v = value; v > 1; v = v >> 1) bits_for = bits_for + 1;
        end
    endfunction
    localparam CW = bits_for(HOLD_CYCLES);
    localparam [1:0] S_IDLE = 2'd0, S_HELD = 2'd1, S_WAIT_RELEASE = 2'd2;

    reg sync1 = 1'b0, sync2 = 1'b0;
    reg [1:0] state;
    reg [CW-1:0] count;

    always @(posedge clk) begin
        sync1 <= btn;
        sync2 <= sync1;
        short_pulse <= 1'b0;
        long_pulse  <= 1'b0;
        if (rst || !enable) begin
            state <= S_IDLE;
            count <= 0;
        end else case (state)
            S_IDLE: if (press_pulse) begin
                state <= S_HELD;
                count <= 1;
            end
            S_HELD: begin
                count <= count + 1'b1;
                if (count == HOLD_CYCLES - 1) begin
                    long_pulse <= 1'b1;
                    state <= S_WAIT_RELEASE;
                end else if (!sync2 && count >= LOCKOUT) begin
                    short_pulse <= 1'b1;
                    state <= S_IDLE;
                end
            end
            S_WAIT_RELEASE: if (!sync2) state <= S_IDLE;
            default: state <= S_IDLE;
        endcase
    end
endmodule
