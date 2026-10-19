module debounce_btn #(
    parameter LOCKOUT = 20_000_000        // ===== MODIFY HERE ===== 200 ms
) (
    input  CLOCK,
    input  btn,
    output reg press_pulse = 0
);
    reg sync1 = 0, sync2 = 0;
    reg armed = 1;
    localparam integer CW = (LOCKOUT < 2) ? 1 : $clog2(LOCKOUT + 1);
    reg [CW-1:0] count = 0;           // auto-sized for configured lockout

    always @(posedge CLOCK) begin
        sync1 <= btn;                     // synchronizer stage 1
        sync2 <= sync1;                   // synchronizer stage 2

        if (count != 0)
            count <= count - 1'b1;

        press_pulse <= 1'b0;              // default: no pulse this cycle

        if (sync2 && armed && count == 0) begin
            press_pulse <= 1'b1;          // valid press
            armed       <= 1'b0;          // ignore until release
            count       <= LOCKOUT;       // start 200 ms debounce window
        end

        if (!sync2 && count == 0)
            armed <= 1'b1;                // re-arm after release
    end
endmodule