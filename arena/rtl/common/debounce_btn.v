// Buttons
module debounce_btn #(
    parameter LOCKOUT = 20_000_000
) (
    input  CLOCK,
    input  btn,
    output reg press_pulse = 0
);
    reg sync1 = 0, sync2 = 0;
    reg armed = 1;
    localparam integer CW = (LOCKOUT < 2) ? 1 : $clog2(LOCKOUT + 1);
    reg [CW-1:0] count = 0;

    always @(posedge CLOCK) begin
        sync1 <= btn;
        sync2 <= sync1;

        if (count != 0)
            count <= count - 1'b1;

        press_pulse <= 1'b0;

        if (sync2 && armed && count == 0) begin
            press_pulse <= 1'b1;
            armed       <= 1'b0;
            count       <= LOCKOUT;
        end

        if (!sync2 && count == 0)
            armed <= 1'b1;
    end
endmodule
