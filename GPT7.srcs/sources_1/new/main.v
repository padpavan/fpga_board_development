`timescale 1ns / 1ps

module top(
    input  CLOCK,
    input  sw0,
    input  sw1,
    input  sw14,
    input  sw15,
    input  BTNC,
    output [7:0] seg,
    output [3:0] an,
    output pmodoledrgb_cs,
    output pmodoledrgb_sdin,
    output pmodoledrgb_sclk,
    output pmodoledrgb_d_cn,
    output pmodoledrgb_resn,
    output pmodoledrgb_vccen,
    output pmodoledrgb_pmoden
);

    wire clk6p25m;
    clk_6p25m u_clk (.CLOCK(CLOCK), .clk6p25m(clk6p25m));

    wire [12:0] pixel_index;
    wire frame_begin, sending_pixels, sample_pixel;
    wire [15:0] oled_data = 16'h0000;

    wire button_press;
    debounce_btn u_debounce (
        .CLOCK(CLOCK),
        .btn(BTNC),
        .press_pulse(button_press)
    );

    wire [6:0] outseg1;
    wire [6:0] segstage;
    wire [3:0] anstage;

    seg7_init initseg(outseg1);
    seg7_game gameseg(CLOCK, button_press, segstage, anstage);
    wire stage2 = sw0 && sw1 && sw14 && sw15;

    assign seg = stage2 ? {1'b1, segstage} : {1'b1, outseg1};
    assign an  = stage2 ? anstage : 4'b1110;

    Oled_Display oled_inst (

        .clk(clk6p25m),
        .reset(1'b0),
        .pixel_data(oled_data),
        .pixel_index(pixel_index),
        .frame_begin(frame_begin),
        .sending_pixels(sending_pixels),
        .sample_pixel(sample_pixel),
        .cs(pmodoledrgb_cs),
        .sdin(pmodoledrgb_sdin),
        .sclk(pmodoledrgb_sclk),
        .d_cn(pmodoledrgb_d_cn),
        .resn(pmodoledrgb_resn),
        .vccen(pmodoledrgb_vccen),
        .pmoden(pmodoledrgb_pmoden)
    );
endmodule


module clk_6p25m(
    input  CLOCK,
    output reg clk6p25m = 1'b0
);

    // toggle every 8 cycles -> period 16 cycles -> 100 MHz / 16 = 6.25 MHz
    reg [2:0] count = 0;

    always @(posedge CLOCK) begin
        if (count == 3'd7) begin
            count    <= 0;
            clk6p25m <= ~clk6p25m;
        end else begin
            count <= count + 1'b1;
        end
    end

endmodule
