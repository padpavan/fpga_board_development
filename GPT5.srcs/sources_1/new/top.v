`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// EE2026 - GPT-5: OLED familiarisation (Section 3.1)
//
// Bring-up design: the Pmod OLED RGB displays a solid background colour.
//   SW4 = 0 -> green background (16'h07E0)
//   SW4 = 1 -> red   background (16'hF800)
//
// Requires the course-provided Oled_Display.v interface module.
//////////////////////////////////////////////////////////////////////////////////

module top(
    input  CLOCK,                       // 100 MHz system clock, pin W5
    input  sw4,                         // SW4 (pin W15): background colour select
    input  reset,                       // BTNU (pin T18) or tie to 1'b0
    output pmodoledrgb_cs,
    output pmodoledrgb_sdin,
    output pmodoledrgb_sclk,
    output pmodoledrgb_d_cn,
    output pmodoledrgb_resn,
    output pmodoledrgb_vccen,
    output pmodoledrgb_pmoden
);

    // ---------------------------------------------------------------
    // 6.25 MHz clock for the OLED SPI interface: 100 MHz / 16
    // clk_div[3] is a 50% duty square wave with period 16 clocks.
    // ---------------------------------------------------------------
    reg [3:0] clk_div = 4'd0;

    always @(posedge CLOCK)
        clk_div <= clk_div + 1'b1;

    wire clk6p25m = clk_div[3];

    // ---------------------------------------------------------------
    // 16-bit RGB565 colour: 5 bits red, 6 bits green, 5 bits blue.
    //   green = 16'b00000_111111_00000 = 16'h07E0
    //   red   = 16'b11111_000000_00000 = 16'hF800
    // ---------------------------------------------------------------
    wire [15:0] oled_data = sw4 ? 16'hF800 : 16'h07E0;

    // ---------------------------------------------------------------
    // OLED interface module (provided by the course template)
    // ---------------------------------------------------------------
    wire [12:0] pixel_index;
    wire        frame_begin;
    wire        sending_pixels;
    wire        sample_pixel;

    Oled_Display oled_inst (
        .clk            (clk6p25m),
        .reset          (reset),
        .pixel_data     (oled_data),
        .pixel_index    (pixel_index),
        .frame_begin    (frame_begin),
        .sending_pixels (sending_pixels),
        .sample_pixel   (sample_pixel),
        .cs             (pmodoledrgb_cs),
        .sdin           (pmodoledrgb_sdin),
        .sclk           (pmodoledrgb_sclk),
        .d_cn           (pmodoledrgb_d_cn),
        .resn           (pmodoledrgb_resn),
        .vccen          (pmodoledrgb_vccen),
        .pmoden         (pmodoledrgb_pmoden)
    );

endmodule
