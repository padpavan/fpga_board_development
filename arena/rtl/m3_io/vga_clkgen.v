`timescale 1ns / 1ps
`include "arena_defs.vh"

// VGA
module vga_clkgen (
    input  wire clk,
    output wire clk_pix,
    output wire locked
);
    wire clk_fb, clk_fb_buf, clk_pix_raw;

    MMCME2_BASE #(
        .CLKIN1_PERIOD(10.0),
        .DIVCLK_DIVIDE(5),
        .CLKFBOUT_MULT_F(37.125),
        .CLKOUT0_DIVIDE_F(5.0)
    ) u_mmcm (
        .CLKIN1(clk), .CLKFBIN(clk_fb_buf), .CLKFBOUT(clk_fb), .CLKFBOUTB(),
        .CLKOUT0(clk_pix_raw), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(),
        .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(),
        .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0)
    );

    BUFG u_fb_buf  (.I(clk_fb),      .O(clk_fb_buf));
    BUFG u_pix_buf (.I(clk_pix_raw), .O(clk_pix));
endmodule
