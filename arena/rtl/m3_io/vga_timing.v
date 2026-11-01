`timescale 1ns / 1ps
`include "arena_defs.vh"
// 640x480 VGA timing; 25 MHz pixel enable is derived from the 100 MHz clock.
module vga_timing #(
    parameter PIX_TICK_DIV=`ARENA_PIX_TICK_DIV,
    parameter H_VISIBLE=`ARENA_VGA_H, parameter V_VISIBLE=`ARENA_VGA_V
) (
    input wire clk, input wire rst,
    output wire pixel_tick, output wire active_video,
    output wire [9:0] x, output wire [9:0] y,
    output wire hsync, output wire vsync
);
    localparam H_FRONT=16, H_SYNC=96, H_BACK=48;
    localparam V_FRONT=10, V_SYNC=2, V_BACK=33;
    localparam H_TOTAL=H_VISIBLE+H_FRONT+H_SYNC+H_BACK;
    localparam V_TOTAL=V_VISIBLE+V_FRONT+V_SYNC+V_BACK;
    localparam TICK_W=(PIX_TICK_DIV<2)?1:$clog2(PIX_TICK_DIV);
    reg [TICK_W-1:0] div_count;
    reg [9:0] h_count, v_count;
    assign pixel_tick = (div_count == PIX_TICK_DIV-1);
    assign x = h_count; assign y = v_count;
    assign active_video = (h_count < H_VISIBLE) && (v_count < V_VISIBLE);
    assign hsync = !((h_count >= H_VISIBLE+H_FRONT) && (h_count < H_VISIBLE+H_FRONT+H_SYNC));
    assign vsync = !((v_count >= V_VISIBLE+V_FRONT) && (v_count < V_VISIBLE+V_FRONT+V_SYNC));
    always @(posedge clk) begin
        if (rst) begin div_count<=0; h_count<=0; v_count<=0; end
        else begin
            if (pixel_tick) begin
                div_count <= 0;
                if (h_count == H_TOTAL-1) begin
                    h_count <= 0;
                    if (v_count == V_TOTAL-1) v_count <= 0;
                    else v_count <= v_count + 1'b1;
                end else h_count <= h_count + 1'b1;
            end else div_count <= div_count + 1'b1;
        end
    end
endmodule
