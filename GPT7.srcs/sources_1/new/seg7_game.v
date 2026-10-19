module seg7_game(
    input CLOCK,
    input button_press,
    output reg [6:0] segstage,
    output reg [3:0] anstage
);

    wire tick_075s;
    reg [2:0] stagecounter = 3'd0;
    reg enabled = 1'b0;

    wire [6:0] segstage0, segstage1, segstage2, segstage3, segstage4, segstage5, segstage6;
    wire [3:0] anstage0, anstage1, anstage2, anstage3, anstage4, anstage5, anstage6;

    stage0 zero(.segstage(segstage0), .anstage(anstage0));
    stage1 one(.segstage(segstage1), .anstage(anstage1));
    stage2 two(.segstage(segstage2), .anstage(anstage2));
    stage3 three(.segstage(segstage3), .anstage(anstage3));
    stage4 four(.segstage(segstage4), .anstage(anstage4));
    stage5 five(.segstage(segstage5), .anstage(anstage5));
    stage6 six(.segstage(segstage6), .anstage(anstage6));

    clock_075s clock075(.CLOCK(CLOCK), .tick(tick_075s));

    always @(posedge CLOCK) begin
        if (button_press)
            enabled <= ~enabled;
    end

    always @(posedge CLOCK) begin
        if (enabled && tick_075s) begin
            if (stagecounter == 3'd6)
                stagecounter <= 3'd0;
            else
                stagecounter <= stagecounter + 1'b1;
        end
    end

    always @(*) begin
        if (enabled) begin
            case (stagecounter)
                3'd0: begin segstage = segstage0; anstage = anstage0; end
                3'd1: begin segstage = segstage1; anstage = anstage1; end
                3'd2: begin segstage = segstage2; anstage = anstage2; end
                3'd3: begin segstage = segstage3; anstage = anstage3; end
                3'd4: begin segstage = segstage4; anstage = anstage4; end
                3'd5: begin segstage = segstage5; anstage = anstage5; end
                3'd6: begin segstage = segstage6; anstage = anstage6; end
                default: begin segstage = 7'b1111111; anstage = 4'b1111; end
            endcase
        end else begin
            segstage = 7'b1111111;
            anstage = 4'b1111;
        end
    end

endmodule


module stage0(output [6:0] segstage, output [3:0] anstage);
    assign segstage = 7'b0111111;
    assign anstage = 4'b1011;
endmodule

module stage1(output [6:0] segstage, output [3:0] anstage);
    assign segstage = 7'b1110111;
    assign anstage = 4'b1011;
endmodule

module stage2(output [6:0] segstage, output [3:0] anstage);
    assign segstage = 7'b1111011;
    assign anstage = 4'b0111;
endmodule

module stage3(output [6:0] segstage, output [3:0] anstage);
    assign segstage = 7'b1011111;
    assign anstage = 4'b1011;
endmodule

module stage4(output [6:0] segstage, output [3:0] anstage);
    assign segstage = 7'b1011111;
    assign anstage = 4'b1101;
endmodule

module stage5(output [6:0] segstage, output [3:0] anstage);
    assign segstage = 7'b0111011;
    assign anstage = 4'b1101;
endmodule

module stage6(output [6:0] segstage, output [3:0] anstage);
    assign segstage = 7'b0111111;
    assign anstage = 4'b1101;
endmodule


module clock_075s(
    input CLOCK,
    output reg tick
);

    reg [26:0] timer = 0;

    always @(posedge CLOCK) begin
        if (timer == 75000000 - 1) begin
            timer <= 0;
            tick <= 1'b1;
        end else begin
            timer <= timer + 1'b1;
            tick <= 1'b0;
        end
    end

endmodule