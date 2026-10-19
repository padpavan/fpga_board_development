`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Self-test for grid_mem : write/read via the two ports and check that the
// write port does not corrupt the read port when enabled separately.
//////////////////////////////////////////////////////////////////////////////
module tb_grid_mem;
    reg        clk = 0;
    reg        we  = 0;
    reg  [7:0] waddr = 0;
    reg  [3:0] wdata = 0;
    reg  [7:0] raddr = 0;
    wire [3:0] rdata;

    grid_mem #(.ADDR_W(8), .DATA_W(4)) dut (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdata),
        .raddr(raddr), .rdata(rdata)
    );

    always #5 clk = ~clk;

    integer errors;

    task write_cell;
        input [7:0] a;
        input [3:0] d;
        begin
            we    = 1'b1;
            waddr = a;
            wdata = d;
            @(posedge clk); #1;
            we    = 1'b0;
        end
    endtask

    task check_cell;
        input [7:0] a;
        input [3:0] exp;
        begin
            raddr = a;
            #1;
            if (rdata !== exp) begin
                $display("FAIL addr %0d: got %h, expected %h", a, rdata, exp);
                errors = errors + 1;
            end else begin
                $display("PASS addr %0d == %h", a, rdata);
            end
        end
    endtask

    initial begin
        errors = 0;
        #20;
        write_cell(8'd10,  4'hA);
        write_cell(8'd255, 4'hF);
        write_cell(8'd0,   4'h1);

        check_cell(8'd10,  4'hA);
        check_cell(8'd255, 4'hF);
        check_cell(8'd0,   4'h1);

        if (errors == 0)
            $display("TB_GRID_MEM PASS");
        else
            $display("TB_GRID_MEM FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
