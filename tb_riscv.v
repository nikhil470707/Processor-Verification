`timescale 1ns/1ps
module tb_riscv;
    reg clk;
    reg reset;

    // instantiate DUT
    riscv_core dut (
        .clk(clk),
        .reset(reset)
    );

    // clock generator
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period
    end

    // load instruction memory and run test
    initial begin
        $dumpfile("riscv.vcd");
        $dumpvars(0, tb_riscv);

        // initial reset
        reset = 1;
        #20;

        // load instruction memory (instr.mem) into DUT.imem
        $readmemh("instr.mem", dut.imem);

        // show loaded instructions
        $display("Loaded instructions:");
        integer k;
        for (k=0; k<9; k=k+1) $display("%0d: %08x", k, dut.imem[k]);

        #20;
        reset = 0;

        #500;

        // checks (read regs from hierarchical DUT)
        $display("Checking registers...");
        check_reg(1, 5);
        check_reg(2, 10);
        check_reg(3, 15);
        check_reg(4, 15);
        check_reg(5, 0);
        check_reg(6, 32);
        check_reg(7, 15);

        $display("dmem[0] = %0d", dut.dmem[0]);

        $display("SIMULATION PASS");
        $finish;
    end

    // helper task 
    task check_reg;
        input integer idx;
        input integer expected;
        integer actual;
        begin
            if (idx == 0) actual = 0;
            else actual = dut.regs[idx];
            if (actual !== expected) begin
                $display("ERROR: x%0d = %0d, expected %0d", idx, actual, expected);
                $finish;
            end else begin
                $display("OK: x%0d == %0d", idx, actual);
            end
        end
    endtask

endmodule
