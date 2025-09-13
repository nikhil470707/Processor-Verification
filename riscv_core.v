// riscv_core.v
// Single-cycle simple RV32I subset core (pure Verilog)

`timescale 1ns/1ps
module riscv_core (
    input  clk,
    input  reset
);

    // Program counter
    reg [31:0] pc, next_pc;

    // Instruction and data memories (behavioral)
    reg [31:0] imem [0:255];
    reg [31:0] dmem [0:255];

    // Register file
    reg [31:0] regs [0:31];

    // instruction fields
    wire [31:0] instr;
    wire [6:0] opcode;
    wire [4:0] rd, rs1, rs2;
    wire [2:0] funct3;
    wire [6:0] funct7;

    assign instr = imem[pc[9:2]]; // word addressed
    assign opcode = instr[6:0];
    assign rd = instr[11:7];
    assign funct3 = instr[14:12];
    assign rs1 = instr[19:15];
    assign rs2 = instr[24:20];
    assign funct7 = instr[31:25];

    // immediates
    wire [31:0] imm_i, imm_s, imm_b, imm_j;

    assign imm_i = {{20{instr[31]}}, instr[31:20]};

    wire [11:0] imm_s_raw;
    assign imm_s_raw = {instr[31:25], instr[11:7]};
    assign imm_s = {{20{imm_s_raw[11]}}, imm_s_raw};

    // B-type immediate: bits [12|10:5|4:1|11] << 1
    wire [12:0] imm_b_raw;
    assign imm_b_raw = {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    assign imm_b = {{19{imm_b_raw[12]}}, imm_b_raw};

    // J-type immediate
    wire [20:0] imm_j_raw;
    assign imm_j_raw = {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
    assign imm_j = {{11{imm_j_raw[20]}}, imm_j_raw};

    // ALU inputs and outputs
    reg [31:0] alu_in1, alu_in2, alu_out;
    reg mem_we, mem_re;
    reg [31:0] mem_rdata;

    // register read function (combinational read)
    function [31:0] rf_read;
        input [4:0] idx;
        begin
            if (idx == 5'd0) rf_read = 32'd0;
            else rf_read = regs[idx];
        end
    endfunction

    // combinational ALU / control
    always @(*) begin
        alu_in1 = rf_read(rs1);
        alu_in2 = rf_read(rs2);
        alu_out = 32'h00000000;
        mem_we = 1'b0;
        mem_re = 1'b0;
        next_pc = pc + 4;

        case (opcode)
            7'b0010011: begin // OP-IMM
                case (funct3)
                    3'b000: alu_out = alu_in1 + imm_i; // ADDI
                    3'b111: alu_out = alu_in1 & imm_i; // ANDI
                    3'b110: alu_out = alu_in1 | imm_i; // ORI
                    3'b100: alu_out = alu_in1 ^ imm_i; // XORI
                    default: alu_out = 32'h0;
                endcase
            end

            7'b0110011: begin // OP (R-type)
                case ({funct7, funct3})
                    {7'b0000000,3'b000}: alu_out = alu_in1 + alu_in2; // ADD
                    {7'b0100000,3'b000}: alu_out = alu_in1 - alu_in2; // SUB
                    {7'b0000000,3'b111}: alu_out = alu_in1 & alu_in2; // AND
                    {7'b0000000,3'b110}: alu_out = alu_in1 | alu_in2; // OR
                    {7'b0000000,3'b100}: alu_out = alu_in1 ^ alu_in2; // XOR
                    default: alu_out = 32'h0;
                endcase
            end

            7'b0000011: begin // LOAD (LW)
                case (funct3)
                    3'b010: begin // LW
                        mem_re = 1'b1;
                        alu_out = alu_in1 + imm_i; // compute address
                    end
                    default: alu_out = 32'h0;
                endcase
            end

            7'b0100011: begin // STORE (SW)
                case (funct3)
                    3'b010: begin // SW
                        mem_we = 1'b1;
                        alu_out = alu_in1 + imm_s; // address
                    end
                    default: alu_out = 32'h0;
                endcase
            end

            7'b1100011: begin // BRANCH
                case (funct3)
                    3'b000: begin // BEQ
                        if (rf_read(rs1) == rf_read(rs2)) next_pc = pc + imm_b;
                    end
                    3'b001: begin // BNE
                        if (rf_read(rs1) != rf_read(rs2)) next_pc = pc + imm_b;
                    end
                    default: ;
                endcase
            end

            7'b1101111: begin // JAL
                next_pc = pc + imm_j;
            end

            7'b1100111: begin // JALR
                if (funct3 == 3'b000) begin
                    next_pc = (rf_read(rs1) + imm_i) & ~32'h1;
                end
            end

            default: begin
                // NOP / unsupported
            end
        endcase
    end

    // sequential logic: PC, memory, writeback
    integer i;
    initial begin
        // initialize memories and regs
        for (i=0; i<256; i=i+1) begin
            imem[i] = 32'h00000013; // addi x0,x0,0 -> NOP
            dmem[i] = 32'h0;
        end
        for (i=0; i<32; i=i+1) regs[i] = 32'h0;
        pc = 32'h0;
        next_pc = 32'h0;
        mem_rdata = 32'h0;
    end

    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'h0;
            // clear regs
            for (i=0; i<32; i=i+1) regs[i] <= 32'h0;
        end else begin
            pc <= next_pc;

            // memory write on clock edge (store)
            if (mem_we) begin
                // word address
                dmem[ alu_out[9:2] ] <= rf_read(rs2);
            end

            // memory read (capture read data)
            if (mem_re) begin
                mem_rdata <= dmem[ alu_out[9:2] ];
            end else begin
                mem_rdata <= mem_rdata;
            end

            // writeback logic - which instructions write registers?
            // We determine by opcode
            case (opcode)
                7'b0010011: begin // OP-IMM -> write alu_out
                    if (rd != 5'd0) regs[rd] <= alu_out;
                end
                7'b0110011: begin // OP -> write alu_out
                    if (rd != 5'd0) regs[rd] <= alu_out;
                end
                7'b0000011: begin // LW -> write mem_rdata
                    if (rd != 5'd0) regs[rd] <= mem_rdata;
                end
                7'b1101111: begin // JAL -> rd = pc + 4 (pc is old pc)
                    if (rd != 5'd0) regs[rd] <= pc + 32'd4;
                end
                7'b1100111: begin // JALR
                    if (funct3 == 3'b000) begin
                        if (rd != 5'd0) regs[rd] <= pc + 32'd4;
                    end
                end
                default: ;
            endcase
        end
    end

    // simple runtime assertions (simulation-only)
    always @(posedge clk) begin
        if (!reset) begin
            if (pc[1:0] != 2'b00) begin
                $display("FATAL: PC misaligned: %h", pc);
                $finish;
            end
            if (regs[0] != 32'h0) begin
                $display("FATAL: Register x0 modified to %h", regs[0]);
                $finish;
            end
        end
    end

    // expose register read for testbench via function
    // (testbench will access via hierarchical reference: dut.regs[idx])
    // No extra code required.

endmodule
