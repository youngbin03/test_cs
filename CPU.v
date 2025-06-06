// ───── CPU.v  (multi-cycle, top level) ─────
`timescale 1ns/1ps
`include "GLOBAL.v"

module CPU (
    input  wire clk,
    input  wire rst,
    output wire halt
);

/*────────────────── 1. Pipeline-like internal registers ──────────────────*/
    reg [31:0] PC, next_PC;
    reg [31:0] IR, MDR;
    reg [31:0] A,  B;
    reg [31:0] ALUOut;

/*────────────────── 2. Control signals ──────────────────*/
    wire [5:0] opcode = IR[31:26];
    wire [5:0] funct  = IR[5:0];

    wire [3:0] ALUOp;
    wire       MemRead, MemWrite, IRWrite;
    wire       RegDst,  RegWrite;
    wire       ALUSrcA;
    wire [1:0] ALUSrcB;
    wire       MemtoReg;
    wire       PCWrite, PCWriteCond;
    wire [1:0] PCSource;
    wire       IorD, SignExtend, SavePC;
    wire [2:0] state;          // debug

/*────────────────── 3. Controller ──────────────────*/
    CTRL ctrl(
        .clk(clk), .rst(rst),
        .opcode(opcode), .funct(funct),
        .ALUOp(ALUOp), .MemRead(MemRead), .MemWrite(MemWrite), .IRWrite(IRWrite),
        .RegDst(RegDst), .RegWrite(RegWrite),
        .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB),
        .MemtoReg(MemtoReg), .PCWrite(PCWrite), .PCWriteCond(PCWriteCond),
        .PCSource(PCSource), .IorD(IorD),
        .SignExtend(SignExtend), .SavePC(SavePC),
        .state(state)
    );

/*────────────────── 4. Memory ──────────────────*/
    wire [31:0] mem_addr  = IorD ? ALUOut : PC;
    wire [31:0] mem_rdata;
    MEM mem (
        .clk(clk), .rst(rst),
        .mem_addr(mem_addr),
        .MemWrite(MemWrite),
        .mem_write_data(B),
        .mem_read_data(mem_rdata)
    );

/*────────────────── 5. Register file ──────────────────*/
    wire [4:0] rd_addr1 = IR[25:21];
    wire [4:0] rd_addr2 = IR[20:16];

    wire [4:0] wr_addr = SavePC  ? 5'd31 :
                         RegDst  ? IR[15:11] :
                                   IR[20:16];

    wire [31:0] wr_data = SavePC   ? PC+4 :
                          MemtoReg ? MDR   :
                                     ALUOut;

    wire [31:0] rd_data1, rd_data2;

    RF rf(
        .clk(clk), .rst(rst),
        .rd_addr1(rd_addr1), .rd_addr2(rd_addr2),
        .rd_data1(rd_data1), .rd_data2(rd_data2),
        .RegWrite(RegWrite), .wr_addr(wr_addr), .wr_data(wr_data)
    );

/*────────────────── 6. ALU ──────────────────*/
    wire [31:0] imm = SignExtend ? {{16{IR[15]}}, IR[15:0]} :
                                   { 16'b0       , IR[15:0]};

    wire [31:0] srcA = ALUSrcA ? A : PC;
    wire [31:0] srcB = (ALUSrcB==2'b00) ? B       :
                       (ALUSrcB==2'b01) ? 32'd4   :
                       (ALUSrcB==2'b10) ? imm     :
                                          (imm<<2);

    wire [31:0] alu_result;
    ALU alu(
        .operand1(srcA), .operand2(srcB),
        .shamt(IR[10:6]), .funct(ALUOp),
        .alu_result(alu_result)
    );
    wire Zero = (alu_result == 32'd0);

/*────────────────── 7. Sequential logic & PC update ──────────────────*/
    wire branch_taken = (opcode==`OP_BEQ &&  Zero) ||
                        (opcode==`OP_BNE && !Zero);
    wire pc_enable = PCWrite | (PCWriteCond & branch_taken);

    always @(*) begin
        case (PCSource)
            2'b00: next_PC = alu_result;                           // PC+4
            2'b01: next_PC = ALUOut;                               // branch target
            2'b10: next_PC = {PC[31:28], IR[25:0], 2'b00};         // jump
            default: next_PC = 32'hxxxx_xxxx;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            PC<=0; IR<=0; MDR<=0; A<=0; B<=0; ALUOut<=0;
        end else begin
            if (MemRead) MDR <= mem_rdata;
            if (IRWrite) IR  <= mem_rdata;
            A      <= rd_data1;
            B      <= rd_data2;
            ALUOut <= alu_result;
            if (pc_enable) PC <= next_PC;

            /* ───── (옵션) WB 로그 ───── */
            if (RegWrite && wr_addr!=0)
                $display("%0t WB  R[%0d] <= %08h  (PC=%08h IR=%08h)",
                         $time, wr_addr, wr_data, PC, IR);
        end
    end

/*────────────────── 8. Halt (all-zero instruction) ──────────────────*/
    assign halt = (IR==32'd0);

endmodule
