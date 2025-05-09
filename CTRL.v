//───────────────────────────────────────────────────────────────
//  CTRL.v  —  multi-cycle MIPS controller  (jump-fix 2025-05-09)
//───────────────────────────────────────────────────────────────
`timescale 1ns/1ps
`include "GLOBAL.v"

module CTRL (
    input             clk,
    input             rst,
    input      [5:0]  opcode,
    input      [5:0]  funct,

    output reg [3:0]  ALUOp,
    output reg        MemRead, MemWrite, IRWrite,
    output reg        RegDst, RegWrite,
    output reg        ALUSrcA,
    output reg [1:0]  ALUSrcB,
    output reg        MemtoReg,
    output reg        PCWrite, PCWriteCond,
    output reg [1:0]  PCSource,
    output reg        IorD, SignExtend, SavePC,
    output reg        JR,              // JR 신호 추가
    output reg        SaveOldPC,       // JAL을 위한 신호 추가

    output reg [2:0]  state          // 디버그(현재 FSM 상태)
);

/*------------------------------------------------------------------
    8-state FSM : IF → ID → EX → MEM → WB …
    ⓐ  J / JAL  은  ID 단계에서   PCSource=2, PCWrite=1  로 즉시 점프
    ⓑ  JAL 은   SavePC=1, SaveOldPC=1로 oldPCplus4 ← PC+4 (WB_I 단계에서 수행)
    ⓒ  JR 은   ID 단계에서 감지하고 PCWrite=1, JR=1로 즉시 점프
------------------------------------------------------------------*/

/* ───── 상태 부호화 ───── */
localparam IF   = 3'd0, ID   = 3'd1,
           EX_R = 3'd2, EX_I = 3'd3,
           MEMR = 3'd4, MEMW = 3'd5,
           WB_R = 3'd6, WB_I = 3'd7;

reg [2:0] next_state;

/* ───── 상태 레지스터 ───── */
always @(posedge clk or posedge rst)
    if (rst) state <= IF;
    else     state <= next_state;

/* ───── 제어 신호 조합 논리 ───── */
always @(*) begin
    /* 기본값 = 0 */
    {ALUOp,MemRead,MemWrite,IRWrite,RegDst,RegWrite,ALUSrcA,ALUSrcB,
     MemtoReg,PCWrite,PCWriteCond,PCSource,IorD,SignExtend,SavePC,JR,SaveOldPC} = 0;
    next_state = state;

    case (state)
    //============================================================ IF
    IF : begin
        MemRead = 1;  IRWrite = 1;  PCWrite = 1;
        ALUOp   = `ALU_ADDU;          // PC+4
        ALUSrcB = 2'b01;
        next_state = ID;
    end
    //============================================================ ID
    ID : begin
        ALUOp   = `ALU_ADDU;          // 분기 타깃 계산
        ALUSrcB = 2'b11;

        case (opcode)
            `OP_RTYPE           : begin
                if (funct == `FUNCT_JR) begin
                    JR = 1;            // JR 신호 활성화
                    PCWrite = 1;       // PC 즉시 갱신
                    next_state = IF;   // ID에서 바로 IF로 이동
                end else begin
                    next_state = EX_R;
                end
            end

            `OP_LW, `OP_SW      : begin SignExtend=1; next_state = EX_I; end
            `OP_BEQ, `OP_BNE    : begin SignExtend=1; next_state = MEMR; end

            //―――――――――― jump / jump-and-link ――――――――――
            `OP_J,  `OP_JAL     : begin
                PCSource = 2'b10;   // jump target
                PCWrite  = 1;       // 즉시 PC 갱신
                
                if (opcode == `OP_JAL) begin
                    SaveOldPC = 1;   // 원래 PC+4 값 저장 (CPU.v의 oldPCplus4에 저장됨)
                    SavePC = 1;      // JAL은 $31에 PC+4 저장
                    RegWrite = 1;    // $31에 쓰기 위해 필요
                end
                
                next_state = IF;   // JAL: $31 저장 / J: RegWrite=0
            end

            // 논리-즉시형 : zero-extend
            `OP_ANDI, `OP_ORI, `OP_XORI
                                 : begin SignExtend=0; next_state = EX_I; end

            default              : begin SignExtend=1; next_state = EX_I; end
        endcase
    end
    //============================================================ EX_R
    EX_R : begin
        ALUSrcA = 1;
        case (funct)
            `FUNCT_ADDU : ALUOp = `ALU_ADDU;
            `FUNCT_SUBU : ALUOp = `ALU_SUBU;
            `FUNCT_AND  : ALUOp = `ALU_AND;
            `FUNCT_OR   : ALUOp = `ALU_OR;
            `FUNCT_XOR  : ALUOp = `ALU_XOR;
            `FUNCT_NOR  : ALUOp = `ALU_NOR;
            `FUNCT_SLT  : ALUOp = `ALU_SLT;
            `FUNCT_SLTU : ALUOp = `ALU_SLTU;
            `FUNCT_SLL  : ALUOp = `ALU_SLL;
            `FUNCT_SRL  : ALUOp = `ALU_SRL;
            `FUNCT_SRA  : ALUOp = `ALU_SRA;
            default     : ALUOp = `ALU_ADDU;
        endcase
        next_state = WB_R;
    end
    //============================================================ EX_I
    EX_I : begin
        ALUSrcA = 1;          // rs
        ALUSrcB = 2'b10;      // +imm
        case (opcode)
            `OP_LUI   : ALUOp = `ALU_LUI;
            `OP_ANDI  : ALUOp = `ALU_AND;
            `OP_ORI   : ALUOp = `ALU_OR;
            `OP_XORI  : ALUOp = `ALU_XOR;
            `OP_SLTI  : ALUOp = `ALU_SLT;
            `OP_SLTIU : ALUOp = `ALU_SLTU;
            default   : ALUOp = `ALU_ADDU;   // ADDIU, LW, SW …
        endcase
        // zero-extend only ANDI/ORI/XORI
        SignExtend = ~(opcode==`OP_ANDI || opcode==`OP_ORI || opcode==`OP_XORI);

        if      (opcode == `OP_LW) next_state = MEMR;
        else if (opcode == `OP_SW) next_state = MEMW;
        else                       next_state = WB_I;
    end
    //============================================================ MEMR
    MEMR : begin
        // ─── Branch (BEQ/BNE) ───
        if (opcode==`OP_BEQ || opcode==`OP_BNE) begin
            ALUSrcA     = 1;      // rs
            ALUSrcB     = 2'b00;  // rt
            ALUOp       = `ALU_SUBU;
            PCSource    = 2'b01;  // ALUOut
            PCWriteCond = 1;
            next_state  = IF;
        end
        // ─── LW ───
        else if (opcode==`OP_LW) begin
            MemRead  = 1;
            IorD     = 1;
            next_state = WB_I;
        end
        // ─── SW ───
        else begin                 // SW
            MemWrite = 1;
            IorD     = 1;
            next_state = IF;
        end
    end
    //============================================================ MEMW
    MEMW : begin
        MemWrite = 1;  IorD = 1;
        next_state = IF;
    end
    //============================================================ WB_R
    WB_R : begin
        RegDst   = 1;  RegWrite = 1;
        next_state = IF;
    end
    //============================================================ WB_I
    WB_I : begin
        RegWrite = (opcode != `OP_J);   // J는 레지스터 쓰기 X
        MemtoReg = (opcode == `OP_LW);  // LW만 메모리→레지스터
        SavePC   = (opcode == `OP_JAL); // JAL: $ra 저장
        next_state = IF;
    end
    endcase
end
endmodule
