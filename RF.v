`timescale 1ns / 100ps

module RF (
    input clk,
    input rst,

    // Read-related ports
    input [4:0] rd_addr1,
    input [4:0] rd_addr2,
    output reg [31:0] rd_data1,
    output reg [31:0] rd_data2,

    // Write-related ports
    input RegWrite,
    input [4:0] wr_addr,
    input [31:0] wr_data
);
    // 32개의 32비트 레지스터
    (* keep = "true" *)
    reg [31:0] register_file [0:31];

    //=============
    // 1) 비동기 읽기
    //=============
    always @(*) begin
        rd_data1 = register_file[rd_addr1];
        rd_data2 = register_file[rd_addr2];
    end

    //=============
    // 2) 동기 쓰기
    //=============
    always @(posedge clk) begin
        if (rst) begin
            // 리셋 시, 파일로부터 초기값 로드
            // 파일 형태: 32줄의 HEX 값, 예) initial_reg.mem
            $readmemh("initial_reg.mem", register_file);
        end
        else begin
            // RegWrite가 1이면 쓰기 수행
            // 0번 레지스터($zero)는 MIPS 규약상 항상 0이므로 쓰기 금지
            if (RegWrite && (wr_addr != 0)) begin
                register_file[wr_addr] <= wr_data;
            end
        end
    end

endmodule