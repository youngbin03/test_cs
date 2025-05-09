`timescale 1ns / 1ps

module MEM(
    input               clk,
    input               rst,

    input  [31:0]       mem_addr,
    input               MemWrite,
    input  [31:0]       mem_write_data,
    output reg [31:0]   mem_read_data
);

    reg [31:0] memory [0:8191];
    integer    i;

    initial begin
        $readmemh("initial_mem.mem", memory);
    end

    // ────────────────────────────────────────────────
    // 1) 비동기 READ  (항상 combinational)
    // ────────────────────────────────────────────────
    always @(*) begin
        mem_read_data = memory[mem_addr >> 2];
    end

    // ────────────────────────────────────────────────
    // 2) 동기 WRITE  &   디버그 로그
    //    - write : rising-edge, MemWrite==1
    //    - read  : rising-edge, MemWrite==0 이면서 주소가 바뀐 경우만 찍음
    // ────────────────────────────────────────────────
    reg [31:0] prev_addr;      // 직전 클럭의 주소(로그용)

    always @(posedge clk) begin
        if (rst) begin
            prev_addr <= 32'hffff_ffff;   // 불가능한 값으로 초기화
        end
        else begin
            /* WRITE */
            if (MemWrite) begin
                memory[mem_addr >> 2] <= mem_write_data;
                $display("%0t MEM: W  addr=%08h data=%08h",
                         $time, mem_addr, mem_write_data);
            end
            /* READ (주소 변동 감지용) */
            else if (mem_addr != prev_addr) begin
                $display("%0t MEM: R  addr=%08h data=%08h",
                         $time, mem_addr, memory[mem_addr >> 2]);
            end
            prev_addr <= mem_addr;
        end
    end

endmodule
