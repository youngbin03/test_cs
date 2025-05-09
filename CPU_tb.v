// ===== CPU_tb.v (with monitor) =====
`timescale 1ns / 1ps
`include "GLOBAL.v"

module CPU_tb;
    integer i;
    integer FAILED;

    reg clk;
    reg rst;
    wire halt;

    // Reference memories
    reg [31:0] register_file [0:31];
    reg [31:0] memory        [0:8191];

    // Instantiate CPU
    CPU cpu (
        .clk(clk),
        .rst(rst),
        .halt(halt)
    );

    // Load references
    initial begin
        $readmemh("reference_mem.mem", memory);
        $readmemh("reference_reg.mem", register_file);
    end

    // Clock generator
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Monitor internal signals
    initial begin
        $display("Time  PC      IR       A       B       ALUOut   MDR");
        $monitor("%0t  %h  %h  %h  %h  %h  %h",
                 $time,
                 cpu.PC,
                 cpu.IR,
                 cpu.A,
                 cpu.B,
                 cpu.ALUOut,
                 cpu.MDR);
    end

    // Testbench settings
    initial begin
        FAILED = 0;
        rst = 1;
        #15 rst = 0;
        @(posedge halt);
        $display("Program Terminate\n");

        // Check registers
        for (i = 0; i < 32; i = i + 1) begin
            if (cpu.rf.register_file[i] !== register_file[i]) begin
                FAILED = 1;
                $display("REG MISMATCH @%0d: cpu=0x%h ref=0x%h",
                         i, cpu.rf.register_file[i], register_file[i]);
            end
        end
        // Check memory
        for (i = 0; i < 8192; i = i + 1) begin
            if (cpu.mem.memory[i] !== memory[i]) begin
                FAILED = 1;
                $display("MEM MISMATCH @%0d: cpu=0x%h ref=0x%h",
                         i, cpu.mem.memory[i], memory[i]);
            end
        end

        if (FAILED) begin
            $display("Simulation failed.");
        end else begin
            $display("Simulation success!!!");
        end
        $finish();
    end

endmodule
