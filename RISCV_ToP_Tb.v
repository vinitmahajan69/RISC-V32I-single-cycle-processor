`timescale 1ns / 1ps

module RISCV_ToP_Tb;
  reg clk,rst;
  
  RISCV_Top UUT(.clk(clk), .rst(rst));
  
  initial begin
  clk = 0;
  end
  
  always #50 clk = ~clk;
  
  initial begin
    rst = 1'b1;
    #50;
    rst = 1'b0;
    #1000;
    $finish;
  end
  
endmodule