`timescale 1ns / 1ps

module program_counter(
input clk,rst,
input [31:0] pc_in,
output reg [31:0] pc_out
);
always@(posedge clk,posedge rst) begin
if (rst) begin
pc_out <= 32'b00;
end else begin
pc_out <= pc_in;
end
end
endmodule



module pc_adder(
input [31:0] pc_in,
output reg [31:0] pc_next
);
always@(*)begin
pc_next <= pc_in + 4;
end
endmodule


module pc_mux(
input [31:0] pc_in,
input [31:0] pc_branch,
input pc_select,
output reg[31:0] pc_out
);
always@(*)begin
if (pc_select == 1'b0) begin
pc_out <= pc_in;
end else begin
pc_out <= pc_branch;
end
end
endmodule


module Instruction_Memory(
input rst,clk,
input [31:0] read_address,
output [31:0] instruction_out
);
reg [31:0] I_Mem [0:63];
wire [5:0] word_idx = read_address[7:2];
integer k;
assign instruction_out = I_Mem[word_idx];
always@(posedge clk,posedge rst)
begin
if (rst) begin
for (k = 0; k < 64; k=k + 1) begin
I_Mem[k] <= 32'b0;
end
end
else begin

I_Mem[0] = 32'b00000000000000000000000000000000; // nop
I_Mem[1] = 32'b00000001100110000000011010110011; // add x13, x16, x25
I_Mem[2] = 32'b01000000001101000000001010110011; // sub x5, x8, x3
I_Mem[3] = 32'b00000000001100010111000010110011; // and x1, x2, x3
I_Mem[4] = 32'b00000000010100011110001000110011; // or x4, x3, x5
I_Mem[5] = 32'b00000000010100011100001000110011; // xor x4, x3, x5
I_Mem[6] = 32'b00000000010100011001001000110011; // sll x4, x3, x5
I_Mem[7] = 32'b00000000010100011101001000110011; // srl x4, x3, x5
I_Mem[8] = 32'b01000000001000011101001010110011; // sra x5, x3, x2
I_Mem[9] = 32'b00000000001000011010001010110011; // slt x5, x3, x2
// I-type
I_Mem[10] = 32'b00000000001010101000101100010011; // addi x22, x21, 2
I_Mem[11] = 32'b00000000001101000110010010010011; // ori x9, x8, 3
I_Mem[12] = 32'b00000000010001000110010010010011; // xori x9, x8, 4 (corrected from 3? but keep)
I_Mem[13] = 32'b00000000010100010111000010010011; // andi x1, x2, 5
I_Mem[14] = 32'b00000000011000011001001000010011; // slli x4, x3, 6
I_Mem[15] = 32'b00000000011100011101001000010011; // srli x4, x3, 7
I_Mem[16] = 32'b00000000100000011101001010010011; // srai x5, x3, 8
I_Mem[17] = 32'b00000000100100011010001010010011; // slti x5, x3, 9
// L-type
I_Mem[18] = 32'b00000000010100011000010010000011; // lb x9, 5(x3)
I_Mem[19] = 32'b00000000001100011001010010000011; // lh x9, 3(x3)
I_Mem[20] = 32'b00000000111100010010010000000011; // lw x8, 15(x2)
// S-type
I_Mem[21] = 32'b00000000111100011000010000100011; // sb x15, 8(x3)
I_Mem[22] = 32'b00000000111000110001010100100011; // sh x14, 10(x6)
I_Mem[23] = 32'b00000000111000110010011000100011; // sw x14, 12(x6)
// B-type
I_Mem[24] = 32'b00000000100101001000011001100011; // beq x9, x9, 12
I_Mem[25] = 32'b00000000100101001001011101100011; // bne x9, x9, 14
// U-type
I_Mem[26] = 32'b00000000000000101000000110110111; // lui x3, 40
I_Mem[27] = 32'b00000000000000101000001010010111; // auipc x5, 20
// J-type
I_Mem[28] = 32'b00000000000000000101000001101111; // jal x1, 20
end
end
endmodule


module Register_File(
input clk,rst, RegWrite,
input [4:0] Rs1, Rs2, Rd,
input [31:0] Write_data,
output [31:0] read_data1, read_data2
);
reg [31:0] Registers [31:0];
initial begin
Registers[0] = 0;
Registers[1] = 3;
Registers[2] = 2;
Registers[3] = 12;
Registers[4] = 20;
Registers[5] = 3;
Registers[6] = 44;
Registers[7] = 4;
Registers[8] = 2;
Registers[9] = 1;
Registers[10] = 23;
Registers[11] = 4;
Registers[12] = 90;
Registers[13] = 10;
Registers[14] = 20;
Registers[15] = 30;
Registers[16] = 40;
Registers[17] = 50;
Registers[18] = 60;
Registers[19] = 70;
Registers[20] = 80;
Registers[21] = 90;
Registers[22] = 10;
Registers[23] = 70;
Registers[24] = 60;
Registers[25] = 65;
Registers[26] = 4;
Registers[27] = 32;
Registers[28] = 12;
Registers[29] = 34;
Registers[30] = 5;
Registers[31] = 10;
end

integer k;
always@(posedge clk) begin
if (rst)
begin
for (k = 0; k < 32; k=k + 1) begin
Registers[k] <= 32'b0;
end
end
else begin
      if (RegWrite && (Rd != 5'd0)) Registers[Rd] <= Write_data;
      Registers[0] <= 32'b0;
end
end

assign read_data1 = Registers[Rs1];
assign read_data2 = Registers[Rs2];

endmodule


module main_control_unit(
input [6:0] opcode,
output reg RegWrite,
output reg MemRead,
output reg MemWrite,
output reg MemToReg,
output reg ALUSrc,
output reg Branch,
output reg [1:0] ALUOp
);

always@(*) begin
  case(opcode)
    7'b0110011: // R-type
      begin 
        RegWrite = 1; ALUSrc = 0; MemToReg = 0; MemRead = 0; MemWrite = 0; Branch = 0; ALUOp = 2'b10;
      end
    7'b0010011: // I-type
      begin 
        RegWrite = 1; ALUSrc = 1; MemToReg = 0; MemRead = 0; MemWrite = 0; Branch = 0; ALUOp = 2'b10;
      end
    7'b0000011: // Load
      begin 
        RegWrite = 1; ALUSrc = 1; MemToReg = 1; MemRead = 1; MemWrite = 0; Branch = 0; ALUOp = 2'b00;
      end
    7'b0100011: // Store
      begin 
        RegWrite = 0; ALUSrc = 1; MemToReg = 0; MemRead = 0; MemWrite = 1; Branch = 0; ALUOp = 2'b00;
      end
    7'b1100011: // Branch
      begin 
        RegWrite = 0; ALUSrc = 0; MemToReg = 0; MemRead = 0; MemWrite = 0; Branch = 1; ALUOp = 2'b01;
      end
    7'b1101111: // JAL
      begin 
        RegWrite = 1; ALUSrc = 0; MemToReg = 0; MemRead = 0; MemWrite = 0; Branch = 0; ALUOp = 2'b10;
      end
    7'b0110111: // LUI
      begin 
        RegWrite = 1; ALUSrc = 1; MemToReg = 0; MemRead = 0; MemWrite = 0; Branch = 0; ALUOp = 2'b10;
      end
    default:
      begin 
        RegWrite = 0; ALUSrc = 0; MemToReg = 0; MemRead = 0; MemWrite = 0; Branch = 0; ALUOp = 2'b00;
      end
  endcase
end

endmodule


module immediate_generator(
  input [31:0] instruction,
  output reg [31:0] imm_out
);

always@(*)begin
case(instruction[6:0])
  7'b0010011, 
  7'b0000011 : imm_out = {{20{instruction[31]}}, instruction[31:20]};
  
  7'b0100011 : imm_out = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type
  
  7'b0110111,
  7'b0010111 : imm_out = {instruction[31:12], 12'b0}; // LUI, AUIPC

  7'b1100011 : imm_out = {{19{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B-type

  default : imm_out = 32'b0;
endcase
end
endmodule


module ALU(
 input [31:0] A,
 input [31:0] B,
 input [3:0] ALUcontrol_In,
 output reg [31:0] Result,
 output reg Zero 
);

always@(A or B or ALUcontrol_In)begin
case(ALUcontrol_In)
4'b0000 : Result = A + B;
4'b0001 : Result = A - B;
4'b0010 : Result = A & B;
4'b0011 : Result = A | B;
4'b0100 : Result = A ^ B;
4'b0101 : Result = A << B[4:0];
4'b0110 : Result = A >> B[4:0];
4'b0111 : Result = $signed(A) >>> B[4:0];
4'b1000 : Result = ($signed(A) < $signed(B)) ? 32'b1 : 32'b0;
default : Result = 32'b0;
endcase

Zero = (Result == 32'b0) ? 1 : 0;
end
endmodule


module ALU_Control(
input [2:0] funct3,
input [6:0] funct7,
input [1:0] ALUOp,
output reg [3:0] ALUcontrol_Out
);

always@(*)begin
case({ALUOp, funct7, funct3})
12'b10_0000000_000 : ALUcontrol_Out <= 4'b0000;
12'b00_0000000_000 : ALUcontrol_Out <= 4'b0000;
12'b00_0000000_001 : ALUcontrol_Out <= 4'b0000;
12'b00_0000000_010 : ALUcontrol_Out <= 4'b0000;
12'b10_0100000_000 : ALUcontrol_Out <= 4'b0001;
12'b10_0000000_111 : ALUcontrol_Out <= 4'b0010;
12'b10_0000000_110 : ALUcontrol_Out <= 4'b0011;
12'b10_0000000_100 : ALUcontrol_Out <= 4'b0100;
12'b10_0000000_001 : ALUcontrol_Out <= 4'b0101;
12'b10_0000000_101 : ALUcontrol_Out <= 4'b0110;
12'b10_0100000_101 : ALUcontrol_Out <= 4'b0111;
12'b10_0000000_010 : ALUcontrol_Out <= 4'b1000;

default            : ALUcontrol_Out <= 4'b0000;
endcase
end
endmodule


module MUX2to1(
input [31:0] input0,
input [31:0] input1,
input select,
output [31:0] out
);
assign out = (select) ? input1 : input0;
endmodule


module Data_Memory(
input clk,rst,
input MemRead, MemWrite,
input [31:0] address,
input [31:0] write_data,
output [31:0] read_data
);

reg [31:0] D_Memory [0:63];
integer k;
wire [5:0] widx = address[7:2];
assign read_data =(MemRead) ? D_Memory[address] : 32'b0;

always@(posedge clk,posedge rst) begin
if(rst) begin
for (k = 0; k<64; k = k+1)begin
D_Memory[k] <= 32'b0;
end
end else if (MemWrite) begin
  D_Memory[widx] <= write_data;
end
end
endmodule


module MUX2to1_DataMemory(
input [31:0] input0,
input [31:0] input1,
input select,
output [31:0] out
);
assign out = (select) ? input1 : input0;
endmodule


module Branch_Adder(
input [31:0] PC,
input [31:0] offset,
output reg [31:0] branch_target
);
always@(*)begin
branch_target <= PC + (offset);
end
endmodule


module RISCV_Top(
input clk, rst
);
wire [31:0] pc_out_wire, pc_next_wire, read_data_wire;
wire [31:0] pc_wire, decode_wire, muxtoAlu;
wire [31:0] read_data1, regtomux, WB_wire;
wire [31:0] branch_target, immgen_wire;
wire [31:0] WB_data_wire;
wire RegWrite, ALUSrc, MemRead; 
wire Memwrite, MemToReg, Branch, Zero;
wire [1:0] ALUOp_wire;
wire [3:0] ALUcontrol_wire;


// Program Counter
program_counter PC(.clk(clk),.rst(rst),.pc_in(pc_wire),.pc_out(pc_out_wire));

// PC Adder
pc_adder PC_Adder(.pc_in(pc_out_wire),.pc_next(pc_next_wire));

// PC Mux
pc_mux pc_mux(.pc_in(pc_next_wire),.pc_branch(branch_target),.pc_select(Branch&Zero),.pc_out(pc_wire));

// Instruction Memory
Instruction_Memory Instr_Mem(.rst(rst),.clk(clk),.read_address(pc_out_wire), .instruction_out(decode_wire));

// Register File
Register_File Reg_File(.rst(rst), .clk(clk), .RegWrite(RegWrite), .Rs1(decode_wire [19:15]), .Rs2(decode_wire [24:20]), .Rd(decode_wire [11:7]), .Write_data(WB_data_wire), .read_data1(read_data1), .read_data2(regtomux));

// Control Unit
main_control_unit Control_Unit(.opcode(decode_wire [6:0]), .RegWrite(RegWrite), .MemRead(MemRead), .MemWrite(MemWrite), .MemToReg(MemToReg), .ALUSrc(ALUSrc), .Branch(Branch), .ALUOp(ALUOp_wire));

// ALU_Control
ALU_Control ALU_Control(.funct3(decode_wire[14:12]), .funct7(decode_wire[31:25]), .ALUOp(ALUOp_wire), .ALUcontrol_Out(ALUcontrol_wire));

// ALU
ALU ALU(.A(read_data1), .B(muxtoAlu), .ALUcontrol_In(ALUcontrol_wire), .Result(WB_wire), .Zero(Zero));

// Immediate Generator
immediate_generator Imm_Gen(.instruction(decode_wire), .imm_out(immgen_wire));

// ALU Mux
MUX2to1 Imm_Mux(.input0(regtomux), .input1(immgen_wire), .select(ALUSrc),.out(muxtoAlu));

// Data Memory
Data_Memory Data_Mem(.clk(clk),.rst(rst), .MemRead(MemRead),.MemWrite(MemWrite),.address(WB_wire),.write_data(regtomux),.read_data(read_data_wire));

//WB Mux
MUX2to1_DataMemory WB_Mux(.input0(WB_wire), .input1(read_data_wire), .select(MemToReg), .out(WB_data_wire));

//Branch Adder
Branch_Adder Branch_Adder(.PC(pc_out_wire), .offset(immgen_wire), .branch_target(branch_target));

endmodule