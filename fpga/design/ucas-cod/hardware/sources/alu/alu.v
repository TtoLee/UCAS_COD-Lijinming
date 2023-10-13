`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ALUOP_AND 3'b000
`define ALUOP_OR 3'b001
`define ALUOP_XOR 3'b100
`define ALUOP_NOR 3'b101
`define ALUOP_ADD 3'b010
`define ALU_SUB 3'b110
`define ALU_SLT 3'b111
`define ALU_SLTU 3'b011

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);

wire op_and = ALUop == `ALUOP_AND;
wire op_or = ALUop == `ALUOP_OR;
wire op_xor = ALUop == `ALUOP_XOR;
wire op_nor = ALUop == `ALUOP_NOR;
wire op_add = ALUop == `ALUOP_ADD;
wire op_sub = ALUop == `ALU_SUB;
wire op_slt = ALUop == `ALU_SLT;
wire op_sltu = ALUop == `ALU_SLTU;

wire [`DATA_WIDTH - 1:0]oper_num = {`DATA_WIDTH{op_add}} & B | {`DATA_WIDTH{op_sub | op_slt | op_sltu}} & (~B);
wire Cin = op_sub | op_slt | op_sltu;

wire [`DATA_WIDTH - 1:0] and_res = A & B;
wire [`DATA_WIDTH - 1:0] or_res = A | B;
wire [`DATA_WIDTH - 1:0] xor_res = A ^ B;
wire [`DATA_WIDTH - 1:0] nor_res = ~(A | B);
wire [`DATA_WIDTH - 1:0] add_res;
wire [`DATA_WIDTH - 1:0] Result_tmp;
wire CarryOut_init;

assign {CarryOut_init, add_res} = A + oper_num + Cin;

assign Result_tmp = {`DATA_WIDTH{op_and}} & and_res |
				{`DATA_WIDTH{op_or}} & or_res |
				{`DATA_WIDTH{op_xor}} & xor_res |
				{`DATA_WIDTH{op_nor}} & nor_res |
				{`DATA_WIDTH{op_add | op_sub | op_slt}} & add_res;

assign Overflow = A[`DATA_WIDTH - 1] & oper_num[`DATA_WIDTH - 1] & ~Result_tmp[`DATA_WIDTH - 1] |
				~A[`DATA_WIDTH - 1] & ~oper_num[`DATA_WIDTH - 1] & Result_tmp[`DATA_WIDTH - 1];

assign CarryOut = {op_add & CarryOut_init} | {op_sub & ~CarryOut_init};

assign Result = ~{`DATA_WIDTH{op_slt | op_sltu}} & Result_tmp |
				{`DATA_WIDTH{op_slt}} & {{`DATA_WIDTH-1{1'b0}}, (Overflow ^ Result_tmp[`DATA_WIDTH - 1])} |
				{`DATA_WIDTH{op_sltu}} & {{`DATA_WIDTH-1{1'b0}}, (~CarryOut_init)};

assign Zero = Result == 32'b0;

endmodule
