`timescale 10ns / 1ns

`define R_Type_opcode 7'b0110011
`define I_Type_opcode 7'b0010011
`define S_Type_opcode 7'b0100011
`define I_Type_load_opcode 7'b0000011
`define I_Type_jump_opcode 7'b1100111
`define B_Type_opcode 7'b1100011
`define U_Type_opcode 5'b10111
`define J_Type_opcode 7'b1101111

`define ALUOP_AND 3'b000
`define ALUOP_OR 3'b001
`define ALUOP_XOR 3'b100
`define ALUOP_NOR 3'b101
`define ALUOP_ADD 3'b010
`define ALU_SUB 3'b110
`define ALU_SLT 3'b111
`define ALU_SLTU 3'b011

module state_ID(
    input         		clk,
    input       		rst,

	input 				complete_pre,		//前序阶段完成信号
	output reg 			complete_this,		//本阶段完成信号

	input      [31:0] 	PC_input,			//前阶段传入PC值
	output reg [31:0] 	PC_output,			//向后阶段传出PC值

	output reg [31:0]   branch_PC_reg,		//跳转/分支PC目标地址(若有)
    input  	   [31:0]	Instruction_reg,	//传入待分析指令

	//寄存器相关信号
    input      [31:0] 	RF_rdata1,
    input 	   [31:0] 	RF_rdata2,
    output     [ 4:0] 	RF_raddr1,
    output     [ 4:0] 	RF_raddr2,
	output reg [ 4:0] 	RF_waddr,			//传出寄存器写地址, 待写回阶段使用

	//向下一阶段传出寄存器读到的数据
	output reg [31:0] 	RF_rdata1_out,
	output reg [31:0] 	RF_rdata2_out,

    output reg [19:0] 	Inst_Decode,		//指令分析结果汇总(见138行)
    output reg [31:0] 	imm_r,				//立即数

	input 			  	fb_ex_branch,		//需要跳转/分支反馈信号
    input               fb_mem,             //访存反馈信号

	input 	   [31:0] 	wb_from_ex,			//旁路输入(非load指令从ex阶段获取)
	input 	   [31:0] 	wb_from_load,		//旁路输入(load指令从mem阶段获取)

	output	   [31:0]	cpu_perf_cnt_1,
	output	   [31:0]	cpu_perf_cnt_4
);
	wire [31:0] branch_PC;
	wire R_Type;
	wire I_Type;
	wire S_Type;
	wire I_Type_load;
	wire I_Type_jump;
	wire B_Type;
	wire U_Type;
	wire J_Type;
    wire shift;
    wire calc;
    wire AUIPC;

    wire [7:0] funct7;
	wire [4:0] rs2;
	wire [4:0] rs1;
	wire [2:0] funct3;
	wire [4:0] rd;
	wire [6:0] opcode;
	wire mul;
	wire [31:0] imm;

	wire RFaddr_read1_equal_write;
	wire RFaddr_read2_equal_write;

    assign opcode = Instruction_reg[6:0];
	assign R_Type = opcode == `R_Type_opcode;
	assign I_Type = (opcode == `I_Type_opcode | opcode == `I_Type_load_opcode | opcode == `I_Type_jump_opcode);
	assign I_Type_load = opcode == `I_Type_load_opcode;
	assign I_Type_jump = opcode == `I_Type_jump_opcode;
	assign S_Type = opcode == `S_Type_opcode;
	assign B_Type = opcode == `B_Type_opcode;
	assign U_Type = opcode[4:0] == `U_Type_opcode;
	assign J_Type = opcode == `J_Type_opcode;
	assign mul = R_Type & funct7[0] & ~funct3[2] & ~funct3[1] & ~funct3[0];

	assign funct7 = Instruction_reg[31:25];
	assign rs2 = Instruction_reg[24:20];
	assign rs1 = Instruction_reg[19:15];
	assign rd = Instruction_reg[11:7] & ({5{R_Type | I_Type | U_Type | J_Type}});
	assign funct3 = Instruction_reg[14:12];
	wire [2:0] ALUop;
	wire [1:0] Shiftop;

    assign imm = {((U_Type)? Instruction_reg[31:20] : {12{Instruction_reg[31]}}), //31-20位
                    ((U_Type | J_Type)? Instruction_reg[19:12] : {8{Instruction_reg[31]}}),    //19-12位
                    (((I_Type | S_Type) & Instruction_reg[31]) | ((B_Type) & Instruction_reg[7]) | ((J_Type) & Instruction_reg[20])),   //11位
                    ({6{~U_Type}} & Instruction_reg[30:25]),    //10-5位
                    ({4{I_Type | J_Type}} & Instruction_reg[24:21] | {4{S_Type | B_Type}} & Instruction_reg[11:8]), //4-1位
                    (I_Type & Instruction_reg[20] | S_Type & Instruction_reg[7])};  //0位
	always @ (posedge clk) begin
		if (complete_pre & ~fb_ex_branch & ~fb_mem) imm_r <= imm;
	end
    
    assign ALUop = ((R_Type || I_Type) && calc)? ((~funct3[2] & ~funct3[1])? {funct7[5] & R_Type, 2'b10} :
												((~funct3[2] & funct3[1])? {~funct3[0], 2'b11} :
												{~funct3[1], 1'b0, funct3[1] & ~funct3[0]})) :
					((B_Type && ~funct3[2])? `ALU_SUB :
					((B_Type && ~funct3[1])? `ALU_SLT :
					((B_Type)? `ALU_SLTU :
					`ALUOP_ADD)));
	assign Shiftop = ({2{(R_Type || I_Type) && shift}}) & {funct3[2], funct7[5]};

    assign shift = ~I_Type_load & ~I_Type_jump & ~funct3[1] & funct3[0];
	assign calc = ~I_Type_load & ~I_Type_jump & (funct3[1] | ~funct3[0]);
    assign AUIPC = U_Type & (~opcode[5]);

	assign branch_PC = (I_Type_jump ? (RFaddr_read1_equal_write? (Inst_Decode[15]? wb_from_load : wb_from_ex) 
																: RF_rdata1): PC_input) + imm;
	
	always @ (posedge clk) begin
		if (complete_pre & ~fb_ex_branch & ~fb_mem) begin
			if (B_Type | J_Type | I_Type_jump)
				branch_PC_reg = branch_PC;
		end
	end

	always @ (posedge clk) begin
		if (complete_pre & ~fb_ex_branch & ~fb_mem)
			PC_output <= PC_input;
	end

    always @ (posedge clk) begin
        if (complete_pre & ~fb_ex_branch & ~fb_mem)
            Inst_Decode <= {
                R_Type, I_Type, calc, shift, I_Type_load, I_Type_jump, mul,	//19-13
                S_Type, B_Type, U_Type, J_Type, AUIPC,	//12-8
                funct3,	//7-5
                ALUop,	//4-2
                Shiftop	//1-0
            };              
    end

	always @ (posedge clk) begin
		if (rst) RF_waddr <= 5'b0;
		else if (complete_pre & ~fb_mem) RF_waddr <= rd;
	end

	assign RF_raddr1 = rs1;
	assign RF_raddr2 = rs2;

	assign RFaddr_read1_equal_write = (|RF_waddr & (RF_raddr1 == RF_waddr));
	assign RFaddr_read2_equal_write = (|RF_waddr & (RF_raddr2 == RF_waddr));

	always @ (posedge clk) begin
		if (RFaddr_read1_equal_write) begin
			if (Inst_Decode[15]) RF_rdata1_out <= wb_from_load;
			else RF_rdata1_out <= wb_from_ex;
		end
		else RF_rdata1_out <= RF_rdata1;
	end

	always @ (posedge clk) begin
		if (RFaddr_read2_equal_write) begin
			if (Inst_Decode[15]) RF_rdata2_out <= wb_from_load;
			else RF_rdata2_out <= wb_from_ex;
		end
		else RF_rdata2_out <= RF_rdata2;
	end

	always @ (posedge clk) begin
		if (rst) complete_this <= 0;
		else if (~fb_mem) begin
			if (complete_pre & ~fb_ex_branch)
				complete_this <= 1;
			else complete_this <= 0;
		end
	end

	reg [31:0] memory_cnt;
	reg [31:0] load_cnt;

	always @(posedge clk) begin
			if (rst) memory_cnt <= 32'b0;
			else if (complete_pre & ~fb_ex_branch & (S_Type | I_Type_load)) memory_cnt = memory_cnt +32'b1;
		end
		assign cpu_perf_cnt_1 = memory_cnt;
	
	always @(posedge clk) begin
		if (rst) load_cnt <= 32'b0;
		else if (complete_pre & ~fb_ex_branch & I_Type_load) load_cnt <= load_cnt + 32'b1;
	end
	assign cpu_perf_cnt_4 = load_cnt;
endmodule