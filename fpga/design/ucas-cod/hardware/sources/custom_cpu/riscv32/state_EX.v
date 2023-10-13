`timescale 10ns / 1ns

module state_EX(
    input 				clk,
    input 				rst,

    input 				complete_pre,		//前一阶段完成信号
    output reg 			complete_this,		//本阶段完成信号

	//寄存器相关信号
    input  		[31:0] 	RF_rdata1,
    input  		[31:0] 	RF_rdata2,
	input  	    [ 4:0] 	RF_waddr_in,
	output reg  [ 4:0]  RF_waddr_out,

    input  		[19:0] 	Inst_Decode,		//指令译码结果
    input  		[31:0] 	imm,				//立即数
    input  		[31:0] 	PC_input,			//输入PC
	output reg 	[31:0] 	PC_output,			//输出PC

    output 				fb_ex_branch,		//需要跳转/分支反馈信号
    input 				fb_mem,				//访存反馈信号

	output reg 	[ 8:0]	mem_info_out,		//包含funct3[2], 确定load/store信号, strb值
	output reg 	[31:0] 	Write_data_out,		//store写入内存数据/写回寄存器数据
	output reg 	[31:0] 	mem_address_out		//store/load地址
);

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

	wire [31:0] ALU_A;
	wire [31:0] ALU_B;
	wire [31:0] ALU_Result;
	wire Zero;
	wire mul;
	wire [31:0] mul_result;
	wire [2:0] ALUop;
	wire [1:0] Shiftop;

	wire [31:0] Shifter_A;
	wire [4:0] Shifter_B;
	wire [31:0] Shifter_Result;

	wire [31:0] Address;
	wire memory_byte;
	wire memory_half;
	wire memory_word;
	wire [3:0] sb_strb;
	wire [3:0] sh_strb;
	wire [3:0] sw_strb;
	wire [31:0] sb_data;
	wire [31:0] sh_data;
	wire [31:0] sw_data;
	wire [31:0] Write_data_mem;
	wire [2:0] funct3;
	wire [3:0] Write_strb;

    assign {R_Type, I_Type, calc, shift, I_Type_load, I_Type_jump, mul,
            S_Type, B_Type, U_Type, J_Type, AUIPC,
            funct3,
            ALUop,
            Shiftop} = Inst_Decode;

    assign ALU_A = ({32{R_Type | I_Type & ~I_Type_jump | S_Type | B_Type}} & RF_rdata1) |
					({32{J_Type | U_Type | I_Type_jump}} & PC_input);
    assign ALU_B = ({32{R_Type | B_Type}} & RF_rdata2) |
					({32{I_Type & ~I_Type_jump | S_Type | U_Type}} & imm) |
                    ({32{J_Type | I_Type_jump}} & 32'd4);
    assign Shifter_A = RF_rdata1;
    assign Shifter_B = ({5{R_Type}} & RF_rdata2[4:0]) |
                        ({5{I_Type}} & imm[4:0]);
    assign mul_result = ALU_A * ALU_B;

 	alu ALU(
		.A(ALU_A),
		.B(ALU_B),
		.ALUop(ALUop),
		.Result(ALU_Result),
		.Overflow(),
		.CarryOut(),
		.Zero(Zero)
	);
    
    shifter Shifter(
		.A(Shifter_A),
		.B(Shifter_B),
		.Shiftop(Shiftop),
		.Result(Shifter_Result)
	);

    assign fb_ex_branch = (complete_pre) & (J_Type | I_Type_jump | (B_Type & (funct3[2] ^ funct3[0] ^ Zero)));

	assign Address = ALU_Result[31:0];
	assign sb_strb = 4'b1000 >> (~ALU_Result[1:0]);
	assign sh_strb = {ALU_Result[1], ALU_Result[1], ~ALU_Result[1], ~ALU_Result[1]};
	assign sw_strb = 4'b1111;
	assign memory_byte = (I_Type_load | S_Type) & (~funct3[1] & ~funct3[0]);
	assign memory_half = (I_Type_load | S_Type) & (~funct3[1] & funct3[0]);
	assign memory_word = (I_Type_load | S_Type) & (funct3[1] & ~funct3[0]);
	assign Write_strb = ({4{S_Type & memory_byte}} & sb_strb) |
						({4{S_Type & memory_half}} & sh_strb) |
						({4{S_Type & memory_word}} & sw_strb);

	assign sb_data = ({32{sb_strb[0]}} & {{24{1'b0}}, RF_rdata2[7:0]}) |
						({32{sb_strb[1]}} & {{16{1'b0}}, RF_rdata2[7:0], {8{1'b0}}}) |
						({32{sb_strb[2]}} & {{8{1'b0}}, RF_rdata2[7:0], {16{1'b0}}}) |
						({32{sb_strb[3]}} & {RF_rdata2[7:0], {24{1'b0}}});
	assign sh_data = ({32{sh_strb[0]}} & {{16{1'b0}}, RF_rdata2[15:0]}) |
						({32{sh_strb[2]}} & {RF_rdata2[15:0], {16{1'b0}}});
	assign sw_data = RF_rdata2;
	assign Write_data_mem = ({32{S_Type & memory_byte}} & sb_data) |
							({32{S_Type & memory_half}} & sh_data) |
							({32{S_Type & memory_word}} & sw_data);

	always @ (posedge clk) begin
		if (rst) mem_info_out <= 32'b0;
		else if (complete_pre && ~fb_mem)
			mem_info_out <= {funct3, S_Type, I_Type_load, Write_strb};
	end

	always @ (posedge clk) begin
		if (complete_pre) begin
			if (S_Type) Write_data_out <= Write_data_mem;
			else if (~fb_mem)begin
				if ((R_Type | I_Type) & shift) Write_data_out <= Shifter_Result;
				else if (U_Type) Write_data_out <= (AUIPC? ALU_Result : imm);
				else if (mul) Write_data_out <= mul_result;
				else Write_data_out <= ALU_Result;
			end
		end
	end

	always @ (posedge clk) begin
		if (complete_pre) mem_address_out <= Address;
	end

	always @(posedge clk ) begin
		if (complete_pre & ~fb_mem) RF_waddr_out <= RF_waddr_in;
	end

	always @ (posedge clk) begin
		if (complete_pre && ~fb_mem) PC_output = PC_input;
	end

    always @ (posedge clk) begin
        if (rst) complete_this <= 0;
        else if (!fb_mem) complete_this <= complete_pre; 
    end
endmodule