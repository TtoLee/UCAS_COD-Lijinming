`timescale 10ns / 1ns

`define R_Type_opcode 6'b000000
`define REGIMM_Type_opcode 6'b000001
`define J_Type_opcode 5'b00001
`define I_Type_calc_opcode 3'b001
`define I_Type_branch_opcode 4'b0001
`define I_mem_b_opcode 3'b000
`define I_mem_h_opcode 3'b001
`define I_mem_w_opcode 3'b011
`define I_memr_lbu_opcode 3'b100
`define I_memr_lhu_opcode 3'b101
`define I_mem_wl_opcode 3'b010
`define I_mem_wr_opcode 3'b110
`define ALUOP_AND 3'b000
`define ALUOP_OR 3'b001
`define ALUOP_XOR 3'b100
`define ALUOP_NOR 3'b101
`define ALUOP_ADD 3'b010
`define ALU_SUB 3'b110
`define ALU_SLT 3'b111
`define ALU_SLTU 3'b011
`define S_IF 5'b00001
`define S_ID 5'b00010
`define S_EX 5'b00100
`define S_MEM 5'b01000
`define S_WB 5'b10000

module simple_cpu(
	input             clk,
	input             rst,

	output reg [31:0]     PC,
	input  [31:0]     Instruction,

	output [31:0]     Address,
	output            MemWrite,
	output [31:0]     Write_data,
	output [ 3:0]     Write_strb,

	input  [31:0]     Read_data,
	output            MemRead
);

	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;

	wire R_Type;
	wire R_Type_calc;
	wire R_Type_shift;
	wire R_Type_jump;
	wire R_Type_mov;
	wire REGIMM;
	wire J_Type;
	wire I_Type_branch;
	wire I_Type_calc;
	wire I_Type_memr;
	wire I_Type_memw;
	wire I_memr_lb;
	wire I_memr_lh;
	wire I_memr_lw;
	wire I_memr_lbu;
	wire I_memr_lhu;
	wire I_memr_lwl;
	wire I_memr_lwr;
	wire I_memw_sb;
	wire I_memw_sh;
	wire I_memw_sw;
	wire I_memw_swl;
	wire I_memw_swr;

	reg [4:0] current_state;
	reg [4:0] next_state;
	reg [31:0] Instruction_reg;
	wire state_IF;
	wire state_ID;
	wire state_EX;
	wire state_MEM;
	wire state_WB;

	wire [5:0] opcode;
	wire [4:0] rs;
	wire [4:0] rt;
	wire [4:0] rd;
	wire [5:0] funct;
	wire [4:0] shamt;
	wire [15:0] imm;
	wire [25:0] instr_index;

	wire [4:0] RF_raddr1;
	wire [4:0] RF_raddr2;
	wire [31:0] RF_rdata1;
	wire [31:0] RF_rdata2;

	wire [31:0] ALU_A;
	wire [31:0] ALU_B;
	reg [31:0] ALU_A_reg;
	reg [31:0] ALU_B_reg; 
	wire [31:0] ALU_Result;
	reg [31:0] ALUOut; 
	wire Zero;
	reg Zero_reg;

	wire [31:0] Shifter_A;
	wire [4:0] Shifter_B;
	wire [31:0] Shifter_Result;
	reg [31:0] Shifter_A_reg;
	reg [4:0] Shifter_B_reg;

	wire [2:0] ALUop;
	wire [1:0] Shiftop;
	wire RegWriteSign;
	wire Jump;
	wire [31:0] Jump_Address;
	wire branch_condition;
	wire [31:0] sign_left_extend;
	wire [31:0] signed_extend;
	wire [31:0] unsigned_extend;
	wire [31:0] extend;
	wire ALUsrc;
	wire [1:0] byte_position;
	wire [7:0] lb_data;
	wire [15:0] lh_data;
	wire [31:0] lwl_data;
	wire [31:0] lwr_data;
	wire [31:0] sb_data;
	wire [31:0] sh_data;
	wire [31:0] sw_data;
	wire [31:0] swl_data;
	wire [31:0] swr_data;
	wire [3:0] sb_strb;
	wire [3:0] sh_strb;
	wire [3:0] sw_strb;
	wire [3:0] swl_strb;
	wire [3:0] swr_strb;
	reg [31:0] Read_data_reg;

	assign state_IF = current_state == `S_IF;
	assign state_ID = current_state == `S_ID;
	assign state_EX = current_state == `S_EX;
	assign state_MEM = current_state == `S_MEM;
	assign state_WB = current_state == `S_WB;

	assign opcode = Instruction_reg[31:26];
	assign R_Type = opcode == `R_Type_opcode;
	assign R_Type_calc = (R_Type && funct[5] == 1'b1);
	assign R_Type_shift = (R_Type && funct[5:3] == 3'b000);
	assign R_Type_jump = (R_Type && {funct[5:3], funct[1]} == 4'b0010);
	assign R_Type_mov = (R_Type && {funct[5:3], funct[1]} == 4'b0011);
	assign REGIMM = (opcode[5:0] == `REGIMM_Type_opcode);
	assign J_Type = opcode[5:1] == `J_Type_opcode;
	assign I_Type_branch = opcode[5:2] == `I_Type_branch_opcode;
	assign I_Type_calc = opcode[5:3] == `I_Type_calc_opcode;
	assign I_Type_memr = opcode[5] & (~opcode[3]);
	assign I_Type_memw = opcode[5] & opcode[3];
	assign I_memr_lb = (I_Type_memr && (opcode[2:0] == `I_mem_b_opcode));
	assign I_memr_lh = (I_Type_memr && (opcode[2:0] == `I_mem_h_opcode));
	assign I_memr_lw = (I_Type_memr && (opcode[2:0] == `I_mem_w_opcode));
	assign I_memr_lbu = (I_Type_memr && (opcode[2:0] == `I_memr_lbu_opcode));
	assign I_memr_lhu = (I_Type_memr && (opcode[2:0] == `I_memr_lhu_opcode));
	assign I_memr_lwl = (I_Type_memr && (opcode[2:0] == `I_mem_wl_opcode));
	assign I_memr_lwr = (I_Type_memr && (opcode[2:0] == `I_mem_wr_opcode));
	assign I_memw_sb = (I_Type_memw && (opcode[2:0] == `I_mem_b_opcode));
	assign I_memw_sh = (I_Type_memw && (opcode[2:0] == `I_mem_h_opcode));
	assign I_memw_sw = (I_Type_memw && (opcode[2:0] == `I_mem_w_opcode));
	assign I_memw_swl = (I_Type_memw && (opcode[2:0] == `I_mem_wl_opcode));
	assign I_memw_swr = (I_Type_memw && (opcode[2:0] == `I_mem_wr_opcode));

	always @ (posedge clk) begin
			if (rst) begin
				current_state <= `S_IF;
			end
			else begin
				current_state <= next_state;
			end
	end

	always @ (*) begin
		case (current_state)
			`S_IF: next_state = `S_ID;
			`S_ID: begin
				if (Instruction_reg == 32'b0) next_state = `S_IF;
				else next_state = `S_EX;
			end
			`S_EX: begin
				if (I_Type_memw || I_Type_memr) next_state = `S_MEM;
				else if (R_Type || I_Type_calc || (J_Type && opcode[0])) next_state = `S_WB;
				else next_state = `S_IF;
			end
			`S_MEM: begin
				if (I_Type_memr) next_state = `S_WB;
				else next_state = `S_IF;
			end
			`S_WB: begin
				next_state = `S_IF;
			end
			default: next_state = `S_IF;
		endcase
	end

	always @ (posedge clk) begin
		if (state_IF) Instruction_reg <= Instruction;
	end

	assign rs = Instruction_reg[26:21];
	assign rt = Instruction_reg[20:16];
	assign rd = Instruction_reg[15:11];
	assign funct = Instruction_reg[5:0];
	assign shamt = Instruction_reg[10:6];
	assign instr_index = Instruction_reg[25:0];
	assign imm = Instruction_reg[15:0];
	assign RF_raddr1 = rs;
	assign RF_raddr2 = (REGIMM)? 32'b0:rt;
	assign RF_wen = (state_WB) & ~((R_Type_jump && ~funct[0]) || REGIMM || (J_Type && ~opcode[0])|| I_Type_memw || I_Type_branch || R_Type_mov && ((~funct[0] && ~Zero_reg) || (funct[0] && Zero_reg)));
	assign RegWriteSign = (I_Type_calc || I_Type_memr)? 1:0;
	assign RF_waddr = (J_Type && opcode[0])? 31:((RegWriteSign)? rt:rd);
	assign RF_wdata = ({32{R_Type_mov & ((~funct[0] & Zero_reg) | (funct[0] & ~Zero_reg))}} & (RF_rdata1)) |
						({32{I_Type_calc & (opcode[2:0] == 3'b111)}}) & {imm[15:0], {16{1'b0}}} |
                        {32{R_Type_calc | R_Type_shift | (I_Type_calc & (opcode[2:0] != 3'b111)) | (J_Type & opcode[0]) | (R_Type_jump & funct[0])}} & ALUOut |
						({32{I_memr_lb}} & {{24{lb_data[7]}}, lb_data}) |
						({32{I_memr_lh}} & {{16{lh_data[15]}}, lh_data}) |
						({32{I_memr_lw}} & Read_data_reg) |
						({32{I_memr_lbu}} & {{24{1'b0}}, lb_data}) |
						({32{I_memr_lhu}} & {{16{1'b0}}, lh_data}) |
						({32{I_memr_lwl}} & lwl_data) |
						({32{I_memr_lwr}} & lwr_data);

	assign byte_position = ALUOut[1:0];
	assign lb_data = (byte_position[1] && byte_position[0])? Read_data_reg[31:24]:
						((byte_position[1] && ~byte_position[0])? Read_data_reg[23:16]:
						(~byte_position[1] && byte_position[0]? Read_data_reg[15:8]:
						Read_data_reg[7:0]));
	assign lh_data = (~byte_position[1] && ~byte_position[0])? Read_data_reg[15:0]:Read_data_reg[31:16];
	assign lwl_data = (byte_position[1] && byte_position[0])? Read_data_reg[31:0]:
						((byte_position[1] && ~byte_position[0])? {Read_data_reg[23:0], RF_rdata2[7:0]}:
						(~byte_position[1] && byte_position[0]? {Read_data_reg[15:0], RF_rdata2[15:0]}:
						{Read_data_reg[7:0], RF_rdata2[23:0]}));
	assign lwr_data = (byte_position[1] && byte_position[0])? {RF_rdata2[31:8], Read_data_reg[31:24]}:
						((byte_position[1] && ~byte_position[0])? {RF_rdata2[31:16], Read_data_reg[31:16]}:
						(~byte_position[1] && byte_position[0]? {RF_rdata2[31:24], Read_data_reg[31:8]}:
						Read_data_reg[31:0]));

	assign MemRead = (state_MEM & opcode[5] & ~opcode[3]);
	assign MemWrite = (state_MEM & opcode[5] & opcode[3]);


	reg_file RegFile(
		.clk(clk),
		.waddr(RF_waddr),
		.raddr1(RF_raddr1),
		.raddr2(RF_raddr2),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(RF_rdata1),
		.rdata2(RF_rdata2)
	);

	assign ALUsrc = (I_Type_calc || I_Type_memr || I_Type_memw)? 1:0;
	assign extend = (I_Type_calc && opcode[2])? unsigned_extend :
					(((I_Type_calc && (opcode[2:1] == 2'b00 || opcode[2:1] == 2'b01)) || I_Type_memr || I_Type_memw)? signed_extend :
					sign_left_extend);

	assign signed_extend = {{16{imm[15]}}, imm[15:0]};
	assign unsigned_extend = {{16{1'b0}}, imm[15:0]};
	assign sign_left_extend = {{14{imm[15]}}, imm[15:0], 2'b00};

	assign ALU_A = ({32{state_IF | state_ID}} & PC) |
					({32{state_EX}} & ALU_A_reg);
	assign ALU_B = ({32{state_IF}} & {{29{1'b0}}, 3'b100}) |
					({32{state_ID}} & {sign_left_extend}) |
					({32{state_EX}} & ALU_B_reg);
	assign ALUop = (state_IF | state_ID)? `ALUOP_ADD : 
					((state_EX)? ((R_Type_calc)? ((funct[3:2] == 2'b00)? {funct[1], 2'b10} :
									((funct[3:2] == 2'b01)? {funct[1], 1'b0, funct[0]} :
									{~funct[0], 2'b11})) :
								((I_Type_calc && opcode[2:0] != 3'b111)? ((opcode[2:1] == 2'b00)? {opcode[1], 2'b10} :
																		((opcode[2])? {opcode[1], 1'b0, opcode[0]} :
																		{~opcode[0], 2'b11})) :
								((R_Type_mov || (I_Type_branch && opcode[1] == 1'b0))? `ALU_SUB :
								((REGIMM || (I_Type_branch && opcode[1] == 1'b1))? `ALU_SLT :
								`ALUOP_ADD)))) : `ALUOP_ADD);

	always @(posedge clk) begin
		if (state_ID) begin
			ALUOut <= ALU_Result;
			ALU_A_reg <= (({32{R_Type_mov | (I_Type_branch & ~opcode[1])}} & RF_rdata2) |
						({32{R_Type_jump | J_Type}} & PC) |
						({32{I_Type_branch & opcode[1]}} & {32{1'b0}}) |
						({32{R_Type_calc | REGIMM | I_Type_calc | I_Type_memw | I_Type_memr}} & RF_rdata1));
			ALU_B_reg <= (({32{R_Type_calc}} & RF_rdata2) |
						({32{I_Type_branch & opcode[1] | (I_Type_branch & ~opcode[1])}} & RF_rdata1) |
						({32{REGIMM | R_Type_mov}}) & {32{1'b0}} |
						({32{ALUsrc}} & extend) |
						({32{R_Type_jump | J_Type}} & {{29{1'b0}}, 3'b100})); 
			Shifter_A_reg <= RF_rdata2;
			Shifter_B_reg <= {5{funct[2]}} & RF_rdata1 |
								{5{~funct[2]}} & shamt;
		end
		if (state_EX) begin
			if (R_Type_shift) ALUOut <= Shifter_Result;
			else ALUOut <= ALU_Result;
		end
	end

	alu ALU(
		.A(ALU_A),
		.B(ALU_B),
		.ALUop(ALUop),
		.Result(ALU_Result),
		.Overflow(),
		.CarryOut(),
		.Zero(Zero)
	);

	always @(posedge clk ) begin
		Zero_reg <= Zero;
	end

	assign Shifter_A = {32{state_EX}} & Shifter_A_reg;
	assign Shifter_B = {32{state_EX}} & Shifter_B_reg;
	assign Shiftop = ({2{R_Type_shift && state_EX}}) & funct[1:0] ;

	shifter Shifter(
		.A(Shifter_A),
		.B(Shifter_B),
		.Shiftop(Shiftop),
		.Result(Shifter_Result)
	);

	always @(posedge clk) begin
		if (rst) PC <= 32'b0;
		else if (state_IF) PC <= ALU_Result;
		else if (state_EX) begin
			if (R_Type_jump) PC <= RF_rdata1;
			else if (J_Type) PC <= {PC[31:28], instr_index[25:0], 2'b00};
			else if (I_Type_branch && (~opcode[1] && (opcode[0] ^ Zero) ||
									(opcode[1] && (opcode[0] ^ (ALU_Result == 32'b0)))) ||(REGIMM && (rt[0] ^ (ALU_Result == 32'b1))))
					PC <= ALUOut;
		end
	end

	always @(posedge clk) begin
		if (state_MEM && I_Type_memr) Read_data_reg <= Read_data;
	end

	assign Address = {ALUOut[31:2], 2'b00};
	assign sb_strb = 4'b1000 >> (~ALUOut[1:0]);
	assign sh_strb = {ALUOut[1], ALUOut[1], ~ALUOut[1], ~ALUOut[1]};
	assign sw_strb = 4'b1111;
	assign swl_strb = {ALUOut[1] & ALUOut[0], ALUOut[1], ALUOut[1] | ALUOut[0], 1'b1};
	assign swr_strb = {1'b1, ~ALUOut[1] | ~ALUOut[0], ~ALUOut[1], ~ALUOut[1] & ~ALUOut[0]};
	assign Write_strb = ({4{I_memw_sb}} & sb_strb) |
						({4{I_memw_sh}} & sh_strb) |
						({4{I_memw_sw}} & sw_strb) |
						({4{I_memw_swl}} & swl_strb) |
						({4{I_memw_swr}} & swr_strb) ;

	assign sb_data = ({32{sb_strb[0]}} & {{24{1'b0}}, RF_rdata2[7:0]}) |
						({32{sb_strb[1]}} & {{16{1'b0}}, RF_rdata2[7:0], {8{1'b0}}}) |
						({32{sb_strb[2]}} & {{8{1'b0}}, RF_rdata2[7:0], {16{1'b0}}}) |
						({32{sb_strb[3]}} & {RF_rdata2[7:0], {24{1'b0}}});
	assign sh_data = ({32{sh_strb[0]}} & {{16{1'b0}}, RF_rdata2[15:0]}) |
						({32{sh_strb[2]}} & {RF_rdata2[15:0], {16{1'b0}}});
	assign sw_data = RF_rdata2;
	assign swl_data = ({32{sb_strb[3]}} & RF_rdata2) |
						({32{sb_strb[2]}} & {{8{1'b0}}, RF_rdata2[31:8]}) |
						({32{sb_strb[1]}} & {{16{1'b0}}, RF_rdata2[31:16]}) |
						({32{sb_strb[0]}} & {{24{1'b0}}, RF_rdata2[31:24]});
	assign swr_data = ({32{sb_strb[0]}} & RF_rdata2) |
						({32{sb_strb[1]}} & {RF_rdata2[23:0], {8{1'b0}}}) |
						({32{sb_strb[2]}} & {RF_rdata2[15:0], {16{1'b0}}}) |
						({32{sb_strb[3]}} & {RF_rdata2[7:0], {24{1'b0}}});

	assign Write_data = ({32{I_memw_sb}} & sb_data) |
						({32{I_memw_sh}} & sh_data) |
						({32{I_memw_sw}} & sw_data) |
						({32{I_memw_swl}} & swl_data) | 
						({32{I_memw_swr}} & swr_data);

endmodule