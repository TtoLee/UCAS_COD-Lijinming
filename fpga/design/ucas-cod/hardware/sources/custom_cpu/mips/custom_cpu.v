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
`define ERET_opcode 6'b010000

`define ALUOP_AND 3'b000
`define ALUOP_OR 3'b001
`define ALUOP_XOR 3'b100
`define ALUOP_NOR 3'b101
`define ALUOP_ADD 3'b010
`define ALU_SUB 3'b110
`define ALU_SLT 3'b111
`define ALU_SLTU 3'b011

`define S_IF 10'b0000000001
`define S_ID 10'b0000000010
`define S_EX 10'b0000000100
`define S_ST 10'b0000001000
`define S_WB 10'b0000010000
`define S_LD 10'b0000100000
`define S_RDW 10'b0001000000
`define S_IW 10'b0010000000
`define S_INIT 10'b0100000000
`define S_INTR 10'b1000000000 

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output reg [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);
  wire [69:0] inst_retire;

// TODO: Please add your custom CPU code here
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
	wire ERET;
	reg shield;

	reg [9:0] current_state;
	reg [9:0] next_state;
	reg [31:0] Instruction_reg;
	wire state_IF;
	wire state_INTR;
	wire state_ID;
	wire state_EX;
	wire state_ST;
	wire state_WB;
	wire state_LD;
	wire state_RDW;
	wire state_IW;
	wire state_INIT;

	wire [5:0] opcode;
	wire [4:0] rs;
	wire [4:0] rt;
	wire [4:0] rd;
	wire [5:0] funct;
	wire [4:0] shamt;
	wire [15:0] imm;
	wire [25:0] instr_index;

	reg [31:0] EPC;
	reg [31:0] PC_4;

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

	reg [31:0] cycle_cnt;
	reg [31:0] memory_cnt;
	reg [31:0] instruction_cnt;
	reg [31:0] wait_cnt;
	reg [31:0] load_cnt;
	reg [31:0] instruction_fetch_cnt;
	reg [31:0] instruction_wait_cnt;
	reg [31:0] mem_wait_cnt;
	reg [31:0] rdw_cnt;
	reg [31:0] cycle_high_cnt;
	reg [31:0] intr_cnt;
	reg [31:0] intr_signal;

	always @(posedge clk) begin
		if (rst) cycle_cnt <= 32'b0;
		else cycle_cnt <= cycle_cnt + 32'b1;
	end
	assign cpu_perf_cnt_0 = cycle_cnt;

	always @(posedge clk) begin
		if (rst) memory_cnt <= 32'b0;
		else if (state_LD || state_ST) memory_cnt <= memory_cnt +32'b1;
	end
	assign cpu_perf_cnt_1 = memory_cnt;

	always @(posedge clk) begin
		if (rst) instruction_cnt <= 32'b0;
		else if (state_IF) instruction_cnt <= instruction_cnt +32'b1;
	end
	assign cpu_perf_cnt_2 = instruction_cnt;

	always @(posedge clk) begin
		if (rst) wait_cnt <= 32'b0;
		else if (state_IF && ~Inst_Req_Ready || state_IW && ~Inst_Valid || (state_LD || state_ST) && ~Mem_Req_Ready || state_RDW && ~Read_data_Valid) wait_cnt <= wait_cnt +32'b1;
	end
	assign cpu_perf_cnt_3 = wait_cnt;

	always @(posedge clk) begin
		if (rst) load_cnt <= 32'b0;
		else if (state_EX && I_Type_memr) load_cnt <= load_cnt + 32'b1;
	end
	assign cpu_perf_cnt_4 = load_cnt;

	always @(posedge clk) begin
		if (rst) instruction_fetch_cnt <= 32'b0;
		else if (state_IF && ~Inst_Req_Ready) instruction_fetch_cnt <= instruction_fetch_cnt +32'b1;
	end
	assign cpu_perf_cnt_5 = instruction_fetch_cnt;

	always @(posedge clk) begin
		if (rst) instruction_wait_cnt <= 32'b0;
		else if (state_IW && ~Inst_Valid) instruction_wait_cnt <= instruction_wait_cnt +32'b1;
	end
	assign cpu_perf_cnt_6 = instruction_wait_cnt;

	always @(posedge clk) begin
		if (rst) mem_wait_cnt <= 32'b0;
		else if ((state_LD || state_ST) && ~Mem_Req_Ready) mem_wait_cnt <= mem_wait_cnt +32'b1;
	end
	assign cpu_perf_cnt_7 = mem_wait_cnt;

	always @(posedge clk) begin
		if (rst) rdw_cnt <= 32'b0;
		else if (state_RDW && ~Read_data_Valid) rdw_cnt <= rdw_cnt +32'b1;
	end
	assign cpu_perf_cnt_8 = rdw_cnt;

	always @(posedge clk) begin
		if (rst) cycle_high_cnt <= ~(32'b0);
		else if (cycle_cnt == 32'b0) cycle_high_cnt <= cycle_high_cnt + 32'b1;
	end
	assign cpu_perf_cnt_9 = cycle_high_cnt;

	always @(posedge clk) begin
		if (rst) intr_cnt <= 32'b0;
		else if (state_INTR) intr_cnt <= intr_cnt +32'b1;
	end
	assign cpu_perf_cnt_10 = intr_cnt;

	always @(posedge clk) begin
		if (rst) intr_signal <= 32'b0;
		else if (intr) intr_signal <= 32'b1;
	end
	assign cpu_perf_cnt_11 = intr_signal;

	assign state_IF = current_state == `S_IF;
	assign state_INTR = current_state == `S_INTR;
	assign state_ID = current_state == `S_ID;
	assign state_EX = current_state == `S_EX;
	assign state_ST = current_state == `S_ST;
	assign state_WB = current_state == `S_WB;
	assign state_LD = current_state == `S_LD;
	assign state_RDW = current_state == `S_RDW;
	assign state_IW = current_state == `S_IW;
	assign state_INIT = current_state == `S_INIT;

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
	assign ERET = opcode == `ERET_opcode;

	always @ (posedge clk) begin
			if (rst) begin
				current_state <= `S_INIT;
			end
			else begin
				current_state <= next_state;
			end
	end

	always @ (*) begin
		case (current_state)
			`S_INIT: next_state = `S_IF;
			`S_IF: begin
				if (intr && ~shield) next_state = `S_INTR;
				else if (Inst_Req_Ready) next_state = `S_IW;
				else next_state = `S_IF;
			end
			`S_INTR: begin
				next_state = `S_IF;
			end
			`S_IW: begin
				if (Inst_Valid) next_state = `S_ID;
				else next_state = `S_IW;
			end	
			`S_ID: begin
				if (Instruction_reg == 32'b0) next_state = `S_IF;
				else next_state = `S_EX;
			end
			`S_EX: begin
				if (I_Type_memw) next_state = `S_ST;
				else if (I_Type_memr) next_state = `S_LD;
				else if (R_Type || I_Type_calc || (J_Type && opcode[0])) next_state = `S_WB;
				else next_state = `S_IF;
			end
			`S_ST: begin
				if (Mem_Req_Ready) next_state = `S_IF;
				else next_state = `S_ST;
			end
			`S_WB: begin
				next_state = `S_IF;
			end
			`S_LD: begin
				if (Mem_Req_Ready) next_state = `S_RDW;
				else next_state = `S_LD;
			end
			`S_RDW: begin
				if (Read_data_Valid) next_state = `S_WB;
				else next_state = `S_RDW;
			end
			default: next_state = `S_INIT;
		endcase
	end

	always @ (posedge clk) begin
		if (rst | state_EX & ERET) shield <= 1'b0;
		else if (state_INTR) begin
			shield <= 1'b1;
			EPC <= PC;
		end
	end

	always @ (posedge clk) begin
		if (state_IW && Inst_Valid) Instruction_reg <= Instruction;
	end

	assign rs = Instruction_reg[25:21];
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

	assign MemRead = state_LD;
	assign Read_data_Ready = state_RDW | state_INIT;
	assign MemWrite = state_ST;
	assign Inst_Req_Valid = state_IF & ~(intr & ~shield);
	assign Inst_Ready = state_IW | state_INIT;

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

	assign ALU_A = ({32{state_IF}} & PC) |
					({32{state_ID}} & PC_4) |
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
						({32{R_Type_jump | J_Type}} & PC_4) |
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
	assign Shiftop = ({2{R_Type_shift && state_EX}}) & funct[1:0];

	shifter Shifter(
		.A(Shifter_A),
		.B(Shifter_B),
		.Shiftop(Shiftop),
		.Result(Shifter_Result)
	);

	always @(posedge clk) begin
		if (rst) PC <= 32'b0;
		else if (state_IF) PC_4 <= ALU_Result;
		else if (state_INTR) PC <= 32'h100;
		else if (state_ID && Instruction_reg == 32'b0) PC <= PC_4;
		else if (state_EX) begin
			if (R_Type_jump) PC <= RF_rdata1;
			else if (J_Type) PC <= {PC_4[31:28], instr_index[25:0], 2'b00};
			else if (I_Type_branch && (~opcode[1] && (opcode[0] ^ Zero) ||
									(opcode[1] && (opcode[0] ^ (ALU_Result == 32'b0)))) ||(REGIMM && (rt[0] ^ (ALU_Result == 32'b1))))
					PC <= ALUOut;
			else if (ERET) PC <= EPC;
			else PC <= PC_4;
		end
	end

	always @(posedge clk) begin
		if (state_RDW && Read_data_Valid) Read_data_reg <= Read_data;
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

	assign inst_retire = {RF_wen, RF_waddr, RF_wdata, PC};
endmodule
