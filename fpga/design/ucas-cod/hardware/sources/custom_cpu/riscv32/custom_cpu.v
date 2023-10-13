`timescale 10ns / 1ns

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output [31:0] PC,
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

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/
// TODO: Please add your custom CPU code here
	//连接到寄存器堆的信号
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [4:0]		RF_raddr1;
	wire [4:0]		RF_raddr2;
	wire [31:0]		RF_wdata;
	wire [31:0]		RF_rdata1;
	wire [31:0]		RF_rdata2;

	//在模块之间传输有关寄存器堆信号的线
	wire [4:0] 		RF_waddr_23;
	wire [4:0]		RF_waddr_34;
	wire [4:0]		RF_waddr_45;
	wire [31:0] 	RF_rdata1_23;
	wire [31:0]		RF_rdata2_23;
	wire [31:0] 	wb_from_ex;
	wire [31:0]		wb_from_load;

	//前一个模块完成工作的信号
	wire complete_12;
	wire complete_23;
	wire complete_34;
	wire complete_45;

	//模块间传输PC的信号
	wire [31:0] PC_12;
	wire [31:0] PC_23;
	wire [31:0] PC_34;
	wire [31:0] PC_45;
	wire [31:0] branch_PC;	//分支/跳转指令的目标地址值

	//待译码指令
	wire [31:0] Instruction_now;

	wire fb_ex_branch;	//分支信号(需将工作任务清除)
	wire fb_mem;		//访存信号(需暂停工作)

	wire [19:0] Inst_Decode;	//指令译码结果
	wire [31:0] imm;			//立即数

	wire [8:0] mem_info;		//存有指令译码后mem阶段所需信息
	wire [31:0] Write_data_34;	//存有store待存值或非访存指令的写回数据
	wire [31:0] mem_address;	//存有访存类指令的地址

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

	state_IF IF(
		.clk(clk),
		.rst(rst),

		.PC(PC_12),
		.Inst_Req_Valid(Inst_Req_Valid),
		.Inst_Req_Ready(Inst_Req_Ready),

		.Instruction(Instruction),
		.Inst_Valid(Inst_Valid),
		.Inst_Ready(Inst_Ready),
		.Instruction_reg(Instruction_now),

		.branch_PC(branch_PC),
		.complete_this(complete_12),

		.fb_ex_branch(fb_ex_branch),
		.fb_mem(fb_mem),
		.cpu_perf_cnt_2(cpu_perf_cnt_2)
	);

	assign PC = PC_12;

	state_ID ID(
		.clk(clk),
		.rst(rst),

		.complete_pre(complete_12),
		.complete_this(complete_23),

		.PC_input(PC_12),
		.PC_output(PC_23),

		.branch_PC_reg(branch_PC),
		.Instruction_reg(Instruction_now),

		.RF_rdata1(RF_rdata1),
		.RF_rdata2(RF_rdata2),
		.RF_raddr1(RF_raddr1),
		.RF_raddr2(RF_raddr2),
		.RF_waddr(RF_waddr_23),
		.RF_rdata1_out(RF_rdata1_23),
		.RF_rdata2_out(RF_rdata2_23),

		.Inst_Decode(Inst_Decode),
		.imm_r(imm),

		.fb_ex_branch(fb_ex_branch),
		.fb_mem(fb_mem),

		.wb_from_ex(wb_from_ex),
		.wb_from_load(wb_from_load),

		.cpu_perf_cnt_1(cpu_perf_cnt_1),
		.cpu_perf_cnt_4(cpu_perf_cnt_4)
	);

	state_EX EX(
		.clk(clk),
		.rst(rst),

		.complete_pre(complete_23),
		.complete_this(complete_34),

		.RF_rdata1(RF_rdata1_23),
		.RF_rdata2(RF_rdata2_23),
		.RF_waddr_in(RF_waddr_23),
		.RF_waddr_out(RF_waddr_34),

		.Inst_Decode(Inst_Decode),
		.imm(imm),

		.PC_input(PC_23),
		.PC_output(PC_34),

		.fb_ex_branch(fb_ex_branch),
		.fb_mem(fb_mem),

		.mem_info_out(mem_info),
		.Write_data_out(Write_data_34),
		.mem_address_out(mem_address)
	);

	assign wb_from_ex = Write_data_34;

	state_MEM MEM(
		.clk(clk),
		.rst(rst),

		.complete_pre(complete_34),
		.complete_this(complete_45),

		.PC_input(PC_34),
		.PC_output(PC_45),

		.RF_waddr_in(RF_waddr_34),
		.mem_info_in(mem_info),
		.Write_data_in(Write_data_34),
		.Write_back_reg(RF_wdata),
		.mem_address_in(mem_address),

		.RF_waddr_out(RF_waddr),
		
		.Address(Address),
		.MemWrite(MemWrite),
		.Write_data(Write_data),
		.Write_strb(Write_strb),
		.MemRead(MemRead),
		.Mem_Req_Ready(Mem_Req_Ready),

		.Read_data(Read_data),
		.Read_data_Valid(Read_data_Valid),
		.Read_data_Ready(Read_data_Ready),

		.fb_mem(fb_mem)
	);

	assign wb_from_load = RF_wdata;


	state_WB WB(
		.clk(clk),
		.rst(rst),

		.complete_pre(complete_45),

		.PC_input(PC_45),
		.RF_waddr(RF_waddr),
		.RF_wdata(RF_wdata),

		.RF_wen(RF_wen),
		.inst_retire(inst_retire)
	);

	reg [31:0] cycle_cnt;
	reg [31:0] cycle_high_cnt;

	always @(posedge clk) begin
		if (rst || cycle_cnt == 32'd999_999_999) cycle_cnt <= 32'b0;
		else cycle_cnt <= cycle_cnt + 32'b1;
	end
	assign cpu_perf_cnt_0 = cycle_cnt;

/*	always @(posedge clk) begin
		if (rst) memory_cnt <= 32'b0;
		else if (state_LD || state_ST) memory_cnt = memory_cnt +32'b1;
	end
	assign cpu_perf_cnt_1 = memory_cnt;

	always @(posedge clk) begin
		if (rst) wait_cnt <= 32'b0;
		else if (state_IF && ~Inst_Req_Ready || state_IW && ~Inst_Valid || (state_LD || state_ST) && ~Mem_Req_Ready || state_RDW && ~Read_data_Valid) wait_cnt = wait_cnt +32'b1;
	end
	assign cpu_perf_cnt_3 = wait_cnt;

	always @(posedge clk) begin
		if (rst) load_cnt <= 32'b0;
		else if (state_EX && I_Type_load) load_cnt <= load_cnt + 32'b1;
	end
	assign cpu_perf_cnt_4 = load_cnt;

	always @(posedge clk) begin
		if (rst) instruction_fetch_cnt <= 32'b0;
		else if (state_IF && ~Inst_Req_Ready) instruction_fetch_cnt = instruction_fetch_cnt +32'b1;
	end
	assign cpu_perf_cnt_5 = instruction_fetch_cnt;

	always @(posedge clk) begin
		if (rst) instruction_wait_cnt <= 32'b0;
		else if (state_IW && ~Inst_Valid) instruction_wait_cnt = instruction_wait_cnt +32'b1;
	end
	assign cpu_perf_cnt_6 = instruction_wait_cnt;

	always @(posedge clk) begin
		if (rst) mem_wait_cnt <= 32'b0;
		else if ((state_LD || state_ST) && ~Mem_Req_Ready) mem_wait_cnt = mem_wait_cnt +32'b1;
	end
	assign cpu_perf_cnt_7 = mem_wait_cnt;

	always @(posedge clk) begin
		if (rst) rdw_cnt <= 32'b0;
		else if (state_RDW && ~Read_data_Valid) rdw_cnt = rdw_cnt +32'b1;
	end
	assign cpu_perf_cnt_8 = rdw_cnt;
	*/
	always @(posedge clk) begin
		if (rst) cycle_high_cnt <= 32'b0;
		else if (cycle_cnt == 32'd999_999_999) cycle_high_cnt <= cycle_high_cnt + 32'b1;
	end
	assign cpu_perf_cnt_9 = cycle_high_cnt;
	
endmodule