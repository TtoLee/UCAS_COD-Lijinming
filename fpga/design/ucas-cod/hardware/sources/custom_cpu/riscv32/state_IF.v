`timescale 10ns / 1ns

`define S_INIT 4'b0001
`define S_IF 4'b0010
`define S_IW 4'b0100
`define S_COM 4'b1000

module state_IF(
    input               clk,
    input               rst,

	//Instruction request channel
	output reg [31:0]   PC,                 //同时传向ID阶段
	output              Inst_Req_Valid,
	input               Inst_Req_Ready,

	//Instruction response channel
	input      [31:0]   Instruction,
	input               Inst_Valid,
	output              Inst_Ready,
    output reg [31:0]   Instruction_reg,    //存有Instruction值, 传给ID以分析

    input      [31:0]   branch_PC,          //跳转/分支PC目标值

    output              complete_this,      //本阶段完成标志

    input               fb_ex_branch,       //需要跳转/分支反馈信号
    input               fb_mem,             //访存反馈信号

    output     [31:0]   cpu_perf_cnt_2
);

    reg [3:0] current_state;
    reg [3:0] next_state;

    reg clear_for_branch;

    wire state_INIT;
    wire state_IF;
    wire state_IW;
    wire state_COM;
    assign {state_COM, state_IW, state_IF, state_INIT} = current_state;

    always @ (posedge clk) begin
        if (rst) current_state <= `S_INIT;
        else current_state <= next_state;
    end

    always @ (*) begin
        case (current_state) 
            `S_INIT: next_state = `S_IF;
            `S_IF: begin
                if (Inst_Req_Ready) next_state = `S_IW;
                else next_state = `S_IF;
            end
            `S_IW: begin
                if (Inst_Valid) begin
                    if (fb_ex_branch | clear_for_branch) next_state = `S_IF;
                    else next_state = `S_COM;
                end
                else next_state = `S_IW;
            end
            `S_COM: begin
                if (fb_mem) next_state = `S_COM;
                else next_state = `S_INIT;
            end
            default: next_state = `S_INIT;
        endcase
    end

    always @ (posedge clk) begin
        if (rst) clear_for_branch <= 0;
        else if (fb_ex_branch) clear_for_branch <= 1;
        else if ((state_IW & Inst_Valid | state_COM) & clear_for_branch) clear_for_branch <= 0;
    end

    always @ (posedge clk) begin
        if (rst) PC <= 32'b0;
        else if (state_IW & (fb_ex_branch | clear_for_branch)) PC <= branch_PC;
        else if (state_COM)
            if (fb_ex_branch | clear_for_branch) PC <= branch_PC;
            else if (~fb_mem) PC <= PC + 4;
    end

    always @ (posedge clk) begin
        if (state_IW && Inst_Valid) Instruction_reg <= Instruction;
    end

    assign complete_this = state_COM;

    assign Inst_Req_Valid = state_IF;
    assign Inst_Ready = state_IW || state_INIT;

    reg [31:0] instruction_cnt;
    always @(posedge clk) begin
		if (rst) instruction_cnt <= 32'b0;
		else if (state_COM & ~fb_mem) instruction_cnt = instruction_cnt +32'b1;
	end
	assign cpu_perf_cnt_2 = instruction_cnt;
endmodule