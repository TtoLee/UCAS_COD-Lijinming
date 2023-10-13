`timescale 10ns / 1ns

`define S_INIT 5'b00001
`define S_LD 5'b00010
`define S_RDW 5'b00100
`define S_ST 5'b01000
`define S_COM 5'b10000

module state_MEM (
    input               clk,
    input               rst,

    input 				complete_pre,		//前一阶段完成信号
    output reg 			complete_this,		//本阶段完成信号

    input       [31:0]  PC_input,           //输入PC值 
    output reg  [31:0]  PC_output,          //输出PC值

    input       [ 4:0]  RF_waddr_in,        //寄存器写地址(输入)
    input       [ 8:0]  mem_info_in,		//包含funct3[2], 确定load/store信号, strb值
	input       [31:0]  Write_data_in,	    //store写入内存数据/写回寄存器数据(输入)
    output reg  [31:0]  Write_back_reg,     //写回寄存器数据(输出)
	input       [31:0]  mem_address_in,	    //store/load地址

    output reg  [4:0]   RF_waddr_out,       //寄存器写地址(输出)

	//Memory request channel
	output reg  [31:0]  Address,
	output              MemWrite,
	output      [31:0]  Write_data,
	output reg  [ 3:0]  Write_strb,
	output              MemRead,
	input               Mem_Req_Ready,

	//Memory data response channel
	input       [31:0]  Read_data,
	input               Read_data_Valid,
	output              Read_data_Ready,

    output              fb_mem               //访存反馈信号          
);

    reg [31:0] memory_address;
    reg [31:0] memory_data;
    
    wire memory_byte;
	wire memory_half;
	wire memory_word;

    reg [4:0] current_state;
    reg [4:0] next_state;

    wire state_INIT;
    wire state_ST;
    wire state_LD;
    wire state_RDW;
    wire state_COM;

    wire [2:0] funct3;
    wire S_Type;
    wire I_Type_load;
    wire [3:0] strb;
    wire [7:0] lb_data;
    wire [15:0] lh_data;

    reg state_cpu_init;

    always @ (posedge clk) begin
        if (rst) current_state <= `S_INIT;
        else current_state <= next_state;
    end

    always @ (*) begin
        case (current_state)
            `S_INIT: begin
                if (~complete_pre) next_state = `S_INIT;
                else if (S_Type) next_state = `S_ST;
                else if (I_Type_load) next_state = `S_LD;
                else next_state = `S_INIT;
            end
			`S_ST: begin
				if (Mem_Req_Ready) next_state = `S_COM;
				else next_state = `S_ST;
			end
			`S_LD: begin
				if (Mem_Req_Ready) next_state = `S_RDW;
				else next_state = `S_LD;
			end
			`S_RDW: begin
				if (Read_data_Valid) next_state = `S_COM;
				else next_state = `S_RDW;
			end
            `S_COM: next_state = `S_INIT;
            default next_state = `S_INIT;
        endcase
    end

    assign {state_COM, state_ST, state_RDW, state_LD, state_INIT} = current_state;
    assign {funct3, S_Type, I_Type_load, strb} = mem_info_in;

    always @ (posedge clk) begin
        Write_strb <= strb;
    end

    assign memory_byte = ~funct3[1] & ~funct3[0];
	assign memory_half = ~funct3[1] & funct3[0];
	assign memory_word = funct3[1] & ~funct3[0];
    assign lb_data = (mem_address_in[1] && mem_address_in[0])? Read_data[31:24]:
						((mem_address_in[1] && ~mem_address_in[0])? Read_data[23:16]:
						(~mem_address_in[1] && mem_address_in[0]? Read_data[15:8]:
						Read_data[7:0]));
	assign lh_data = (~mem_address_in[1] && ~mem_address_in[0])? Read_data[15:0]: Read_data[31:16];

    always @ (posedge clk) begin
        if (complete_pre & state_INIT & S_Type)
            memory_data <= Write_data_in;
        else if (state_RDW & Read_data_Valid)
            Write_back_reg <= ({32{memory_byte & ~funct3[2]}} & {{24{lb_data[7]}}, lb_data}) |
						({32{memory_half & ~funct3[2]}} & {{16{lh_data[15]}}, lh_data}) |
						({32{memory_word & ~funct3[2]}} & Read_data) |
						({32{memory_byte & funct3[2]}} & {{24{1'b0}}, lb_data}) |
						({32{memory_half & funct3[2]}} & {{16{1'b0}}, lh_data});
        else if (complete_pre & state_INIT & ~(S_Type | I_Type_load))
            Write_back_reg <= Write_data_in;
    end

    always @ (posedge clk) begin
        if (complete_pre & state_INIT) Address <= {mem_address_in[31:2], 2'b0};
    end

    always @ (posedge clk) begin
        if (rst) RF_waddr_out <= 5'b0;
        else if (complete_pre && state_INIT) RF_waddr_out <= RF_waddr_in;
    end

    always @ (posedge clk) begin
        if (complete_pre & state_INIT) PC_output <= PC_input;
    end

    assign Write_data = memory_data;
    assign MemWrite = state_ST;
    assign MemRead = state_LD;

    assign fb_mem = ~rst & (state_LD | state_RDW | state_ST);

    always @ (posedge clk) begin
        if (rst) complete_this <= 0;
        else if (state_INIT & complete_pre & (state_COM | (state_INIT & (~complete_pre | (~S_Type & ~I_Type_load)))) | state_COM) complete_this <= 1;
        else complete_this <= 0;
    end

    assign Read_data_Ready = state_RDW | state_cpu_init;

    always @ (posedge clk) begin
        state_cpu_init <= rst;
    end

endmodule