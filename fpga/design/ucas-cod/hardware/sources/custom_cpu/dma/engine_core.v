`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Xu Zhang (zhangxu415@mails.ucas.ac.cn)
// 
// Create Date: 06/14/2018 11:39:09 AM
// Design Name: 
// Module Name: dma_core
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define S_IDLE 4'b0001
`define S_REQ 4'b0010
`define S_RD_WR 4'b0100
`define S_FIFO2R 4'b1000

module engine_core #(
	parameter integer  DATA_WIDTH       = 32
)
(
	input    clk,
	input    rst,
	
	output reg [31:0]       src_base,
	output reg [31:0]       dest_base,
	output reg [31:0]       tail_ptr,
	output reg [31:0]       head_ptr,
	output reg [31:0]       dma_size,
	output reg [31:0]       ctrl_stat,

	input  [31:0]	    reg_wr_data,
	input  [ 5:0]       reg_wr_en,
  
	output              intr,
  
	output [31:0]       rd_req_addr,
	output [ 4:0]       rd_req_len,
	output              rd_req_valid,
	
	input               rd_req_ready,
	input  [31:0]       rd_rdata,
	input               rd_last,
	input               rd_valid,
	output              rd_ready,
	
	output [31:0]       wr_req_addr,
	output [ 4:0]       wr_req_len,
	output              wr_req_valid,
	input               wr_req_ready,
	output [31:0]       wr_data,
	output              wr_valid,
	input               wr_ready,
	output              wr_last,
	
	output              fifo_rden,
	output [31:0]       fifo_wdata,
	output              fifo_wen,
	
	input  [31:0]       fifo_rdata,
	input               fifo_is_empty,
	input               fifo_is_full
);
	// TODO: Please add your logic design here
	reg [3:0] rd_current_state;
	reg [3:0] rd_next_state;
	reg [3:0] wr_current_state;
	reg [3:0] wr_next_state;

	wire state_rd_IDLE;
	wire state_rd_REQ;
	wire state_rd_RD;
	wire state_wr_IDLE;
	wire state_wr_REQ;
	wire state_wr_WR;
	wire state_wr_FIFO2R;

	wire [2:0] last_burst;
	wire [31:0] total_burst_times;
	wire rd_complete_burst;
	wire wr_complete_burst;

	wire EN;
	wire equal;

	reg [31:0] fifo_data;

	reg [31:0] rd_counter;
	reg [31:0] wr_counter;
	reg [31:0] wdata_counter;

	always @ (posedge clk) begin
		if (reg_wr_en[0]) src_base <= reg_wr_data;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[1]) dest_base <= reg_wr_data;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[2]) tail_ptr <= reg_wr_data;
		else if (rd_complete_burst & wr_complete_burst & state_rd_IDLE & state_wr_IDLE)
			tail_ptr <= tail_ptr + dma_size;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[3]) head_ptr <= reg_wr_data;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[4]) dma_size <= reg_wr_data;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[5]) ctrl_stat <= reg_wr_data;
		else if (EN & rd_complete_burst & wr_complete_burst & state_rd_IDLE & state_wr_IDLE)
			ctrl_stat[31] = 1'b1;
	end
	assign intr = ctrl_stat[31];
	assign EN = ctrl_stat[0];
	assign equal = head_ptr == tail_ptr;

	assign last_burst = dma_size[4:2] + |dma_size[1:0];
	assign total_burst_times = {5'b0, dma_size[31:5]} + |dma_size[4:0];
	assign rd_complete_burst = rd_counter == total_burst_times;
	assign wr_complete_burst = wr_counter == total_burst_times;

	assign state_rd_IDLE = rd_current_state == `S_IDLE;
	assign state_rd_REQ = rd_current_state == `S_REQ;
	assign state_rd_RD = rd_current_state == `S_RD_WR;
	assign state_wr_IDLE = wr_current_state == `S_IDLE;
	assign state_wr_REQ = wr_current_state == `S_REQ;
	assign state_wr_WR = wr_current_state == `S_RD_WR;
	assign state_wr_FIFO2R = wr_current_state == `S_FIFO2R;

	always @ (posedge clk) begin
		if (rst) rd_current_state <= `S_IDLE;
		else rd_current_state <= rd_next_state;
	end

	always @ (*) begin
		case (rd_current_state)
			`S_IDLE: begin
				if (EN & state_wr_IDLE & ~equal & ~rd_complete_burst & fifo_is_empty)
					rd_next_state = `S_REQ;
				else rd_next_state = `S_IDLE;
			end
			`S_REQ: begin
				if (fifo_is_full | rd_complete_burst) rd_next_state = `S_IDLE;
				else if (rd_req_ready) rd_next_state = `S_RD_WR;
				else rd_next_state = `S_REQ;
			end
			`S_RD_WR: begin
				if (rd_valid & rd_last & ~fifo_is_full) rd_next_state = `S_REQ;
				else rd_next_state = `S_RD_WR;
			end
			default: rd_next_state = `S_IDLE;
		endcase
	end	

	always @ (posedge clk) begin
		if (rst | state_rd_IDLE & state_wr_IDLE & EN & ~equal & rd_complete_burst & wr_complete_burst)
			rd_counter <= 32'b0;
		else if (state_rd_RD & rd_valid & rd_last)
			rd_counter <= rd_counter + 1;
	end

	assign rd_req_addr = src_base + tail_ptr + (rd_counter << 5);
	assign rd_req_len = (rd_counter == total_burst_times - 1 && |last_burst)? {2'b0, {3{last_burst - 1'b1}}} : 5'b111;
	assign rd_req_valid = state_rd_REQ & ~fifo_is_full & ~rd_complete_burst;
	assign rd_ready = state_rd_RD;

	assign fifo_wen = rd_ready & rd_valid & ~fifo_is_full;
	assign fifo_wdata = rd_rdata;

	always @ (posedge clk) begin
		if (rst) wr_current_state <= `S_IDLE;
		else wr_current_state <= wr_next_state;
	end

	always @ (*) begin
		case (wr_current_state)
			`S_IDLE: begin
				if (EN & state_rd_IDLE & ~equal & ~wr_complete_burst & fifo_is_full)
					wr_next_state = `S_REQ;
				else wr_next_state = `S_IDLE;
			end
			`S_REQ: begin
				if (wr_complete_burst | fifo_is_empty) wr_next_state = `S_IDLE;
				else if (wr_req_ready) wr_next_state = `S_FIFO2R;
				else wr_next_state = `S_REQ;
			end
			`S_FIFO2R: begin
				wr_next_state = `S_RD_WR;
			end
				`S_RD_WR: begin
				if (wr_ready & wr_last | fifo_is_empty) wr_next_state = `S_REQ;
				else if (wr_ready & ~fifo_is_empty) wr_next_state = `S_FIFO2R;
				else wr_next_state = `S_RD_WR;
			end
			default: wr_next_state = `S_IDLE;
		endcase
	end

	assign fifo_rden = wr_next_state == `S_FIFO2R;
	always @ (posedge clk) begin
		if (state_wr_FIFO2R) fifo_data <= fifo_rdata;
	end

	always @ (posedge clk) begin
		if (rst | state_rd_IDLE & state_wr_IDLE & EN & ~equal & ~intr & rd_complete_burst & wr_complete_burst)
			wr_counter <= 32'b0;
		else if (state_wr_WR & wr_ready & wr_last)
			wr_counter <= wr_counter + 1;
	end

	always @ (posedge clk) begin
		if (rst | state_wr_REQ) wdata_counter <= 3'b0;
		else if (state_wr_WR & wr_ready) wdata_counter <= wdata_counter + 1;
	end

	assign wr_req_valid = state_wr_REQ & ~fifo_is_empty;
	assign wr_req_addr = dest_base + tail_ptr + (wr_counter << 5);
	assign wr_req_len = (wr_counter == total_burst_times - 1 && |last_burst)? {2'b0, {3{last_burst - 1'b1}}} : 5'b111;
	assign wr_valid = state_wr_WR;
	assign wr_data = fifo_data;
	assign wr_last = wdata_counter == wr_req_len[2:0];
endmodule