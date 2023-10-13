`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

`define S_WAIT 		8'b00000001
`define S_TAG_RD	8'b00000010
`define S_EVICT		8'b00000100
`define S_MEM_RD	8'b00001000
`define S_RECV		8'b00010000
`define S_REFILL	8'b00100000
`define S_RESP		8'b01000000
`define S_CACHE_RD	8'b10000000

module icache_top (
	input	      clk,
	input	      rst,
	
	//CPU interface
	/** CPU instruction fetch request to Cache: valid signal */
	input         from_cpu_inst_req_valid,
	/** CPU instruction fetch request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_inst_req_addr,
	/** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
	output        to_cpu_inst_req_ready,
	
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit Instruction value */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive Instruction */
	input	      from_cpu_cache_rsp_ready,

	//Memory interface (32 byte aligned address)
	/** Cache sending memory read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address (32 byte alignment) */
	output [31:0] to_mem_rd_req_addr,
	/** Acknowledgement from memory: ready to receive memory read request */
	input         from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input         from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input         from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready
);

//TODO: Please add your I-Cache code here
	reg [7:0] current_state;
	reg [7:0] next_state;

	wire state_WAIT;
	wire state_TAG_RD;
	wire state_EVICT;
	wire state_MEM_RD;
	wire state_RECV;
	wire state_REFILL;
	wire state_RESP;
	wire state_CACHE_RD;

	wire [23:0] tag;
	wire [ 2:0] index;
	wire [ 4:0] offset;

	reg [`CACHE_SET - 1 : 0] valid_array [`CACHE_WAY - 1 : 0];
	wire [23:0] tag_rdata [`CACHE_WAY - 1 : 0];
	wire [`LINE_LEN - 1 : 0] data_rdata [`CACHE_WAY - 1 : 0];
	wire [`LINE_LEN - 1 : 0] data_wdata;
	wire [`LINE_LEN - 1 : 0] data_hit;

	reg [7:0] call_cnt [`CACHE_WAY - 1:0][`CACHE_SET - 1:0];
	wire [1:0] replace_position;

	wire [3:0] hit_way;
	wire read_hit;

	reg [2:0] read_cnt;
	reg [31:0] read_data [7:0];

	wire [3:0] wen;

	assign {state_CACHE_RD, state_RESP, state_REFILL, state_RECV, state_MEM_RD,
			state_EVICT, state_TAG_RD, state_WAIT} = current_state;

	always @ (posedge clk) begin
		if (rst) current_state <= 8'b0;
		else current_state <= next_state;
	end

	always @ (*) begin
		case (current_state)
			`S_WAIT: begin
				if (from_cpu_inst_req_valid) next_state = `S_TAG_RD;
				else next_state = `S_WAIT;
			end
			`S_TAG_RD: begin
				if (read_hit) next_state = `S_CACHE_RD;
				else next_state = `S_EVICT;
			end
			`S_EVICT: next_state = `S_MEM_RD;
			`S_MEM_RD: begin
				if (from_mem_rd_req_ready) next_state = `S_RECV;
				else next_state = `S_MEM_RD;
			end
			`S_RECV: begin
				if (from_mem_rd_rsp_valid & from_mem_rd_rsp_last) next_state = `S_REFILL;
				else next_state = `S_RECV;
			end
			`S_REFILL: next_state = `S_RESP;
			`S_CACHE_RD: next_state = `S_RESP;
			`S_RESP: begin
				if (from_cpu_cache_rsp_ready) next_state = `S_WAIT;
				else next_state = `S_RESP;
			end
			default: next_state = `S_WAIT;
		endcase
	end

	assign to_cpu_inst_req_ready = state_WAIT;
	assign to_cpu_cache_rsp_valid = state_RESP;
	assign to_mem_rd_req_valid = state_MEM_RD;
	assign to_mem_rd_rsp_ready = state_RECV;

	assign {tag, index, offset} = from_cpu_inst_req_addr;
	assign to_mem_rd_req_addr = {from_cpu_inst_req_addr[31:5], 5'b0};

	integer i_valid;
	always @ (posedge clk) begin
		if (rst) begin
			for (i_valid = 0; i_valid < `CACHE_WAY; i_valid = i_valid + 1)
				valid_array[i_valid] <= 8'b0;
			end
		else if (state_EVICT) valid_array[replace_position][index] <= 1'b0;
		else if (state_REFILL) valid_array[replace_position][index] <= 1'b1;
	end

	assign wen[0] = state_REFILL & (~replace_position[1] & ~replace_position[0]);
	assign wen[1] = state_REFILL & (~replace_position[1] & replace_position[0]);
	assign wen[2] = state_REFILL & (replace_position[1] & ~replace_position[0]);
	assign wen[3] = state_REFILL & (replace_position[1] & replace_position[0]);
	assign data_wdata = {read_data[7], read_data[6], read_data[5], read_data[4],
						read_data[3], read_data[2], read_data[1], read_data[0]};

	assign read_hit = |hit_way;
	tag_array tag_0 (
        .clk(clk),
        .waddr(index),
        .raddr(index),
        .wen(wen[0]),
        .wdata(tag),
        .rdata(tag_rdata[0])
    );
    tag_array tag_1 (
        .clk(clk),
        .waddr(index),
        .raddr(index),
        .wen(wen[1]),
        .wdata(tag),
        .rdata(tag_rdata[1])
    );
    tag_array tag_2 (
        .clk(clk),
        .waddr(index),
        .raddr(index),
        .wen(wen[2]),
        .wdata(tag),
        .rdata(tag_rdata[2])
    );
    tag_array tag_3 (
        .clk(clk),
        .waddr(index),
        .raddr(index),
        .wen(wen[3]),
        .wdata(tag),
        .rdata(tag_rdata[3])
    );
	assign hit_way[0] = valid_array[0][index] & (tag_rdata[0] == tag);
	assign hit_way[1] = valid_array[1][index] & (tag_rdata[1] == tag);
	assign hit_way[2] = valid_array[2][index] & (tag_rdata[2] == tag);
	assign hit_way[3] = valid_array[3][index] & (tag_rdata[3] == tag);

	data_array data_0 (
		.clk(clk),
		.waddr(index),
		.raddr(index),
		.wen(wen[0]),
		.wdata(data_wdata),
		.rdata(data_rdata[0])
	);
	data_array data_1 (
		.clk(clk),
		.waddr(index),
		.raddr(index),
		.wen(wen[1]),
		.wdata(data_wdata),
		.rdata(data_rdata[1])
	);
	data_array data_2 (
		.clk(clk),
		.waddr(index),
		.raddr(index),
		.wen(wen[2]),
		.wdata(data_wdata),
		.rdata(data_rdata[2])
	);
	data_array data_3 (
		.clk(clk),
		.waddr(index),
		.raddr(index),
		.wen(wen[3]),
		.wdata(data_wdata),
		.rdata(data_rdata[3])
	);
	assign data_hit = {`LINE_LEN{hit_way[0]}} & data_rdata[0] |
						{`LINE_LEN{hit_way[1]}} & data_rdata[1] |
						{`LINE_LEN{hit_way[2]}} & data_rdata[2] |
						{`LINE_LEN{hit_way[3]}} & data_rdata[3];
	assign to_cpu_cache_rsp_data = data_hit >> {offset, 3'b0};

	integer i_call;
	always @ (posedge clk) begin
		if (rst) begin
			for (i_call = 0; i_call < `CACHE_SET; i_call = i_call + 1) begin
				call_cnt[0][i_call] <= 8'b0;
				call_cnt[1][i_call] <= 8'b0;
				call_cnt[2][i_call] <= 8'b0;
				call_cnt[3][i_call] <= 8'b0;
			end
		end
		else if (state_RESP && from_cpu_cache_rsp_ready)begin
			if (~hit_way[0]) call_cnt[0][index] <= call_cnt[0][index] + 1;
			if (~hit_way[1]) call_cnt[1][index] <= call_cnt[1][index] + 1;
			if (~hit_way[2]) call_cnt[2][index] <= call_cnt[2][index] + 1;
			if (~hit_way[3]) call_cnt[3][index] <= call_cnt[3][index] + 1;
		end
		else if (state_REFILL) call_cnt[replace_position][index] <= 8'b0;
	end

	assign replace_position = (~valid_array[0][index])? 2'd0 :
								(~valid_array[1][index])? 2'd1 :
								(~valid_array[2][index])? 2'd2 :
								(~valid_array[3][index])? 2'd3 :
								(call_cnt[0][index] >= call_cnt[1][index])? 
								(call_cnt[0][index] >= call_cnt[2][index])?
								(call_cnt[0][index] >= call_cnt[3][index])? 2'd0 : 2'd3 :
								(call_cnt[2][index] >= call_cnt[3][index])? 2'd2 : 2'd3 :
								(call_cnt[1][index] >= call_cnt[2][index])?
								(call_cnt[1][index] >= call_cnt[3][index])? 2'd1 : 2'd3 :
								(call_cnt[2][index] >= call_cnt[3][index])? 2'd2 : 2'd3;

	always @ (posedge clk) begin
		if (rst || state_MEM_RD && from_mem_rd_req_ready) read_cnt <= 2'b0;
		else if (state_RECV && from_mem_rd_rsp_valid) read_cnt <= read_cnt + 1'b1;
	end

	always @ (posedge clk) begin
		if (state_RECV && from_mem_rd_rsp_valid) read_data[read_cnt] <= from_mem_rd_rsp_data;
	end

endmodule
