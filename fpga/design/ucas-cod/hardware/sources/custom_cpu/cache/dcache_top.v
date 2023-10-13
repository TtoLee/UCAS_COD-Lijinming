`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

`define S_WAIT 			16'b0000_0000_0000_0001
`define S_TAG_RD		16'b0000_0000_0000_0010
`define S_CACHE_RD		16'b0000_0000_0000_0100
`define S_CACHE_RESP	16'b0000_0000_0000_1000
`define S_CACHE_WR		16'b0000_0000_0001_0000
`define S_EVICT			16'b0000_0000_0010_0000
`define S_MEM_WR		16'b0000_0000_0100_0000
`define S_CACHE_SEND	16'b0000_0000_1000_0000
`define S_MEM_RD		16'b0000_0001_0000_0000
`define S_RECV			16'b0000_0010_0000_0000
`define S_REFILL		16'b0000_0100_0000_0000
`define S_COMEM_RD		16'b0000_1000_0000_0000
`define S_MEM_RECV		16'b0001_0000_0000_0000
`define S_MEM_RESP		16'b0010_0000_0000_0000
`define S_COMEM_WR		16'b0100_0000_0000_0000
`define S_CPU_SEND		16'b1000_0000_0000_0000

module dcache_top (
	input	      clk,
	input	      rst,
  
	//CPU interface
	/** CPU memory/IO access request to Cache: valid signal */
	input         from_cpu_mem_req_valid,
	/** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
	input         from_cpu_mem_req,
	/** CPU memory/IO access request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_mem_req_addr,
	/** CPU memory/IO access request to Cache: 32-bit write data */
	input  [31:0] from_cpu_mem_req_wdata,
	/** CPU memory/IO access request to Cache: 4-bit write strobe */
	input  [ 3:0] from_cpu_mem_req_wstrb,
	/** Acknowledgement from Cache: ready to receive CPU memory access request */
	output        to_cpu_mem_req_ready,
		
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit read data */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive read data */
	input         from_cpu_cache_rsp_ready,
		
	//Memory/IO read interface
	/** Cache sending memory/IO read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address
	  * 4 byte alignment for I/O read 
	  * 32 byte alignment for cache read miss */
	output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
	  * 0 for I/O read (read only one data beat)
	  * 7 for cache read miss (read eight data beats) */
	output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
	input	      from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input	      from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input	      from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready,

	//Memory/IO write interface
	/** Cache sending memory/IO write request: valid signal */
	output        to_mem_wr_req_valid,
	/** Cache sending memory write request: address
	  * 4 byte alignment for I/O write 
	  * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
	output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
	output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
	input         from_mem_wr_req_ready,

	/** Cache sending memory/IO write data: valid signal for current data beat */
	output        to_mem_wr_data_valid,
	/** Cache sending memory/IO write data: current data beat */
	output [31:0] to_mem_wr_data,
	/** Cache sending memory/IO write data: write strobe
	  * 4'b1111 for cache write-back 
	  * other values for I/O write and cache write miss according to the original CPU request*/ 
	output [ 3:0] to_mem_wr_data_strb,
	/** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
	output        to_mem_wr_data_last,
	/** Acknowledgement from memory/IO: ready to receive current data beat */
	input	      from_mem_wr_data_ready
);

  //TODO: Please add your D-Cache code here
  	reg [15:0] current_state;
  	reg [15:0] next_state;
	wire state_WAIT;
	wire state_TAG_RD;
	wire state_CACHE_RD;
	wire state_CACHE_RESP;
	wire state_CACHE_WR;
	wire state_EVICT;
	wire state_MEM_WR;
	wire state_CACHE_SEND;
	wire state_MEM_RD;
	wire state_RECV	;
	wire state_REFILL;
	wire state_COMEM_RD;
	wire state_MEM_RECV;
	wire state_MEM_RESP;
	wire state_COMEM_WR;
	wire state_CPU_SEND;

	reg [31:0] cpu_data;
	reg [3:0] cpu_strb;
	reg [31:0] cpu_address;

  	wire able_to_cache;
	reg mem_req;
	wire [23:0] tag;
	wire [2:0] index;
	wire [4:0] offset;

	wire [3:0] wen;
	wire [1:0] write_way;
	wire [1:0] replace_position;

	reg [31:0] cache_to_mem_data [7:0];

	wire [3:0] hit_way;
	wire read_hit;
	wire dirty;

	reg [7:0] call_cnt [`CACHE_WAY - 1:0][`CACHE_SET - 1:0];
	reg [2:0] operate_cnt;

	reg [`CACHE_SET - 1 : 0] valid_array [`CACHE_WAY - 1 : 0];
	reg [`CACHE_SET - 1 : 0] dirty_array [`CACHE_WAY - 1 : 0];
	wire [23:0] tag_rdata [`CACHE_WAY - 1 : 0];
	wire [`LINE_LEN - 1 : 0] data_rdata [`CACHE_WAY - 1 : 0];
	wire [`LINE_LEN - 1 : 0] data_wdata;
	wire [`LINE_LEN - 1 : 0] data_hit;
	reg [31:0] read_data [7:0];
	reg [31:0] bypass_read_data;

	assign {state_CPU_SEND, state_COMEM_WR, state_MEM_RESP, state_MEM_RECV, state_COMEM_RD,
			state_REFILL, state_RECV, state_MEM_RD, state_CACHE_SEND, state_MEM_WR, state_EVICT,
			state_CACHE_WR, state_CACHE_RESP, state_CACHE_RD, state_TAG_RD, state_WAIT} = current_state;

	always @ (posedge clk) begin
		if (rst) current_state <= 16'b0;
		else current_state <= next_state;
	end

	always @ (*) begin
		case (current_state)
			`S_WAIT: begin
				if (!from_cpu_mem_req_valid) next_state = `S_WAIT;
				else if (able_to_cache) next_state = `S_TAG_RD;
				else if (from_cpu_mem_req) next_state = `S_COMEM_WR;
				else next_state = `S_COMEM_RD;
			end
			`S_TAG_RD: begin
				if (~read_hit) next_state = `S_EVICT;
				else if (~mem_req) next_state = `S_CACHE_RD;
				else next_state = `S_CACHE_WR;
			end
			`S_CACHE_RD: next_state = `S_CACHE_RESP;
			`S_CACHE_RESP: next_state = `S_WAIT;
			`S_CACHE_WR: next_state = `S_WAIT;
			`S_EVICT: begin
				if (dirty) next_state = `S_MEM_WR;
				else next_state = `S_MEM_RD;
			end
			`S_MEM_WR: begin
				if (from_mem_wr_req_ready) next_state = `S_CACHE_SEND;
				else next_state = `S_MEM_WR;
			end
			`S_CACHE_SEND: begin
				if (from_mem_wr_data_ready & to_mem_wr_data_last) next_state = `S_MEM_RD;
				else next_state = `S_CACHE_SEND;
			end
			`S_MEM_RD: begin
				if (from_mem_rd_req_ready) next_state = `S_RECV;
				else next_state = `S_MEM_RD;
			end
			`S_RECV: begin
				if (from_mem_rd_rsp_valid & from_mem_rd_rsp_last) next_state = `S_REFILL;
				else next_state = `S_RECV;
			end
			`S_REFILL: begin
				if (mem_req) next_state = `S_CACHE_WR;
				else next_state = `S_CACHE_RESP;
			end
			`S_COMEM_WR: begin
				if (from_mem_wr_req_ready) next_state = `S_CPU_SEND;
				else next_state = `S_COMEM_WR;
			end
			`S_CPU_SEND: begin
				if (from_mem_wr_data_ready & to_mem_wr_data_last) next_state = `S_WAIT;
				else next_state = `S_CPU_SEND;
			end
			`S_COMEM_RD: begin
				if (from_mem_rd_req_ready) next_state = `S_MEM_RECV;
				else next_state = `S_COMEM_RD;
			end
			`S_MEM_RECV: begin
				if (from_mem_rd_rsp_valid & from_mem_rd_rsp_last) next_state = `S_MEM_RESP;
				else next_state = `S_MEM_RECV;
			end
			`S_MEM_RESP: begin
				if (from_cpu_cache_rsp_ready) next_state = `S_WAIT;
				else next_state = `S_MEM_RESP;
			end
			default: next_state = `S_WAIT;
		endcase
	end

	integer i_valid;
	always @ (posedge clk) begin
		if (rst) begin
			for (i_valid = 0; i_valid < `CACHE_WAY; i_valid = i_valid + 1)
				valid_array[i_valid] <= 8'b0;
			end
		else if (state_EVICT) valid_array[replace_position][index] <= 1'b0;
		else if (state_REFILL) valid_array[replace_position][index] <= 1'b1;
	end

	integer i_dirty;
	always @ (posedge clk) begin
		if (rst) begin
			for (i_dirty = 0; i_dirty <`CACHE_WAY; i_dirty = i_dirty + 1)
				dirty_array[i_dirty] <= 8'b0;
		end
		else if (state_CACHE_WR)
			dirty_array[write_way][index] <= 1'b1;
		else if (state_CACHE_SEND & from_mem_wr_data_ready & to_mem_wr_data_last)
			dirty_array[replace_position][index] <= 1'b0;
	end

	always @ (posedge clk) begin
		if (state_WAIT & from_cpu_mem_req_valid) begin
			cpu_data <= from_cpu_mem_req_wdata;
			cpu_strb <= from_cpu_mem_req_wstrb;
			cpu_address <= from_cpu_mem_req_addr;
		end
	end
	assign {tag, index, offset} = cpu_address;

	assign able_to_cache = (|from_cpu_mem_req_addr[31:5]) & ~(from_cpu_mem_req_addr[31] | from_cpu_mem_req_addr[30]);
	assign dirty = dirty_array[replace_position][index];

	always @ (posedge clk) begin
		if (state_WAIT && from_cpu_mem_req_valid)
			mem_req <= from_cpu_mem_req;
	end

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
	assign read_hit = |hit_way;

	assign write_way = {2{hit_way[0]}} & 2'd0 |
						{2{hit_way[1]}} & 2'd1 |
						{2{hit_way[2]}} & 2'd2 |
						{2{hit_way[3]}} & 2'd3;

	assign wen[0] = state_CACHE_WR & hit_way[0] | state_REFILL & (~replace_position[1] & ~replace_position[0]);
	assign wen[1] = state_CACHE_WR & hit_way[1] | state_REFILL & (~replace_position[1] & replace_position[0]);
	assign wen[2] = state_CACHE_WR & hit_way[2] | state_REFILL & (replace_position[1] & ~replace_position[0]);
	assign wen[3] = state_CACHE_WR & hit_way[3] | state_REFILL & (replace_position[1] & replace_position[0]);

	assign data_wdata = (state_CACHE_WR)? (data_rdata[write_way] &
						~({224'b0, {8{cpu_strb[3]}}, {8{cpu_strb[2]}}, {8{cpu_strb[1]}}, {8{cpu_strb[0]}}} << {offset, 3'b0}) |
						({224'b0, cpu_data} << {offset, 3'b0})) :
						(state_REFILL)? {read_data[7], read_data[6], read_data[5], read_data[4], read_data[3], read_data[2], read_data[1], read_data[0]} : 256'b0;

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
	always @ (posedge clk) begin
		if (state_MEM_WR & from_mem_wr_req_ready)
			{cache_to_mem_data[7], cache_to_mem_data[6], cache_to_mem_data[5], cache_to_mem_data[4],
			cache_to_mem_data[3], cache_to_mem_data[2], cache_to_mem_data[1], cache_to_mem_data[0]} <= data_rdata[replace_position];
	end

	assign to_mem_wr_data = {32{state_CACHE_SEND}} & cache_to_mem_data[operate_cnt] |
							{32{state_CPU_SEND}} & cpu_data;
	assign to_mem_wr_data_last = (state_CPU_SEND) ? (operate_cnt == 3'b0)? 1 : 0 :
								(operate_cnt == 3'd7)? 1 : 0;
	assign to_mem_wr_req_addr = {32{state_MEM_WR}} & {tag_rdata[replace_position], index, 5'b0} |
								{32{state_COMEM_WR}} & cpu_address;
	assign to_mem_wr_data_strb = {4{state_MEM_WR | state_CACHE_SEND}} & 4'b1111 |
									{4{state_COMEM_WR | state_CPU_SEND}} & cpu_strb;
	assign to_mem_wr_req_len = {8{state_MEM_WR}} & 8'b111 |
									{8{state_COMEM_WR}} & 8'b0;

	always @ (posedge clk) begin
		if (state_RECV && from_mem_rd_rsp_valid) read_data[operate_cnt] <= from_mem_rd_rsp_data;
	end

	always @ (posedge clk) begin
		if (state_MEM_RECV & from_mem_rd_rsp_valid) bypass_read_data <= from_mem_rd_rsp_data;
	end

	assign to_cpu_cache_rsp_data = {32{state_CACHE_RESP}} & (data_hit >> {offset, 3'b0}) |
									{32{state_MEM_RESP}} & bypass_read_data;

	assign to_cpu_cache_rsp_valid = state_CACHE_RESP | state_MEM_RESP;
	assign to_mem_wr_req_valid = state_MEM_WR | state_COMEM_WR;
	assign to_mem_wr_data_valid = state_CACHE_SEND | state_CPU_SEND;
	assign to_cpu_mem_req_ready = state_WAIT;
	assign to_mem_rd_req_valid = state_MEM_RD | state_COMEM_RD;
	assign to_mem_rd_rsp_ready = state_RECV | state_MEM_RECV;

	assign to_mem_rd_req_len = {8{state_MEM_RD}} & 8'b111 |
								{8{state_COMEM_RD}} & 8'b0;
	assign to_mem_rd_req_addr = {32{state_MEM_RD}} & {cpu_address[31:5], 5'b0} |
								{32{state_COMEM_RD}} & cpu_address;

	always @ (posedge clk) begin
		if (rst || state_MEM_WR && from_mem_wr_req_ready || state_MEM_RD && from_mem_rd_req_ready)
			operate_cnt <= 3'b0;
		else if (state_CACHE_SEND && from_mem_wr_data_ready || state_RECV && from_mem_rd_rsp_valid)
			operate_cnt <= operate_cnt + 3'b1;
	end

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
		else if (state_CACHE_RESP && from_cpu_cache_rsp_ready || state_CACHE_WR)begin
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

endmodule
