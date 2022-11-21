`define CLOG2(x) \
	(x <= 8'd2) ? 8'd1 : \
	(x <= 8'd4) ? 8'd2 : \
	(x <= 8'd8) ? 8'd3 : \
	(x <= 8'd16) ? 8'd4 : \
	(x <= 8'd32) ? 8'd5 : \
	(x <= 8'd64) ? 8'd6 : \
	(x <= 8'd128) ? 8'd7 : \
	-1

module MyDesign (
//---------------------------------------------------------------------------
//Control signals
  input   wire dut_run                    , 
  output  reg dut_busy                   ,
  input   wire reset_b                    ,  
  input   wire clk                        ,
 
//---------------------------------------------------------------------------
//Input SRAM interface
  output reg        input_sram_write_enable    ,
  output reg [11:0] input_sram_write_addresss  ,
  output reg [15:0] input_sram_write_data      ,
  output reg [11:0] input_sram_read_address    ,
  input wire [15:0] input_sram_read_data       ,

//---------------------------------------------------------------------------
//Output SRAM interface
  output reg        output_sram_write_enable    ,
  output reg [11:0] output_sram_write_addresss  ,
  output reg [15:0] output_sram_write_data      ,
  output reg [11:0] output_sram_read_address    ,
  input wire [15:0] output_sram_read_data       ,

//---------------------------------------------------------------------------
//Scratchpad SRAM interface
  output reg        scratchpad_sram_write_enable    ,
  output reg [11:0] scratchpad_sram_write_addresss  ,
  output reg [15:0] scratchpad_sram_write_data      ,
  output reg [11:0] scratchpad_sram_read_address    ,
  input wire [15:0] scratchpad_sram_read_data       ,

//---------------------------------------------------------------------------
//Weights SRAM interface                                                       
  output reg        weights_sram_write_enable    ,
  output reg [11:0] weights_sram_write_addresss  ,
  output reg [15:0] weights_sram_write_data      ,
  output reg [11:0] weights_sram_read_address    ,
  input wire [15:0] weights_sram_read_data       

);

	localparam ADDRW = 12;
	localparam DATAW = 16;

	localparam [13:0]
		STATE_INPUT_IDLE  			= 14'b00_0000_0000_0001,
		STATE_INPUT_READ_SIZE  		= 14'b00_0000_0000_0010,
		STATE_INPUT_CHECK_SIZE 		= 14'b00_0000_0000_0100,
		STATE_INPUT_READ_ADDR1 		= 14'b00_0000_0000_1000,
		STATE_INPUT_READ_ADDR2 		= 14'b00_0000_0001_0000,
		STATE_INPUT_READ_ADDR3 		= 14'b00_0000_0010_0000,
		STATE_INPUT_READ_ADDR4 		= 14'b00_0000_0100_0000,
		STATE_INPUT_READ_ADDR5 		= 14'b00_0000_1000_0000,
		STATE_INPUT_READ_ADDR6 		= 14'b00_0001_0000_0000,
		STATE_INPUT_READ_ADDR7 		= 14'b00_0010_0000_0000,
		STATE_INPUT_READ_ADDR8 		= 14'b00_0100_0000_0000,
		STATE_INPUT_CHECK1  		= 14'b00_1000_0000_0000,
		STATE_INPUT_CHECK2 			= 14'b01_0000_0000_0000,
		STATE_INPUT_DONE 			= 14'b10_0000_0000_0000;

	localparam [16:0]
		STATE_IBUF_EMPTY			= 17'b0_0000_0000_0000_0001,
		STATE_IBUF_FILL2			= 17'b0_0000_0000_0000_0010,
		STATE_IBUF_FILL4 			= 17'b0_0000_0000_0000_0100,
		STATE_IBUF_FILL6			= 17'b0_0000_0000_0000_1000,
		STATE_IBUF_FILL8			= 17'b0_0000_0000_0001_0000,
		STATE_IBUF_FILL10			= 17'b0_0000_0000_0010_0000,
		STATE_IBUF_FILL12			= 17'b0_0000_0000_0100_0000,
		STATE_IBUF_FILL14			= 17'b0_0000_0000_1000_0000,
		STATE_IBUF_MULT1			= 17'b0_0000_0001_0000_0000,
		STATE_IBUF_MULT2			= 17'b0_0000_0010_0000_0000,
		STATE_IBUF_MULT3			= 17'b0_0000_0100_0000_0000,
		STATE_IBUF_MULT4			= 17'b0_0000_1000_0000_0000,
		STATE_IBUF_MULT5			= 17'b0_0001_0000_0000_0000,
		STATE_IBUF_MULT6			= 17'b0_0010_0000_0000_0000,
		STATE_IBUF_MULT7			= 17'b0_0100_0000_0000_0000,
		STATE_IBUF_MULT8			= 17'b0_1000_0000_0000_0000,
		STATE_IBUF_MULT9			= 17'b1_0000_0000_0000_0000;

	localparam [2:0]
		STATE_MATRIX_INCOMPLETE0	= 3'b001,
		STATE_MATRIX_INCOMPLETE1	= 3'b010,
		STATE_MATRIX_COMPLETE  		= 3'b100;

	localparam [2:0]
		STATE_RELU_STORE			= 1'b0,
		STATE_RELU_READY			= 1'b1;

	////////////////////////////////////////////////////////

	reg  [13:0] current_input_state;
	reg  [13:0] next_input_state;

	reg  [16:0] current_ibuf_state;
	reg  [16:0] next_ibuf_state;

	reg  [2:0] current_matrix_state;
	reg  [2:0] next_matrix_state;

	reg  [1:0] current_relu_state;
	reg  [1:0] next_relu_state;

	// Stage 1: Control
	reg  input_req_size;
	reg  input_req_valid;
	reg  input_req_fire;
	reg  input_set_done;
	wire input_col_done;
	wire input_matrix_done;
	reg  input_matrix_done_r;
	reg [4:0] input_addr_sel;

	// Stage 1: Datapath
	reg  [ADDRW-1:0] input_base_addr;
	wire [ADDRW-1:0] input_set_addr;
	wire [ADDRW-1:0] input_sram_raddr;

	reg  [DATAW-1:0] input_matrix_size;
	wire [7:0]	     N;
	wire [7:0]		 logN;

	reg  [7:0] row;
	reg  [7:0] col;

	// Stage 2: Control
	reg  ibuf_ready;
	reg  ibuf_fire;
	wire ibuf_push;
	reg  ibuf_pop;
	reg  ibuf_empty;
	reg  ibuf_multiply;
	reg  ibuf_set_done;
	reg  ibuf_matrix_done;

	reg [2:0] kernel_addr_sel;
	reg  kernel_byteen;

	// Stage 2: Datapath
	reg  [7:0] ibuf [15:0]; 
	reg  input_data_size;
	reg  input_data_valid;

	// Stage 3: Control
	reg  conv_valid;
	reg  conv_matrix_done;

	reg  signed [7:0]  ibuf_out [3:0];
	wire signed [7:0]  kernel_rdata;
	reg  signed [19:0] macc [3:0];

	wire signed [19:0] max_pool1;
	wire signed [19:0] max_pool2;
	wire signed [19:0] max_pool;
	reg         [19:0] max_pool_r;
	reg  			   max_pool_valid;
	reg                max_pool_matrix_done;

	wire [7:0] relu;
	reg  [7:0] relu_old;
	reg        relu_valid;
	reg        relu_ready;
	wire       relu_fire;
	reg    	   relu_matrix_done;
	reg        relu_store;
	
	reg output_matrix_done;
	reg output_done;

	integer i;

	// STAGE 1 //////////////////////////////////////////////////////

	// Controller

	always @(*) begin
	dut_busy				= 1'b1;
	input_req_valid 		= 1'b0;
	input_req_fire			= 1'b0;
	input_req_size 			= 1'b0;
	input_set_done 			= 1'b0;
	input_addr_sel			= 4'hx;

	casex (current_input_state)
	STATE_INPUT_IDLE: begin
		if (dut_run) begin
			next_input_state = STATE_INPUT_READ_SIZE;
		end else begin 
			next_input_state = STATE_INPUT_IDLE;
		end
		dut_busy 			 = 1'b0;
	end

	STATE_INPUT_DONE: begin
		if (output_done) begin
			next_input_state = STATE_INPUT_IDLE;
		end else begin
			next_input_state = STATE_INPUT_DONE;
		end	
	end

	STATE_INPUT_READ_SIZE: begin
		next_input_state 	= STATE_INPUT_CHECK_SIZE;
		input_req_valid 	= 1'b1;
		input_req_size		= 1'b1;
		input_addr_sel		= 4'b0000;
	end

	STATE_INPUT_CHECK_SIZE: begin
		if (input_sram_read_data == 16'hffff) begin
			next_input_state = STATE_INPUT_DONE;
		end else if (ibuf_empty) begin
			next_input_state = STATE_INPUT_READ_ADDR1;
		end else begin
			next_input_state = STATE_INPUT_CHECK_SIZE;
		end
	end

	STATE_INPUT_READ_ADDR1: begin
		next_input_state	= STATE_INPUT_READ_ADDR2;
		input_req_valid 	= 1'b1;
		input_req_fire		= 1'b1;
		input_addr_sel		= 4'b0001;
	end

	STATE_INPUT_READ_ADDR2: begin
		next_input_state	= STATE_INPUT_READ_ADDR3;
		input_req_valid		= 1'b1;
		input_addr_sel		= 4'b0010;
	end

	STATE_INPUT_READ_ADDR3: begin
		next_input_state 	= STATE_INPUT_READ_ADDR4;
		input_req_valid 	= 1'b1;
		input_addr_sel		= 4'b0011;
	end

	STATE_INPUT_READ_ADDR4: begin
		next_input_state 	= STATE_INPUT_READ_ADDR5;
		input_req_valid 	= 1'b1;
		input_addr_sel		= 4'b0100;
	end

	STATE_INPUT_READ_ADDR5: begin
		next_input_state 	= STATE_INPUT_READ_ADDR6;
		input_req_valid 	= 1'b1;
		input_addr_sel		= 4'b0101;
	end

	STATE_INPUT_READ_ADDR6: begin
		next_input_state 	= STATE_INPUT_READ_ADDR7;
		input_req_valid 	= 1'b1;
		input_addr_sel		= 4'b0110;
	end

	STATE_INPUT_READ_ADDR7: begin
		next_input_state 	= STATE_INPUT_READ_ADDR8;
		input_req_valid 	= 1'b1;
		input_addr_sel		= 4'b0111;
	end

	STATE_INPUT_READ_ADDR8: begin
		next_input_state 	= STATE_INPUT_CHECK1;
		input_req_valid 	= 1'b1;
		input_addr_sel		= 4'b1000;
		input_set_done		= 1'b1;
	end

	STATE_INPUT_CHECK1: begin
		next_input_state 	= STATE_INPUT_CHECK2;
	end

	STATE_INPUT_CHECK2: begin
		if (input_matrix_done) begin
			next_input_state = STATE_INPUT_READ_SIZE;
		end else if (ibuf_set_done) begin
			next_input_state = STATE_INPUT_READ_ADDR1;
		end else begin
			next_input_state = STATE_INPUT_CHECK2;
		end
	end

	default: next_input_state = STATE_INPUT_IDLE;
	endcase
	end

	assign input_col_done    = ((col + 8'h2) > (N - 8'h2));
	assign input_matrix_done = ((row + 8'h2) > (N - 8'h2));

	always @(posedge clk) begin
		if (~reset_b) begin
			current_input_state <= STATE_INPUT_IDLE;
		end else begin
			current_input_state <= next_input_state;
		end
	end

	// Datapath

	assign input_set_addr 	= ((row << logN) + col) >> 1; // (N x r + c) / 2
	assign input_sram_raddr = input_base_addr + input_set_addr + 12'h1;

	always @(*) begin
	casex (input_addr_sel)
		4'b0000: input_sram_read_address = input_base_addr;
		4'b0001: input_sram_read_address = input_sram_raddr;
		4'b0010: input_sram_read_address = input_sram_raddr + 1;
		4'b0011: input_sram_read_address = input_sram_raddr + (N >> 1);
		4'b0100: input_sram_read_address = input_sram_raddr + (N >> 1) + 1;
		4'b0101: input_sram_read_address = input_sram_raddr + N;
		4'b0110: input_sram_read_address = input_sram_raddr + N + 1;
		4'b0111: input_sram_read_address = input_sram_raddr + (N + (N >> 1)); // (3 * N) / 2
		4'b1000: input_sram_read_address = input_sram_raddr + (N + (N >> 1) + 1);
		default: input_sram_read_address = 4'bxxxx;
	endcase
	end

	always @(posedge clk) begin
		if (~reset_b) begin
			input_base_addr  <= 12'h0;
			row <= 8'h0;
			col <= 8'h0;
		end else begin
			if (input_set_done) begin
				col <= col + 8'h2;
			end

			if (input_col_done) begin
				col <= 8'h0;
				row <= row + 8'h2;
			end

			if (input_matrix_done) begin
				input_base_addr <= input_base_addr + ((N << logN) >> 1) + 1; // Go to the next input matrix
				row <= 8'h0;
				col <= 8'h0;
			end

			// Reset input address once we're done with all input matrices
			if (~dut_busy) begin
				input_base_addr <= 12'h0;
				row <= 8'h0;
				col <= 8'h0;
			end
		end
	end

	// Pipeline Register
	always @(posedge clk) begin : pipe_reg_st_1_to_2
		if (~reset_b) begin
			input_data_valid 	<= 1'b0;
			input_data_size 	<= 1'b0;
			input_matrix_done_r <= 1'b0;
			ibuf_fire           <= 1'b0;
		end else begin
			input_data_valid 	<= input_req_valid;
			input_data_size 	<= input_req_size;
			input_matrix_done_r <= input_matrix_done;
			ibuf_fire 			<= input_req_fire;
		end
	end

	// STAGE 2 //////////////////////////////////////////////////////

	// Controller

	always @(*) begin
	ibuf_ready 		= 1'b0;
	ibuf_pop 		= 1'b0;
	ibuf_empty 		= 1'b0;
	ibuf_set_done 	= 1'b0;
	ibuf_multiply 	= 1'b0;
	kernel_byteen 	= 1'bx;
	kernel_addr_sel = 3'hx;

	casex (current_ibuf_state)	
	STATE_IBUF_EMPTY: begin
		if (ibuf_fire) begin
			next_ibuf_state = STATE_IBUF_FILL2;
		end else begin
			next_ibuf_state = STATE_IBUF_EMPTY;
		end
		ibuf_ready	= 1'b1;
		ibuf_empty 	= 1'b1;
	end

	STATE_IBUF_FILL2: begin
		next_ibuf_state = STATE_IBUF_FILL4;
		ibuf_ready 		= 1'b1;
	end

	STATE_IBUF_FILL4: begin
		next_ibuf_state = STATE_IBUF_FILL6;
		ibuf_ready 		= 1'b1;
	end

	STATE_IBUF_FILL6: begin
		next_ibuf_state = STATE_IBUF_FILL8;
		ibuf_ready 		= 1'b1;
	end

	STATE_IBUF_FILL8: begin
		next_ibuf_state = STATE_IBUF_FILL10;
		ibuf_ready 		= 1'b1;
	end

	STATE_IBUF_FILL10: begin
		next_ibuf_state = STATE_IBUF_FILL12;
		ibuf_ready 		= 1'b1;
	end

	STATE_IBUF_FILL12: begin
		next_ibuf_state = STATE_IBUF_FILL14;
		ibuf_ready 		= 1'b1;
	end

	STATE_IBUF_FILL14: begin
		next_ibuf_state = STATE_IBUF_MULT1;
		ibuf_ready 		= 1'b1;
		kernel_addr_sel = 3'b000;
	end

	STATE_IBUF_MULT1: begin
		next_ibuf_state	= STATE_IBUF_MULT2;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b1;
		kernel_addr_sel = 3'b000;
	end

	STATE_IBUF_MULT2: begin
		next_ibuf_state	= STATE_IBUF_MULT3;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b0;
		kernel_addr_sel = 3'b001;
	end

	STATE_IBUF_MULT3: begin
		next_ibuf_state	= STATE_IBUF_MULT4;
		ibuf_multiply 	= 1'b1;
		kernel_byteen 	= 1'b1;
		kernel_addr_sel = 3'b001;
	end

	STATE_IBUF_MULT4: begin
		next_ibuf_state	= STATE_IBUF_MULT5;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b0;
		kernel_addr_sel = 3'b010;
	end

	STATE_IBUF_MULT5: begin
		next_ibuf_state	= STATE_IBUF_MULT6;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b1;
		kernel_addr_sel = 3'b010;
	end

	STATE_IBUF_MULT6: begin
		next_ibuf_state	= STATE_IBUF_MULT7;
		ibuf_multiply 	= 1'b1;
		kernel_byteen 	= 1'b0;
		kernel_addr_sel = 3'b011;
	end

	STATE_IBUF_MULT7: begin
		next_ibuf_state	= STATE_IBUF_MULT8;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b1;
		kernel_addr_sel = 3'b011;
	end

	STATE_IBUF_MULT8: begin
		next_ibuf_state	= STATE_IBUF_MULT9;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b0;
		kernel_addr_sel = 3'b100;
	end

	STATE_IBUF_MULT9: begin
		next_ibuf_state	= STATE_IBUF_EMPTY;
		ibuf_multiply 	= 1'b1;
		ibuf_set_done	= 1'b1;
		kernel_byteen 	= 1'b1;
	end

	default: next_ibuf_state = STATE_IBUF_EMPTY;
	endcase
	end

	always @(*) begin
	ibuf_matrix_done = 1'b0;

	casex (current_matrix_state)
	STATE_MATRIX_INCOMPLETE0: begin
		if (input_matrix_done_r) begin
			next_matrix_state 	= STATE_MATRIX_INCOMPLETE1;
		end else begin
			next_matrix_state	= STATE_MATRIX_INCOMPLETE0;
		end
	end

	STATE_MATRIX_INCOMPLETE1: begin
		if (ibuf_set_done) begin
			next_matrix_state 	= STATE_MATRIX_COMPLETE;
		end else begin
			next_matrix_state	= STATE_MATRIX_INCOMPLETE1;
		end
	end

	STATE_MATRIX_COMPLETE: begin
		next_matrix_state 		= STATE_MATRIX_INCOMPLETE0;
		ibuf_matrix_done 		= 1'b1;
	end

	default: next_matrix_state = STATE_MATRIX_INCOMPLETE0;
	endcase
	end

	always @(posedge clk) begin
		if (~reset_b) begin
			current_ibuf_state <= STATE_IBUF_EMPTY;
			current_matrix_state <= STATE_MATRIX_INCOMPLETE0;
		end else begin
			current_ibuf_state <= next_ibuf_state;
			current_matrix_state <= next_matrix_state;
		end
	end

	// Datapath

	// Store input SRAM read data in a shift register
	assign ibuf_push 	= input_data_valid & ~input_data_size & ibuf_ready;
	assign N 			= input_matrix_size[7:0];
	assign logN 		= `CLOG2(N);

	always @(*) begin
		ibuf_out[0] = $signed(ibuf[15]);
		ibuf_out[1] = $signed(ibuf[14]);
		ibuf_out[2] = $signed(ibuf[11]);
		ibuf_out[3] = $signed(ibuf[10]);

		casex (kernel_addr_sel) 
			3'b000: weights_sram_read_address = 12'h0;
			3'b001: weights_sram_read_address = 12'h1;
			3'b010: weights_sram_read_address = 12'h2;
			3'b011: weights_sram_read_address = 12'h3;
			3'b100: weights_sram_read_address = 12'h4;
			default: weights_sram_read_address = 12'hx;
		endcase
	end

	always @(posedge clk) begin
		if (~reset_b) begin
			input_matrix_size <= 16'h0;
			for (i = 0; i < 16; i = i + 1) begin
				ibuf[i] <= 8'h0;
			end
		end else begin
			if (input_data_size) begin
				input_matrix_size <= input_sram_read_data;
			end

			if (ibuf_pop) begin
				for (i = 15; i > 0; i = i - 1) begin
					ibuf[i] <= ibuf[i - 1]; // Shift one value
				end
			end else begin
				for (i = 15; i > 0; i = i - 1) begin
					ibuf[i] <= ibuf[i - 2]; // Shift two values
				end
			end

			if (ibuf_push) begin
				ibuf[0] <= input_sram_read_data[0 +: 8];
				ibuf[1] <= input_sram_read_data[8 +: 8];
			end 
		end
	end

	// Pipeline Register
	always @(posedge clk) begin : pipe_reg_st_2_to_3
		if (~reset_b) begin
			conv_valid 			<= 1'b0;
			conv_matrix_done 	<= 1'b0;
		end else begin
			conv_valid 			<= ibuf_set_done;
			conv_matrix_done 	<= ibuf_matrix_done;
		end
	end

	// STAGE 3: Convolution //////////////////////////////////////////////////////

	// Datapath

	assign kernel_rdata = kernel_byteen ? weights_sram_read_data[8 +: 8] : weights_sram_read_data[0 +: 8];

	genvar j;
	generate
		for (j = 0; j < 4; j = j + 1) begin : muladd
			wire [19:0] mult = kernel_rdata  * ibuf_out[j]; // Multiply

			always @(posedge clk) begin
				if (~reset_b) begin
					macc[j] <= 20'h0;
				end else begin
					if (ibuf_empty) begin
						macc[j] <= 20'h0;
					end
					if (ibuf_multiply) begin
						macc[j] <= macc[j] + mult; // Accumulate
					end
				end
			end
		end
	endgenerate

	// Pipeline Register
	always @(posedge clk) begin : pipe_reg_st_3_to_4
		if (~reset_b) begin
			max_pool_valid 			<= 1'b0;
			max_pool_matrix_done 	<= 1'b0;
		end else begin
			max_pool_valid 			<= conv_valid;
			max_pool_matrix_done 	<= conv_matrix_done;
		end
	end

	// STAGE 4: Max Pool  //////////////////////////////////////////////////////

	assign max_pool1 = (macc[0] > macc[1]) ? macc[0] : macc[1];
	assign max_pool2 = (macc[2] > macc[3]) ? macc[2] : macc[3];
	assign max_pool = (max_pool1 > max_pool2) ? max_pool1 : max_pool2;

	// Pipeline Register
	always @(posedge clk) begin : pipe_reg_st_4_to_5
		if (~reset_b) begin
			max_pool_r 			<= 20'h0;
			relu_valid 			<= 1'b0;
			relu_matrix_done 	<= 1'b0;
		end else begin
			max_pool_r 			<= max_pool;
			relu_valid 			<= conv_valid;
			relu_matrix_done 	<= conv_matrix_done;
		end
	end

	// STAGE 5: ReLu //////////////////////////////////////////////////////////

	// Controller

	always @(*) begin
	relu_store		= 1'b0;
	relu_ready		= 1'b0;

	casex (current_relu_state)
	STATE_RELU_STORE: begin
		if (relu_valid) begin
			next_relu_state = STATE_RELU_READY;
		end else begin
			next_relu_state = STATE_RELU_STORE;
		end
		relu_store = 1'b1;
	end

	STATE_RELU_READY: begin
		if (relu_valid | relu_matrix_done) begin
			next_relu_state = STATE_RELU_STORE;
		end else begin
			next_relu_state = STATE_RELU_READY;
		end
		relu_ready = 1'b1;
	end

	default: next_relu_state = STATE_RELU_STORE;
	endcase
	end

	always @(posedge clk) begin
		if (~reset_b) begin
			current_relu_state 			<= STATE_RELU_STORE;
		end else begin
			current_relu_state 			<= next_relu_state;
		end
	end

	// Datapath

	assign relu = max_pool_r[19] ? 8'h0 : (max_pool_r > 8'h7f) ? 8'h7f : {1'b0, max_pool_r[0 +: 7]};
	assign relu_fire = relu_valid & relu_ready;

	// Write to output SRAM
	always @(posedge clk) begin
		if (~reset_b) begin
			output_sram_write_enable 	<= 1'b0;
			output_sram_write_addresss 	<= 12'hfff;
			output_sram_write_data 		<= 16'h0;
		end else begin

			if (~dut_busy) begin
				output_sram_write_addresss <= 12'hfff;
			end

			output_matrix_done 			<= relu_matrix_done;
			output_done 				<= output_matrix_done;
			output_sram_write_enable    <= relu_fire | relu_matrix_done;

			if (relu_store & relu_valid) begin
				relu_old <= relu;
			end

			if (relu_fire | relu_matrix_done) begin
				output_sram_write_addresss 	<= output_sram_write_addresss + 12'b1;
			end

			if (relu_fire) begin
				output_sram_write_data 		<= {relu_old, relu};
			end	

			if (relu_matrix_done) begin
				output_sram_write_data 		<= {relu_old, 8'h0};
			end
		end
	end

	// DEBUGGING //////////////////////////////////////////////////////

	always @(posedge clk) begin

		// if (input_data_valid) begin
		// 	$display ("%d: SRAM raddr = %h, SRAM rdata = %h", $time, input_sram_read_address, input_sram_read_data);
		// 	$display ("ibuf state = %0d", current_ibuf_state);
		// 	for (i = 0; i < 16; i = i + 1) begin
		// 		$display ("ibuf[%0d] = %h", i, ibuf[i]);
		// 	end
		// end
		
		// if (ibuf_set_done) begin
		// 	$display ("ibuf_out_0 = %h, ibuf_out1 = %h, ibuf_out2 = %h, ibuf_out3 = %h", ibuf_out[0], ibuf_out[1], ibuf_out[2], ibuf_out[3]);
		// end

		// if (ibuf_matrix_done) begin
		// 	$display ("\nmatrix multiply done!\n");
		// end

		// if (conv_valid) begin
		// 	for (i = 0; i < 4; i = i + 1) begin
		// 		$display ("macc[%0d] = %h", i, macc[i]);
		// 	end	
		// end

		// if (conv_valid) begin
		// 	$display ("max_pool = %d", max_pool);
		// end

		// if (relu_valid) begin
		// 	$display ("relu = %d", relu);
		// end

		if (output_sram_write_enable) begin
			$display ("@%0h  %h", output_sram_write_addresss, output_sram_write_data);
		end

		if (output_matrix_done) begin
			$display ("\noutput matrix done!\n");
		end
	end


endmodule

