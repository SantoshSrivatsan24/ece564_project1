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

	// TODO: Can onehot encoding reduce decode logic?
	localparam [3:0]
		STATE_INPUT_IDLE  			= 4'h0,
		STATE_INPUT_READ_SIZE  		= 4'h1,
		STATE_INPUT_CHECK_SIZE 		= 4'h2,
		STATE_INPUT_READ_ADDR1 		= 4'h3,
		STATE_INPUT_READ_ADDR2 		= 4'h4,
		STATE_INPUT_READ_ADDR3 		= 4'h5,
		STATE_INPUT_READ_ADDR4 		= 4'h6,
		STATE_INPUT_READ_ADDR5 		= 4'h7,
		STATE_INPUT_READ_ADDR6 		= 4'h8,
		STATE_INPUT_READ_ADDR7 		= 4'h9,
		STATE_INPUT_READ_ADDR8 		= 4'ha,
		STATE_INPUT_CHECK1  		= 4'hb,
		STATE_INPUT_CHECK2 			= 4'hc,
		STATE_INPUT_DONE 			= 4'hd;

	localparam [4:0]
		STATE_IBUF_EMPTY			= 5'h0,
		STATE_IBUF_FILL2			= 5'h1,
		STATE_IBUF_FILL4 			= 5'h2,
		STATE_IBUF_FILL6			= 5'h3,
		STATE_IBUF_FILL8			= 5'h4,
		STATE_IBUF_FILL10			= 5'h5,
		STATE_IBUF_FILL12			= 5'h6,
		STATE_IBUF_FILL14			= 5'h7,
		STATE_IBUF_MULT1			= 5'h8,
		STATE_IBUF_MULT2			= 5'ha,
		STATE_IBUF_MULT3			= 5'hb,
		STATE_IBUF_MULT4			= 5'hc,
		STATE_IBUF_MULT5			= 5'hd,
		STATE_IBUF_MULT6			= 5'he,
		STATE_IBUF_MULT7			= 5'hf,
		STATE_IBUF_MULT8			= 5'h10,
		STATE_IBUF_MULT9			= 5'h11;

	localparam [2:0]
		STATE_MATRIX_INCOMPLETE0	= 2'h0,
		STATE_MATRIX_INCOMPLETE1	= 2'h1,
		STATE_MATRIX_COMPLETE  		= 2'h2;

	localparam [2:0]
		STATE_RELU_STORE			= 1'b0,
		STATE_RELU_READY			= 1'b1;

	////////////////////////////////////////////////////////

	reg  [3:0] current_input_state;
	reg  [3:0] next_input_state;

	reg  [4:0] current_ibuf_state;
	reg  [4:0] next_ibuf_state;

	reg  [2:0] current_matrix_state;
	reg  [2:0] next_matrix_state;

	reg  [1:0] current_relu_state;
	reg  [1:0] next_relu_state;

	reg  [ADDRW-1:0] input_base_addr;
	wire [ADDRW-1:0] input_set_addr;
	wire [ADDRW-1:0] input_sram_raddr;

	reg  [DATAW-1:0] input_matrix_size;
	wire [7:0]	     N;
	wire [7:0]		 logN;

	reg  [7:0] row;
	reg  [7:0] col;
	wire [7:0] next_row;
	wire [7:0] next_col;

	reg  input_req_size;
	reg  input_req_valid;
	reg  input_req_fire;

	reg  input_data_size;
	reg  input_data_valid;

	reg  input_set_done;
	wire input_col_done;
	wire input_matrix_done;
	reg  input_matrix_done_r;

	reg  [7:0] ibuf [15:0]; // Shift register
	reg  ibuf_ready;
	reg  ibuf_fire;
	wire ibuf_push;
	reg  ibuf_pop;
	reg  ibuf_empty;
	reg  ibuf_multiply;
	reg  ibuf_set_done;
	reg  ibuf_matrix_done;

	reg  kernel_byteen;

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

	// Compute input SRAM read address
	assign next_col 	  = col + 8'h2;
	assign next_row 	  = row + 8'h2;

	assign input_col_done    = (next_col > (N - 2));
	assign input_matrix_done = (next_row > (N - 2));

	assign input_set_addr 	= ((row << logN) + col) >> 1; // (N x r + c) / 2
	// assign input_set_addr 	= ((row * N) + col) >> 1; // (N x r + c) / 2
	assign input_sram_raddr = input_base_addr + input_set_addr + 12'h1;

	always @(*) begin

	dut_busy				= 1'b1;
	input_req_valid 		= 1'b0;
	input_req_fire			= 1'b0;
	input_req_size 			= 1'b0;
	input_set_done 			= 1'b0;
	input_sram_read_address = 12'hx;

	casex (current_input_state)
	STATE_INPUT_IDLE: begin
		if (dut_run) begin
			next_input_state 	= STATE_INPUT_READ_SIZE;
		end else begin 
			next_input_state 	= STATE_INPUT_IDLE;
		end
		dut_busy 				= 1'b0;
	end

	STATE_INPUT_DONE: begin
		if (output_done) begin
			next_input_state 	= STATE_INPUT_IDLE;
		end else begin
			next_input_state 	= STATE_INPUT_DONE;
		end	
	end

	STATE_INPUT_READ_SIZE: begin
		next_input_state 		= STATE_INPUT_CHECK_SIZE;
		input_req_valid 		= 1'b1;
		input_req_size			= 1'b1;
		input_sram_read_address	= input_base_addr;
	end

	STATE_INPUT_CHECK_SIZE: begin
		if (input_sram_read_data == 16'hffff) begin
			next_input_state 	= STATE_INPUT_DONE;
		end else if (ibuf_empty) begin
			next_input_state 	= STATE_INPUT_READ_ADDR1;
		end else begin
			next_input_state	= STATE_INPUT_CHECK_SIZE;
		end
	end

	STATE_INPUT_READ_ADDR1: begin
		next_input_state		= STATE_INPUT_READ_ADDR2;
		input_req_valid 		= 1'b1;
		input_req_fire			= 1'b1;
		input_sram_read_address	= input_sram_raddr;
	end

	STATE_INPUT_READ_ADDR2: begin
		next_input_state		= STATE_INPUT_READ_ADDR3;
		input_req_valid			= 1'b1;
		input_sram_read_address	= input_sram_raddr + 1;
	end

	STATE_INPUT_READ_ADDR3: begin
		next_input_state 		= STATE_INPUT_READ_ADDR4;
		input_req_valid 		= 1'b1;
		input_sram_read_address	= input_sram_raddr + (N >> 1);
	end

	STATE_INPUT_READ_ADDR4: begin
		next_input_state 		= STATE_INPUT_READ_ADDR5;
		input_req_valid 		= 1'b1;
		input_sram_read_address	= input_sram_raddr + (N >> 1) + 1;
	end

	STATE_INPUT_READ_ADDR5: begin
		next_input_state 		= STATE_INPUT_READ_ADDR6;
		input_req_valid 		= 1'b1;
		input_sram_read_address	= input_sram_raddr + N;
	end

	STATE_INPUT_READ_ADDR6: begin
		next_input_state 		= STATE_INPUT_READ_ADDR7;
		input_req_valid 		= 1'b1;
		input_sram_read_address	= input_sram_raddr + N + 1;
	end

	STATE_INPUT_READ_ADDR7: begin
		next_input_state 		= STATE_INPUT_READ_ADDR8;
		input_req_valid 		= 1'b1;
		input_sram_read_address	= input_sram_raddr + (N + (N >> 1)); // (3 * N) / 2
	end

	STATE_INPUT_READ_ADDR8: begin
		next_input_state 		= STATE_INPUT_CHECK1;
		input_req_valid 		= 1'b1;
		input_sram_read_address	= input_sram_raddr + (N + (N >> 1) + 1);
		input_set_done			= 1'b1;
	end

	STATE_INPUT_CHECK1: begin
		next_input_state 		= STATE_INPUT_CHECK2;
	end

	STATE_INPUT_CHECK2: begin
		if (input_matrix_done) begin
			next_input_state 	= STATE_INPUT_READ_SIZE;
		end else if (ibuf_set_done) begin
			next_input_state 	= STATE_INPUT_READ_ADDR1;
		end else begin
			next_input_state	= STATE_INPUT_CHECK2;
		end
	end
	endcase
	end

	always @(posedge clk) begin
		if (~reset_b) begin
			current_input_state <= STATE_INPUT_IDLE;
		end else begin
			current_input_state <= next_input_state;
		end
	end

	always @(posedge clk) begin
		if (~reset_b) begin
			input_base_addr  <= 12'h0;
			row 		<= 8'h0;
			col 		<= 8'h0;
		end else begin
			if (input_set_done) begin
				col <= next_col;
			end

			if (input_col_done) begin
				col <= 8'h0;
				row <= next_row;
			end

			if (input_matrix_done) begin
				input_base_addr <= input_base_addr + ((N << logN) >> 1) + 1; // Go to the next input matrix
				// input_base_addr <= input_base_addr + ((N * N) >> 1) + 1; // Go to the next input matrix
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

	// STAGE 2 //////////////////////////////////////////////////////

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

	// Store input SRAM read data in a shift register
	assign ibuf_push 	= input_data_valid & ~input_data_size & ibuf_ready;
	assign N 			= input_matrix_size[7:0];
	assign logN 		= `CLOG2(N);

	always @(*) begin

	ibuf_ready 		= 1'b0;
	ibuf_pop 		= 1'b0;
	ibuf_empty 		= 1'b0;
	ibuf_set_done 	= 1'b0;
	ibuf_multiply 	= 1'b0;
	kernel_byteen 	= 1'bx;
	weights_sram_read_address = 12'hx;

	ibuf_out[0] = ibuf[15];
	ibuf_out[1] = ibuf[14];
	ibuf_out[2] = ibuf[11];
	ibuf_out[3] = ibuf[10];

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
		weights_sram_read_address = 12'h0;
	end

	STATE_IBUF_MULT1: begin
		next_ibuf_state	= STATE_IBUF_MULT2;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b1;
		weights_sram_read_address = 12'h0;
	end

	STATE_IBUF_MULT2: begin
		next_ibuf_state	= STATE_IBUF_MULT3;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b0;
		weights_sram_read_address = 12'h1;
	end

	STATE_IBUF_MULT3: begin
		next_ibuf_state	= STATE_IBUF_MULT4;
		ibuf_multiply 	= 1'b1;
		kernel_byteen 	= 1'b1;
		weights_sram_read_address = 12'h1;
	end

	STATE_IBUF_MULT4: begin
		next_ibuf_state	= STATE_IBUF_MULT5;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b0;
		weights_sram_read_address = 12'h2;
	end

	STATE_IBUF_MULT5: begin
		next_ibuf_state	= STATE_IBUF_MULT6;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b1;
		weights_sram_read_address = 12'h2;
	end

	STATE_IBUF_MULT6: begin
		next_ibuf_state	= STATE_IBUF_MULT7;
		ibuf_multiply 	= 1'b1;
		kernel_byteen 	= 1'b0;
		weights_sram_read_address = 12'h3;
	end

	STATE_IBUF_MULT7: begin
		next_ibuf_state	= STATE_IBUF_MULT8;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b1;
		weights_sram_read_address = 12'h3;
	end

	STATE_IBUF_MULT8: begin
		next_ibuf_state	= STATE_IBUF_MULT9;
		ibuf_multiply 	= 1'b1;
		ibuf_pop		= 1'b1;
		kernel_byteen 	= 1'b0;
		weights_sram_read_address = 12'h4;
	end

	STATE_IBUF_MULT9: begin
		next_ibuf_state	= STATE_IBUF_EMPTY;
		ibuf_multiply 	= 1'b1;
		ibuf_set_done	= 1'b1;
		kernel_byteen 	= 1'b1;
	end
	endcase
	end

	always @(*) begin
	ibuf_matrix_done = 1'b0;

	casex (current_matrix_state)
	STATE_MATRIX_INCOMPLETE0: begin
		if (input_matrix_done_r) begin
			next_matrix_state 	= STATE_MATRIX_INCOMPLETE1;
		end
	end

	STATE_MATRIX_INCOMPLETE1: begin
		if (ibuf_set_done) begin
			next_matrix_state 	= STATE_MATRIX_COMPLETE;
		end
	end

	STATE_MATRIX_COMPLETE: begin
		next_matrix_state 		= STATE_MATRIX_INCOMPLETE0;
		ibuf_matrix_done 		= 1'b1;
	end
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
				for (i = 16; i > 0; i = i - 1) begin
					ibuf[i] <= ibuf[i - 1]; // Shift one value
				end
			end else begin
				for (i = 16; i > 0; i = i - 1) begin
					ibuf[i] <= ibuf[i - 2]; // Shift two values
				end
			end

			if (ibuf_push) begin
				ibuf[0] <= input_sram_read_data[0 +: 8];
				ibuf[1] <= input_sram_read_data[8 +: 8];
			end 
		end
	end

	// STAGE 3: Convolution //////////////////////////////////////////////////////

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

	// STAGE 4: Max Pool  //////////////////////////////////////////////////////

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

	assign max_pool1 = (macc[0] > macc[1]) ? macc[0] : macc[1];
	assign max_pool2 = (macc[2] > macc[3]) ? macc[2] : macc[3];
	assign max_pool = (max_pool1 > max_pool2) ? max_pool1 : max_pool2;

	// STAGE 5: ReLu //////////////////////////////////////////////////////////

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

	assign relu = max_pool_r[19] ? 8'h0 : (max_pool_r > 8'h7f) ? 8'h7f : {1'b0, max_pool_r[0 +: 7]};
	assign relu_fire = relu_valid & relu_ready;

	always @(*) begin
	casex (current_relu_state)
	STATE_RELU_STORE: begin
		if (relu_valid) begin
			next_relu_state = STATE_RELU_READY;
		end else begin
			next_relu_state = STATE_RELU_STORE;
		end
		relu_store = 1'b1;
		relu_ready = 1'b0;
	end

	STATE_RELU_READY: begin
		if (relu_valid | relu_matrix_done) begin
			next_relu_state = STATE_RELU_STORE;
		end else begin
			next_relu_state = STATE_RELU_READY;
		end
		relu_store = 1'b0;
		relu_ready = 1'b1;
	end
	endcase
	end

	// Write to output SRAM
	always @(posedge clk) begin
		if (~reset_b) begin
			current_relu_state 			<= STATE_RELU_STORE;
			output_sram_write_enable 	<= 1'b0;
			output_sram_write_addresss 	<= 12'hfff;
			output_sram_write_data 		<= 16'h0;
		end else begin

			if (~dut_busy) begin
				output_sram_write_addresss <= 12'hfff;
			end

			current_relu_state 			<= next_relu_state;
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

	// STAGE 6 //////////////////////////////////////////////////////////



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
			$display ("@%0h \t %h", output_sram_write_addresss, output_sram_write_data);
		end

		if (output_matrix_done) begin
			$display ("\noutput matrix done!\n");
		end
	end


endmodule

