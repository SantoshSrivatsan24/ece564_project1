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
		STATE_INPUT_CHECK2 			= 4'hc;

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

	////////////////////////////////////////////////////////

	reg  [ADDRW-1:0] base_addr;
	wire [ADDRW-1:0] set_addr;
	wire [ADDRW-1:0] start_addr;

	reg  [7:0] row;
	reg  [7:0] col;
	wire [7:0] next_row;
	wire [7:0] next_col;

	// Control signals
	reg  is_size_st0;
	reg  is_size_st1;
	reg  is_valid_st0;
	reg  is_valid_st1;
	reg  set_done;
	wire col_done;
	wire matrix_done;

	reg  [DATAW-1:0] size;
	wire [7:0]	     N;

	reg  [7:0] ibuf [15:0]; // Shift register
	reg  signed [7:0] ibuf_out [3:0];
	reg  ibuf_ready;
	reg  ibuf_pop; // Pop one entry from ibuf instead of 2 entries
	reg  do_multiply;
	reg  set_done_st1;
	wire ibuf_push;

	integer i;

	////////////////////////////////////////////////////////

	// STAGE 0

	//  Keep track of which row and column we're on

	always @(posedge clk) begin

		if (~reset_b) begin
			base_addr  	<= 12'h0;
			row 		<= 8'h0;
			col 		<= 8'h0;
		end else begin

			if (set_done) begin
				col <= next_col;
			end

			if (col_done) begin
				col <= 8'h0;
				row <= next_row;
			end

			if (matrix_done) begin
				// TODO: Check if it's ok to use $clog2
				base_addr <= base_addr + ((N << $clog2(N)) >> 1) + 1; // Go to the next input matrix
				row <= 8'h0;
				col <= 8'h0;
			end
		end
	end

	assign next_col 	= col + 8'h2;
	assign next_row 	= row + 8'h2;
	assign col_done 	= (next_col > (N - 2));
	assign matrix_done 	= (next_row > (N - 2)); // Input to FSM

	assign set_addr 	= ((row << $clog2(N)) + col) >> 1; // (N x r + c) / 2
	assign start_addr 	= base_addr + set_addr + 12'h1;

	////////////////////////////////////////////////////////

	// Compute input SRAM read address

	reg [3:0] current_input_state;
	reg [3:0] next_input_state;

	always @(posedge clk) begin
		if (~reset_b) begin
			current_input_state <= STATE_INPUT_IDLE;
		end else begin
			current_input_state <= next_input_state;
		end
	end

	always @(*) begin

	dut_busy				= 1'b1;
	is_valid_st0 			= 1'b0;
	is_size_st0 			= 1'b0;
	set_done 				= 1'b0;
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

	STATE_INPUT_READ_SIZE: begin
		next_input_state 		= STATE_INPUT_CHECK_SIZE;
		is_valid_st0 			= 1'b1;
		is_size_st0				= 1'b1;
		input_sram_read_address	= base_addr;
	end

	STATE_INPUT_CHECK_SIZE: begin
		if (input_sram_read_data == 16'hff) begin
			next_input_state 	= STATE_INPUT_IDLE;
		end else begin
			next_input_state 	= STATE_INPUT_READ_ADDR1;
		end
	end

	STATE_INPUT_READ_ADDR1: begin
		if (ibuf_ready) begin
			next_input_state 	= STATE_INPUT_READ_ADDR2;
		end else begin
			next_input_state	= STATE_INPUT_READ_ADDR1;
		end
		is_valid_st0 			= 1'b1;
		input_sram_read_address	= start_addr;
	end

	STATE_INPUT_READ_ADDR2: begin
		next_input_state 		= STATE_INPUT_READ_ADDR3;
		is_valid_st0			= 1'b1;
		input_sram_read_address	= start_addr + 1;
	end

	STATE_INPUT_READ_ADDR3: begin
		next_input_state 		= STATE_INPUT_READ_ADDR4;
		is_valid_st0 			= 1'b1;
		input_sram_read_address	= start_addr + (N >> 1);
	end

	STATE_INPUT_READ_ADDR4: begin
		next_input_state 		= STATE_INPUT_READ_ADDR5;
		is_valid_st0 			= 1'b1;
		input_sram_read_address	= start_addr + (N >> 1) + 1;
	end

	STATE_INPUT_READ_ADDR5: begin
		next_input_state 		= STATE_INPUT_READ_ADDR6;
		is_valid_st0 			= 1'b1;
		input_sram_read_address	= start_addr + N;
	end

	STATE_INPUT_READ_ADDR6: begin
		next_input_state 		= STATE_INPUT_READ_ADDR7;
		is_valid_st0 			= 1'b1;
		input_sram_read_address	= start_addr + N + 1;
	end

	STATE_INPUT_READ_ADDR7: begin
		next_input_state 		= STATE_INPUT_READ_ADDR8;
		is_valid_st0 			= 1'b1;
		input_sram_read_address	= start_addr + (N + (N >> 1)); // (3 * N) / 2
	end

	STATE_INPUT_READ_ADDR8: begin
		next_input_state 		= STATE_INPUT_CHECK1;
		is_valid_st0 			= 1'b1;
		input_sram_read_address	= start_addr + (N + (N >> 1) + 1);
		set_done				= 1'b1;
	end

	STATE_INPUT_CHECK1: begin
		next_input_state 		= STATE_INPUT_CHECK2;
	end

	STATE_INPUT_CHECK2: begin
		if (matrix_done) begin
			next_input_state 	= STATE_INPUT_READ_SIZE;
		end else begin
			next_input_state 	= STATE_INPUT_READ_ADDR1;
		end
	end
			
	endcase
	end

	////////////////////////////////////////////////////////

	// Pipeline Register: Stage 0 -> Stage 1

	always @(posedge clk) begin : pipe_reg_st_0_to_1

		if (~reset_b) begin
			is_valid_st1 	<= 1'b0;
			is_size_st1 	<= 1'b0;
		end else begin
			is_valid_st1 	<= is_valid_st0;
			is_size_st1 	<= is_size_st0;
		end
	end

	////////////////////////////////////////////////////////

	// STAGE 1

	// Store SRAM read data in a shift register

	// FIXME: ~set_done_st1 = Hacky solution to avoid duplicate entries in ibuf
	assign ibuf_push = is_valid_st1 & ~is_size_st1 & ibuf_ready & ~set_done_st1;

	always @(posedge clk) begin

		if (~reset_b) begin
			size <= 16'b0;
		end else begin
			if (is_size_st1) begin
				size <= input_sram_read_data;
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

	assign N = size[7:0];

	////////////////////////////////////////////////////////

	// Keep track of ibuffer state

	reg [4:0] current_ibuf_state;
	reg [4:0] next_ibuf_state;
	reg kernel_pos;

	always @(posedge clk) begin
		if (~reset_b) begin
			current_ibuf_state <= STATE_IBUF_EMPTY;
		end else begin
			current_ibuf_state <= next_ibuf_state;
		end
	end

	always @(*) begin

	ibuf_ready 	= 1'b0;
	ibuf_pop 	= 1'b0;
	do_multiply = 1'b0;
	set_done_st1 = 1'b0;
	weights_sram_read_address = 12'hx;
	kernel_pos = 1'bx;

	ibuf_out[0] = ibuf[15];
	ibuf_out[1] = ibuf[14];
	ibuf_out[2] = ibuf[11];
	ibuf_out[3] = ibuf[10];

	casex (current_ibuf_state)	
	STATE_IBUF_EMPTY: begin
		if (is_valid_st1 & ~is_size_st1) begin
			next_ibuf_state = STATE_IBUF_FILL2;
		end else begin
			next_ibuf_state = STATE_IBUF_EMPTY;
		end
		ibuf_ready			= 1'b1;
	end

	STATE_IBUF_FILL2: begin
		if (is_valid_st1 & ~is_size_st1) begin
			next_ibuf_state = STATE_IBUF_FILL4;
		end else begin
			next_ibuf_state = STATE_IBUF_FILL2;
		end
		ibuf_ready 			= 1'b1;
	end

	STATE_IBUF_FILL4: begin
		if (is_valid_st1 & ~is_size_st1) begin
			next_ibuf_state = STATE_IBUF_FILL6;
		end else begin
			next_ibuf_state = STATE_IBUF_FILL4;
		end
		ibuf_ready 			= 1'b1;
	end

	STATE_IBUF_FILL6: begin
		if (is_valid_st1 & ~is_size_st1) begin
			next_ibuf_state = STATE_IBUF_FILL8;
		end else begin
			next_ibuf_state = STATE_IBUF_FILL6;
		end
		ibuf_ready 			= 1'b1;
	end

	STATE_IBUF_FILL8: begin
		if (is_valid_st1 & ~is_size_st1) begin
			next_ibuf_state = STATE_IBUF_FILL10;
		end else begin
			next_ibuf_state = STATE_IBUF_FILL8;
		end
		ibuf_ready 			= 1'b1;
	end

	STATE_IBUF_FILL10: begin
		if (is_valid_st1 & ~is_size_st1) begin
			next_ibuf_state = STATE_IBUF_FILL12;
		end else begin
			next_ibuf_state = STATE_IBUF_FILL10;
		end
		ibuf_ready 			= 1'b1;
	end

	STATE_IBUF_FILL12: begin
		if (is_valid_st1 & ~is_size_st1) begin
			next_ibuf_state = STATE_IBUF_FILL14;
		end else begin
			next_ibuf_state = STATE_IBUF_FILL12;
		end
		ibuf_ready 			= 1'b1;
	end

	STATE_IBUF_FILL14: begin
		if (is_valid_st1 & ~is_size_st1) begin
			next_ibuf_state = STATE_IBUF_MULT1;
		end else begin
			next_ibuf_state = STATE_IBUF_FILL14;
		end
		ibuf_ready 			= 1'b1;
		weights_sram_read_address = 12'h0;
	end

	STATE_IBUF_MULT1: begin
		next_ibuf_state		= STATE_IBUF_MULT2;
		do_multiply 		= 1'b1;
		ibuf_pop			= 1'b1;
		weights_sram_read_address = 12'h0;
		kernel_pos = 1'b1;
	end

	STATE_IBUF_MULT2: begin
		next_ibuf_state		= STATE_IBUF_MULT3;
		do_multiply 		= 1'b1;
		ibuf_pop			= 1'b1;
		weights_sram_read_address = 12'h1;
		kernel_pos = 1'b0;
	end

	STATE_IBUF_MULT3: begin
		next_ibuf_state		= STATE_IBUF_MULT4;
		do_multiply 		= 1'b1;
		weights_sram_read_address = 12'h1;
		kernel_pos = 1'b1;
	end

	STATE_IBUF_MULT4: begin
		next_ibuf_state		= STATE_IBUF_MULT5;
		do_multiply 		= 1'b1;
		ibuf_pop			= 1'b1;
		weights_sram_read_address = 12'h2;
		kernel_pos = 1'b0;
	end

	STATE_IBUF_MULT5: begin
		next_ibuf_state		= STATE_IBUF_MULT6;
		do_multiply 		= 1'b1;
		ibuf_pop			= 1'b1;
		weights_sram_read_address = 12'h2;
		kernel_pos = 1'b1;
	end


	STATE_IBUF_MULT6: begin
		next_ibuf_state		= STATE_IBUF_MULT7;
		do_multiply 		= 1'b1;
		weights_sram_read_address = 12'h3;
		kernel_pos = 1'b0;
	end

	STATE_IBUF_MULT7: begin
		next_ibuf_state		= STATE_IBUF_MULT8;
		do_multiply 		= 1'b1;
		ibuf_pop			= 1'b1;
		weights_sram_read_address = 12'h3;
		kernel_pos = 1'b1;
	end

	STATE_IBUF_MULT8: begin
		next_ibuf_state		= STATE_IBUF_MULT9;
		do_multiply 		= 1'b1;
		ibuf_pop			= 1'b1;
		weights_sram_read_address = 12'h4;
		kernel_pos = 1'b0;
	end

	STATE_IBUF_MULT9: begin
		next_ibuf_state		= STATE_IBUF_EMPTY;
		do_multiply 		= 1'b1;
		set_done_st1		= 1'b1;
		ibuf_ready			= 1'b1;
		kernel_pos = 1'b1;
	end
	endcase
	end

	////////////////////////////////////////////////////////

	// STAGE 2: Multiplication with kernel

	wire signed [7:0] kernel_rdata;

	assign kernel_rdata = kernel_pos ? weights_sram_read_data[8 +: 8] : weights_sram_read_data[0 +: 8];

	reg signed [19:0] multiply_r [3:0];

	genvar j;
	generate
		for (j = 0; j < 4; j = j + 1) begin : mul
			wire [19:0] multiply = kernel_rdata  * ibuf_out[j];

			always @(posedge clk) begin
				if (~reset_b) begin
					multiply_r[j] <= 20'h0;
				end
				if (do_multiply) begin
					multiply_r[j] <= multiply_r[j] + multiply;
				end
			end
		end
	endgenerate

	// Pipeline register
	reg is_valid_st2;
	always @(posedge clk) begin
		if (~reset_b) begin
			is_valid_st2 <= 0;
		end else begin
			is_valid_st2 <= set_done_st1;
		end
	end

	////////////////////////////////////////////////////////

	// STAGE 4: Max Pool

	wire [19:0] max_pool1;
	wire [19:0] max_pool2;
	wire [19:0] max_pool;

	assign max_pool1 = (multiply_r[0] > multiply_r[1]) ? multiply_r[0] : multiply_r[1];
	assign max_pool2 = (multiply_r[2] > multiply_r[3]) ? multiply_r[2] : multiply_r[3];
	assign max_pool = (max_pool1 > max_pool2) ? max_pool1 : max_pool2;

	////////////////////////////////////////////////////////

	// Debugging

	always @(posedge clk) begin

		// if (is_valid_st1) begin
		// 	$display ("%d: SRAM raddr = %h, SRAM rdata = %h", $time, input_sram_read_address, input_sram_read_data);
		// 	$display ("ibuf state = %0d", current_ibuf_state);
		// 	for (i = 0; i < 16; i = i + 1) begin
		// 		$display ("ibuf[%0d] = %h", i, ibuf[i]);
		// 	end
		// end
		
		if (set_done_st1) begin
			$display ("ibuf_out_0 = %d, ibuf_out1 = %d, ibuf_out2 = %d, ibuf_out3 = %d", ibuf_out[0], ibuf_out[1], ibuf_out[2], ibuf_out[3]);
		end

		if (matrix_done) begin
			$display ("\nmatrix done!\n");
		end

		// if (is_valid_st2) begin
		// 	for (i = 0; i < 4; i = i + 1) begin
		// 		$display ("multiply_r[%0d] = %h", i, multiply_r[i]);
		// 	end	
		// end

		if (is_valid_st2) begin
			$display ("max_pool = %d", max_pool);
		end
	end


endmodule

