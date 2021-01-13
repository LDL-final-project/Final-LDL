module OnePulse (
	output reg signal_single_pulse,
	input wire signal,
	input wire clock
	);
	
	reg signal_delay;

	always @(posedge clock) begin
		if (signal == 1'b1 & signal_delay == 1'b0)
		  signal_single_pulse <= 1'b1;
		else
		  signal_single_pulse <= 1'b0;

		signal_delay <= signal;
	end
endmodule

module KeyboardDecoder(
	output reg [511:0] key_down,
	output wire [8:0] last_change,
	output reg key_valid,
	inout wire PS2_DATA,
	inout wire PS2_CLK,
	input wire rst,
	input wire clk
    );
    
    parameter [1:0] INIT			= 2'b00;
    parameter [1:0] WAIT_FOR_SIGNAL = 2'b01;
    parameter [1:0] GET_SIGNAL_DOWN = 2'b10;
    parameter [1:0] WAIT_RELEASE    = 2'b11;
    
		parameter [7:0] IS_INIT			= 8'hAA;
    parameter [7:0] IS_EXTEND		= 8'hE0;
    parameter [7:0] IS_BREAK		= 8'hF0;
    
    reg [9:0] key;		// key = {been_extend, been_break, key_in}
    reg [1:0] state;
    reg been_ready, been_extend, been_break;
    
    wire [7:0] key_in;
    wire is_extend;
    wire is_break;
    wire valid;
    wire err;
    
    wire [511:0] key_decode = 1 << last_change;
    assign last_change = {key[9], key[7:0]};
    
    KeyboardCtrl_0 inst (
		.key_in(key_in),
		.is_extend(is_extend),
		.is_break(is_break),
		.valid(valid),
		.err(err),
		.PS2_DATA(PS2_DATA),
		.PS2_CLK(PS2_CLK),
		.rst(rst),
		.clk(clk)
	);

	OnePulse op (
		.signal_single_pulse(pulse_been_ready),
		.signal(been_ready),
		.clock(clk)
	);
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		state <= INIT;
    		been_ready  <= 1'b0;
    		been_extend <= 1'b0;
    		been_break  <= 1'b0;
    		key <= 10'b0_0_0000_0000;
    	end else begin
    		state <= state;
				been_ready  <= been_ready;
				been_extend <= (is_extend) ? 1'b1 : been_extend;
				been_break  <= (is_break ) ? 1'b1 : been_break;
				key <= key;
    		case (state)
    			INIT : begin
    				if (key_in == IS_INIT) begin
    					state <= WAIT_FOR_SIGNAL;
    					been_ready  <= 1'b0;
							been_extend <= 1'b0;
							been_break  <= 1'b0;
							key <= 10'b0_0_0000_0000;
    				end else begin
    						state <= INIT;
						end
					end
    			WAIT_FOR_SIGNAL : begin
						if (valid == 0) begin
							state <= WAIT_FOR_SIGNAL;
							been_ready <= 1'b0;
						end else begin
							state <= GET_SIGNAL_DOWN;
						end
					end
    			GET_SIGNAL_DOWN : begin
						state <= WAIT_RELEASE;
						key <= {been_extend, been_break, key_in};
						been_ready  <= 1'b1;
    			end
    			WAIT_RELEASE : begin
						if (valid == 1) begin
							state <= WAIT_RELEASE;
						end else begin
							state <= WAIT_FOR_SIGNAL;
							been_extend <= 1'b0;
							been_break  <= 1'b0;
						end
					end
    			default : begin
						state <= INIT;
						been_ready  <= 1'b0;
						been_extend <= 1'b0;
						been_break  <= 1'b0;
						key <= 10'b0_0_0000_0000;
    			end
    		endcase
    	end
    end
    
    always @ (posedge clk, posedge rst) begin
    	if (rst) begin
    		key_valid <= 1'b0;
    		key_down <= 511'b0;
    	end else if (key_decode[last_change] && pulse_been_ready) begin
    		key_valid <= 1'b1;
    		if (key[8] == 0) begin
    			key_down <= key_down | key_decode;
    		end else begin
    			key_down <= key_down & (~key_decode);
    		end
    	end else begin
    		key_valid <= 1'b0;
				key_down <= key_down;
    	end
    end

endmodule

module keyboard (
	inout wire PS2_DATA,
	inout wire PS2_CLK,
	input wire rst,
	input wire clk,
  output reg [3:0] color_id,
	output wire confirm
	);
	
	parameter [8:0] KEY_CODES [0:12] = {
		9'b0_0010_1101,	// R => 2D
		9'b0_0010_1100,	// T => 2C
		9'b0_0011_0101,	// Y => 35
		9'b0_0011_1100,	// U => 3C
		9'b0_0010_1011,	// F => 2B
		9'b0_0011_0100,	// G => 34
		9'b0_0011_0011,	// H => 33
		9'b0_0011_1011,	// J => 3B
		9'b0_0010_1010,	// V => 2A
		9'b0_0011_0001,	// B => 31
		9'b0_0011_0010, // N => 32
		9'b0_0011_1010, // M => 3A
		9'b0_0101_1010  // Enter
	};
	
  reg [4:0] nx_color_id;
	wire [511:0] key_down;
	wire [8:0] last_change;
	wire been_ready;
	

	KeyboardDecoder key_de (
		.key_down(key_down),
		.last_change(last_change),
		.key_valid(been_ready),
		.PS2_DATA(PS2_DATA),
		.PS2_CLK(PS2_CLK),
		.rst(rst),
		.clk(clk)
	);

	always @ (posedge clk, posedge rst) begin
		if (rst) begin
			color_id <= 4'b1100;
			confirm = 1'b0;
		end else begin
			color_id <= color_id;
			confirm = 1'b0;
			if (been_ready && key_down[last_change] == 1'b1) begin
				if (last_change == KEY_CODES[12] && nx_color_id != 4'b1100) 
					confirm <= 1'b1;
				color_id <= nx_color_id;
			end
		end
	end
	
	always @ (*) begin
		case (last_change)
			KEY_CODES[00] : nx_color_id = 4'b0000;
			KEY_CODES[01] : nx_color_id = 4'b0001;
			KEY_CODES[02] : nx_color_id = 4'b0010;
			KEY_CODES[03] : nx_color_id = 4'b0011;
			KEY_CODES[04] : nx_color_id = 4'b0100;
			KEY_CODES[05] : nx_color_id = 4'b0101;
			KEY_CODES[06] : nx_color_id = 4'b0110;
			KEY_CODES[07] : nx_color_id = 4'b0111;
			KEY_CODES[08] : nx_color_id = 4'b1000;
			KEY_CODES[09] : nx_color_id = 4'b1001;
			KEY_CODES[10] : nx_color_id = 4'b1010;
			KEY_CODES[11] : nx_color_id = 4'b1011;
			KEY_CODES[12] : nx_color_id = color_id;
			default		  : nx_color_id = 4'b1100;
		endcase
	end
	
endmodule
