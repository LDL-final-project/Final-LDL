module top (
    input wire clk, rst,
    inout wire PS2_DATA,
    inout wire PS2_CLK,
    output wire [3:0] vgaRed,
    output wire [3:0] vgaGreen,
    output wire [3:0] vgaBlue,
    output wire hsync, vsync,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue,
    output signal_left,
    output signal_right,
    output [5:0] led
);
    wire [3:0] _color_id;
    wire confirm;

    keyboard kb (.PS2_DATA(PS2_DATA), .PS2_CLK(PS2_CLK),
	             .rst(rst), .clk(clk), .color_id(_color_id), .confirm(confirm));

    screen scn (.clk(clk), .rst(rst), .color_id(_color_id),
              .vgaRed(vgaRed), .vgaGreen(vgaGreen), .vgaBlue(vgaBlue),
              .hsync(hsync), .vsync(vsync));
    
    colors_divide smtr(
        .clk(clk), .rst(rst), .color_id(_color_id),
        .red(red), .yellow(yellow), .blue(blue),
        .signal_left(signal_left), .signal_right(signal_right), .led(led),
        .confirm(confirm)
    );
    
endmodule
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
	output reg confirm
	);
	
	reg [25:0] confirm_cnt, next_confirm_cnt;
    reg next_confirm;
	
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
			confirm_cnt <= 26'd0;
		end else begin
			confirm_cnt <= next_confirm_cnt;
			color_id <= nx_color_id;
			if (been_ready && key_down[last_change] == 1'b1) begin
                if (last_change == KEY_CODES[12] && nx_color_id != 4'b1100) 
                    confirm <= 1'b1;
                 else confirm <= next_confirm;
            end else confirm <= next_confirm;
		end
	end
	
	always @(*) begin
        
        if (confirm_cnt[25]) next_confirm = 1'b0;
        else next_confirm = confirm;
	    if (confirm) next_confirm_cnt = confirm_cnt + 1;
	    else next_confirm_cnt = 26'd0;
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

module color(
    input wire clk, rst,
    input wire [9:0] h_cnt,
    input wire [9:0] v_cnt,
    input wire [3:0] color_id,
    output reg [23:0] RGB
  );
  parameter C00 = 24'h764689;
  parameter C01 = 24'h5854A1;
  parameter C02 = 24'h66876B;
  parameter C03 = 24'h33AF79;
  parameter C10 = 24'h793F3F;
  parameter C11 = 24'hFD7041;
  parameter C12 = 24'h3C49D8;
  parameter C13 = 24'h2E714D;
  parameter C20 = 24'hB9662C;
  parameter C21 = 24'h296ED0;
  parameter C22 = 24'hFDF447;
  parameter C23 = 24'hF90303;
  parameter C00_s = 24'hb18cbf;
  parameter C01_s = 24'h8a85e6;
  parameter C02_s = 24'ha6e3af;
  parameter C03_s = 24'h9aedc9;
  parameter C10_s = 24'hbd7777;
  parameter C11_s = 24'hfa9e7f;
  parameter C12_s = 24'h7c85f2;
  parameter C13_s = 24'h79d4a3;
  parameter C20_s = 24'he6a67a;
  parameter C21_s = 24'h87ace0;
  parameter C22_s = 24'hfff87a;
  parameter C23_s = 24'hf27272;


  always @(*) begin
    if (v_cnt >= 80 && v_cnt < 160 && h_cnt >= 100 && h_cnt < 180) RGB = color_id == 4'd0 ? C00_s : C00;
    else if (v_cnt >= 80 && v_cnt < 160 && h_cnt >= 220 && h_cnt < 300) RGB = color_id == 4'd1 ? C01_s : C01;
    else if (v_cnt >= 80 && v_cnt < 160 && h_cnt >= 340 && h_cnt < 420) RGB = color_id == 4'd2 ? C02_s : C02;
    else if (v_cnt >= 80 && v_cnt < 160 && h_cnt >= 460 && h_cnt < 540) RGB = color_id == 4'd3 ? C03_s : C03;
    else if (v_cnt >= 200 && v_cnt < 280 && h_cnt >= 100 && h_cnt < 180) RGB = color_id == 4'd4 ? C10_s : C10;
    else if (v_cnt >= 200 && v_cnt < 280 && h_cnt >= 220 && h_cnt < 300) RGB = color_id == 4'd5 ? C11_s : C11;
    else if (v_cnt >= 200 && v_cnt < 280 && h_cnt >= 340 && h_cnt < 420) RGB = color_id == 4'd6 ? C12_s : C12;
    else if (v_cnt >= 200 && v_cnt < 280 && h_cnt >= 460 && h_cnt < 540) RGB = color_id == 4'd7 ? C13_s : C13;
    else if (v_cnt >= 320 && v_cnt < 400 && h_cnt >= 100 && h_cnt < 180) RGB = color_id == 4'd8 ? C20_s : C20;
    else if (v_cnt >= 320 && v_cnt < 400 && h_cnt >= 220 && h_cnt < 300) RGB = color_id == 4'd9 ? C21_s : C21;
    else if (v_cnt >= 320 && v_cnt < 400 && h_cnt >= 340 && h_cnt < 420) RGB = color_id == 4'd10 ? C22_s : C22;
    else if (v_cnt >= 320 && v_cnt < 400 && h_cnt >= 460 && h_cnt < 540) RGB = color_id == 4'd11 ? C23_s : C23;
    else RGB = 24'd0;
  end
    
endmodule

module vga_controller (
    input wire pclk, reset,
    output wire hsync, vsync, valid,
    output wire [9:0] h_cnt,
    output wire [9:0] v_cnt
    );

    reg [9:0]pixel_cnt;
    reg [9:0]line_cnt;
    reg hsync_i,vsync_i;

    parameter HD = 640;
    parameter HF = 16;
    parameter HS = 96;
    parameter HB = 48;
    parameter HT = 800; 
    parameter VD = 480;
    parameter VF = 10;
    parameter VS = 2;
    parameter VB = 33;
    parameter VT = 525;
    parameter hsync_default = 1'b1;
    parameter vsync_default = 1'b1;

    always @(posedge pclk)
        if (reset)
            pixel_cnt <= 0;
        else
            if (pixel_cnt < (HT - 1))
                pixel_cnt <= pixel_cnt + 1;
            else
                pixel_cnt <= 0;

    always @(posedge pclk)
        if (reset)
            hsync_i <= hsync_default;
        else
            if ((pixel_cnt >= (HD + HF - 1)) && (pixel_cnt < (HD + HF + HS - 1)))
                hsync_i <= ~hsync_default;
            else
                hsync_i <= hsync_default; 

    always @(posedge pclk)
        if (reset)
            line_cnt <= 0;
        else
            if (pixel_cnt == (HT -1))
                if (line_cnt < (VT - 1))
                    line_cnt <= line_cnt + 1;
                else
                    line_cnt <= 0;

    always @(posedge pclk)
        if (reset)
            vsync_i <= vsync_default; 
        else if ((line_cnt >= (VD + VF - 1)) && (line_cnt < (VD + VF + VS - 1)))
            vsync_i <= ~vsync_default; 
        else
            vsync_i <= vsync_default; 

    assign hsync = hsync_i;
    assign vsync = vsync_i;
    assign valid = ((pixel_cnt < HD) && (line_cnt < VD));

    assign h_cnt = (pixel_cnt < HD) ? pixel_cnt : 10'd0;
    assign v_cnt = (line_cnt < VD) ? line_cnt : 10'd0;

endmodule

module screen (
  input wire clk, rst,
  input wire [3:0] color_id,
  output wire [3:0] vgaRed,
  output wire [3:0] vgaGreen,
  output wire [3:0] vgaBlue,
  output wire hsync, vsync
);

  wire clk_22, clk_2, clk_16;
  wire valid, dbRst;
  wire [9:0] h_cnt; //640
  wire [9:0] v_cnt; //480
  wire [23:0] RGB;
  reg  [7:0] Red, Green, Blue;

  clock_divider #(.n (2)) clk2 (.clk (clk), .clk_div (clk_2));
  clock_divider #(.n (22)) clk22 (.clk (clk), .clk_div (clk_22));
  clock_divider #(.n (16)) clk16 (.clk (clk), .clk_div (clk_16));
  debounce db_rst (.pb_debounced (dbRst), .pb (rst), .clk (clk_16));  
  assign {vgaRed, vgaGreen, vgaBlue} = (valid == 1'b1) ? {Red[3:0], Green[3:0], Blue[3:0]} : 12'h0;

  vga_controller   vga_inst(
    .pclk(clk_2),
    .reset(rst),
    .hsync(hsync),
    .vsync(vsync),
    .valid(valid),
    .h_cnt(h_cnt),
    .v_cnt(v_cnt)
  );

  color color_init(
    .clk(clk_22),
    .rst(dbRst),
    .h_cnt(h_cnt),
    .v_cnt(v_cnt),
    .color_id(color_id),
    .RGB(RGB)
  );

  always @(*) begin
    Red = RGB[23:16] >> 4;
    Green = RGB[15:8] >> 4;
    Blue = RGB[7:0] >> 4;    
  end

endmodule
module clock_divider (clk, clk_div);

  parameter n = 25;
  input clk;
  output clk_div;

  reg [n-1:0] num;
  wire [n-1:0] nxt_num;

  assign nxt_num = num + 1;
  assign clk_div = num[n-1];

  always @ (posedge clk) begin
    num = nxt_num;
  end

endmodule

module debounce (pb_debounced, pb, clk);

  input pb, clk;
  output pb_debounced;

  reg [3:0] shift_reg;

  always @(posedge clk) begin
    shift_reg[3:1] <= shift_reg[2:0];
    shift_reg[0]  <= pb;
  end

  assign pb_debounced = ((shift_reg == 4'b1111) ? 1'b1 : 1'b0);

endmodule

module onepulse (
  input wire rst, clk, pb_debounced,
  output reg pb_1pulse
);

  reg pb_1pulse_nxt, pb_debounced_delay;

  always @(*) begin
    pb_1pulse_nxt = pb_debounced & ~pb_debounced_delay;
  end

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      pb_1pulse <= 1'b0;
      pb_debounced_delay <= 1'b0;
    end else begin
      pb_1pulse <= pb_1pulse_nxt;
      pb_debounced_delay <= pb_debounced;
    end
  end
endmodule
module colors_divide(
    input clk,
    input rst,
    input [3:0] color_id,
    input confirm,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue,
    output signal_left,
    output signal_right,
    output [5:0] led
);
    reg [9:0] freq_for_red, freq_for_yellow, freq_for_blue;
    
    always@* begin
        case(color_id)
            4'd0: begin //#764689
                freq_for_red = 10'd3;
                freq_for_yellow = 10'd0;
                freq_for_blue = 10'd2;
            end
            4'd1: begin //#5854A1
                freq_for_red = 10'd3;
                freq_for_yellow = 10'd0;
                freq_for_blue = 10'd4;
            end
            4'd2: begin //#66876B
                freq_for_red = 10'd3;
                freq_for_yellow = 10'd6;
                freq_for_blue = 10'd4;
            end
            4'd3: begin //#33AF79
                freq_for_red = 10'd0;
                freq_for_yellow = 10'd4;
                freq_for_blue = 10'd3;
            end
            4'd4: begin //#793F3F
                freq_for_red = 10'd4;
                freq_for_yellow = 10'd4;
                freq_for_blue = 10'd3;
            end
            4'd5: begin //#FD7041
                freq_for_red = 10'd3;
                freq_for_yellow = 10'd5;
                freq_for_blue = 10'd0;
            end
            4'd6: begin //#3C49D8
                freq_for_red = 10'd2;
                freq_for_yellow = 10'd0;
                freq_for_blue = 10'd5;
            end
            4'd7: begin //#2E714D
                freq_for_red = 10'd3;
                freq_for_yellow = 10'd2;
                freq_for_blue = 10'd5;
            end
            4'd8: begin //#B9662C
                freq_for_red = 10'd3;
                freq_for_yellow = 10'd6;
                freq_for_blue = 10'd1;
            end
            4'd9: begin //#296ED0
                freq_for_red = 10'd0;
                freq_for_yellow = 10'd0;
                freq_for_blue = 10'd6;
            end
            4'd10: begin //#FDF447
                freq_for_red = 10'd0;
                freq_for_yellow = 10'd6;
                freq_for_blue = 10'd0;
            end
            4'd11: begin //#F90303
                freq_for_red = 10'd6;
                freq_for_yellow = 10'd0;
                freq_for_blue = 10'd0;
            end
            default: begin
                freq_for_red = 10'd0;
                freq_for_yellow = 10'd0;
                freq_for_blue = 10'd0;
            end
        endcase
    end
    
    hole_motors_states stepp_ind(
        .clk(clk), .rst(rst), .confirm(confirm),
        .freq_for_red(freq_for_red), .freq_for_yellow(freq_for_yellow), .freq_for_blue(freq_for_blue),
        .red(red), .yellow(yellow), .blue(blue),
        .signal_left(signal_left), .signal_right(signal_right), .led(led)
    );
endmodule

module hole_motors_states(
    input clk,
    input rst,
    input confirm,
    input [9:0] freq_for_red,
    input [9:0] freq_for_yellow,
    input [9:0] freq_for_blue,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue,
    output signal_left,
    output signal_right,
    output [5:0] led
);
    parameter INIT = 3'd0;
    parameter RED = 3'd1;
    parameter RtoY = 3'd2;
    parameter YELLOW = 3'd3;
    parameter YtoB = 3'd4;
    parameter BLUE = 3'd5;
    parameter BtoR = 3'd6;
    
    parameter depth = 6'd20;
    parameter car_move_time = 6'd7;
    
    wire clk_19, clk_cnt;
    reg [2:0] state, next_state;
    reg [5:0] count_sec, next_count_sec;
    reg [9:0] count_rnd, next_count_rnd;
    reg en_for_red, en_for_yellow, en_for_blue, en_for_car;
    reg dir_for_motor, dir_for_car;
    
    assign led[3:0] = {1'b0, state};
    assign led[5:4] = {1'b0, confirm};
    
    clock_divider #(.n(19)) clk19( .clk(clk), .clk_div(clk_19));
    clock_divider #(.n(25)) clkcnt( .clk(clk), .clk_div(clk_cnt));
    
    always@(posedge clk_cnt, posedge rst) begin
        if (rst) begin
            count_sec <= 6'b0;
            count_rnd <= 10'b0;
        end else begin
            count_sec <= next_count_sec;
            count_rnd <= next_count_rnd;
        end
    end
    always@(posedge clk_cnt, posedge rst) begin
        if (rst) begin
            state <= INIT;
        end else begin;
            state <= next_state;
        end
    end
    
    always@* begin
        dir_for_motor = (count_sec <= depth) ? 1'b0 : 1'b1;
    end
    
    always@* begin
        en_for_red = 1'b0;
        en_for_yellow = 1'b0;
        en_for_blue = 1'b0;
        en_for_car = 1'b0;
        dir_for_car = 1'b0;
        next_state = state;
        next_count_sec = count_sec;
        next_count_rnd = count_rnd;
        case(state)
            INIT: begin
                next_state = (confirm) ? RED : INIT;
                next_count_sec = 6'd0;
                next_count_rnd = 10'd0;
            end
            RED: begin
                if (count_rnd == freq_for_red) begin
                    next_state = RtoY;
                    next_count_rnd = 10'd0;
                    next_count_sec = 6'd0;
                end else begin
                    next_count_sec = (count_sec < depth*2) ? count_sec + 6'd1 : 6'd0;
                    if (count_sec == depth*2) next_count_rnd = count_rnd + 10'd1;
                end
                en_for_red = 1'b1;
            end
            RtoY: begin
                if(count_rnd == 10'd1) begin
                    next_state = YELLOW;
                    next_count_rnd = 10'd0;
                    next_count_sec = 6'd0;
                end else begin
                    next_count_sec = (count_sec < car_move_time) ? count_sec + 6'd1 : 6'd0;
                    if (count_sec == car_move_time) next_count_rnd = count_rnd + 10'd1;
                end
                en_for_car = 1'b1;
                dir_for_car = 1'b1;
            end
            YELLOW: begin
                if (count_rnd == freq_for_yellow) begin
                    next_state = YtoB;
                    next_count_rnd = 10'd0;
                    next_count_sec = 6'd0;
                end else begin
                    next_count_sec = (count_sec < depth*2) ? count_sec + 6'd1 : 6'd0;
                    if (count_sec == depth*2) next_count_rnd = count_rnd + 10'd1;
                end
                en_for_yellow = 1'b1;
            end
            YtoB: begin
                if(count_rnd == 10'd1) begin
                    next_state = BLUE;
                    next_count_rnd = 10'd0;
                    next_count_sec = 6'd0;
                end else begin
                    next_count_sec = (count_sec < car_move_time) ? count_sec + 6'd1 : 6'd0;
                    if (count_sec == car_move_time) next_count_rnd = count_rnd + 10'd1;
                end
                en_for_car = 1'b1;
                dir_for_car = 1'b1;
            end
            BLUE: begin
                if (count_rnd == freq_for_blue) begin
                    next_state = BtoR;
                    next_count_rnd = 10'd0;
                    next_count_sec = 6'd0;
                end else begin
                    next_count_sec = (count_sec < depth*2) ? count_sec + 6'd1 : 6'd0;
                    if (count_sec == depth*2) next_count_rnd = count_rnd + 10'd1;
                end
                en_for_blue = 1'b1;
            end
            BtoR: begin
                if(count_rnd == 10'd2) begin
                    next_state = INIT;
                    next_count_rnd = 10'd0;
                    next_count_sec = 6'd0;
                end else begin
                    next_count_sec = (count_sec < car_move_time) ? count_sec + 6'd1 : 6'd0;
                    if (count_sec == car_move_time) next_count_rnd = count_rnd + 10'd1;
                end
                en_for_car = 1'b1;
                dir_for_car = 1'b0;
            end
            default: begin
                next_state = INIT;
            end
        endcase
    end
    Servo_Motor servot(
        .clk(clk), .rst(rst), .en(en_for_car), .dir(dir_for_car), .signal_left(signal_left), .signal_right(signal_right)
    );
    stepper_motor_driver motorr(
        .clk(clk_19), .rst(rst), . en(en_for_red), .dir(dir_for_motor), .signal(red)
    );
    stepper_motor_driver motory(
        .clk(clk_19), .rst(rst), . en(en_for_yellow), .dir(dir_for_motor), .signal(yellow)
    );
    stepper_motor_driver motorb(
        .clk(clk_19), .rst(rst), . en(en_for_blue), .dir(dir_for_motor), .signal(blue)
    );
    
endmodule

module stepper_motor_driver(
    input clk,
    input rst,
    input dir,
    input en,
    output reg [3:0] signal
    );
    
    parameter sig4 = 3'b001;
    parameter sig3 = 3'b011;
    parameter sig2 = 3'b010;
    parameter sig1 = 3'b110;
    parameter sig0 = 3'b000;
    
    reg [2:0] present_state, next_state;
    
    always @ (posedge clk, posedge rst) begin
        if (rst == 1'b1)
            present_state = sig0;
        else 
            present_state = next_state;
    end
    
    always @ (posedge clk) begin
        if (present_state == sig4)
            signal = 4'b1000;
        else if (present_state == sig3)
            signal = 4'b0100;
        else if (present_state == sig2)
            signal = 4'b0010;
        else if (present_state == sig1)
            signal = 4'b0001;
        else
            signal = 4'b0000;
    end
    
    always @* begin
        case(present_state)
            sig4: begin
                if (dir == 1'b0 && en == 1'b1)
                    next_state = sig3;
                else if (dir == 1'b1 && en == 1'b1)
                    next_state = sig1;
                else 
                    next_state = sig0;
            end  
            sig3: begin
                if (dir == 1'b0&& en == 1'b1)
                    next_state = sig2;
                else if (dir == 1'b1 && en == 1'b1)
                    next_state = sig4;
                else 
                    next_state = sig0;
            end
            sig2: begin
                if (dir == 1'b0&& en == 1'b1)
                    next_state = sig1;
                else if (dir == 1'b1 && en == 1'b1)
                    next_state = sig3;
                else 
                    next_state = sig0;
            end 
            sig1: begin
                if (dir == 1'b0&& en == 1'b1)
                    next_state = sig4;
                else if (dir == 1'b1 && en == 1'b1)
                    next_state = sig2;
                else 
                    next_state = sig0;
            end
            sig0: begin
                if (en == 1'b1)
                    next_state = sig1;
                else 
                    next_state = sig0;
            end
            default:
                next_state = sig0; 
        endcase
    end
endmodule
module Server_top (clk, rst, en, dir, signal_left, signal_right);
    input clk, rst, dir, en;
    output signal_left;
    output signal_right;

    Servo_Motor Ser(.clk(clk), .rst(rst), .en(en), .dir(dir), .signal_left(signal_left), .signal_right(signal_right));
endmodule


module Servo_Motor (clk, rst, en, dir, signal_left, signal_right);
    input clk, rst, en, dir;
	output reg signal_left;
	output reg signal_right;
	reg [29:0] count, count2;
	parameter POS = 30'd15_4000, NEG = 30'd15_6100, STOP = 30'd0, WAVELENGTH = 30'd200_0000;
	reg [29:0] dir_count_left, dir_count_right;
	
	always @ (posedge clk) begin
		if(rst) begin
			count <= 30'd0;
			signal_left <= 1'b0;
			signal_right <= 1'b0;
		end
		else begin
			count <= (count < WAVELENGTH) ? count + 1'b1 : 30'd0;
			signal_left <= (count < dir_count_left) ? 1'b1 : 1'b0;
			signal_right <= (count < dir_count_right) ? 1'b1 : 1'b0;
		end
	end
	
	always @ (*) begin
		if(en) begin
			if(dir) begin
				dir_count_right = POS;
				dir_count_left = NEG;
			end
			else begin
				dir_count_right = NEG;
				dir_count_left = POS;
			end
        end
        else begin
            dir_count_left = STOP;
            dir_count_right = STOP;
        end
	end
endmodule