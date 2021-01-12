module controll_car(
    input clk,
    input rst,
    output signal_left,
    output signal_right
);
    
    controll_car servo(
        .clk(clk),
        .rst(rst),
        .location_0(5),
        .location_1(5),
        .location_2(5),
        .signal_left(signal_left),
        .signal_right(signal_right)
    );

endmodule

module servo_motor (
    input clk,
    input rst,
    input location_0,
    input location_1,
    input location_2,
    output signal_left,
    output signal_right
);

    wire clk_26;
    reg [1:0] dir;
    reg [10:0] counter, next_counter;
    
    parameter move = 3'd4;
    
    clock_divider clk25(.clk(clk), .clk_div(clk_25));
    
    always @(posedge clk_26) begin
        if (rst) begin
            counter <= 10'd0;
        end else begin
            counter <= next_counter;
        end
    end

    always @(*) begin
        next_counter = counter + 1;
    end

    always @(*) begin
        if (counter < location_0)
            dir = 2'b00;
        else if(counter > location_0 && counter < location_0 + move)
            dir = 2'b01;
        else if(counter > location_0 + move && counter < location_0 + move + location_1)
            dir = 2'b00;
        else if(counter > location_0 + move + location_1 && counter < location_0 + move + location_1 + move)
            dir = 2'b01;
        else if(counter > location_0 + move + location_1 + move && counter < location_0 + move + location_1 + move + location_2)
            dir = 2'b00;
        else if(counter > location_0 + move + location_1 + move + location_2 && counter < location_0 + move + location_1 + move + location_2 + move + move)
            dir = 2'b10;
    end

    servo_motor_driver Ser_1(
        .clk(clk),
        .rst(rst),
        .dir(dir),
        .signal_left(signal_left),
        .signal_right(signal_right)
    );

endmodule

module servo_moror_driver (clk, rst, dir, signal_left, signal_right);
    input clk, rst;
	input [1:0] dir;
	output reg signal_left;
	output reg signal_right;
	reg [29:0] count, count2;
	reg [29:0] dir_count_left, dir_count_right;
	
	parameter POS = 30'd15_0000;
	parameter NEG = 30'd15_7000;
	parameter STOP = 30'd0;
	parameter WAVELENGTH = 30'd200_0000;
	
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
		case(dir)
			2'b01: begin
				dir_count_left = NEG;
				dir_count_right = POS;
			end
			2'b10: begin
				dir_count_left = POS;
				dir_count_right = NEG;
			end
			default: begin
				dir_count_left = STOP;
				dir_count_right = STOP;
			end
		endcase
	end
endmodule