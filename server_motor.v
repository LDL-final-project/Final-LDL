// MG-996R嚙踝蕭嚙璀嚙踝蕭嚙瘤
module Server_top (clk, rst, runtime, dir, signal, signal2);
		input clk, rst, dir;
		input reg [3:0] runtime;
    output signal;
    output signal2;
		reg [3:0] counter, next_counter;

    clock_divider #(.n(27)) clock_div( .clk(clk), .clk_div(clk_27));

		always @(posedge clk_27, posedge rst) begin
			if (rst) begin
				counter <= 4'd0;
				en = 0;
			end else begin
				if (counter >= runtime) begin
					counter <= 4'd0;
					en <= 0;
				end else begin	
					counter <= next_counter;
					en <= 1;
				end
			end
		end

		always @(*) begin
			next_counter = counter + 1;
		end

    Servo_Motor Ser(.clk(clk), .rst(rst), .en(en) .dir(dir), .signal(signal), .signal2(signal2));
endmodule


module Servo_Motor (clk, rst, en, dir, signal, signal2);
  input clk, rst, en, dir;
	output reg signal;
	output reg signal2;
	reg [29:0] count, count2;
	parameter POS = 30'd15_0000, NEG = 30'd15_7000, STOP = 30'd15_0000, WAVELENGTH = 30'd200_0000;
	reg [29:0] dir_count, dir_count2;
	
	always @ (posedge clk) begin
		if(rst) begin
			count <= 30'd0;
			signal <= 1'b0;
			signal2 <= 1'b0;
		end
		else begin
			count <= (count < WAVELENGTH) ? count + 1'b1 : 30'd0;
			signal <= (count < dir_count) ? 1'b1 : 1'b0;
			signal2 <= (count < dir_count2) ? 1'b1 : 1'b0;
		end
	end
	
	always @ (*) begin
		case(dir)
			2'b01: begin
				dir_count2 = POS;
				dir_count = NEG;
			end
			2'b10: begin
				dir_count2 = NEG;
				dir_count = POS;
			end
			default: begin
				dir_count = STOP;
				dir_count2 = STOP;
			end
		endcase
	end
endmodule