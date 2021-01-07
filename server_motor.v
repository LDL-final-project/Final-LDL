// MG-996R嚙踝蕭嚙璀嚙踝蕭嚙瘤
module top (clk, rst, signal, signal2);
    input clk;
    input rst;
    output signal;
    output signal2;
    Servo_Motor_PWM_Gen Ser_1(clk, rst, 2'b10, signal, signal2);
endmodule


module Servo_Motor_PWM_Gen (clk, rst, dir, signal, signal2);
    input clk, rst;
	input [1:0] dir;
	output reg signal;
	output reg signal2;
	reg [29:0] count, count2;
	parameter POS = 30'd10_0000, NEG = 30'd20_0000, STOP = 30'd15_0000, WAVELENGTH = 30'd200_0000;
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