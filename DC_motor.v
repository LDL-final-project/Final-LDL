module pwm (input clk, input rst, output reg pwm);
    reg [7:0] counter;
    always @ (posedge clk) begin
        if (rst)
            counter = 8'h00;
        else if (counter < vc) begin
            pwm = 1'b1;
            counter = counter + 1;
        end else begin
            pwm = 1'b0;
            counter = counter + 1;
        end
    end
endmodule
