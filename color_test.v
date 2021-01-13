module color_test(
    input clk,
    input rst,
    input [3:0] sw,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue
);
    wire en_for_red, en_for_yellow, en_for_blue;
    wire clk_19;
    
    assign {en_for_red, en_for_yellow, en_for_blue, dir} = sw;
    
    clock_divider #(.n(19)) clk19( .clk(clk), .clk_div(clk_19));
    
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
