module color_test(
    input clk,
    input rst,
    input [3:0] sw,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue
);
    wire en_for_red, en_for_yellow, en_for_blue, dir_for_motor;
    wire clk_19, clk_cnt;
    
    reg [9:0] count, next_count;
    reg en;
    
    assign {en_for_red, en_for_yellow, en_for_blue, dir_for_motor} = sw;
    
    clock_divider #(.n(19)) clk19( .clk(clk), .clk_div(clk_19));
    clock_divider #(.n(25)) clkcnt( .clk(clk), .clk_div(clk_cnt));
    
    always@(posedge clk_cnt, posedge rst) begin
        if (rst) begin
            count <= 10'd0;
        end else begin
            count <= next_count;
        end
    end
    always@* begin
        if(en_for_red)
            next_count = 10'd0;
        else
            next_count = count+ 10'd1;
    end
    always@* begin
        if(count<10'd20)
            en = 1'd1;
        else
            en = 1'd0;
    end
    
    stepper_motor_driver motorr(
        .clk(clk_19), .rst(rst), . en(en), .dir(dir_for_motor), .signal(red)
    );
    stepper_motor_driver motory(
        .clk(clk_19), .rst(rst), . en(en_for_yellow), .dir(dir_for_motor), .signal(yellow)
    );
    stepper_motor_driver motorb(
        .clk(clk_19), .rst(rst), . en(en_for_blue), .dir(dir_for_motor), .signal(blue)
    );

endmodule
