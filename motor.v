module stepper_motor(
    input clk,
    input rst,
//    input color_id,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue,
    output signal_left,
    output signal_right,
    output wire [2:0] led
);
    reg [9:0] freq_for_red, freq_for_yellow, freq_for_blue;
    
    stepper_motor_individual stepp_ind(
        .clk(clk), .rst(rst),
        .freq_for_red(1), .freq_for_yellow(1), .freq_for_blue(1),
        .red(red), .yellow(yellow), .blue(blue), .led(led),
        .signal_left(signal_left), .signal_right(signal_right)
    );
endmodule

module stepper_motor_individual(
    input clk,
    input rst,
    input [9:0] freq_for_red,
    input [9:0] freq_for_yellow,
    input [9:0] freq_for_blue,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue,
    output signal_left,
    output signal_right,
    output wire [2:0] led
);
    parameter INIT = 3'd0;
    parameter RED = 3'd1;
    parameter RtoY = 3'd2;
    parameter YELLOW = 3'd3;
    parameter YtoB = 3'd4;
    parameter BLUE = 3'd5;
    parameter BtoR = 3'd6;
    
    parameter depth = 5'd24;
    parameter car_move_time = 5'd7;
    
    wire clk_19, clk_cnt;
    reg [2:0] state, next_state;
    reg [5:0] count_sec, next_count_sec;
    reg [9:0] count_rnd, next_count_rnd;
    reg en_for_red, en_for_yellow, en_for_blue, en_for_car;
    reg dir_for_motor, dir_for_car;
    
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
    always@(posedge clk_19, posedge rst) begin
        if (rst) begin
            state <= INIT;
        end else begin;
            state <= next_state;
        end
    end
    
    always@* begin
        dir_for_motor = (count_sec <= depth) ? 1'b0 : 1'b1;
    end
    
    assign led = state;
    
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
                next_state = RED;
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
                if(count_rnd == 10'd1) begin
                    next_state = INIT;
                    next_count_rnd = 10'd0;
                    next_count_sec = 6'd0;
                end else begin
                    next_count_sec = (count_sec < car_move_time*2) ? count_sec + 6'd1 : 6'd0;
                    if (count_sec == car_move_time*2) next_count_rnd = count_rnd + 10'd1;
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