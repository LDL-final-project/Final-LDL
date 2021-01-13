module top(
    input clk,
    input rst,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue
);
    
    stepper_motor step_mt(
        .clk(clk), .rst(rst), .color_id(0),
        .red(red), .yellow(yellow), .blue(blue)
    );
endmodule

module stepper_motor(
    input clk,
    input rst,
    input color_id,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue
);
    wire clk_cnt;
    reg [9:0] r_time, y_time, b_time;
    reg [9:0] next_r_time, next_y_time, next_b_time;
    
    always@(posedge clk, posedge rst) begin
       if(rst) begin
            r_time <= 4'b0;
            y_time <= 4'b0;
            b_time <= 4'b0;
        end
        else begin
            r_time <= next_r_time;
            y_time <= next_y_time;
            b_time <= next_b_time;
        end
    end 
    always@* begin
        case(color_id)
            4'd0: begin
                next_r_time = 5;
                next_y_time = 9;
                next_b_time = 9;
            end
            default: begin
                next_r_time = 0;
                next_y_time = 0;
                next_b_time = 0;
            end
        endcase
    end
    
    stepper_motor_individual stepp_ind(
        .clk(clk), .rst(rst),
        .r_time(r_time), .y_time(y_time), .b_time(b_time),
        .red(red), .yellow(yellow), .blue(blue)
    );
endmodule

module stepper_motor_individual(
    input clk,
    input rst,
    input r_time,
    input y_time,
    input b_time,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue
);
    parameter INIT = 3'd0;
    parameter R_drop = 3'd1;
    parameter RtoY = 3'd2;
    parameter Y_drop = 3'd3;
    parameter YtoB = 3'd4;
    parameter B_drop = 3'd5;
    parameter BtoR = 3'd6;
    
    parameter depth = 3'd5;
    parameter move = 3'd2;
    
    wire clk_19, clk_cnt;
    reg [2:0] state, next_state;
    reg [3:0] count_sec, next_count_sec;
    reg [9:0] count_rnd, next_count_rnd;
    reg en_r, en_y, en_b, dir;
    
    clock_divider #(.n(19)) clk19( .clk(clk), .clk_div(clk_19));
    clock_divider #(.n(27)) clkcnt( .clk(clk), .clk_div(clk_cnt));
    
    always@(posedge clk_cnt, posedge rst) begin
        if(rst)
            count_sec <= 4'b0;
        else
            count_sec <= next_count_sec;
    end
    
    always@(posedge clk_cnt, posedge rst) begin
        if(rst)
            count_rnd <= 10'b0;
        else
            count_rnd <= next_count_rnd;
    end
    
    always@(posedge clk, posedge rst) begin
        if(rst)
            state = INIT;
        else
            state = next_state;
    end
    
    always@* begin
        dir = (count_sec < depth) ? 1'b0 : 1'b1;
    end
    
    always@* begin
        en_r = 1'b0;
        en_y = 1'b0;
        en_b = 1'b0;
        next_state = state;
        next_count_sec = count_sec;
        next_count_rnd = count_rnd;
        case(state)
            INIT: begin
                next_state = R_drop;
                next_count_sec = 4'd0;
                next_count_rnd = 10'd0;
            end
            R_drop: begin
                next_state = (count_rnd < r_time) ? R_drop : RtoY;
                next_count_sec = (count_sec < depth*2) ? count_sec + 1'b1 : 4'd0;
                if (count_sec < depth*2)
                    next_count_rnd = (count_rnd < r_time) ? count_rnd + 1'b1 : 10'd0;
                en_r = 1'b1;
            end
            RtoY: begin
                next_state = (count_sec < move) ? RtoY : Y_drop;
                next_count_sec = (count_sec < move) ? count_sec + 1'b1 : 4'd0;
            end
            Y_drop: begin
                next_state = (count_rnd < y_time) ? Y_drop : YtoB;
                next_count_sec = (count_sec < depth*2) ? count_sec + 1'b1 : 4'd0;
                if(count_sec < depth*2)
                    next_count_rnd = (count_rnd < y_time) ? count_rnd + 1'b1 : 10'd0;
                en_y = 1'b1;
            end
            YtoB: begin
                next_state = (count_sec < move) ? YtoB : B_drop;
                next_count_sec = (count_sec < move) ? count_sec + 1'b1 : 4'd0;
            end
            B_drop: begin
                next_state = (count_rnd < b_time) ? B_drop : BtoR;
                next_count_sec = (count_sec < depth*2) ? count_sec + 1'b1 : 4'd0;
                if(count_sec < depth*2)
                    next_count_rnd = (count_rnd < b_time) ? count_rnd + 1'b1 : 10'd0;
                en_b = 1'b1;
            end
            BtoR: begin
                next_state = (count_sec < move) ? BtoR : INIT;
                next_count_sec = (count_sec < move) ? count_sec + 1 : 4'd0;
            end
            default: begin
                next_state = INIT;
            end
        endcase
    end
    
    stepper_motor_driver motorr(
        .clk(clk_19), .rst(rst), . en(en_r), .dir(dir), .signal(red)
    );
    stepper_motor_driver motory(
        .clk(clk_19), .rst(rst), . en(en_y), .dir(dir), .signal(yellow)
    );
    stepper_motor_driver motorb(
        .clk(clk_19), .rst(rst), . en(en_b), .dir(dir), .signal(blue)
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
