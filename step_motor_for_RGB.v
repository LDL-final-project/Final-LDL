module box(
    input clk,
    input rst,
    input color_id,
    output [3:0] red,
    output [3:0] yellow,
    output [3:0] blue,
    output location_0,
    output location_1,
    output location_2
);

    wire r_depth, r_stay;
    wire y_depth, y_stay;
    wire b_depth, b_stay;
    reg r, y, b;
    reg next_r_depth, next_r_stay;
    reg next_y_depth, next_y_stay;
    reg next_b_depth, next_b_stay;
    
    assign r_depth = next_r_depth;
    assign y_depth = next_y_depth;
    assign b_depth = next_b_depth;
    assign r_stay = next_r_stay;
    assign y_stay = next_y_stay;
    assign b_stay = next_b_stay;
    
    always@(posedge clk) begin
        case(color_id)
            4'd0: begin
                next_r_depth <= 10'd5;
                next_y_depth <= 10'd5;
                next_b_depth <= 10'd5;
            end
            4'd1: begin
                next_r_depth <= 10'd10;
                next_y_depth <= 10'd5;
                next_b_depth <= 10'd5;
            end
            4'd2: begin
                next_r_depth <= 10'd5;
                next_y_depth <= 10'd10;
                next_b_depth <= 10'd5;
            end
            4'd3: begin
                next_r_depth <= 10'd5;
                next_y_depth <= 10'd5;
                next_b_depth <= 10'd10;
            end
            4'd4: begin
                next_r_depth <= 10'd10;
                next_y_depth <= 10'd10;
                next_b_depth <= 10'd5;
            end
            4'd5: begin
                next_r_depth <= 10'd5;
                next_y_depth <= 10'd10;
                next_b_depth <= 10'd10;
            end
            4'd6: begin
                next_r_depth <= 10'd10;
                next_y_depth <= 10'd5;
                next_b_depth <= 10'd10;
            end
            4'd7: begin
                next_r_depth <= 10'd15;
                next_y_depth <= 10'd10;
                next_b_depth <= 10'd5;
            end
            4'd8: begin
                next_r_depth <= 10'd15;
                next_y_depth <= 10'd5;
                next_b_depth <= 10'd10;
            end
            4'd9: begin
                next_r_depth <= 10'd10;
                next_y_depth <= 10'd15;
                next_b_depth <= 10'd5;
            end
            4'd10: begin
                next_r_depth <= 10'd5;
                next_y_depth <= 10'd15;
                next_b_depth <= 10'd10;
            end
            4'd11: begin
                next_r_depth <= 10'd10;
                next_y_depth <= 10'd5;
                next_b_depth <= 10'd15;
            end
            default: begin
            
            end
        endcase
    end
    always@(posedge clk) begin
        case(color_id)
            4'd0: begin
                next_r_stay <= 10'd5;
                next_y_stay <= 10'd5;
                next_b_stay <= 10'd5;
            end
            4'd1: begin
                next_r_stay <= 10'd10;
                next_y_stay <= 10'd5;
                next_b_stay <= 10'd5;
            end
            4'd2: begin
                next_r_stay <= 10'd5;
                next_y_stay <= 10'd10;
                next_b_stay <= 10'd5;
            end
            4'd3: begin
                next_r_stay <= 10'd5;
                next_y_stay <= 10'd5;
                next_b_stay <= 10'd10;
            end
            4'd4: begin
                next_r_stay <= 10'd10;
                next_y_stay <= 10'd10;
                next_b_stay <= 10'd5;
            end
            4'd5: begin
                next_r_stay <= 10'd5;
                next_y_stay <= 10'd10;
                next_b_stay <= 10'd10;
            end
            4'd6: begin
                next_r_stay <= 10'd10;
                next_y_stay <= 10'd5;
                next_b_stay <= 10'd10;
            end
            4'd7: begin
                next_r_stay <= 10'd15;
                next_y_stay <= 10'd10;
                next_b_stay <= 10'd5;
            end
            4'd8: begin
                next_r_stay <= 10'd15;
                next_y_stay <= 10'd5;
                next_b_stay <= 10'd10;
            end
            4'd9: begin
                next_r_stay <= 10'd10;
                next_y_stay <= 10'd15;
                next_b_stay <= 10'd5;
            end
            4'd10: begin
                next_r_stay <= 10'd5;
                next_y_stay <= 10'd15;
                next_b_stay <= 10'd10;
            end
            4'd11: begin
                next_r_stay <= 10'd10;
                next_y_stay <= 10'd5;
                next_b_stay <= 10'd15;
            end
            default: begin
            
            end
        endcase
    end
    
    stepper_motor motorr(
        .clk(clk),
        .rst(rst),
        . _wait(0),
        .sec(r_stay),
        .depth(r_depth),
        .signal_out(red)
    );
    stepper_motor motory(
        .clk(clk),
        .rst(rst),
        . _wait(r_depth + r_stay + r_depth),
        .sec(y_stay),
        .depth(y_depth),
        .signal_out(yellow)
    );
    stepper_motor motorb(
        .clk(clk),
        .rst(rst),
        . _wait(r_depth + r_stay + r_depth),
        .sec(b_stay),
        .depth(b_depth),
        .signal_out(blue)
    );
endmodule

module stepper_motor(
    input clk,
    input rst,
    input _wait,
    input sec,
    input depth,
    output [3:0] signal_out
);
    
    wire clk_22;
    wire _en, _dir;
    reg [31:0] count, next_count;
    reg en, dir;
    
    clock_divider #(.n(22)) clock_div( .clk(clk), .clk_div(clk_22));
    
    assign _en = en;
    assign _dir = _dir;
    
    always@(posedge clk, posedge rst) begin
        if(rst)
            count <= 32'b0;
        else
            count <= next_count;
    end
    
    always@* begin
        next_count = count + 1'b1;
    end
    
    always@* begin
        if(count < _wait)
            en = 1'b0;
        else if(count > _wait && count < _wait + depth)
            en = 1'b1;
        else if(count > _wait + depth && count < _wait + depth + sec)
            en = 1'b0;
        else if(count > _wait + depth + sec && count < _wait + depth + sec + depth)
            en = 1'b1;
        else
            en = 1'b0;
    end
    
    always@* begin
        if(count < _wait + depth + (sec>>1))
            dir = 1'b0;
        else
            dir = 1'b1;
    end
    
    Stepper_motor control(
        .clk(clk_22),
        .rst(rst),
        .dir(_dir),
        .en(_en),        
        .signal(signal_out)
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