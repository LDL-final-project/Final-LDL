module top(
    input clk,
    input rst,
    output [3:0] signal_out
);
    stepper_motor sp(
        .clk(clk), .rst(rst), . _wait(0), .sec(10), .depth(5), .signal_out(signal_out));
endmodule

module stepper_motor(
    input clk,
    input rst,
    input [3:0] _wait,
    input [3:0] sec,
    input [3:0] depth,
    output [3:0] signal_out
);
    
    wire clk_21, clk_25;
    wire _en, _dir;
    reg [31:0] count, next_count;
    reg en, dir;
    
    clock_divider #(.n(19)) clock_div( .clk(clk), .clk_div(clk_21));
    clock_divider #(.n(27)) clk25( .clk(clk), .clk_div(clk_25));
    
    assign _en = en;
    assign _dir = _dir;
    
    always@(posedge clk_25, posedge rst) begin
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
            dir = 1'b1;
        else
            dir = 1'b0;
    end
    
    stepper_motor_driver sd(
        .clk(clk_21),
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