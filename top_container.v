module top_container(
  input wire clk, rst,
  inout wire PS2_DATA,
  inout wire PS2_CLK,
  output wire [3:0] vgaRed,
  output wire [3:0] vgaGreen,
  output wire [3:0] vgaBlue,
  output wire hsync, vsync,
  output wire confirm,
  output wire [3:0] color_id
);
  wire [3:0] _color_id;
  assign color_id = _color_id;
  reg state, next_state;
  parameter INIT = 3'd0;
  parameter COLOR1 = 3'd1;
  parameter CAR1 = 3'd2;
  parameter COLOR2 = 3'd3;
  parameter CAR2 = 3'd4;
  parameter COLOR = 3'd5;
  parameter CAR3 = 3'd6;

  keyboard kb (.PS2_DATA(PS2_DATA), .PS2_CLK(PS2_CLK),
	             .rst(rst), .clk(clk), .color_id(_color_id)), .confirm(confirm);

  screen scn (.clk(clk), .rst(rst), .color_id(_color_id),
              .vgaRed(vgaRed), .vgaGreen(vgaGreen), .vgaBlue(vgaBlue),
              .hsync(hsync), .vsync(vsync));
  stepper_motor motorR(
      .clk(clk),
      .rst(rst),
      . _wait(0),
      .sec(r_stay),
      .depth(r_depth),
      .signal_out(red)
  );
  stepper_motor motorY(
      .clk(clk),
      .rst(rst),
      . _wait(r_depth + r_stay + r_depth),
      .sec(y_stay),
      .depth(y_depth),
      .signal_out(yellow)
  );
  stepper_motor motorB(
      .clk(clk),
      .rst(rst),
      . _wait(r_depth + r_stay + r_depth),
      .sec(b_stay),
      .depth(b_depth),
      .signal_out(blue)
  );
  Servo_Motor Ser_1(clk, rst, dir, signal, signal2);

  always @(*) begin
    case (state)

      INIT: begin
        
      end
      COLOR1: begin
        
      end
      CAR1: begin
        
      end
      COLOR2 begin
        
      end
      CAR2: begin
        
      end
      COLOR3: begin
        
      end
      CAR3: begin
        
      end
      default: begin
        
      end

    endcase
  end
  

end module