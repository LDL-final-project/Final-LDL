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
  stepper_motor redMotor(
        .clk(clk_19), .rst(rst),
        .dir(dir_for_red_motor),
        .en(en_for_red_motor),        
        .signal(signal_out_red)
        );    
    
  stepper_motor yellowMotor(
      .clk(clk_19), .rst(rst),
      .dir(dir_for_yellow_motor),
      .en(en_for_yellow_motor),        
      .signal(signal_out_yellow)
  );
  stepper_motor BlueMotor(
      .clk(clk_19), .rst(rst),
      .dir(dir_for_blue_motor),
      .en(en_for_blue_motor),        
      .signal(signal_out_blue)
  );
  Server_top Ser_1(
      .clk(clk), .rst(rst), .runtime(runtime), .dir(dir_for_car), 
      .signal(signal), .signal2(signal2)
  );

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