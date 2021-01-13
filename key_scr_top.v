module top (
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

  keyboard kb (.PS2_DATA(PS2_DATA), .PS2_CLK(PS2_CLK),
	             .rst(rst), .clk(clk), .color_id(_color_id)), .confirm(confirm);

  screen scn (.clk(clk), .rst(rst), .color_id(_color_id),
              .vgaRed(vgaRed), .vgaGreen(vgaGreen), .vgaBlue(vgaBlue),
              .hsync(hsync), .vsync(vsync));

endmodule