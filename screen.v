
module color(
    input wire clk, rst,
    input wire [9:0] h_cnt,
    input wire [9:0] v_cnt,
    input wire [3:0] color_id,
    output reg [23:0] RGB
  );
  parameter C00 = 24'hEE190E;
  parameter C01 = 24'hF38B39;
  parameter C02 = 24'hF6E12E;
  parameter C03 = 24'hAAF12E;
  parameter C10 = 24'h36982D;
  parameter C11 = 24'h2BB080;
  parameter C12 = 24'h36E5DA;
  parameter C13 = 24'h3BB5EA;
  parameter C20 = 24'h3E7FF2;
  parameter C21 = 24'h3949EE;
  parameter C22 = 24'h4B2FBD;
  parameter C23 = 24'h8139CD;
  parameter C00_s = 24'hF3908B;
  parameter C01_s = 24'hEEB181;
  parameter C02_s = 24'hF5EB93;
  parameter C03_s = 24'hD7EEAF;
  parameter C10_s = 24'h88C384;
  parameter C11_s = 24'h94DFC4;
  parameter C12_s = 24'h9DF7F2;
  parameter C13_s = 24'hA4DDF5;
  parameter C20_s = 24'hA2C0F6;
  parameter C21_s = 24'hA2A9F6;
  parameter C22_s = 24'hAFA1E8;
  parameter C23_s = 24'hC9A9EC;


  always @(*) begin
    if (v_cnt >= 80 && v_cnt < 160 && h_cnt >= 100 && h_cnt < 180) RGB = color_id == 4'd0 ? C00_s : C00;
    else if (v_cnt >= 80 && v_cnt < 160 && h_cnt >= 220 && h_cnt < 300) RGB = color_id == 4'd1 ? C01_s : C01;
    else if (v_cnt >= 80 && v_cnt < 160 && h_cnt >= 340 && h_cnt < 420) RGB = color_id == 4'd2 ? C02_s : C02;
    else if (v_cnt >= 80 && v_cnt < 160 && h_cnt >= 460 && h_cnt < 540) RGB = color_id == 4'd3 ? C03_s : C03;
    else if (v_cnt >= 200 && v_cnt < 280 && h_cnt >= 100 && h_cnt < 180) RGB = color_id == 4'd4 ? C10_s : C10;
    else if (v_cnt >= 200 && v_cnt < 280 && h_cnt >= 220 && h_cnt < 300) RGB = color_id == 4'd5 ? C11_s : C11;
    else if (v_cnt >= 200 && v_cnt < 280 && h_cnt >= 340 && h_cnt < 420) RGB = color_id == 4'd6 ? C12_s : C12;
    else if (v_cnt >= 200 && v_cnt < 280 && h_cnt >= 460 && h_cnt < 540) RGB = color_id == 4'd7 ? C13_s : C13;
    else if (v_cnt >= 320 && v_cnt < 400 && h_cnt >= 100 && h_cnt < 180) RGB = color_id == 4'd8 ? C20_s : C20;
    else if (v_cnt >= 320 && v_cnt < 400 && h_cnt >= 220 && h_cnt < 300) RGB = color_id == 4'd9 ? C21_s : C21;
    else if (v_cnt >= 320 && v_cnt < 400 && h_cnt >= 340 && h_cnt < 420) RGB = color_id == 4'd10 ? C22_s : C22;
    else if (v_cnt >= 320 && v_cnt < 400 && h_cnt >= 460 && h_cnt < 540) RGB = color_id == 4'd11 ? C23_s : C23;
    else RGB = 24'd0;
  end
    
endmodule

module vga_controller (
    input wire pclk, reset,
    output wire hsync, vsync, valid,
    output wire [9:0] h_cnt,
    output wire [9:0] v_cnt
    );

    reg [9:0]pixel_cnt;
    reg [9:0]line_cnt;
    reg hsync_i,vsync_i;

    parameter HD = 640;
    parameter HF = 16;
    parameter HS = 96;
    parameter HB = 48;
    parameter HT = 800; 
    parameter VD = 480;
    parameter VF = 10;
    parameter VS = 2;
    parameter VB = 33;
    parameter VT = 525;
    parameter hsync_default = 1'b1;
    parameter vsync_default = 1'b1;

    always @(posedge pclk)
        if (reset)
            pixel_cnt <= 0;
        else
            if (pixel_cnt < (HT - 1))
                pixel_cnt <= pixel_cnt + 1;
            else
                pixel_cnt <= 0;

    always @(posedge pclk)
        if (reset)
            hsync_i <= hsync_default;
        else
            if ((pixel_cnt >= (HD + HF - 1)) && (pixel_cnt < (HD + HF + HS - 1)))
                hsync_i <= ~hsync_default;
            else
                hsync_i <= hsync_default; 

    always @(posedge pclk)
        if (reset)
            line_cnt <= 0;
        else
            if (pixel_cnt == (HT -1))
                if (line_cnt < (VT - 1))
                    line_cnt <= line_cnt + 1;
                else
                    line_cnt <= 0;

    always @(posedge pclk)
        if (reset)
            vsync_i <= vsync_default; 
        else if ((line_cnt >= (VD + VF - 1)) && (line_cnt < (VD + VF + VS - 1)))
            vsync_i <= ~vsync_default; 
        else
            vsync_i <= vsync_default; 

    assign hsync = hsync_i;
    assign vsync = vsync_i;
    assign valid = ((pixel_cnt < HD) && (line_cnt < VD));

    assign h_cnt = (pixel_cnt < HD) ? pixel_cnt : 10'd0;
    assign v_cnt = (line_cnt < VD) ? line_cnt : 10'd0;

endmodule

module screen (
  input wire clk, rst,
  input wire [3:0] color_id,
  output wire [3:0] vgaRed,
  output wire [3:0] vgaGreen,
  output wire [3:0] vgaBlue,
  output wire hsync, vsync
);

  wire clk_22, clk_2, clk_16;
  wire valid, dbRst;
  wire [9:0] h_cnt; //640
  wire [9:0] v_cnt; //480
  wire [23:0] RGB;
  reg  [7:0] Red, Green, Blue;

  clock_divider #(.n (2)) clk2 (.clk (clk), .clk_div (clk_2));
  clock_divider #(.n (22)) clk22 (.clk (clk), .clk_div (clk_22));
  clock_divider #(.n (16)) clk16 (.clk (clk), .clk_div (clk_16));
  debounce db_rst (.pb_debounced (dbRst), .pb (rst), .clk (clk_16));  
  assign {vgaRed, vgaGreen, vgaBlue} = (valid == 1'b1) ? {Red[3:0], Green[3:0], Blue[3:0]} : 12'h0;

  vga_controller   vga_inst(
    .pclk(clk_2),
    .reset(rst),
    .hsync(hsync),
    .vsync(vsync),
    .valid(valid),
    .h_cnt(h_cnt),
    .v_cnt(v_cnt)
  );

  color color_init(
    .clk(clk_22),
    .rst(dbRst),
    .h_cnt(h_cnt),
    .v_cnt(v_cnt),
    .color_id(color_id),
    .RGB(RGB)
  );

  always @(*) begin
    Red = RGB[23:16] >> 4;
    Green = RGB[15:8] >> 4;
    Blue = RGB[7:0] >> 4;    
  end

endmodule