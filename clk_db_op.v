module clock_divider (clk, clk_div);

  parameter n = 25;
  input clk;
  output clk_div;

  reg [n-1:0] num;
  wire [n-1:0] nxt_num;

  assign nxt_num = num + 1;
  assign clk_div = num[n-1];

  always @ (posedge clk) begin
    num = nxt_num;
  end

endmodule

module debounce (pb_debounced, pb, clk);

  input pb, clk;
  output pb_debounced;

  reg [3:0] shift_reg;

  always @(posedge clk) begin
    shift_reg[3:1] <= shift_reg[2:0];
    shift_reg[0]  <= pb;
  end

  assign pb_debounced = ((shift_reg == 4'b1111) ? 1'b1 : 1'b0);

endmodule

module onepulse (
  input wire rst, clk, pb_debounced,
  output reg pb_1pulse
);

  reg pb_1pulse_nxt, pb_debounced_delay;

  always @(*) begin
    pb_1pulse_nxt = pb_debounced & ~pb_debounced_delay;
  end

  always @(posedge clk, posedge rst) begin
    if (rst) begin
      pb_1pulse <= 1'b0;
      pb_debounced_delay <= 1'b0;
    end else begin
      pb_1pulse <= pb_1pulse_nxt;
      pb_debounced_delay <= pb_debounced;
    end
  end
endmodule