
module pmod_step_interface(
    input clk,
    input rst,
    input direction,
    input en,
    output [3:0] signal_out
    );
    
    wire new_clk;
    
    clock_divider #(.n(20)) clock_div(
        .clk(clk),
        .clk_div(new_clk)
        );
    
    pmod_step_driver control(
        .rst(rst),
        .dir(direction),
        .clk(new_clk),
        .en(en),
        .signal(signal_out)
        );    
    
endmodule
