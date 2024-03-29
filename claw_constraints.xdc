## Clock signal
set_property PACKAGE_PIN W5 [get_ports clk]
	set_property IOSTANDARD LVCMOS33 [get_ports clk]
#Buttons
    set_property PACKAGE_PIN U18 [get_ports rst]
        set_property IOSTANDARD LVCMOS33 [get_ports rst]
# Switches
set_property PACKAGE_PIN V17 [get_ports {direction}]
	set_property IOSTANDARD LVCMOS33 [get_ports {direction}]
set_property PACKAGE_PIN V16 [get_ports {en}]
    set_property IOSTANDARD LVCMOS33 [get_ports {en}]

set_property PACKAGE_PIN L17 [get_ports {signal_out[0]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {signal_out[0]}]
set_property PACKAGE_PIN M19 [get_ports {signal_out[1]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {signal_out[1]}]
set_property PACKAGE_PIN P17 [get_ports {signal_out[2]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {signal_out[2]}]
set_property PACKAGE_PIN R18 [get_ports {signal_out[3]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {signal_out[3]}]
           
set_property CFGBVS Vcco [current_design]
set_property config_voltage 3.3 [current_design]
