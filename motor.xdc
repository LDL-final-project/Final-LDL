## Clock signal
set_property PACKAGE_PIN W5 [get_ports clk]
	set_property IOSTANDARD LVCMOS33 [get_ports clk]
#Buttons
    set_property PACKAGE_PIN U18 [get_ports rst]
        set_property IOSTANDARD LVCMOS33 [get_ports rst]
## Switches
#set_property PACKAGE_PIN V17 [get_ports {direction}]
#	set_property IOSTANDARD LVCMOS33 [get_ports {direction}]
#set_property PACKAGE_PIN V16 [get_ports {en}]
#    set_property IOSTANDARD LVCMOS33 [get_ports {en}]

set_property PACKAGE_PIN H1 [get_ports {red[0]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {red[0]}]
set_property PACKAGE_PIN K2 [get_ports {red[1]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {red[1]}]
set_property PACKAGE_PIN H2 [get_ports {red[2]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {red[2]}]
set_property PACKAGE_PIN G3 [get_ports {red[3]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {red[3]}]
    
set_property PACKAGE_PIN A15 [get_ports {yellow[0]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {yellow[0]}]
set_property PACKAGE_PIN A17 [get_ports {yellow[1]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {yellow[1]}]
set_property PACKAGE_PIN C15 [get_ports {yellow[2]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {yellow[2]}]
set_property PACKAGE_PIN C16 [get_ports {yellow[3]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {yellow[3]}]
           
    
set_property PACKAGE_PIN L17 [get_ports {blue[0]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {blue[0]}]
set_property PACKAGE_PIN M19 [get_ports {blue[1]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {blue[1]}]
set_property PACKAGE_PIN P17 [get_ports {blue[2]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {blue[2]}]
set_property PACKAGE_PIN R18 [get_ports {blue[3]}]
    set_property IOSTANDARD LVCMOS33 [get_ports {blue[3]}]
    
set_property PACKAGE_PIN B16 [get_ports signal_left]
    set_property IOSTANDARD LVCMOS33 [get_ports signal_left]
set_property PACKAGE_PIN P18 [get_ports signal_right]
    set_property IOSTANDARD LVCMOS33 [get_ports signal_right]
    
 set_property PACKAGE_PIN U16 [get_ports {led[0]}]                    
   set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
#Sch name = JC8
set_property PACKAGE_PIN E19 [get_ports {led[1]}]                    
   set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
#Sch name = JC9
set_property PACKAGE_PIN U19 [get_ports {led[2]}]                    
   set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
           
set_property CFGBVS Vcco [current_design]
set_property config_voltage 3.3 [current_design]