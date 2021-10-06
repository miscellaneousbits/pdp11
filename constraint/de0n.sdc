create_clock -period 20ns clkin
create_clock -period 100ns cpuclk
#create_clock -period 100ns dram_clk
create_clock -period 100ns unibus:pdp11|rh11:rh0|sdspi:sd1|clk -name clk_rk

derive_pll_clocks -use_net_name
derive_clock_uncertainty

set_false_path -from [get_clocks {*}] -to [get_pins {*}]
set_false_path -to [get_clocks {*}] -from [get_pins {*}]
#set_false_path -to * -from [get_ports {key0}]
#set_false_path -to * -from [get_ports {key1}]
#set_false_path -from * -to [get_ports {panel_col*}]
#set_false_path -from * -to [get_ports {panel_xled*}]

set_clock_groups -asynchronous -group clkin -group cpuclk -group clk_rk
