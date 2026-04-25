# operating conditions and boundary conditions #

set cycle  10.0         ;#clock period defined by designer

create_clock -period $cycle [get_ports  clk]
set_dont_touch_network      [get_clocks clk]
set_clock_uncertainty  0.1  [get_clocks clk]
set_clock_latency      0.5  [get_clocks clk]
    

set_input_delay  [expr $cycle * 0.5] -clock clk [remove_from_collection [all_inputs] {clk}]
set_output_delay [expr $cycle * 0.5] -clock clk [all_outputs]
set_drive 1 [all_inputs]
set_load 0.05 [all_outputs]

               

set_max_fanout 20 [all_inputs]
