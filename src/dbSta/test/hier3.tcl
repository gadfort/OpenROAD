# hierarchical verilog sample with unconnected pins
source "helpers.tcl"
read_lef example1.lef
read_liberty example1_typ.lib
read_verilog hier3.v
link_design top -hier

puts "Find b1/out2: [get_property [get_pins b1/out2] full_name]"
# b2/out2 is not connected in top, but should still be findable
puts "Find b2/out2: [get_property [get_pins b2/out2] full_name]"
