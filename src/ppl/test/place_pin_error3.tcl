# gcd_nangate45 IO placement
source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_def gcd.def

catch { place_pin -pin_name clk -layer metal7 -location {40 30} -pin_size {1.6 0.35} } error
puts $error
