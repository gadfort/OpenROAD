# The macro grid of polygon_macro_grid_with_rings with the macro rotated 180
# degrees.
#
# nangate_polygon/floorplan_macro_r180.def places polygon_macro_L at the same
# bounding box (57 50.4) - (114 92.4) but with orient S, which moves the notch
# from the upper right of the bounding box to the lower left:
#   (85.5 50.4) (114 50.4) (114 92.4) (57 92.4) (57 71.4) (85.5 71.4)
#
# The outline has to be read through inst->getTransform() like the supply pins
# already are.  An implementation that takes the master polygon as-drawn gets
# this design exactly backwards -- it would build the ring around the notch and
# leave the body of the macro bare -- and the bounding box is symmetric under
# R180, so nothing else in the flow would notice.
#
# Today the result is byte-identical to polygon_macro_grid_with_rings in the
# ring geometry -- same four rectangles around (49 42.4) - (122 100.4) -- which
# is the tell: the bounding box is invariant under R180 and nothing else is
# consulted.  Regenerate the golden when polygon support lands.
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef nangate_polygon/polygon_macro.lef
read_def nangate_polygon/floorplan_macro_r180.def

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VSS -pin_pattern {^VSS$} -ground

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_stripe -followpins -layer metal1
add_pdn_stripe -layer metal4 -width 0.48 -pitch 20.0 -offset 2.0
add_pdn_stripe -layer metal7 -width 1.4 -pitch 20.0 -offset 2.0 \
  -extend_to_core_ring

add_pdn_connect -layers {metal1 metal4}
add_pdn_connect -layers {metal4 metal7}

define_pdn_grid -macro -name "polygon_macro" -instances "macro_L"
add_pdn_ring -grid "polygon_macro" -layers {metal5 metal6} -widths 2.0 \
  -spacings 2.0 -core_offsets 2.0
add_pdn_stripe -layer metal5 -width 0.93 -pitch 10.0 -offset 2.0 \
  -extend_to_core_ring
add_pdn_stripe -layer metal6 -width 0.93 -pitch 10.0 -offset 2.0 \
  -extend_to_core_ring

add_pdn_connect -layers {metal4 metal5}
add_pdn_connect -layers {metal5 metal6}
add_pdn_connect -layers {metal6 metal7}

pdngen

set def_file [make_result_file polygon_macro_grid_r180.def]
write_def $def_file
diff_files polygon_macro_grid_r180.defok $def_file
