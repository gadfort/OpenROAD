# An L-shaped macro on an L-shaped die.
#
# nangate_polygon/floorplan_polygon_macro.def tucks polygon_macro_L into the
# corner of the L-shaped die, so that the *bounding box* of the macro,
# (11.4 30.8) - (68.4 72.8), reaches 20.9um into the die notch while the body
# of the macro does not:
#
#   body   (11.4 30.8) (68.4 30.8) (68.4 51.8) (39.9 51.8) (39.9 72.8)
#          (11.4 72.8)
#   die    (0 0) (95 0) (95 56) (47.5 56) (47.5 112) (0 112)
#
# Every part of the macro is inside the die.  Reasoning in bounding boxes says
# otherwise, and the two polygons interact: the macro grid extends over the
# notch, where odb has put system-reserved obstructions on every routing layer,
# so the macro straps and ring in that region are cut away rather than simply
# being wasted.
#
# The macro grid must stay on the body of the macro, the core grid must keep
# the area between the macro and the die edge, and no shape may land in the
# notch.
#
# Today neither polygon is honoured: the golden below records the same
# two-sided core ring as polygon_core_grid_with_rings and the same macro-grid
# shapes over the macro notch.  The one thing that is already right is the die
# notch itself -- none of the 1185 shapes reaches it, because odb obstructs it.
# Regenerate the golden when polygon support lands.
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef nangate_polygon/polygon_macro.lef
read_def nangate_polygon/floorplan_polygon_macro.def

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VSS -pin_pattern {^VSS$} -ground

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_stripe -followpins -layer metal1 -extend_to_core_ring
add_pdn_stripe -layer metal4 -width 0.48 -pitch 20.0 -offset 2.0
add_pdn_ring -grid "Core" -layers {metal5 metal6} -widths 0.8 -spacings 0.8 \
  -core_offsets 0.2
add_pdn_stripe -layer metal7 -width 1.4 -pitch 20.0 -offset 2.0

add_pdn_connect -layers {metal1 metal4}
add_pdn_connect -layers {metal1 metal6}
add_pdn_connect -layers {metal5 metal6}
add_pdn_connect -layers {metal4 metal7}

define_pdn_grid -macro -name "polygon_macro" -instances "macro_L"
add_pdn_stripe -layer metal5 -width 0.93 -pitch 10.0 -offset 2.0
add_pdn_stripe -layer metal6 -width 0.93 -pitch 10.0 -offset 2.0

add_pdn_connect -layers {metal4 metal5}
add_pdn_connect -layers {metal5 metal6}
add_pdn_connect -layers {metal6 metal7}

pdngen

set def_file [make_result_file polygon_die_and_macro.def]
write_def $def_file
diff_files polygon_die_and_macro.defok $def_file
