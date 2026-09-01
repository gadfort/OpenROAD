# A macro grid over an L-shaped hard macro.
#
# nangate_polygon/polygon_macro.lef declares
#   SIZE 57 BY 42 ;
#   OBS LAYER OVERLAP ; POLYGON 0 0 57 0 57 21 28.5 21 28.5 42 0 42 ;
# LEF has no polygonal MACRO SIZE, so SIZE is the bounding box and the
# OVERLAP-layer obstruction carries the real outline -- which is what LEF/DEF
# defines it for.  The notch x >= 28.5, y >= 21 of the bounding box is not part
# of the macro; the macro draws nothing there and no supply pin reaches into
# it.
#
# InstanceGrid::getDomainArea is inst->getBBox(), so the macro straps are
# generated across the whole 57x42 bounding box.  In the notch there is no
# macro metal4 pin to land a via on, so the metal5 and metal6 straps there
# connect to nothing: they are dangling supply metal over what should be free
# area.
#
# The straps must cover the L body only, and every strap must reach a via.
#
# Today they do not: the golden below records 11 metal5/metal6 macro-grid
# shapes over the notch, several of them reaching past the last macro supply
# pin at y = 71.4 with no via to land on.  The metal1 rails and the metal4 and
# metal7 core straps in the notch are correct and must stay -- the notch is
# ordinary free core area.  Regenerate the golden when polygon support lands.
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef nangate_polygon/polygon_macro.lef
read_def nangate_polygon/floorplan_macro.def

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VSS -pin_pattern {^VSS$} -ground

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_stripe -followpins -layer metal1
add_pdn_stripe -layer metal4 -width 0.48 -pitch 20.0 -offset 2.0
add_pdn_stripe -layer metal7 -width 1.4 -pitch 20.0 -offset 2.0

add_pdn_connect -layers {metal1 metal4}
add_pdn_connect -layers {metal4 metal7}

define_pdn_grid -macro -name "polygon_macro" -instances "macro_L"
add_pdn_stripe -layer metal5 -width 0.93 -pitch 10.0 -offset 2.0
add_pdn_stripe -layer metal6 -width 0.93 -pitch 10.0 -offset 2.0

add_pdn_connect -layers {metal4 metal5}
add_pdn_connect -layers {metal5 metal6}
add_pdn_connect -layers {metal6 metal7}

pdngen

set def_file [make_result_file polygon_macro_grid.def]
write_def $def_file
diff_files polygon_macro_grid.defok $def_file
