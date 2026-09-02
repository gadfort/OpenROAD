# Straps extended to the boundary of an L-shaped die.
#
# -extend_to_boundary resolves through Grid::getGridBoundary, which is
# block->getDieArea() -- the bounding box.  On the L-shaped die of
# nangate_polygon/floorplan.def that runs every strap the full 95x112 extent of
# the bounding box, including the part of each strap that lies in the notch
# where there is no die.
#
# A metal4 strap at x > 47.5 must stop at y = 56 and a metal7 strap at y > 56
# must stop at x = 47.5.  The system obstructions over the notch do trim the
# overhang, so the surviving geometry is right; what this pins is that nothing
# is left protruding and that the straps still reach the die edge on the sides
# where the die really does extend that far.
#
# Verified to hold today, by the same obstruction cut as polygon_core_grid:
# none of the 787 shapes in the result reaches the notch.  The golden should
# survive the polygon work unchanged -- and it did.  What changed is the wasted
# work, not the geometry.
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_def nangate_polygon/floorplan.def

add_global_connection -net VDD -pin_pattern VDD -power
add_global_connection -net VSS -pin_pattern VSS -ground

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_stripe -followpins -layer metal1
add_pdn_stripe -layer metal4 -width 0.48 -pitch 20.0 -offset 2.0 \
  -extend_to_boundary
add_pdn_stripe -layer metal7 -width 1.4 -pitch 20.0 -offset 2.0 \
  -extend_to_boundary

add_pdn_connect -layers {metal1 metal4}
add_pdn_connect -layers {metal4 metal7}

pdngen

set def_file [make_result_file polygon_core_grid_extend_to_boundary.def]
write_def $def_file
diff_files polygon_core_grid_extend_to_boundary.defok $def_file
