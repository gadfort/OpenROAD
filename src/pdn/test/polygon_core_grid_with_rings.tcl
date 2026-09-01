# A ring around an L-shaped core.
#
# The core of nangate_polygon/floorplan.def is
#   (9.5 8.4) (85.5 8.4) (85.5 53.2) (43.7 53.2) (43.7 103.6) (9.5 103.6)
# inside the die
#   (0 0) (95 0) (95 56) (47.5 56) (47.5 112) (0 112)
# so the core-to-die margin is 9.5um/8.4um at the outer edges but only
# 3.8um/2.8um at the concave corner.  These ring dimensions -- 0.2um offset
# plus 2.4um of metal -- fit everywhere including the notch, so the ring is
# buildable as drawn and nothing has to be trimmed.
#
# The ring must follow the L: eight sides per layer, not four, joined at the
# concave corner, with the metal5/metal6 vias placed at all six corners.  A
# ring built from the core bounding box instead runs through the notch, where
# the system obstructions chop it into disconnected segments.
#
# Today it does not: the golden below records a metal5 ring with only its
# bottom side -- the top side is cut by the notch obstructions and then trimmed
# away as dangling metal -- and a metal6 right side that stops at y = 51.885
# instead of 106.2.  Two of the four sides of the ring are gone, with no
# diagnostic.  Regenerate the golden when polygon support lands.
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_def nangate_polygon/floorplan.def

add_global_connection -net VDD -pin_pattern VDD -power
add_global_connection -net VSS -pin_pattern VSS -ground

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_stripe -followpins -layer metal1 -extend_to_core_ring

add_pdn_ring -grid "Core" -layers {metal5 metal6} -widths 0.8 -spacings 0.8 \
  -core_offsets 0.2

add_pdn_connect -layers {metal5 metal6}
add_pdn_connect -layers {metal1 metal6}

pdngen

set def_file [make_result_file polygon_core_grid_with_rings.def]
write_def $def_file
diff_files polygon_core_grid_with_rings.defok $def_file
