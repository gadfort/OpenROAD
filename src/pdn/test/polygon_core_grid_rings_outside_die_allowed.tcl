# The ring of polygon_core_grid_rings_outside_die, opted in with
# -allow_out_of_die.
#
# -allow_out_of_die is documented as "the ring shapes are allowed to be outside
# the die boundary. This should be used with caution", so the warning has to be
# the one the user asked for -- PDN-0239 -- and not silence.  Without a
# polygon-aware containment test neither branch of Rings::checkDieArea is
# reached on this design, so passing the flag changes nothing.
#
# Today neither branch is reached, so no PDN-0239 appears in the golden below,
# and the geometry it records is a ring with only a bottom and a left side --
# the top and right sides were cut at the notch and trimmed away.  Regenerate
# the golden when the containment test becomes polygon-aware.
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_def nangate_polygon/floorplan.def

add_global_connection -net VDD -pin_pattern VDD -power
add_global_connection -net VSS -pin_pattern VSS -ground

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_stripe -followpins -layer metal1 -extend_to_core_ring

add_pdn_ring -grid "Core" -layers {metal5 metal6} -widths 2.0 \
  -spacings 2.0 -core_offsets 2.0 -add_connect -allow_out_of_die

pdngen

set def_file [make_result_file polygon_core_grid_rings_outside_die_allowed.def]
write_def $def_file
diff_files polygon_core_grid_rings_outside_die_allowed.defok $def_file
