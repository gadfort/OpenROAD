# test that a right-angle rotated instance honors only the flip, not the rotation
#
# Right-angle rotated macros are normally given their own define_pdn_grid, whose
# parameters are already expressed in the rotated frame, so PDN must not re-map
# them for the rotation.  The flip half still has to be honored:
#   - R90 (W) is a pure rotation, so nothing moves
#   - MXR90 (FW) is R90 flipped, so the x axis mirrors and the vertical (metal6)
#     offsets are measured from the right edge instead
# The horizontal (metal5) offsets stay put for both, so each macro is the
# control for the other's unmirrored axis.
#
# No halo and no rings here: a rotated macro on this floorplan overlaps rows it
# does not cover (PDN-0008) and reaches within 1.2 um of the die edge, so
# neither can be exercised on this fixture.
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_lef nangate_macros/fakeram45_64x32.lef

read_def nangate_macros/floorplan.def

# rotate the macros in place; setLocationOrient keeps the lower-left corner of
# the bounding box.  FIXED instances have to be released to be re-oriented.
proc flip_inst { name orient } {
  set inst [[ord::get_db_block] findInst $name]
  set status [$inst getPlacementStatus]
  $inst setPlacementStatus PLACED
  $inst setLocationOrient $orient
  $inst setPlacementStatus $status
}

flip_inst "frontend.icache.data_arrays_0.data_arrays_0_0_ext.mem" "R90"
flip_inst "dcache.data.data_arrays_0.data_arrays_0_ext.mem" "MXR90"

add_global_connection -net VDD -pin_pattern {^VDD$} -power
add_global_connection -net VDD -pin_pattern {^VDDPE$}
add_global_connection -net VDD -pin_pattern {^VDDCE$}
add_global_connection -net VSS -pin_pattern {^VSS$} -ground
add_global_connection -net VSS -pin_pattern {^VSSE$}

set_voltage_domain -power VDD -ground VSS

define_pdn_grid -name "Core"
add_pdn_stripe -followpins -layer metal1

add_pdn_stripe -layer metal4 -width 0.48 -spacing 4.0 -pitch 49.0 -offset 2.5
add_pdn_stripe -layer metal7 -width 1.4 -pitch 40.0 -offset 2.5

add_pdn_connect -layers {metal1 metal4}
add_pdn_connect -layers {metal4 metal7}

define_pdn_grid -macro -name "sram_r90" \
  -instances "frontend.icache.data_arrays_0.data_arrays_0_0_ext.mem"
add_pdn_stripe -grid "sram_r90" -layer metal5 -width 0.93 -spacing 1.5 -pitch 6.0 -offset 1.5
add_pdn_stripe -grid "sram_r90" -layer metal6 -width 0.93 -spacing 1.5 -pitch 15.0 -offset 2.5

add_pdn_connect -grid "sram_r90" -layers {metal4 metal5}
add_pdn_connect -grid "sram_r90" -layers {metal5 metal6}
add_pdn_connect -grid "sram_r90" -layers {metal6 metal7}

define_pdn_grid -macro -name "sram_fw" \
  -instances "dcache.data.data_arrays_0.data_arrays_0_ext.mem"
add_pdn_stripe -grid "sram_fw" -layer metal5 -width 0.93 -spacing 1.5 -pitch 6.0 -offset 1.5
add_pdn_stripe -grid "sram_fw" -layer metal6 -width 0.93 -spacing 1.5 -pitch 15.0 -offset 2.5

add_pdn_connect -grid "sram_fw" -layers {metal4 metal5}
add_pdn_connect -grid "sram_fw" -layers {metal5 metal6}
add_pdn_connect -grid "sram_fw" -layers {metal6 metal7}

pdngen

set def_file [make_result_file macros_flipped_right_angle.def]
write_def $def_file
diff_files macros_flipped_right_angle.defok $def_file
