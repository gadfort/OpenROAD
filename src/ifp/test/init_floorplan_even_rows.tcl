# init_floorplan
source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_liberty Nangate45/Nangate45_typ.lib
read_verilog reg1.v
link_design top
initialize_floorplan -die_area "0 0 1000 1000" \
  -core_area "100 100 900 898" \
  -site FreePDK45_38x28_10R_NP_162NW_34O \
  -row_parity EVEN

set def_file [make_result_file init_floorplan_even_rows.def]
write_def $def_file
diff_files init_floorplan_even_rows.defok $def_file
