# Generator for floorplan_polygon.def in this directory.  Run it from
# src/pdn/test with
#
#   openroad -no_splash -no_init -exit \
#     sky130_power_switch/make_polygon_fixture.tcl
#
# The design is the gcd netlist of floorplan.def; only the floorplan is
# replaced, by an L-shaped die and core.  Every dimension below is a whole
# number of sites (0.46um) in X and of double rows (5.44um) in Y, measured from
# the core origin, so both row sets land exactly on the requested outline: the
# switch cell is two rows tall and sits in the unithddbl rows, so a core height
# that is an odd number of single rows would leave it a partial row.
set site unithd
set dbl_site unithddbl

# L-shaped die: a 276x136 lower arm plus a 138x277.44 left arm.
set die {0 0  276 0  276 136  138 136  138 277.44  0 277.44}
# The core is inset 10.12um (22 sites) and 10.88um (4 rows) from the outer
# edges of the die but only 5.52um (12 sites) and 5.44um (2 rows) at the
# concave corner, so a shape that fits inside the core bounding box can still
# cross the real die boundary at the notch.
set core {10.12 10.88  265.88 10.88  265.88 130.56  132.48 130.56
  132.48 266.56  10.12 266.56}

read_lef sky130hd/sky130hd.tlef
read_lef sky130hd/sky130_fd_sc_hd_merged.lef
read_lef sky130_power_switch/power_switch.lef
read_liberty sky130hd/sky130hd_tt.lib
read_def sky130_power_switch/floorplan.def

# The design arrives with 873 well taps and 54 signal pins.  A power grid test
# needs neither; the taps are FIXED at positions in the old core, so most of
# them would land in the notch of the new die, and both put a great deal of
# unrelated geometry in the golden.  The nPWRUP control net has no terminal, so
# it survives this.
proc strip_unneeded { } {
  set block [ord::get_db_block]
  set drop {}
  foreach inst [$block getInsts] {
    set type [[$inst getMaster] getType]
    if { $type eq "CORE_WELLTAP" || $type eq "CORE_SPACER" } {
      lappend drop $inst
    }
  }
  foreach inst $drop {
    odb::dbInst_destroy $inst
  }
  foreach bterm [$block getBTerms] {
    odb::dbBTerm_destroy $bterm
  }
}
strip_unneeded

initialize_floorplan -die_area $die -core_area $core -site $site \
  -additional_sites $dbl_site

source sky130hd/sky130hd.tracks
global_placement -skip_initial_place
detailed_placement

write_def sky130_power_switch/floorplan_polygon.def
