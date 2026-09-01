# Generator for the fixtures in this directory.  One fixture per process; run
# all four from src/pdn/test with
#
#   for f in floorplan floorplan_macro floorplan_macro_r180 \
#            floorplan_polygon_macro ; do
#     FIXTURE=$f openroad -no_splash -no_init -exit \
#       nangate_polygon/make_fixtures.tcl
#   done
#
# The netlist is gcd, borrowed from nangate_gcd/floorplan.def; only the
# floorplan is replaced.  Every dimension below is a whole number of sites
# (0.19um) in X and of rows (1.4um) in Y, so the generated row set lands
# exactly on the requested core outline.
set site FreePDK45_38x28_10R_NP_162NW_34O

# L-shaped die: a 95x56 lower arm plus a 47.5x112 left arm.
set poly_die {0 0  95 0  95 56  47.5 56  47.5 112  0 112}
# L-shaped core.  The inset is deliberately uneven -- 9.5um/8.4um at the outer
# edges but only 3.8um/2.8um at the concave corner -- so that a ring can fit
# inside the die *bounding box* and still cross the real die boundary at the
# notch.  That is the case PDN-0351 has to catch.
set poly_core {9.5 8.4  85.5 8.4  85.5 53.2  43.7 53.2  43.7 103.6  9.5 103.6}

# Rectangular floorplan for the macro-only fixtures.
set rect_die {0 0 133 112}
set rect_core {9.5 8.4 123.5 103.6}

read_lef Nangate45/Nangate45.lef
read_lef nangate_polygon/polygon_macro.lef
read_liberty Nangate45/Nangate45_typ.lib
read_def nangate_gcd/floorplan.def

# Place polygon_macro_L so that its *bounding box* starts at (x, y) um,
# whatever the orientation.  setOrigin moves the master origin, which for a
# rotated master is not the lower-left corner of the placed bounding box, so
# the offset has to be measured after the orientation is applied.
proc place_macro { x y orient } {
  set block [ord::get_db_block]
  set master [[ord::get_db] findMaster polygon_macro_L]
  set inst [odb::dbInst_create $block $master "macro_L"]
  $inst setOrient $orient
  $inst setOrigin 0 0
  set bbox [$inst getBBox]
  $inst setOrigin [expr { int($x * 2000) - [$bbox xMin] }] \
    [expr { int($y * 2000) - [$bbox yMin] }]
  $inst setPlacementStatus FIRM
}

set fixture $::env(FIXTURE)
switch -exact -- $fixture {
  floorplan {
    # polygon die and core, no macro
    initialize_floorplan -die_area $poly_die -core_area $poly_core -site $site
  }
  floorplan_macro {
    # rectangular die and core, one L-shaped macro.  The notch of the macro is
    # at the upper right of its bounding box.
    initialize_floorplan -die_area $rect_die -core_area $rect_core -site $site
    place_macro 57 50.4 R0
  }
  floorplan_macro_r180 {
    # the same bounding box rotated 180 degrees, so the notch moves to the
    # lower left.  Any outline handling that ignores the instance transform
    # gets this one backwards.
    initialize_floorplan -die_area $rect_die -core_area $rect_core -site $site
    place_macro 57 50.4 R180
  }
  floorplan_polygon_macro {
    # polygon die and core with the L-shaped macro tucked into the corner of
    # the L.  The bounding box of the macro, (11.4 30.8) - (68.4 72.8), reaches
    # 20.9um into the die notch; the body of the macro does not.  The macro is
    # kept ~2um clear of the core boundary so the core ring is not cut by the
    # min-spacing bloat of the macro obstructions, which would confuse the
    # polygon question with an unrelated one.
    initialize_floorplan -die_area $poly_die -core_area $poly_core -site $site
    place_macro 11.4 30.8 R0
  }
  default {
    puts "unknown fixture: $fixture"
    exit 1
  }
}

source Nangate45/Nangate45.tracks
global_placement -skip_initial_place
detailed_placement
write_def nangate_polygon/$fixture.def
