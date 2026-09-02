# Generator for the fixtures in this directory.  One fixture per process; run
# them all from src/pdn/test with
#
#   for f in floorplan floorplan_u floorplan_macro floorplan_macro_r180 \
#            floorplan_macro_r90 floorplan_polygon_macro \
#            floorplan_macro_tall floorplan_two_macros ; do
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

# U-shaped die: a 152x56 base with two 32.3um arms rising from its ends.  The
# notch is cut inwards from the top edge, so the core has two concave corners
# and the empty area between the arms is enclosed on three sides -- a different
# topology from the single concave corner of the L above.
set u_die {0 0  152 0  152 112  104.5 112  104.5 56  47.5 56  47.5 112  0 112}
set u_core {9.5 8.4  142.5 8.4  142.5 103.6  110.2 103.6  110.2 50.4
  41.8 50.4  41.8 103.6  9.5 103.6}

# The same U-shaped die with a core whose arms are exactly as wide as the
# macro placed in each of them (28.5um), so that a macro fills its arm from
# side to side.  A wider arm would leave a strip of rows a couple of microns
# wide alongside the macro, too narrow for a strap and so an unrepairable
# followpin channel (PDN-0179), which has nothing to do with the polygon
# question.  The arms are inset 9.5um from the walls of the die notch, as the
# outer edges are.
set two_macro_core {9.5 8.4  142.5 8.4  142.5 103.6  114 103.6  114 50.4
  38 50.4  38 103.6  9.5 103.6}

# A padring around a core whose four corners are cut away.  Each cut turns one
# convex corner into two convex corners and one concave one, so this shape has
# four concave corners, one in each of the four orientations -- the L has one
# and the U has two, both facing the same way.
#
# The die is 570x571.2 with a 140um pad row on each side, and the core is inset
# far enough from the pads to leave room for the ring between them: the ring
# reaches 14um out of the core and the nearest pad edge is 21.5um away.
# given as four vertices rather than two corners: polygon mode wants the die
# spelled out the same way as the core
set pad_die {0 0  570 0  570 571.2  0 571.2}
set pad_core {199.5 162.4  370.5 162.4  370.5 201.6  408.5 201.6
  408.5 369.6  370.5 369.6  370.5 408.8  199.5 408.8
  199.5 369.6  161.5 369.6  161.5 201.6  199.5 201.6}

# Rectangular floorplan for the macro-only fixtures.
set rect_die {0 0 133 112}
set rect_core {9.5 8.4 123.5 103.6}

read_lef Nangate45/Nangate45.lef
read_lef nangate_bsg_black_parrot/dummy_pads.lef
read_lef nangate_polygon/polygon_macro.lef
read_lef nangate_polygon/polygon_macro_tall.lef
read_liberty Nangate45/Nangate45_typ.lib
read_def nangate_gcd/floorplan.def

# The gcd netlist arrives with 168 filler cells and 54 signal pins.  A power
# grid test needs neither, and carrying them puts a few thousand lines of
# unrelated geometry in every golden, so they go before the floorplan is
# rebuilt.  Removing the fillers is what dpl asks for anyway (DPL-0037).
proc strip_unneeded { } {
  set block [ord::get_db_block]
  set fillers {}
  foreach inst [$block getInsts] {
    if { [[$inst getMaster] getType] eq "CORE_SPACER" } {
      lappend fillers $inst
    }
  }
  foreach inst $fillers {
    odb::dbInst_destroy $inst
  }
  foreach bterm [$block getBTerms] {
    odb::dbBTerm_destroy $bterm
  }
}
strip_unneeded

# Place a macro so that its *bounding box* starts at (x, y) um, whatever the
# orientation.  setOrigin moves the master origin, which for a rotated master
# is not the lower-left corner of the placed bounding box, so the offset has to
# be measured after the orientation is applied.
proc place_macro { master_name name x y orient } {
  set block [ord::get_db_block]
  set master [[ord::get_db] findMaster $master_name]
  set inst [odb::dbInst_create $block $master $name]
  $inst setOrient $orient
  $inst setOrigin 0 0
  set bbox [$inst getBBox]
  $inst setOrigin [expr { int($x * 2000) - [$bbox xMin] }] \
    [expr { int($y * 2000) - [$bbox yMin] }]
  $inst setPlacementStatus FIRM
  return $inst
}

# Place a pad so that its bounding box starts at (x, y) um.  The _H masters
# carry their core-side pin along the top edge as drawn, so the orientation has
# to turn that edge towards the core: R270 for a pad on the west side and R90
# for one on the east, which are DEF's E and W.  R90 on the west would face the
# pin at the die instead.  The bounding box has to be measured after the turn.
proc place_pad { master_name name x y orient } {
  set block [ord::get_db_block]
  set master [[ord::get_db] findMaster $master_name]
  set inst [odb::dbInst_create $block $master $name]
  $inst setOrient $orient
  $inst setOrigin 0 0
  set bbox [$inst getBBox]
  $inst setOrigin [expr { int($x * 2000) - [$bbox xMin] }] \
    [expr { int($y * 2000) - [$bbox yMin] }]
  $inst setPlacementStatus FIRM
  return $inst
}

# Return the rectangles of the macro's real outline, in placed coordinates, by
# reading the OVERLAP-layer obstruction that LEF/DEF defines as the occupied
# area of a non-rectangular macro.  odb has already decomposed the polygon into
# rectangles for getObstructions().
#
# The corners are moved with dbTransform::apply(Point&); the Rect overload is
# unusable from Tcl because odb's SWIG interface maps any `odb::Rect &r`
# parameter to an output-only argument (dbtypes.i, WRAP_OBJECT_RETURN_REF).
proc macro_body_rects { inst } {
  set xform [$inst getTransform]
  set rects {}
  foreach ob [[$inst getMaster] getObstructions] {
    set layer [$ob getTechLayer]
    if { $layer eq "NULL" || [$layer getName] ne "OVERLAP" } {
      continue
    }
    set xs {}
    set ys {}
    foreach corner [list [list [$ob xMin] [$ob yMin]] [list [$ob xMax] [$ob yMax]]] {
      set pt [odb::new_Point [lindex $corner 0] [lindex $corner 1]]
      $xform apply $pt
      lappend xs [$pt getX]
      lappend ys [$pt getY]
    }
    lappend rects [list [tcl::mathfunc::min {*}$xs] [tcl::mathfunc::min {*}$ys] \
      [tcl::mathfunc::max {*}$xs] [tcl::mathfunc::max {*}$ys]]
  }
  if { [llength $rects] == 0 } {
    error "[[$inst getMaster] getName] has no OVERLAP obstruction"
  }
  # the outline has to cover exactly the placed bounding box, which catches a
  # wrong transform before it can quietly bias a fixture
  set bx {}
  set by {}
  foreach r $rects {
    lappend bx [lindex $r 0] [lindex $r 2]
    lappend by [lindex $r 1] [lindex $r 3]
  }
  set bbox [$inst getBBox]
  if {
    [tcl::mathfunc::min {*}$bx] != [$bbox xMin]
    || [tcl::mathfunc::min {*}$by] != [$bbox yMin]
    || [tcl::mathfunc::max {*}$bx] != [$bbox xMax]
    || [tcl::mathfunc::max {*}$by] != [$bbox yMax]
  } {
    error "transformed outline does not span the instance bounding box"
  }
  return $rects
}

# Remove the rows that fall under the macro, cutting against its real outline
# rather than its bounding box.  tap's cut_rows would use the bounding box
# (Tapcell::findBlockages pushes inst->getBBox()), which would also strip the
# rows out of the notch -- and the notch is placeable area.
#
# A row is a run of sites at one y, so cutting it against a rectangle leaves at
# most a left and a right remainder.  Both are snapped outward to the site grid
# so the surviving rows stay on pitch.
proc cut_rows_to_body { inst halo_x halo_y } {
  set block [ord::get_db_block]
  set body [macro_body_rects $inst]
  foreach row [lreverse [$block getRows]] {
    set rb [$row getBBox]
    set site [$row getSite]
    set sw [$site getWidth]
    set orient [$row getOrient]
    set name [$row getName]
    set y1 [$rb yMin]
    set y2 [$rb yMax]
    set spans [list [list [$rb xMin] [$rb xMax]]]
    foreach b $body {
      lassign $b bx1 by1 bx2 by2
      set bx1 [expr { $bx1 - $halo_x }]
      set bx2 [expr { $bx2 + $halo_x }]
      set by1 [expr { $by1 - $halo_y }]
      set by2 [expr { $by2 + $halo_y }]
      if { $y2 <= $by1 || $y1 >= $by2 } {
        continue
      }
      set next {}
      foreach s $spans {
        lassign $s sx1 sx2
        if { $sx2 <= $bx1 || $sx1 >= $bx2 } {
          lappend next $s
          continue
        }
        # snap the surviving ends inward to a whole number of sites
        if { $sx1 < $bx1 } {
          set n [expr { ($bx1 - $sx1) / $sw }]
          if { $n > 0 } {
            lappend next [list $sx1 [expr { $sx1 + $n * $sw }]]
          }
        }
        if { $sx2 > $bx2 } {
          set n [expr { ($sx2 - $bx2) / $sw }]
          if { $n > 0 } {
            lappend next [list [expr { $sx2 - $n * $sw }] $sx2]
          }
        }
      }
      set spans $next
    }
    if {
      [llength $spans] == 1
      && [lindex $spans 0 0] == [$rb xMin]
      && [lindex $spans 0 1] == [$rb xMax]
    } {
      continue
    }
    odb::dbRow_destroy $row
    set i 0
    foreach s $spans {
      lassign $s sx1 sx2
      set count [expr { ($sx2 - $sx1) / $sw }]
      if { $count <= 0 } {
        continue
      }
      odb::dbRow_create $block "${name}_$i" $site $sx1 $y1 $orient \
        "HORIZONTAL" $count $sw
      incr i
    }
  }
}

set fixture $::env(FIXTURE)
set macros {}
# how far clear of the macro body the rows are cut, in dbu
set halo_x 0
set halo_y 0
switch -exact -- $fixture {
  floorplan {
    # polygon die and core, no macro
    initialize_floorplan -die_area $poly_die -core_area $poly_core -site $site
  }
  floorplan_macro {
    # rectangular die and core, one L-shaped macro.  The notch of the macro is
    # at the upper right of its bounding box.
    initialize_floorplan -die_area $rect_die -core_area $rect_core -site $site
    lappend macros [place_macro polygon_macro_L macro_L 57 50.4 R0]
  }
  floorplan_macro_r180 {
    # the same bounding box rotated 180 degrees, so the notch moves to the
    # lower left.  Any outline handling that ignores the instance transform
    # gets this one backwards.
    initialize_floorplan -die_area $rect_die -core_area $rect_core -site $site
    lappend macros [place_macro polygon_macro_L macro_L 57 50.4 R180]
  }
  floorplan_macro_tall {
    # rectangular die and core with a macro 70um high in a 95.2um core, so a
    # core strap crossing it cannot be routed around.  Straps at x in
    # 61.75..76 meet the body up to y = 65.8 and then pass through the notch.
    initialize_floorplan -die_area $rect_die -core_area $rect_core -site $site
    lappend macros [place_macro polygon_macro_tall macro_L 47.5 19.6 R0]
  }
  floorplan_u {
    # U-shaped die and core, no macro
    initialize_floorplan -die_area $u_die -core_area $u_core -site $site
  }
  floorplan_cut_corners_pads {
    # Cut-corner core inside a padring.  Two pads per side: one facing a cut
    # corner, where the core is 38um further away than along the middle of the
    # edge, and one facing the middle.  The pad at the cut has to reach a ring
    # that is not where the rest of that edge's ring is.
    initialize_floorplan -die_area $pad_die -core_area $pad_core -site $site
    place_pad PADCELL_VDD_V pad_vdd_s 170 0 R0
    place_pad PADCELL_VSS_V pad_vss_s 280 0 R0
    place_pad PADCELL_VSS_V pad_vss_n 170 431.2 MX
    place_pad PADCELL_VDD_V pad_vdd_n 280 431.2 MX
    place_pad PADCELL_VDD_H pad_vdd_w 0 170 R270
    place_pad PADCELL_VSS_H pad_vss_w 0 280 R270
    place_pad PADCELL_VSS_H pad_vss_e 430 170 R90
    place_pad PADCELL_VDD_H pad_vdd_e 430 280 R90
  }
  floorplan_macro_halo {
    # as floorplan_macro, but with the rows cut 3um clear of the body so that a
    # 3um halo has somewhere to sit.  PDN-0008 refuses a halo that overlaps a
    # row, and the other macro fixtures cut their rows right up to the body.
    initialize_floorplan -die_area $rect_die -core_area $rect_core -site $site
    lappend macros [place_macro polygon_macro_L macro_L 57 50.4 R0]
    set halo_x [expr { int(3.0 * 2000) }]
    set halo_y $halo_x
  }
  floorplan_macro_r90 {
    # rectangular die and core with the L-shaped macro turned a quarter, so
    # that its bounding box is 42x57 rather than 57x42.  R0 and R180 leave the
    # axes alone; only a quarter turn swaps them.
    initialize_floorplan -die_area $rect_die -core_area $rect_core -site $site
    lappend macros [place_macro polygon_macro_L macro_L 57 25.2 R90]
  }
  floorplan_polygon_macro {
    # polygon die and core with the L-shaped macro tucked into the corner of
    # the L.  The bounding box of the macro, (11.4 30.8) - (68.4 72.8), reaches
    # 20.9um into the die notch; the body of the macro does not.  The macro is
    # kept ~2um clear of the core boundary so the core ring is not cut by the
    # min-spacing bloat of the macro obstructions, which would confuse the
    # polygon question with an unrelated one.
    initialize_floorplan -die_area $poly_die -core_area $poly_core -site $site
    lappend macros [place_macro polygon_macro_L macro_L 11.4 30.8 R0]
  }
  floorplan_two_macros {
    # U-shaped die and core with a tall macro in each arm, mirrored so that
    # their notches face each other across the gap between the arms.  Both
    # macros run up to the top of the core, so the area they occupy is a notch
    # in the row footprint that opens onto the core boundary rather than a hole
    # inside it: filling holes does not recover it, and only the union of the
    # macro outlines does.  That is the branch of
    # VoltageDomain::getDomainRegion that a single macro never reaches.
    #
    # Left macro bbox (9.5 33.6) - (38 103.6), notch at its upper right; right
    # macro bbox (114 33.6) - (142.5 103.6), notch at its upper left.  Each
    # fills its arm from side to side, so the only rows left in an arm are the
    # ones in the macro's own notch.
    initialize_floorplan -die_area $u_die -core_area $two_macro_core \
      -site $site
    lappend macros [place_macro polygon_macro_tall macro_left 9.5 33.6 R0]
    lappend macros [place_macro polygon_macro_tall macro_right 114 33.6 MY]
    # Half a row clear in Y.  A row that abuts the body exactly puts its
    # followpin rail half over the macro's own metal1, and the rail on the
    # floor of the notch then has no metal4 above it either -- the body blocks
    # metal1 through metal5, so the strap crossing the notch cannot start until
    # a spacing past the body.  Neither has anything to do with the polygon
    # question; a real flow keeps a keepout around a macro for the same reason.
    set halo_y [expr { int(0.7 * 2000) }]
  }
  default {
    puts "unknown fixture: $fixture"
    exit 1
  }
}

foreach macro $macros {
  cut_rows_to_body $macro $halo_x $halo_y
}

source Nangate45/Nangate45.tracks
global_placement -skip_initial_place
detailed_placement
write_def nangate_polygon/$fixture.def
