or_integration_tests(
  "mpl"
  TESTS
    macro_only
    no_unfixed_macros
    guides1
    guides2
    fixed_ios1
    io_constraints1
    io_constraints2
    io_constraints3
    io_constraints4
    io_constraints5
    io_constraints6
    io_constraints7
    io_constraints8
    io_constraints9
    io_constraints10
    mixed_ios1
    io_pads1
    orientation_improve1
    orientation_improve2
    orientation_improve3
    boundary_push1
    boundary_push2
    boundary_push3
    centralization1
    clocked_macro
)

# Skipped
#mpl_man_tcl_check
#mpl_readme_msgs_check

add_subdirectory(cpp)
