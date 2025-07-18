# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2019-2025, The OpenROAD Authors

sta::define_cmd_args "configure_cts_characterization" {[-max_cap cap] \
                                                       [-max_slew slew] \
                                                       [-slew_steps slew_steps] \
                                                       [-cap_steps cap_steps] \
                                                      }

proc configure_cts_characterization { args } {
  sta::parse_key_args "configure_cts_characterization" args \
    keys {-max_cap -max_slew -slew_steps -cap_steps} flags {}

  sta::check_argc_eq0 "configure_cts_characterization" $args

  if { [info exists keys(-max_cap)] } {
    set max_cap_value $keys(-max_cap)
    cts::set_max_char_cap $max_cap_value
  }

  if { [info exists keys(-max_slew)] } {
    set max_slew_value $keys(-max_slew)
    cts::set_max_char_slew $max_slew_value
  }

  if { [info exists keys(-slew_steps)] } {
    set steps $keys(-slew_steps)
    sta::check_cardinal "-slew_steps" $steps
    cts::set_slew_steps $slew
  }

  if { [info exists keys(-cap_steps)] } {
    set steps $keys(-cap_steps)
    sta::check_cardinal "-cap_steps" $steps
    cts::set_cap_steps $cap
  }
}

sta::define_cmd_args "clock_tree_synthesis" {[-wire_unit unit]
                                             [-buf_list buflist] \
                                             [-root_buf buf] \
                                             [-clk_nets nets] \
                                             [-tree_buf buf] \
                                             [-distance_between_buffers] \
                                             [-branching_point_buffers_distance] \
                                             [-clustering_exponent] \
                                             [-clustering_unbalance_ratio] \
                                             [-sink_clustering_size] \
                                             [-sink_clustering_max_diameter] \
                                             [-macro_clustering_size] \
                                             [-macro_clustering_max_diameter] \
                                             [-sink_clustering_enable] \
                                             [-balance_levels] \
                                             [-sink_clustering_levels levels] \
                                             [-num_static_layers] \
                                             [-sink_clustering_buffer] \
                                             [-obstruction_aware] \
                                             [-no_obstruction_aware] \
                                             [-apply_ndr] \
                                             [-sink_buffer_max_cap_derate] \
                                             [-dont_use_dummy_load] \
                                             [-delay_buffer_derate] \
                                             [-library] \
                                             [-repair_clock_nets] \
                                             [-no_insertion_delay]
} ;# checker off

proc clock_tree_synthesis { args } {
  sta::parse_key_args "clock_tree_synthesis" args \
    keys {-root_buf -buf_list -wire_unit -clk_nets -sink_clustering_size \
          -num_static_layers -sink_clustering_buffer \
          -distance_between_buffers -branching_point_buffers_distance \
          -clustering_exponent \
          -clustering_unbalance_ratio -sink_clustering_max_diameter \
          -macro_clustering_size -macro_clustering_max_diameter \
          -sink_clustering_levels -tree_buf \
          -sink_buffer_max_cap_derate -delay_buffer_derate -library} \
    flags {-post_cts_disable -sink_clustering_enable -balance_levels \
           -obstruction_aware -no_obstruction_aware -apply_ndr \
           -dont_use_dummy_load -repair_clock_nets -no_insertion_delay
  } ;# checker off

  sta::check_argc_eq0 "clock_tree_synthesis" $args

  if { [info exists keys(-library)] } {
    set cts_library $keys(-library)
    cts::set_cts_library $cts_library
  }

  if { [info exists flags(-post_cts_disable)] } {
    utl::warn CTS 115 "-post_cts_disable is obsolete."
  }

  cts::set_sink_clustering [info exists flags(-sink_clustering_enable)]

  if { [info exists keys(-sink_clustering_size)] } {
    set size $keys(-sink_clustering_size)
    cts::set_sink_clustering_size $size
  }

  if { [info exists keys(-sink_clustering_max_diameter)] } {
    set distance $keys(-sink_clustering_max_diameter)
    cts::set_clustering_diameter $distance
  }

  if { [info exists keys(-macro_clustering_size)] } {
    set size $keys(-macro_clustering_size)
    cts::set_macro_clustering_size $size
  }

  if { [info exists keys(-macro_clustering_max_diameter)] } {
    set distance $keys(-macro_clustering_max_diameter)
    cts::set_macro_clustering_diameter $distance
  }

  cts::set_balance_levels [info exists flags(-balance_levels)]

  if { [info exists keys(-sink_clustering_levels)] } {
    set levels $keys(-sink_clustering_levels)
    cts::set_sink_clustering_levels $levels
  }

  if { [info exists keys(-num_static_layers)] } {
    set num $keys(-num_static_layers)
    cts::set_num_static_layers $num
  }

  if { [info exists keys(-distance_between_buffers)] } {
    set distance $keys(-distance_between_buffers)
    cts::set_distance_between_buffers [ord::microns_to_dbu $distance]
  }

  if { [info exists keys(-branching_point_buffers_distance)] } {
    set distance $keys(-branching_point_buffers_distance)
    cts::set_branching_point_buffers_distance [ord::microns_to_dbu $distance]
  }

  if { [info exists keys(-clustering_exponent)] } {
    set exponent $keys(-clustering_exponent)
    cts::set_clustering_exponent $exponent
  }

  if { [info exists keys(-clustering_unbalance_ratio)] } {
    set unbalance $keys(-clustering_unbalance_ratio)
    cts::set_clustering_unbalance_ratio $unbalance
  }

  if { [info exists keys(-buf_list)] } {
    set buf_list $keys(-buf_list)
    cts::set_buffer_list $buf_list
  } else {
    cts::set_buffer_list ""
  }

  if { [info exists keys(-wire_unit)] } {
    set wire_unit $keys(-wire_unit)
    cts::set_wire_segment_distance_unit $wire_unit
  }

  if { [info exists keys(-clk_nets)] } {
    set clk_nets $keys(-clk_nets)
    set fail [cts::set_clock_nets $clk_nets]
    if { $fail } {
      utl::error CTS 56 "Error when finding -clk_nets in DB."
    }
  }

  if { [info exists keys(-tree_buf)] } {
    set buf $keys(-tree_buf)
    cts::set_tree_buf $buf
  }

  if { [info exists keys(-root_buf)] } {
    set root_buf $keys(-root_buf)
    cts::set_root_buffer $root_buf
  } else {
    cts::set_root_buffer ""
  }

  if { [info exists keys(-sink_clustering_buffer)] } {
    set sink_buf $keys(-sink_clustering_buffer)
    cts::set_sink_buffer $sink_buf
  } else {
    cts::set_sink_buffer ""
  }

  if { [info exists keys(-sink_buffer_max_cap_derate)] } {
    set derate $keys(-sink_buffer_max_cap_derate)
    if { $derate > 1.0 || $derate < 0.0 } {
      utl::error CTS 109 "sink_buffer_max_cap_derate needs to be between 0 and 1.0."
    }
    cts::set_sink_buffer_max_cap_derate $derate
  }

  if { [info exists keys(-delay_buffer_derate)] } {
    set buffer_derate $keys(-delay_buffer_derate)
    if { $buffer_derate < 0.0 } {
      utl::error CTS 123 "delay_buffer_derate needs to be greater than or equal to 0."
    }
    cts::set_delay_buffer_derate $buffer_derate
  }

  if { [info exists flags(-obstruction_aware)] } {
    utl::warn CTS 128 "-obstruction_aware is obsolete."
  }

  if { [info exists flags(-no_obstruction_aware)] } {
    cts::set_obstruction_aware false
  }
  if { [info exists flags(-dont_use_dummy_load)] } {
    cts::set_dummy_load false
  } else {
    cts::set_dummy_load true
  }

  cts::set_apply_ndr [info exists flags(-apply_ndr)]

  if { [info exists flags(-repair_clock_nets)] } {
    cts::set_repair_clock_nets true
  } else {
    cts::set_repair_clock_nets false
  }

  if { [info exists flags(-no_insertion_delay)] } {
    cts::set_insertion_delay false
  } else {
    cts::set_insertion_delay true
  }

  if { [ord::get_db_block] == "NULL" } {
    utl::error CTS 103 "No design block found."
  }
  cts::run_triton_cts
}

sta::define_cmd_args "report_cts" {[-out_file file] \
                                  }
proc report_cts { args } {
  sta::parse_key_args "report_cts" args \
    keys {-out_file} flags {}

  sta::check_argc_eq0 "report_cts" $args

  if { [info exists keys(-out_file)] } {
    set outFile $keys(-out_file)
    cts::set_metric_output $outFile
  }

  cts::report_cts_metrics
}

namespace eval cts {
proc clock_tree_synthesis_debug { args } {
  sta::parse_key_args "clock_tree_synthesis_debug" args \
    keys {} flags {-plot} ;# checker off

  sta::check_argc_eq0 "clock_tree_synthesis_debug" $args
  cts::set_plot_option [info exists flags(-plot)]

  cts::set_debug_cmd
}
}
