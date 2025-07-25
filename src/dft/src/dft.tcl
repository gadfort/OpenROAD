# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2023-2025, The OpenROAD Authors

sta::define_cmd_args "report_dft_plan" {[-verbose]}

proc report_dft_plan { args } {
  sta::parse_key_args "report_dft_plan" args \
    keys {} \
    flags {-verbose}

  sta::check_argc_eq0 "report_dft_plan" $args

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 1 "No design block found."
  }

  set verbose [info exists flags(-verbose)]

  dft::report_dft_plan $verbose
}

sta::define_cmd_args "scan_replace" { }
proc scan_replace { args } {
  sta::parse_key_args "scan_replace" args \
    keys {} flags {}

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 8 "No design block found."
  }
  dft::scan_replace
}

sta::define_cmd_args "execute_dft_plan" {}
proc execute_dft_plan { args } {
  sta::parse_key_args "execute_dft_plan" args \
    keys {} \
    flags {}

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 9 "No design block found."
  }
  dft::execute_dft_plan
}

sta::define_cmd_args "set_dft_config" { [-max_length max_length]
                                        [-max_chains max_chains]
                                        [-clock_mixing clock_mixing]
                                        [-scan_enable_name_pattern scan_enable_name_pattern]
                                        [-scan_in_name_pattern scan_in_name_pattern]
                                        [-scan_out_name_pattern scan_out_name_pattern]
                                        }
proc set_dft_config { args } {
  sta::parse_key_args "set_dft_config" args \
    keys {
      -max_length
      -max_chains
      -clock_mixing
      -scan_enable_name_pattern
      -scan_in_name_pattern
      -scan_out_name_pattern
    } \
    flags {}

  sta::check_argc_eq0 "set_dft_config" $args

  if { [info exists keys(-max_length)] } {
    set max_length $keys(-max_length)
    sta::check_positive_integer "-max_length" $max_length
    dft::set_dft_config_max_length $max_length
  }

  if { [info exists keys(-max_chains)] } {
    set max_chains $keys(-max_chains)
    sta::check_positive_integer "-max_chains" $max_chains
    dft::set_dft_config_max_chains $max_chains
  }

  if { [info exists keys(-clock_mixing)] } {
    set clock_mixing $keys(-clock_mixing)
    dft::set_dft_config_clock_mixing $clock_mixing
  }

  foreach {flag signal} {
    -scan_enable_name_pattern "scan_enable"
    -scan_in_name_pattern "scan_in"
    -scan_out_name_pattern "scan_out"
  } {
    if { [info exists keys($flag)] } {
      dft::set_dft_config_scan_signal_name_pattern $signal $keys($flag)
    }
  }
}

sta::define_cmd_args "report_dft_config" { }
proc report_dft_config { args } {
  sta::parse_key_args "report_dft_config" args keys {} flags {}
  dft::report_dft_config
}


sta::define_cmd_args "scan_opt" { }
proc scan_opt { args } {
  sta::parse_key_args "scan_opt" args \
    keys {} flags {}

  if { [ord::get_db_block] == "NULL" } {
    utl::error DFT 13 "No design block found."
  }
  dft::scan_opt
}
