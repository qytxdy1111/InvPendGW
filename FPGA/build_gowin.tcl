set root [file normalize [file dirname [info script]]]
set build_dir [file join $root build]
file mkdir $build_dir
create_project -name pid_pendulum -dir $build_dir -pn GW2A-LV55PG484C8/I7 -device_version C -force
set_device -name GW2A-55C -device_version C GW2A-LV55PG484C8/I7
foreach source {adc.v encoder.v hmi_display.v hmi_display8.v hmi_command_parser.v arm_position_trajectory.v arm_sine_reference.v main.v motor_ctrl.v pid_controller_q16.sv state_q16.v telemetry_3ms.v uart_tx.v uart_rx_screen.v} {
    add_file [file join $root src $source]
}
add_file [file join $root src pin.cst]
add_file [file join $root timing.sdc]
set_option -include_path [file join $root src]
set_option -top_module main
set_option -verilog_std sysv2017
set_option -frequency 50
set_option -global_freq 50
set_option -output_base_name attitude_control
# Use timing-driven placement and routing for the existing arm-loop timing path.
set_option -place_option 1
set_option -route_option 1
set_option -timing_driven 1
run all
