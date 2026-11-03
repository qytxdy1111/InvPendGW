# FPGA Real-Time Attitude Control

Minimal reproducible source project for an FPGA-based real-time attitude control system.

## Repository layout

```text
FPGA/
  Attitude_Control.gprj   Gowin IDE project
  build_gowin.tcl        Reproducible command-line build script
  timing.sdc             Timing constraint
  src/                   Verilog/SystemVerilog sources and pin constraint
  sim/                   Simulation testbenches
HMI/
  HMI_Interface.HMI      Serial touch-screen project
```

Generated synthesis, place-and-route, simulation, and programming files are intentionally excluded from version control.

## FPGA target

- Device: `GW2A-LV55PG484C8/I7`
- Device family/version: `GW2A-55C`, version `C`
- Top module: `main`
- System clock constraint: `50 MHz`
- HDL standard: SystemVerilog 2017

## Build

Open `FPGA/Attitude_Control.gprj` in Gowin EDA, or build from the `FPGA` directory with:

```powershell
gw_sh build_gowin.tcl
```

The script creates `FPGA/build/`, which is ignored by Git. A successful build produces programming files under the generated build directory.

For Gowin EDA on Windows, place the repository and the Gowin installation in paths containing ASCII characters only. Some Gowin tool versions cannot read project or device files through paths containing non-ASCII characters.

## Required source files

The Gowin project and build script use the following design files:

- `src/adc.v`
- `src/encoder.v`
- `src/hmi_display.v`
- `src/hmi_display8.v`
- `src/hmi_command_parser.v`
- `src/arm_position_trajectory.v`
- `src/arm_sine_reference.v`
- `src/main.v`
- `src/motor_ctrl.v`
- `src/pid_controller_q16.sv`
- `src/state_q16.v`
- `src/telemetry_3ms.v`
- `src/uart_tx.v`
- `src/uart_rx_screen.v`
- `src/pin.cst`
- `timing.sdc`

## Notes

- Runtime control gains are initialized from parameters in `FPGA/src/main.v`.
- Runtime gain changes are volatile and reset to source defaults after FPGA reset or power cycling.
- The HMI communicates with the FPGA through the dedicated serial interface defined by the RTL and pin constraint.
- Generated bitstreams are not committed; rebuild them from the tracked sources using Gowin EDA.
