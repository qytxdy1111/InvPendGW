`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// J280 inverted pendulum, top level (T44 baseline; comment set slimmed).
//   * Rod loop: PID on the pendulum angle in BALANCE, PD in manual PD, then
//     CONTROL_SIGN -> +-UMAX_Q16 -> global /2 (DUTY_SCALE_SHIFT) -> DUTY_DZ
//     dead-zone offset -> DUTY_MAX_CTRL cap. A zero command releases the motor.
//   * Arm loop: P + clamped D (+30 duty/rad outward -- the rod loop's booster)
//     plus a slow clamped integral anchor pulling the arm back to the position
//     latched when SW1 was pressed.
//   * Safety: the arm limit is an IMMEDIATE latch; the rod-angle fault needs 3
//     consecutive over-limit samples. Both latch STATE_FAULT until SW5.
//   * Units: gains are Q16 duty per rad, Q16 angles are radians * 65536, duty
//     is 1/270 of the PWM period (PWM_PERIOD = 5400 at 50 MHz), sample = 3 ms.
//   * Tuning loop: edit a parameter -> gw_sh build_gowin.tcl -> flash -> CSV.
// Full bench history and per-parameter rationale: reports/MANUAL_PD_VALIDATION.md
// and reports/T44_BASELINE_LOCK.md.
// ---------------------------------------------------------------------------
 
module main #(  
    parameter signed [31:0] KP_Q16 = 32'sd65536000,   // 870.0 duty/rad rod P (applied = /2 = 475)
    parameter signed [31:0] KI_Q16 = 32'sd0,   // 180.0 duty/(rad*s) rod I. 2026-10-13 (V9-A/B): the rod integrator is NOT what blocks an angle-only loop -- with KI=0 the cart still ran away at -411 counts/s, because the cart's velocity mode is only marginally stable and the rod angle carries no information about it. The position loop is structurally required; only its AUTHORITY is negotiable.
    parameter signed [31:0] KD_Q16 = 32'sd0,   // 28.5 duty/(rad/s) rod D (40 ms window rate, DFILT)
    parameter signed [31:0] UMAX_Q16 = 32'sd27525120,   // 420 duty: control-path saturation, == (DUTY_MAX_CTRL-DUTY_DZ)<<16
    parameter signed [31:0] I_LIMIT_Q16 = 32'sd32768,   // 0.5 rad*s rod-loop integrator clamp. 2026-10-12: tried 2.0 (131072) — the rod improved (|dev|<=5deg 96.1 -> 98.9%, |dev|<=1deg 28.5 -> 51.3%) BUT the mean tilt stayed +0.44 deg (so "the rail causes the residual tilt" was WRONG) and the bigger integral became a 127 applied-duty constant thrust that pushed the arm to the +-1000 limit (Fault). The integral's thrust and the position loop's hold are a zero-sum fight over the same actuator.
    parameter signed [31:0] BALANCE_CAPTURE_ANGLE_Q16 = 32'sd22877,   // 20.0 deg capture window (relaxed from 5.0 deg)
    parameter signed [31:0] BALANCE_CAPTURE_RATE_Q16 = 32'sd655360,   // 10.0 rad/s, checked on the 40 ms window rate (relaxed from 1.0)
    parameter integer CAPTURE_HOLD_CYCLES = 3,   // consecutive valid samples needed to capture
    parameter integer SWINGUP_TIMEOUT_CYCLES = 900,   // 900 samples = 18 s at 20 ms, then FAULT latches
    parameter signed [31:0] SWINGUP_INITIAL_CMD = 32'sd170,   // task-43: first half-cycle 170 duty (+10%), second half stays 155
    parameter integer SWINGUP_INITIAL_HALF_SAMPLES = 30,   // task-37: 30 samples = 0.60 s per initial half-cycle at 20 ms (was 200 at 3 ms)
    parameter integer SWINGUP_INITIAL_HALVES = 2,   // task-36: one complete round trip (out + back), then coast
    parameter signed [31:0] SWINGUP_ARM_REVERSAL_Q16 = 32'sd80066,   // task-37: 70 deg arm soft reversal (C41 hit 35 deg first after only 0.33 s)
    parameter signed [31:0] FAULT_ANGLE_Q16 = 32'sd34315,   // 30 deg delivery fault threshold; checked only in BALANCE (and manual PD when enabled)
    parameter signed [31:0] ARM_ZERO_COUNTS = 0,   // encoder assembly zero; the loop zero is arm_zero_run (latched on SW1)
    parameter signed [31:0] ARM_RAD_PER_COUNT_Q30 = 6116518,   // 1 count = 0.0056965 rad = 0.32638 deg (1103 counts/rev measured)
    parameter integer PEND_ZERO_COUNTS = 817,   // 2026-11-02 hardware capture: controller direction flips at 817 counts (was 841).
    // The 841 value is retained in rollback_verified_841 as the previously tested baseline. 826 is not a validated bitstream yet.
    // 1 count = 0.08502 deg, so 841 -> 817 moves the pendulum zero by -2.04 deg. Recheck the physical vertical reference after any sensor change.
    parameter signed [31:0] PEND_RAD_PER_COUNT_Q30 = 1593415,   // 1 count = 0.0014839 rad = 0.08502 deg
    parameter signed [31:0] ARM_RATE_PER_COUNT_Q16 = 18666,   // 0.2848 rad/s per count (20 ms); /2 on the 40 ms window = 0.1424
    parameter signed [31:0] PEND_RATE_PER_COUNT_Q16 = 4863,   // 0.0742 rad/s per count (20 ms); /2 on the 40 ms window = 0.0371
    parameter signed [31:0] ARM_LIMIT_Q16 = 373323,   // +-1000 counts = +-5.696 rad; the limit logic uses counts, this value is kept for reference
    parameter integer ARM_CABLE_FAULT_ENABLE = 0,   // 0 = 去掉 ±1000 计数(约一整圈)的电缆硬限位故障；1 = 恢复该保护
    parameter integer STATE_TIMEOUT_CYCLES = 1500000,   // 1500000 = 30 ms without a state sample -> FAULT (20 ms sampling)
    parameter integer BUTTON_DEBOUNCE_CYCLES = 249999,   // 249999 = 5 ms debounce
    parameter signed [31:0] MANUAL_KP_Q16 = 32'sd78643200,   // 手动起摆P1200.0 duty/rad manual PD
    parameter signed [31:0] MANUAL_KD_Q16 = 32'sd983040,   // 手动起摆D15.0 duty/(rad/s) manual PD (was 18.0 at 3 ms; 20 ms loop has more lag)
    parameter integer ARM_LOOP_ENABLE = 1,   // 1 = 摆臂位置环生效; 0 = 完全关闭(P+D+I+远场辅助全部清零, I 锚同步清零)
    parameter signed [31:0] ARM_LOOP_KP_Q16 = -32'sd1966080,   // NEGATIVE param; leading minus => effective +30 duty/rad (T44 value, restored after V9-A/B)
    parameter signed [31:0] ARM_LOOP_KD_Q16 = -32'sd0,   // task-44 C49: NEGATIVE param; effective +50 duty/(rad/s) (damping up from +30) to suppress the 0.45 Hz sway
    parameter signed [31:0] ARM_LOOP_D_CLAMP_Q16 = 32'sd4915200,   // 75 duty pre-scale (37.5 applied) ceiling (T44 value; the 150 raise that went with the doubled KD was reverted with it)
    parameter integer ARM_LOOP_I_SHIFT = 6,   // 4 = 20.83 duty/(rad*s); >>3 = 41.7, >>2 = 83.3 (no multiplier)
    // W3 = option A (2026-10-13): ANCHOR ANTI-WINDUP with a RELAXED window. W2 used 5 deg and made things WORSE:
    // the capture transient at BALANCE entry routinely exceeds 5 deg, so the anchor stopped accumulating exactly
    // when the arm needed holding. 10 deg keeps the whole capture transient integrating while still blocking the
    // windup that a real hand push produces. Only the ACCUMULATION is gated; the OUTPUT is untouched (V9-A/V9-B
    // proved that removing the position loop fails in 2-5 s).
    parameter signed [31:0] ARM_I_ANGLE_Q16 = 32'sd22908,   // ~20.0 deg: anchor integration window
    parameter signed [31:0] ARM_LOOP_I_CLAMP_Q16 = 32'sd6553600,   // 100 duty pre-scale (50 applied) anchor authority. W1 (2026-10-13): V8-E left a slow arm drift to -145 counts with the anchor railed at 25 applied, i.e. the DC demanded more than the clamp allowed. Precedent: task-32 with this clamp held the arm STILL for 91 s; task-33 at 250 pre-scale (=125 applied) destabilised the slow mode -> 100 pre-scale is the documented sweet spot.
    // Far-field position priority restore (owner request 2026-10-12, sign fixed
    // after the first bench run): beyond ARM_PRI_ON_COUNTS from the latched start a
    // bounded restoring bias is added on top of the arm loop, so getting back to the
    // start wins over anything pushing the arm out. SIGN: a POSITIVE error must
    // produce a POSITIVE command, because on this machine a positive command drives
    // pos DOWN (measured). Hysteresis (engage at ON, release at OFF) stops the
    // boundary from chattering; inside the band the control law is exactly T44.
    parameter signed [31:0] ARM_PRI_ON_COUNTS = 32'sd60,     // ramp starts beyond this |pos-start|
    parameter integer ARM_PRI_SLOPE_SHIFT = 2,               // ramp slope: 1/4 duty pre-scale per count (= 20.7 applied duty/rad)
    parameter signed [31:0] ARM_PRI_DUTY_Q16 = 32'sd1638400, // 25 duty pre-scale (12.5 applied) ramp cap
    parameter signed [31:0] ARM_MOTION_TILT_ENABLE_Q16 = 32'sd3432,  // resume trajectory below 3 deg
    parameter signed [31:0] ARM_MOTION_TILT_DISABLE_Q16 = 32'sd6859, // pause trajectory above 6 deg
    parameter signed [31:0] ARM_MOTION_RATE_ENABLE_Q16 = 32'sd32768, // resume below 0.5 rad/s
    parameter signed [31:0] ARM_MOTION_RATE_DISABLE_Q16 = 32'sd65536, // pause above 1.0 rad/s
    parameter integer DFILT_SHIFT = 1,   // 20 ms samples: 1 = 40 ms one-pole on the D terms (was 3 = 24 ms at 3 ms samples)
    parameter integer DUTY_SCALE_SHIFT = 1,   // 1 = /2 global output scale for 12 V (2 => /4, 0 => /1)
    parameter signed [31:0] DUTY_DZ = 32'sd15,   // 10 duty dead-zone offset (12 V breakaway)
    parameter signed [31:0] DUTY_MAX_CTRL = 32'sd220,   // 220 = 81.5% of the 270-step PWM range
    parameter signed [31:0] CONTROL_SIGN = -32'sd1,   // -1 measured polarity; do not flip unless the wiring changes
    parameter integer AUTO_SWINGUP_ENABLE = 1,   // 1 = allow one initial-excitation attempt on release
    parameter integer MANUAL_PEND_FAULT = 1,   // 1 = the rod-angle fault also applies in manual PD
    parameter integer MANUAL_PD_HOLD_CYCLES = 15000000,   // 15000000 = 0.3 s long press enters manual PD
    // C37/C38/C39 (task-34): PWM balance capture soft start + swing-up final-half trim.
    parameter integer BAL_SOFT_START_SAMPLES = 0,         // task-35: soft start off (full authority after the kick)
    parameter signed [31:0] BAL_SOFT_START_MIN_Q16 = 32'sd26214,  // ramp start 0.4 in Q16
    parameter signed [31:0] SWINGUP_FINAL_CMD = 32'sd155,   // task-41: matches the 155 duty kick (last half uses it)
    parameter integer SWINGUP_FINAL_HALF_SAMPLES = 15,     // last half-cycle cap; == HALF means unchanged (15 x 20 ms = 300 ms)
    parameter integer SWINGUP_COAST_ENABLE = 1,             // task-35: 1 = coast in AUTO after the initial segment
    parameter integer SWINGUP_COAST_BRAKE = 1,              // task-40: 1 = short-brake (mode 11) during the AUTO coast
    parameter integer SWINGUP_HALF_DELAY_SAMPLES = 10,      // task-42: pause between initial half-cycles (10 x 20 ms = 200 ms; 0 = off)
    parameter integer SWINGUP_HALF_DELAY_BRAKE = 1          // task-42: 1 = short-brake (mode 11) during the inter-half pause
)(
    input wire clk,
    input wire rst_n,
    input wire enc_a,
    input wire enc_b,
    input wire [9:0] adc_data_in,
    input wire mode_button_n,
    input wire hmi_rx,
    output wire adc_clk,
    output wire adc_oe_n,
    output wire adc_stby_n,
    output wire uart_tx,
    output wire hmi_tx,
    output wire ain1,
    output wire ain2,
    output wire pwma,
    output wire bn1,
    output wire bn2,
    output wire pwmb,
    output wire [3:0] led
);
// FSM: OFF -> MANUAL_WAIT -> (long press) MANUAL_PD | (release) AUTO_SWINGUP
//      -> BALANCE; any fault or timeout latches FAULT until SW5.
    localparam [2:0] STATE_OFF = 3'd0;
    localparam [2:0] STATE_MANUAL_WAIT = 3'd1;
    localparam [2:0] STATE_AUTO_SWINGUP = 3'd2;
    localparam [2:0] STATE_BALANCE = 3'd3;
    localparam [2:0] STATE_FAULT = 3'd4;
    localparam [2:0] STATE_MANUAL_PD = 3'd5;
    localparam integer CAPTURE_HOLD_LIMIT = CAPTURE_HOLD_CYCLES > 0 ? CAPTURE_HOLD_CYCLES : 1;
    localparam integer SWINGUP_TIMEOUT_LIMIT = SWINGUP_TIMEOUT_CYCLES > 0 ? SWINGUP_TIMEOUT_CYCLES : 1;
    localparam integer SWINGUP_INITIAL_HALF_LIMIT =
        SWINGUP_INITIAL_HALF_SAMPLES > 0 ? SWINGUP_INITIAL_HALF_SAMPLES : 1;
    localparam integer SWINGUP_INITIAL_HALVES_LIMIT =
        SWINGUP_INITIAL_HALVES > 0 ? SWINGUP_INITIAL_HALVES : 1;
    localparam integer STATE_TIMEOUT_LIMIT = STATE_TIMEOUT_CYCLES > 0 ? STATE_TIMEOUT_CYCLES : 1;
    // task-34: last-half sample cap (only differs when FINAL_HALF is set and != HALF).
    localparam integer SWINGUP_FINAL_HALF_LIMIT =
        (SWINGUP_FINAL_HALF_SAMPLES > 0 && SWINGUP_FINAL_HALF_SAMPLES != SWINGUP_INITIAL_HALF_SAMPLES)
        ? SWINGUP_FINAL_HALF_SAMPLES : SWINGUP_INITIAL_HALF_LIMIT;
    // Power-of-two soft-start ramp shift (= log2(BAL_SOFT_START_SAMPLES)).
    localparam integer BAL_SOFT_START_SHIFT =
        (BAL_SOFT_START_SAMPLES > 1) ? $clog2(BAL_SOFT_START_SAMPLES) : 0;
    localparam signed [31:0] PEND_ZERO_Q16 = PEND_ZERO_COUNTS * 65536;
    // Declare reset nets before their first use (including the HMI UART instances).
    reg [15:0] reset_count;
    reg reset_done;
    wire rst_n_sys;
    reg [2:0] state;
    reg control_fault;
    reg state_timeout;
    reg swingup_direction_positive;
    reg manual_wait_screen;
    reg signed [15:0] target_position_deg;
    // Q16 encoder counts need 48 bits so the full signed 32-bit encoder position
    // remains representable when latched as an absolute target.
    reg signed [47:0] target_position_counts_q16;
    reg signed [31:0] target_position_angle_q16;
    wire signed [31:0] motion_position_counts_q16;
    wire signed [31:0] motion_position_angle_q16;
    wire signed [31:0] motion_velocity_counts_q16;
    wire signed [31:0] arm_reference_counts_q16;
    reg target_command_seen;
    reg signed [31:0] hmi_arm_absolute_q16;
    reg signed [31:0] hmi_arm_angle_deg, hmi_pend_angle_deg;
    reg signed [31:0] hmi_arm_rate_deg, hmi_pend_rate_deg;

    wire hmi_rx_valid;
    wire [7:0] hmi_rx_data;
    wire hmi_mode_valid;
    wire [1:0] hmi_mode_value;
    wire hmi_delta_valid;
    wire signed [15:0] hmi_delta_degrees;
    wire hmi_delta_apply_valid;
    wire hmi_delta_cancel_valid;
    wire hmi_sine_period_valid;
    wire [7:0] hmi_sine_period_seconds;
    wire hmi_sine_start_valid;
    wire hmi_sine_stop_valid;
    wire hmi_pid_gain_valid;
    wire [1:0] hmi_pid_gain_index;
    wire hmi_pid_gain_increment;
    wire hmi_arm_gain_valid;
    wire [1:0] hmi_arm_gain_index;
    wire hmi_arm_gain_increment;
    wire hmi_stop_command = hmi_mode_valid && (hmi_mode_value == 2'd0);
    uart_rx_screen u_hmi_uart_rx (
        .clk(clk), .rst_n(rst_n_sys), .rx(hmi_rx),
        .data(hmi_rx_data), .valid(hmi_rx_valid));
    hmi_command_parser u_hmi_command_parser (
        .clk(clk), .rst_n(rst_n_sys), .rx_valid(hmi_rx_valid), .rx_data(hmi_rx_data),
        .mode_valid(hmi_mode_valid), .mode_value(hmi_mode_value),
        .delta_valid(hmi_delta_valid), .delta_degrees(hmi_delta_degrees),
        .delta_apply_valid(hmi_delta_apply_valid),
        .delta_cancel_valid(hmi_delta_cancel_valid),
        .sine_period_valid(hmi_sine_period_valid),
        .sine_period_seconds(hmi_sine_period_seconds),
        .sine_start_valid(hmi_sine_start_valid),
        .sine_stop_valid(hmi_sine_stop_valid),
        .pid_gain_valid(hmi_pid_gain_valid),
        .pid_gain_index(hmi_pid_gain_index),
        .pid_gain_increment(hmi_pid_gain_increment),
        .arm_gain_valid(hmi_arm_gain_valid),
        .arm_gain_index(hmi_arm_gain_index),
        .arm_gain_increment(hmi_arm_gain_increment));

    // Runtime rod-loop gains. They are volatile by design: a power cycle
    // restores the three synthesis-time defaults above. Keep the three tuning
    // increments independent so KP, KI and KD can use different step sizes.
    // Q16.16: 0.1 * 65536 = 6553.6, rounded here to 6554.
    localparam signed [31:0] PID_KP_STEP_Q16 = 32'sd6553600;
    localparam signed [31:0] PID_KI_STEP_Q16 = 32'sd65536;
    localparam signed [31:0] PID_KD_STEP_Q16 = 32'sd655360;
    localparam signed [31:0] PID_GAIN_MAX_Q16 = 32'sh7fff0000;
    reg signed [31:0] pid_kp_runtime_q16;
    reg signed [31:0] pid_ki_runtime_q16;
    reg signed [31:0] pid_kd_runtime_q16;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            pid_kp_runtime_q16 <= KP_Q16;
            pid_ki_runtime_q16 <= KI_Q16;
            pid_kd_runtime_q16 <= KD_Q16;
        end else if (hmi_pid_gain_valid) begin
            case (hmi_pid_gain_index)
                2'd0: begin
                    if (hmi_pid_gain_increment)
                        pid_kp_runtime_q16 <=
                            (pid_kp_runtime_q16 > PID_GAIN_MAX_Q16 - PID_KP_STEP_Q16) ?
                            PID_GAIN_MAX_Q16 : pid_kp_runtime_q16 + PID_KP_STEP_Q16;
                    else
                        pid_kp_runtime_q16 <= (pid_kp_runtime_q16 < PID_KP_STEP_Q16) ?
                            32'sd0 : pid_kp_runtime_q16 - PID_KP_STEP_Q16;
                end
                2'd1: begin
                    if (hmi_pid_gain_increment)
                        pid_ki_runtime_q16 <=
                            (pid_ki_runtime_q16 > PID_GAIN_MAX_Q16 - PID_KI_STEP_Q16) ?
                            PID_GAIN_MAX_Q16 : pid_ki_runtime_q16 + PID_KI_STEP_Q16;
                    else
                        pid_ki_runtime_q16 <= (pid_ki_runtime_q16 < PID_KI_STEP_Q16) ?
                            32'sd0 : pid_ki_runtime_q16 - PID_KI_STEP_Q16;
                end
                2'd2: begin
                    if (hmi_pid_gain_increment)
                        pid_kd_runtime_q16 <=
                            (pid_kd_runtime_q16 > PID_GAIN_MAX_Q16 - PID_KD_STEP_Q16) ?
                            PID_GAIN_MAX_Q16 : pid_kd_runtime_q16 + PID_KD_STEP_Q16;
                    else
                        pid_kd_runtime_q16 <= (pid_kd_runtime_q16 < PID_KD_STEP_Q16) ?
                            32'sd0 : pid_kd_runtime_q16 - PID_KD_STEP_Q16;
                end
                default: ;
            endcase
        end
    end

// Runtime ARM-loop (position loop) gains, volatile like the rod gains.
// The stored P/D values keep the synthesis-time sign convention: NEGATIVE
// means a positive effective gain (the loop applies a leading minus). An
// "increment" command therefore moves the stored value toward -max, a
// "decrement" moves it toward 0. The I "gain" is the >>shift applied to the
// anchor accumulator (1/2^shift): a smaller shift is a BIGGER gain, so LI+
// decrements the shift and LI- increments it. Steps are preset here in the
// FPGA program, not on the screen.
    localparam signed [31:0] ARM_LOOP_KP_STEP_Q16 = 32'sd65536;   // +1.00 duty/rad per LP+ press
    localparam signed [31:0] ARM_LOOP_KD_STEP_Q16 = 32'sd655360;   // +10.00 duty/(rad/s) per LD+ press
    localparam signed [31:0] ARM_LOOP_GAIN_MAX_Q16 = 32'sh7fff0000; // ~32767.0 duty/rad ceiling
    localparam [3:0] ARM_LOOP_I_SHIFT_MIN = 4'd0; // shift 0 = 100 duty/(rad*s), the strongest I
    localparam [3:0] ARM_LOOP_I_SHIFT_MAX = 4'd8; // shift 8 = 0.39 duty/(rad*s), the weakest I
    reg signed [31:0] arm_loop_kp_runtime_q16;
    reg signed [31:0] arm_loop_kd_runtime_q16;
    reg [3:0] arm_loop_i_shift_runtime;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            arm_loop_kp_runtime_q16 <= ARM_LOOP_KP_Q16;
            arm_loop_kd_runtime_q16 <= ARM_LOOP_KD_Q16;
            arm_loop_i_shift_runtime <= ARM_LOOP_I_SHIFT[3:0];
        end else if (hmi_arm_gain_valid) begin
            case (hmi_arm_gain_index)
                2'd0: begin
                    if (hmi_arm_gain_increment) begin
                        // Effective gain UP: stored value goes more negative.
                        if (arm_loop_kp_runtime_q16 - ARM_LOOP_KP_STEP_Q16 <
                            -ARM_LOOP_GAIN_MAX_Q16)
                            arm_loop_kp_runtime_q16 <= -ARM_LOOP_GAIN_MAX_Q16;
                        else
                            arm_loop_kp_runtime_q16 <=
                                arm_loop_kp_runtime_q16 - ARM_LOOP_KP_STEP_Q16;
                    end else begin
                        // Effective gain DOWN: stored value goes toward 0.
                        if (arm_loop_kp_runtime_q16 + ARM_LOOP_KP_STEP_Q16 > 32'sd0)
                            arm_loop_kp_runtime_q16 <= 32'sd0;
                        else
                            arm_loop_kp_runtime_q16 <=
                                arm_loop_kp_runtime_q16 + ARM_LOOP_KP_STEP_Q16;
                    end
                end
                2'd1: begin
                    if (hmi_arm_gain_increment) begin
                        // I gain UP = shift DOWN.
                        if (arm_loop_i_shift_runtime > ARM_LOOP_I_SHIFT_MIN)
                            arm_loop_i_shift_runtime <= arm_loop_i_shift_runtime - 4'd1;
                    end else begin
                        // I gain DOWN = shift UP.
                        if (arm_loop_i_shift_runtime < ARM_LOOP_I_SHIFT_MAX)
                            arm_loop_i_shift_runtime <= arm_loop_i_shift_runtime + 4'd1;
                    end
                end
                2'd2: begin
                    if (hmi_arm_gain_increment) begin
                        if (arm_loop_kd_runtime_q16 - ARM_LOOP_KD_STEP_Q16 <
                            -ARM_LOOP_GAIN_MAX_Q16)
                            arm_loop_kd_runtime_q16 <= -ARM_LOOP_GAIN_MAX_Q16;
                        else
                            arm_loop_kd_runtime_q16 <=
                                arm_loop_kd_runtime_q16 - ARM_LOOP_KD_STEP_Q16;
                    end else begin
                        if (arm_loop_kd_runtime_q16 + ARM_LOOP_KD_STEP_Q16 > 32'sd0)
                            arm_loop_kd_runtime_q16 <= 32'sd0;
                        else
                            arm_loop_kd_runtime_q16 <=
                                arm_loop_kd_runtime_q16 + ARM_LOOP_KD_STEP_Q16;
                    end
                end
                default: ;
            endcase
        end
    end

// Power-on reset stretch (32768 clocks).
    assign rst_n_sys = reset_done & rst_n;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_count <= 16'd0;
            reset_done <= 1'b0;
        end else if (!reset_count[15]) begin
            reset_count <= reset_count + 1'b1;
            reset_done <= 1'b0;
        end else begin
            reset_done <= 1'b1;
        end
    end

    wire [31:0] pos;
    wire encoder_dir;
    wire [11:0] theta;
    wire sample_valid;
    wire signed [31:0] arm_angle, arm_rate, pend_angle, pend_rate;
    wire state_valid;
    wire signed [31:0] control_u_q16;
    wire pid_valid;
    wire signed [31:0] integral_q16;
    wire pid_rst_n = rst_n_sys && (state == STATE_BALANCE);

// Sensor front ends.
    encoder u_encoder(
        .clk(clk), .rst_n(rst_n_sys), .a(enc_a), .b(enc_b),
        .pos(pos), .dir(encoder_dir));
    adc u_adc(
        .clk_50m(clk), .rst_n(rst_n_sys), .adc_clk(adc_clk),
        .adc_oe_n(adc_oe_n), .adc_stby_n(adc_stby_n), .adc_otr(),
        .adc_data_in(adc_data_in), .theta_out(theta), .sample_valid(sample_valid));
    reg signed [31:0] arm_zero_run;
    // Absolute encoder-count equilibrium/target.  On the first start it is
    // initialized to the current arm position; subsequent HMI deltas move it.
    reg signed [31:0] arm_eq_counts;
    state_q16 #(
        .ARM_ZERO_COUNTS(ARM_ZERO_COUNTS),
        .ARM_RAD_PER_COUNT_Q30(ARM_RAD_PER_COUNT_Q30),
        .PEND_ZERO_Q16(PEND_ZERO_Q16),
        .PEND_RAD_PER_COUNT_Q30(PEND_RAD_PER_COUNT_Q30),
        .ARM_RATE_PER_COUNT_Q16(ARM_RATE_PER_COUNT_Q16),
        .PEND_RATE_PER_COUNT_Q16(PEND_RATE_PER_COUNT_Q16)
    ) u_state(
        .clk(clk), .rst_n(rst_n_sys), .sample_valid(sample_valid),
        .pos(pos), .arm_reference_counts_q16(arm_reference_counts_q16),
        .theta(theta),
        .arm_angle(arm_angle), .arm_rate(arm_rate), .pend_angle(pend_angle),
        .pend_rate(pend_rate), .out_valid(state_valid));
// 40 ms window rate (2-sample average at 20 ms) + one-pole D filter.
    reg signed [31:0] pend_rate_h0, pend_rate_h1;
    reg signed [31:0] arm_rate_h0, arm_rate_h1;
    reg [2:0] rate_hist_count;
    wire rate_filter_valid = (rate_hist_count == 3'd2);
    wire signed [32:0] pend_rate_window = pend_rate_h0 + pend_rate_h1;
    wire signed [32:0] arm_rate_window = arm_rate_h0 + arm_rate_h1;
    reg signed [31:0] pend_rate_filt_q16;
    reg signed [31:0] arm_rate_filt_q16;
    reg signed [31:0] pend_rate_dfilt_q16;
    reg signed [31:0] arm_rate_dfilt_q16;
    wire signed [63:0] pend_dflt_err =
        $signed(pend_rate_filt_q16) - $signed(pend_rate_dfilt_q16);
    wire signed [63:0] arm_dflt_err =
        $signed(arm_rate_filt_q16) - $signed(arm_rate_dfilt_q16);
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            pend_rate_h0 <= 32'sd0; pend_rate_h1 <= 32'sd0;
            arm_rate_h0 <= 32'sd0; arm_rate_h1 <= 32'sd0;
            rate_hist_count <= 3'd0;
            pend_rate_filt_q16 <= 32'sd0;
            arm_rate_filt_q16 <= 32'sd0;
            pend_rate_dfilt_q16 <= 32'sd0;
            arm_rate_dfilt_q16 <= 32'sd0;
        end else if (sample_valid) begin
            pend_rate_filt_q16 <= rate_filter_valid ? pend_rate_window[32:1] : 32'sd0;
            arm_rate_filt_q16 <= rate_filter_valid ? arm_rate_window[32:1] : 32'sd0;
            pend_rate_dfilt_q16 <= pend_rate_dfilt_q16 +
                pend_dflt_err[63:DFILT_SHIFT];
            arm_rate_dfilt_q16 <= arm_rate_dfilt_q16 +
                arm_dflt_err[63:DFILT_SHIFT];
            pend_rate_h0 <= pend_rate; pend_rate_h1 <= pend_rate_h0;
            arm_rate_h0 <= arm_rate; arm_rate_h1 <= arm_rate_h0;
            if (!rate_filter_valid) rate_hist_count <= rate_hist_count + 1'b1;
        end
    end

// Rod PID core; reset unless BALANCE, so its output is 0 elsewhere.
    pid_controller_q16 #(
        .KP_Q16(KP_Q16), .KI_Q16(KI_Q16), .KD_Q16(KD_Q16),
        .UMAX_Q16(UMAX_Q16), .I_LIMIT_Q16(I_LIMIT_Q16), .SAMPLE_PERIOD_MS(20)
    ) u_pid(
        .clk(clk), .rst_n(pid_rst_n), .in_valid(state_valid && state == STATE_BALANCE),
        .pend_angle_q16(pend_angle), .pend_rate_q16(pend_rate_dfilt_q16),
        .kp_runtime_q16(pid_kp_runtime_q16),
        .ki_runtime_q16(pid_ki_runtime_q16),
        .kd_runtime_q16(pid_kd_runtime_q16),
        .out_valid(pid_valid),
        .control_u_q16(control_u_q16), .integral_q16(integral_q16));

    reg [1:0] button_sync;
    reg button_stable, button_previous;
    reg [17:0] button_count;
    wire button_press = button_previous && !button_stable;
    wire button_release = !button_previous && button_stable;

    // The pending position change is a modulo-360 net offset. Each incoming
    // D command updates this cache only; A commits it and X discards it.
    // Keeping both the committed angle and pending offset in [-180, 180]
    // makes a single add/subtract sufficient to wrap each update.
    reg signed [15:0] target_position_pending_deg;
    wire signed [16:0] target_pending_sum =
        $signed({target_position_pending_deg[15], target_position_pending_deg}) +
        $signed({hmi_delta_degrees[15], hmi_delta_degrees});
    wire signed [16:0] target_pending_wrapped =
        (target_pending_sum > 17'sd180) ? (target_pending_sum - 17'sd360) :
        (target_pending_sum < -17'sd180) ? (target_pending_sum + 17'sd360) :
        target_pending_sum;
    wire signed [16:0] target_position_commit_sum =
        $signed({target_position_deg[15], target_position_deg}) +
        $signed({target_position_pending_deg[15], target_position_pending_deg});
    wire signed [16:0] target_position_commit_wrapped =
        (target_position_commit_sum > 17'sd180) ?
            (target_position_commit_sum - 17'sd360) :
        (target_position_commit_sum < -17'sd180) ?
            (target_position_commit_sum + 17'sd360) :
        target_position_commit_sum;
    // At startup the cable-protected arm span is less than one revolution, so
    // one fold is sufficient to keep the displayed target in [-180, 180].
    wire signed [31:0] hmi_initial_target_wrapped =
        (hmi_arm_angle_deg > 32'sd180) ? (hmi_arm_angle_deg - 32'sd360) :
        (hmi_arm_angle_deg < -32'sd180) ? (hmi_arm_angle_deg + 32'sd360) :
        hmi_arm_angle_deg;
    wire target_delta_commit = hmi_delta_apply_valid &&
        (target_position_pending_deg != 16'sd0);
    reg signed [16:0] target_delta_applied_pending;
    reg target_delta_pending;
    wire signed [31:0] target_count_delta_q16 = target_delta_applied_pending * 32'sd200795;   // counts Q16 per degree = round(1103/360*65536)
    wire signed [31:0] target_angle_delta_q16 = target_delta_applied_pending * 32'sd1144;
    reg signed [31:0] target_count_delta_stage;
    reg signed [31:0] target_angle_delta_stage;
    reg target_product_pending;
    wire hmi_start_command = hmi_mode_valid && (hmi_mode_value != 2'd0) &&
        !control_fault && !state_timeout && (state != STATE_FAULT);
    wire run_start_now = hmi_start_command || ((state == STATE_OFF) && button_press);
    wire initialize_target_now = !target_command_seen &&
        run_start_now;

    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            target_position_deg <= 16'sd0;
            target_position_pending_deg <= 16'sd0;
            target_position_counts_q16 <= 32'sd0;
            target_position_angle_q16 <= 32'sd0;
            target_command_seen <= 1'b0;
            target_delta_applied_pending <= 17'sd0;
            target_delta_pending <= 1'b0;
            target_count_delta_stage <= 32'sd0;
            target_angle_delta_stage <= 32'sd0;
            target_product_pending <= 1'b0;
        end else if (initialize_target_now) begin
            target_position_deg <= hmi_initial_target_wrapped[15:0];
            target_position_counts_q16 <= {pos, 16'b0};
            target_position_angle_q16 <= hmi_arm_absolute_q16;
            target_command_seen <= 1'b1;
            target_delta_pending <= 1'b0;
            target_product_pending <= 1'b0;
        end else begin
            if (target_product_pending) begin
                target_position_counts_q16 <= target_position_counts_q16 + target_count_delta_stage;
                target_position_angle_q16 <= target_position_angle_q16 + target_angle_delta_stage;
            end
            target_product_pending <= target_delta_pending;
            if (target_delta_pending) begin
                target_count_delta_stage <= target_count_delta_q16;
                target_angle_delta_stage <= target_angle_delta_q16;
            end
            target_delta_pending <= target_delta_commit;
            if (hmi_delta_valid) begin
                target_position_pending_deg <= target_pending_wrapped[15:0];
            end
            if (hmi_delta_cancel_valid) begin
                target_position_pending_deg <= 16'sd0;
            end
            if (target_delta_commit) begin
                target_position_deg <= target_position_commit_wrapped[15:0];
                target_position_pending_deg <= 16'sd0;
                target_delta_applied_pending <=
                    $signed({target_position_pending_deg[15], target_position_pending_deg});
                target_command_seen <= 1'b1;
            end
        end
    end

    reg [31:0] button_held_count;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) button_held_count <= 32'd0;
        else begin
            if (button_stable) button_held_count <= 32'd0;
            else if (button_held_count != 32'hFFFFFFFF) button_held_count <= button_held_count + 1'b1;
        end
    end
    wire button_held_long = button_held_count >= MANUAL_PD_HOLD_CYCLES;

    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            button_sync <= 2'b11;
            button_stable <= 1'b1;
            button_previous <= 1'b1;
            button_count <= 18'd0;
        end else begin
            button_sync <= {button_sync[0], mode_button_n};
            button_previous <= button_stable;
            if (button_sync[1] == button_stable) begin
                button_count <= 18'd0;
            end else if (button_count == BUTTON_DEBOUNCE_CYCLES) begin
                button_stable <= button_sync[1];
                button_count <= 18'd0;
            end else begin
                button_count <= button_count + 1'b1;
            end
        end
    end

    reg [31:0] capture_count;
    reg [31:0] swingup_count;
    // One finite initial-excitation attempt. Capture may finish it early;
    // otherwise the final half-cycle returns to OFF with zero motor command.
    reg [31:0] swingup_initial_half_count;
    reg [31:0] swingup_initial_halves_done;
    wire signed [31:0] pend_angle_abs = pend_angle[31] ? -pend_angle : pend_angle;
    wire signed [31:0] arm_angle_abs = arm_angle[31] ? -arm_angle : arm_angle;
    wire signed [31:0] pend_rate_filt_abs = pend_rate_filt_q16[31] ?
        -pend_rate_filt_q16 : pend_rate_filt_q16;
    wire signed [31:0] pend_rate_dfilt_abs = pend_rate_dfilt_q16[31] ?
        -pend_rate_dfilt_q16 : pend_rate_dfilt_q16;
    reg position_motion_permitted;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            position_motion_permitted <= 1'b0;
        end else if (state != STATE_BALANCE) begin
            position_motion_permitted <= 1'b0;
        end else if (state_valid) begin
            if (position_motion_permitted) begin
                if ((pend_angle_abs >= ARM_MOTION_TILT_DISABLE_Q16) ||
                    (pend_rate_dfilt_abs >= ARM_MOTION_RATE_DISABLE_Q16))
                    position_motion_permitted <= 1'b0;
            end else if ((pend_angle_abs <= ARM_MOTION_TILT_ENABLE_Q16) &&
                         (pend_rate_dfilt_abs <= ARM_MOTION_RATE_ENABLE_Q16)) begin
                position_motion_permitted <= 1'b1;
            end
        end
    end
    wire motion_reanchor = initialize_target_now || hmi_start_command ||
        hmi_stop_command || ((state == STATE_OFF) && button_press);
    arm_position_trajectory #(
        .MAX_SPEED_COUNTS_Q16(32'sd13107), // 0.20 count per 3 ms sample (~23 deg/s)
        .ACCEL_COUNTS_Q16(32'sd64),        // limit reference acceleration
        .ERROR_TO_SPEED_SHIFT(8),
        .ARM_RAD_PER_COUNT_Q30(ARM_RAD_PER_COUNT_Q30)
    ) u_arm_position_trajectory (
        .clk(clk), .rst_n(rst_n_sys), .sample_valid(state_valid),
        .enable((state == STATE_BALANCE) && position_motion_permitted),
        .reanchor(motion_reanchor),
        .actual_counts_q16({pos[15:0], 16'b0}), .actual_angle_q16(hmi_arm_absolute_q16),
        .target_counts_q16(target_position_counts_q16[31:0]),
        .target_angle_q16(target_position_angle_q16),
        .reference_counts_q16(motion_position_counts_q16),
        .reference_angle_q16(motion_position_angle_q16),
        .velocity_counts_q16(motion_velocity_counts_q16)
    );
    wire capture_now = state_valid &&
        (pend_angle_abs < BALANCE_CAPTURE_ANGLE_Q16) &&
        (pend_rate_filt_abs < BALANCE_CAPTURE_RATE_Q16);
    wire pend_fault_check = (state == STATE_BALANCE) ||
        ((MANUAL_PEND_FAULT != 0) && (state == STATE_MANUAL_PD));
    // C77 (task-75): cable protection keeps its own absolute reference (the
    // SW1-latched arm_zero_run), so moving the balance equilibrium cannot move
    // the physical limits.
    wire signed [31:0] cable_rel_counts = $signed(pos) - arm_zero_run;
    wire [31:0] cable_abs_counts = cable_rel_counts[31] ? -cable_rel_counts : cable_rel_counts;
    wire arm_limit_now = (ARM_CABLE_FAULT_ENABLE != 0) && (cable_abs_counts >= 32'sd1000);
    wire cable_soft_now = cable_abs_counts >= 32'sd900;
    wire sine_start_allowed = (state == STATE_BALANCE) && position_motion_permitted &&
        !control_fault && !state_timeout && (cable_rel_counts >= -32'sd600) &&
        (cable_rel_counts <= 32'sd600);
    // During swing-up and while the capture gate is still settling, the
    // balance reference must remain at the position where this run started.
    // Once the pendulum has been captured and the motion gate opens, switch
    // the reference to the committed absolute target.  The sine-reference
    // module then slews from the captured position to that target.
    wire signed [31:0] arm_start_reference_counts_q16 = $signed(arm_zero_run) <<< 16;
    wire use_start_reference = (state != STATE_BALANCE) || !position_motion_permitted;
    wire signed [31:0] arm_eq_counts_q16 = $signed(arm_eq_counts) <<< 16;
    wire signed [31:0] arm_base_reference_counts_q16 = use_start_reference ?
        arm_start_reference_counts_q16 : arm_eq_counts_q16;
    arm_sine_reference u_arm_sine_reference (
        .clk(clk), .rst_n(rst_n_sys), .sample_valid(state_valid),
        .balance_active(state == STATE_BALANCE),
        .motion_allowed(position_motion_permitted && !cable_soft_now &&
                        !control_fault && !state_timeout),
        .start_request(hmi_sine_start_valid), .start_allowed(sine_start_allowed),
        .stop_request(hmi_sine_stop_valid),
        .clear_request(hmi_start_command || ((state == STATE_OFF) && button_press)),
        .period_valid(hmi_sine_period_valid), .period_seconds(hmi_sine_period_seconds),
        .safety_origin_counts_q16($signed(arm_zero_run) <<< 16),
        .base_reference_counts_q16(arm_base_reference_counts_q16),
        .actual_counts_q16($signed(pos) <<< 16),
        .reference_counts_q16(arm_reference_counts_q16), .running(),
        .period_seconds_active());
    // C77: HMI D/A commits move the equilibrium by the same count delta as the
    // displayed target. Keep the equilibrium offset relative to the current
    // run's start point. This gives a constant +/-900 clamp and avoids putting
    // a wide absolute-origin subtract/compare chain on the 50 MHz path.
    reg signed [31:0] arm_eq_rel_counts;
    wire signed [31:0] eq_delta_counts = target_count_delta_stage >>> 16;
    wire signed [31:0] arm_eq_rel_sum = arm_eq_rel_counts + eq_delta_counts;
    wire signed [31:0] arm_eq_rel_clamped = (arm_eq_rel_sum > 32'sd900) ? 32'sd900 :
        (arm_eq_rel_sum < -32'sd900) ? -32'sd900 : arm_eq_rel_sum;
    wire [31:0] arm_eq_rel_next_abs = arm_eq_rel_clamped[31] ?
        -arm_eq_rel_clamped : arm_eq_rel_clamped;
    wire [31:0] arm_eq_rel_cur_abs = arm_eq_rel_counts[31] ?
        -arm_eq_rel_counts : arm_eq_rel_counts;
    wire signed [31:0] arm_eq_next_rel =
        (cable_soft_now && (arm_eq_rel_next_abs > arm_eq_rel_cur_abs)) ?
        arm_eq_rel_counts : arm_eq_rel_clamped;
    wire signed [31:0] arm_eq_next = arm_zero_run + arm_eq_next_rel;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            arm_eq_counts <= 32'sd0;
            arm_eq_rel_counts <= 32'sd0;
        end else begin
            if (run_start_now) begin
                if (!target_command_seen) begin
                    arm_eq_counts <= $signed(pos);
                    arm_eq_rel_counts <= 32'sd0;
                end else begin
                    // Keep an already committed absolute target across a
                    // stop/restart, but express its offset from the new
                    // run origin for the relative cable clamp below.
                    arm_eq_rel_counts <= arm_eq_counts - $signed(pos);
                end
            end else if (target_product_pending) begin
                arm_eq_counts <= arm_eq_next;
                arm_eq_rel_counts <= arm_eq_next_rel;
            end
        end
    end
    wire pend_angle_over = pend_fault_check && (pend_angle_abs >= FAULT_ANGLE_Q16);
    reg [1:0] pend_fault_count;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) pend_fault_count <= 2'd0;
        else if (state_valid) begin
            if (!pend_angle_over) pend_fault_count <= 2'd0;
            else if (pend_fault_count != 2'd3)
                pend_fault_count <= pend_fault_count + 1'b1;
        end
    end
    wire pend_fault_confirmed = pend_angle_over && (pend_fault_count >= 2'd2);
    wire fault_now = state_valid && state != STATE_OFF && state != STATE_FAULT &&
        (arm_limit_now || pend_fault_confirmed);

    reg [31:0] state_age;
    reg state_seen;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            state_age <= 32'd0;
            state_seen <= 1'b0;
            state_timeout <= 1'b0;
        end else if (hmi_stop_command) begin
            state_age <= 32'd0;
            state_seen <= 1'b0;
            state_timeout <= 1'b0;
        end else if (state_valid) begin
            state_age <= 32'd0;
            state_seen <= 1'b1;
            state_timeout <= 1'b0;
        end else if (state_seen && !state_timeout) begin
            if (state_age >= STATE_TIMEOUT_LIMIT - 1)
                state_timeout <= 1'b1;
            else
                state_age <= state_age + 1'b1;
        end
    end

    wire swingup_timeout_now = state == STATE_AUTO_SWINGUP && state_valid &&
        !capture_now && (swingup_count >= SWINGUP_TIMEOUT_LIMIT - 1);
    wire swingup_initial_active = (state == STATE_AUTO_SWINGUP) &&
        (swingup_initial_halves_done < SWINGUP_INITIAL_HALVES_LIMIT);
    // task-42: pause between initial half-cycles. After a completed half-cycle with more halves to
    // go, the FSM waits SWINGUP_HALF_DELAY_SAMPLES valid samples before flipping the direction.
    reg [15:0] swingup_pause_count;
    wire swingup_pause_active = (SWINGUP_HALF_DELAY_SAMPLES > 0) && (swingup_pause_count != 16'd0);
    // task-34: the last initial half-cycle may use a trimmed kick and a different sample cap.
    wire swingup_final_half_active = swingup_initial_active &&
        (swingup_initial_halves_done == SWINGUP_INITIAL_HALVES_LIMIT - 1);
    wire [31:0] swingup_half_sample_limit = swingup_final_half_active ?
        SWINGUP_FINAL_HALF_LIMIT : SWINGUP_INITIAL_HALF_LIMIT;
    // The existing soft reversal also completes a drive half-cycle. During the inter-half pause the
    // half-done decision is gated (the arm can still be outside the reversal angle while stopping).
    wire swingup_initial_half_done_raw = state_valid && swingup_initial_active &&
        (((arm_angle >= SWINGUP_ARM_REVERSAL_Q16) && swingup_direction_positive) ||
         ((arm_angle <= -SWINGUP_ARM_REVERSAL_Q16) && !swingup_direction_positive) ||
         (swingup_initial_half_count >= swingup_half_sample_limit - 1));
    wire swingup_initial_half_done = swingup_initial_half_done_raw && !swingup_pause_active;
    wire swingup_initial_complete_now = swingup_initial_half_done &&
        (swingup_initial_halves_done >= SWINGUP_INITIAL_HALVES_LIMIT - 1);
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            state <= STATE_OFF;
            capture_count <= 32'd0;
            swingup_count <= 32'd0;
            swingup_initial_half_count <= 32'd0;
            swingup_initial_halves_done <= 32'd0;
            swingup_pause_count <= 16'd0;
            arm_zero_run <= 32'sd0;   //  task-33: cleared on reset, set on SW1
            control_fault <= 1'b0;
            swingup_direction_positive <= 1'b1;
            manual_wait_screen <= 1'b0;
        end else begin
            if (hmi_stop_command) begin
                state <= STATE_OFF;
                capture_count <= 32'd0;
                swingup_count <= 32'd0;
                swingup_initial_half_count <= 32'd0;
                swingup_initial_halves_done <= 32'd0;
                swingup_pause_count <= 16'd0;
                control_fault <= 1'b0;
                manual_wait_screen <= 1'b0;
            end else if (hmi_mode_valid && (hmi_mode_value == 2'd1) &&
                         !control_fault && !state_timeout && (state != STATE_FAULT)) begin
                state <= STATE_MANUAL_WAIT;
                arm_zero_run <= pos;
                capture_count <= 32'd0;
                swingup_count <= 32'd0;
                swingup_initial_half_count <= 32'd0;
                swingup_initial_halves_done <= 32'd0;
                swingup_pause_count <= 16'd0;
                swingup_direction_positive <= 1'b1;
                manual_wait_screen <= 1'b1;
            end else if (hmi_mode_valid && (hmi_mode_value == 2'd2) &&
                         !control_fault && !state_timeout && (state != STATE_FAULT)) begin
                state <= STATE_AUTO_SWINGUP;
                arm_zero_run <= pos;
                capture_count <= 32'd0;
                swingup_count <= 32'd0;
                swingup_initial_half_count <= 32'd0;
                swingup_initial_halves_done <= 32'd0;
                swingup_pause_count <= 16'd0;
                swingup_direction_positive <= 1'b1;
                manual_wait_screen <= 1'b0;
            end else if (state_timeout && state != STATE_OFF && state != STATE_FAULT) begin
                state <= STATE_FAULT;
                control_fault <= 1'b1;
                swingup_initial_half_count <= 32'd0;
                swingup_initial_halves_done <= 32'd0;
            end else if (fault_now || swingup_timeout_now) begin
                state <= STATE_FAULT;
                control_fault <= 1'b1;
                capture_count <= 32'd0;
                swingup_count <= 32'd0;
                swingup_initial_half_count <= 32'd0;
                swingup_initial_halves_done <= 32'd0;
            end else begin
                case (state)
                    STATE_OFF: begin
                        capture_count <= 32'd0;
                        swingup_count <= 32'd0;
                        swingup_initial_half_count <= 32'd0;
                        swingup_initial_halves_done <= 32'd0;
                        swingup_direction_positive <= 1'b1;
                        if (button_press) begin
                            state <= STATE_MANUAL_WAIT;
                            arm_zero_run <= pos;
                            manual_wait_screen <= 1'b0;
                        end
                    end
                    STATE_MANUAL_WAIT: begin
                        swingup_count <= 32'd0;
                        swingup_initial_half_count <= 32'd0;
                        swingup_initial_halves_done <= 32'd0;
                        if (button_held_long) begin
                            state <= STATE_MANUAL_PD;
                            capture_count <= 32'd0;
                        end else if (button_release && !manual_wait_screen) begin
                            if (AUTO_SWINGUP_ENABLE != 0) begin
                                state <= STATE_AUTO_SWINGUP;
                                manual_wait_screen <= 1'b0;
                                capture_count <= 32'd0;
                                swingup_initial_half_count <= 32'd0;
                                swingup_initial_halves_done <= 32'd0;
                            end else begin
                                state <= STATE_OFF;
                                manual_wait_screen <= 1'b0;
                                capture_count <= 32'd0;
                            end
// Capture streak: a SAMPLE that misses the window breaks it (not every clock).
                        end else if (capture_now) begin
                            if (capture_count >= CAPTURE_HOLD_LIMIT - 1) begin
                                state <= STATE_BALANCE;
                                capture_count <= 32'd0;
                            end else capture_count <= capture_count + 1'b1;
                        end else if (state_valid) begin
                            capture_count <= 32'd0;
                        end
                    end
                    STATE_AUTO_SWINGUP: begin
                        manual_wait_screen <= 1'b0;
                        if (SWINGUP_HALF_DELAY_SAMPLES > 0) begin
                            // task-42: inter-half pause. Only the down-counter moves (half_count is
                            // frozen and no new half-done decision is taken); the direction flips on
                            // the last pause sample so the next half-cycle starts right after.
                            if (swingup_pause_active) begin
                                if (state_valid) begin
                                    if (swingup_pause_count == 16'd1) begin
                                        swingup_pause_count <= 16'd0;
                                        swingup_direction_positive <= ~swingup_direction_positive;
                                        swingup_initial_half_count <= 32'd0;
                                        swingup_initial_halves_done <= swingup_initial_halves_done + 1'b1;
                                    end else begin
                                        swingup_pause_count <= swingup_pause_count - 1'b1;
                                    end
                                end
                            end else if (state_valid && swingup_initial_active) begin
                                if (swingup_initial_half_done_raw) begin
                                    if (swingup_initial_halves_done < SWINGUP_INITIAL_HALVES_LIMIT - 1) begin
                                        // more halves remain: hold the current direction and pause
                                        swingup_pause_count <= SWINGUP_HALF_DELAY_SAMPLES;
                                    end else begin
                                        // last half finished: normal completion (then coast)
                                        swingup_direction_positive <= ~swingup_direction_positive;
                                        swingup_initial_half_count <= 32'd0;
                                        swingup_initial_halves_done <= swingup_initial_halves_done + 1'b1;
                                    end
                                end else begin
                                    swingup_initial_half_count <= swingup_initial_half_count + 1'b1;
                                end
                            end
                        end else if (state_valid && swingup_initial_active) begin
                            // DELAY=0: exact legacy behavior (bit-for-bit)
                            if (swingup_initial_half_done) begin
                                swingup_direction_positive <= ~swingup_direction_positive;
                                swingup_initial_half_count <= 32'd0;
                                swingup_initial_halves_done <= swingup_initial_halves_done + 1'b1;
                            end else begin
                                swingup_initial_half_count <= swingup_initial_half_count + 1'b1;
                            end
                        end
                        if (button_press) begin
                            state <= STATE_OFF;
                            capture_count <= 32'd0;
                            swingup_count <= 32'd0;
                            swingup_initial_half_count <= 32'd0;
                            swingup_initial_halves_done <= 32'd0;
                            swingup_pause_count <= 16'd0;
                        end else if (capture_now && (capture_count >= CAPTURE_HOLD_LIMIT - 1)) begin
                            // A fully qualified capture wins even on the final sample.
                            state <= STATE_BALANCE;
                            capture_count <= 32'd0;
                            swingup_pause_count <= 16'd0;
                        end else if ((SWINGUP_COAST_ENABLE == 0) &&
                                     (swingup_initial_complete_now || !swingup_initial_active)) begin
                            // task-35: with SWINGUP_COAST_ENABLE the initial segment end is NOT an exit;
                            // the FSM keeps coasting in AUTO with auto_cmd=0 until capture / 18 s timeout /
                            // button. COAST_ENABLE=0 keeps the old OFF behavior bit-for-bit.
                            state <= STATE_OFF;
                            capture_count <= 32'd0;
                            swingup_count <= 32'd0;
                            swingup_pause_count <= 16'd0;
                        end else if (capture_now) begin
                            capture_count <= capture_count + 1'b1;
                        end else if (state_valid) begin
                            capture_count <= 32'd0;
                            if (swingup_count < SWINGUP_TIMEOUT_LIMIT)
                                swingup_count <= swingup_count + 1'b1;
                        end
                    end
                    STATE_BALANCE: begin
                        manual_wait_screen <= 1'b0;
                        capture_count <= 32'd0;
                        swingup_count <= 32'd0;
                        swingup_initial_half_count <= 32'd0;
                        swingup_initial_halves_done <= 32'd0;
                        if (button_press) state <= STATE_OFF;
                    end
                    STATE_MANUAL_PD: begin
                        manual_wait_screen <= 1'b0;
                        capture_count <= 32'd0;
                        swingup_count <= 32'd0;
                        swingup_initial_half_count <= 32'd0;
                        swingup_initial_halves_done <= 32'd0;
                        if (button_press) state <= STATE_OFF;
                    end
                    default: begin
                        state <= STATE_FAULT;
                        capture_count <= 32'd0;
                        swingup_count <= 32'd0;
                    end
                endcase
            end
        end
    end

// Arming: which path is allowed to drive the motor.
    wire balance_enabled = state == STATE_BALANCE;
    wire auto_swing_enabled = state == STATE_AUTO_SWINGUP;
    wire manual_pd_enabled = state == STATE_MANUAL_PD;
    wire swingup_ready = !state_timeout && !control_fault;
    // task-34: trimmed final-half kick (direction sign still follows the half-cycle).
    wire signed [31:0] swingup_half_cmd = swingup_final_half_active ?
        SWINGUP_FINAL_CMD : SWINGUP_INITIAL_CMD;
    wire signed [31:0] auto_cmd = swingup_initial_active && swingup_ready && !swingup_pause_active ?
        (swingup_direction_positive ? swingup_half_cmd : -swingup_half_cmd) : 32'sd0;
    wire pid_armed = balance_enabled && !state_timeout && !control_fault;
    wire manual_pd_armed = manual_pd_enabled && !state_timeout && !control_fault;
    wire control_path_enabled = pid_armed || manual_pd_armed;

// task-34: BALANCE capture soft start. bal_entry_count counts state_valid samples inside BALANCE
// (0 on the first BALANCE sample); the gain ramps BAL_SOFT_START_MIN_Q16 -> 1.0 over
// BAL_SOFT_START_SAMPLES samples (power of two, shift based). Cleared outside BALANCE.
    reg [31:0] bal_entry_count;
    reg signed [31:0] bal_soft_gain_q16;
    // Narrow ramp multiplies: the command magnitude is bounded by DUTY_MAX_CTRL (<=220) and the
    // gain by 65536, so 18x18-bit products are exact; 32x32 LUT multipliers failed timing here.
    wire signed [31:0] bal_soft_step_q16 = 32'sd65536 - BAL_SOFT_START_MIN_Q16;
    wire [17:0] bal_step_u = bal_soft_step_q16[17:0];       // positive ramp step (unsigned)
    wire [6:0] bal_count_u = bal_entry_count[6:0];          // counter saturates well before 128
    // Split the 18x7 ramp multiply into two unsigned 9x7 halves (exact): step*(lo+hi*512).
    wire [8:0] bal_step_lo_u = bal_step_u[8:0];
    wire [8:0] bal_step_hi_u = bal_step_u[17:9];
    wire [15:0] bal_step_prod_lo = bal_step_lo_u * bal_count_u;
    wire [15:0] bal_step_prod_hi = bal_step_hi_u * bal_count_u;
    wire [24:0] bal_soft_prod = bal_step_prod_lo + (bal_step_prod_hi << 9);
    wire signed [63:0] bal_soft_gain_raw = $signed(BAL_SOFT_START_MIN_Q16) +
        ($signed({38'd0, bal_soft_prod}) >>> BAL_SOFT_START_SHIFT);
    wire signed [31:0] bal_soft_gain_comb =
        (BAL_SOFT_START_SAMPLES == 0 || bal_entry_count >= BAL_SOFT_START_SAMPLES ||
         bal_soft_gain_raw > 64'sd65536) ? 32'sd65536 : bal_soft_gain_raw[31:0];
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            bal_entry_count <= 32'd0;
            bal_soft_gain_q16 <= 32'sd65536;
        end else if (state != STATE_BALANCE) begin
            bal_entry_count <= 32'd0;
            bal_soft_gain_q16 <= (BAL_SOFT_START_SAMPLES == 0) ? 32'sd65536 : BAL_SOFT_START_MIN_Q16;
        end else begin
            if (state_valid) bal_entry_count <= bal_entry_count + 1'b1;
            bal_soft_gain_q16 <= bal_soft_gain_comb;
        end
    end

// Controller -> duty mapping (shared by BALANCE and manual PD).
    wire signed [63:0] umax_wide = $signed({32'sd0, UMAX_Q16});
    wire signed [63:0] manual_pd_raw_q32 =
        MANUAL_KP_Q16 * pend_angle + MANUAL_KD_Q16 * pend_rate_dfilt_q16;

    reg signed [63:0] manual_pd_norm_q16;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) manual_pd_norm_q16 <= 64'sd0;
        else manual_pd_norm_q16 <= manual_pd_raw_q32 >>> 16;
    end

    wire signed [63:0] manual_pd_signed_q16 = (CONTROL_SIGN >= 0) ?
        -manual_pd_norm_q16 : manual_pd_norm_q16;
    wire signed [63:0] pid_signed_q16 = (CONTROL_SIGN >= 0) ?
        $signed(control_u_q16) : -$signed(control_u_q16);
    wire signed [63:0] pendulum_loop_q16 = pid_armed ? pid_signed_q16 :
        manual_pd_signed_q16;
// Arm position loop: P + clamped D (outward push, rod-loop booster).
    wire signed [63:0] arm_d_clamp_wide = $signed({32'sd0, ARM_LOOP_D_CLAMP_Q16});
    wire signed [31:0] arm_position_error_q16 = hmi_arm_absolute_q16 - motion_position_angle_q16;
    // Split the position error from its gain multiply. This adds one 50 MHz
    // cycle (20 ns, negligible beside the 3 ms control sample) and shortens
    // the C53 timing path without changing the controller's sample rate.
    reg signed [31:0] arm_position_error_q16_r;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) arm_position_error_q16_r <= 32'sd0;
        else arm_position_error_q16_r <= arm_position_error_q16;
    end
    // C76 (task-74): restore the original handover arm loop - P is driven by
    // arm_angle = pos - arm_zero_run (the FIXED anchor latched on SW1), not by the
    // position-loop error (measured - trajectory reference was positive feedback).
    reg signed [31:0] arm_angle_p;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) arm_angle_p <= 32'sd0;
        else arm_angle_p <= arm_angle;
    end
    wire signed [63:0] arm_loop_p_pos_q16 = ((-$signed(arm_loop_kp_runtime_q16)) * $signed(arm_angle_p)) >>> 16;   //  T44b: leading-minus shape (fast)
    wire signed [63:0] arm_loop_d_raw = ((-$signed(arm_loop_kd_runtime_q16)) * $signed(arm_rate_dfilt_q16)) >>> 16;   //  T44b: leading-minus shape
    wire signed [63:0] arm_loop_d_pos_q16 = (arm_loop_d_raw > arm_d_clamp_wide) ?
        arm_d_clamp_wide :
        (arm_loop_d_raw < -arm_d_clamp_wide) ? -arm_d_clamp_wide : arm_loop_d_raw;
// Arm position loop: slow clamped integral anchor (sign opposite to P).
    wire arm_i_armed = ((state == STATE_BALANCE) || (state == STATE_MANUAL_PD)) &&
        ARM_LOOP_ENABLE;
    reg signed [31:0] arm_i_acc;
    wire signed [31:0] arm_i_acc_clamp =
        $signed(ARM_LOOP_I_CLAMP_Q16) <<< arm_loop_i_shift_runtime;
    // SIGN: POSITIVE (restoring). The bench on 2026-10-12 disproved the old note:
    // on this wiring a POSITIVE command drives pos DOWN, so with acc accumulating
    // arm_angle a POSITIVE contribution pulls the arm back to the start. The old
    // negative sign was the drift engine (it pushed the arm away with up to 25
    // applied duty). Evidence: adding a 75 duty term with the old sign took the
    // drift from -9 counts/s to -169/-231 counts/s; T32 (bigger old-sign clamp)
    // "held still at the wrong place"; T33 (250) ran away.
    wire signed [31:0] arm_loop_i_q16 = (arm_i_acc >>> arm_loop_i_shift_runtime);
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            arm_i_acc <= 32'sd0;
        end else if (!arm_i_armed) begin
            arm_i_acc <= 32'sd0;
        end else if (hmi_arm_gain_valid && (hmi_arm_gain_index == 2'd1)) begin
            // An LI command changed the integral authority: re-clamp the
            // accumulator to the new bound so a burst of authority does not
            // reach the output all at once.
            arm_i_acc <= (arm_i_acc > arm_i_acc_clamp) ? arm_i_acc_clamp :
                         (arm_i_acc < -arm_i_acc_clamp) ? -arm_i_acc_clamp :
                         arm_i_acc;
        end else if (sample_valid && (pend_angle_abs < ARM_I_ANGLE_Q16)) begin
            if (($signed(arm_angle) > 0) && (arm_i_acc < arm_i_acc_clamp)) begin
                arm_i_acc <= (arm_i_acc + $signed(arm_angle) > arm_i_acc_clamp) ?
                    arm_i_acc_clamp : arm_i_acc + $signed(arm_angle);
            end else if (($signed(arm_angle) < 0) && (arm_i_acc > -arm_i_acc_clamp)) begin
                arm_i_acc <= (arm_i_acc + $signed(arm_angle) < -arm_i_acc_clamp) ?
                    -arm_i_acc_clamp : arm_i_acc + $signed(arm_angle);
            end
        end
    end
    // Continuous far-field position priority assist (2026-10-12 v2). The FIRST
    // version was a constant-force relay with hysteresis; the bench showed it
    // sustained a limit cycle (arm pinned to +-80 counts, rod swinging +-17 deg at
    // 3 Hz). With the relay off the same rig balanced 55.5 s at 90.7% within +-5 deg
    // and held the arm within ~90 counts. This version keeps "a slight offset is
    // corrected immediately" but is CONTINUOUS: no step, no hysteresis, saturated
    // ramp. SIGN: positive error -> positive command (on this machine a positive
    // command drives pos DOWN).
    // C77: the +/-1000-count cable hard limit is now switchable (ARM_CABLE_FAULT_ENABLE
    // = 0 by default), so the arm can pass +/-2048 counts. The old signed 12-bit window
    // wrapped there and would have reversed the assist; keep the full 32-bit pos.
    // C76 (task-74): original handover far-field assist - 32-bit pos - arm_zero_run.
    wire signed [31:0] arm_pri_actual_counts_q16 = $signed(pos) <<< 16;
    reg signed [31:0] arm_pri_err_counts_q16;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) arm_pri_err_counts_q16 <= 32'sd0;
        else arm_pri_err_counts_q16 <=
            arm_pri_actual_counts_q16 - arm_reference_counts_q16;
    end
    wire signed [31:0] arm_pri_abs_q16 = arm_pri_err_counts_q16[31] ?
        -arm_pri_err_counts_q16 : arm_pri_err_counts_q16;
    wire signed [31:0] arm_pri_on_q16 = ARM_PRI_ON_COUNTS <<< 16;
    wire signed [31:0] arm_pri_over_q16 = (arm_pri_abs_q16 > arm_pri_on_q16) ?
        (arm_pri_abs_q16 - arm_pri_on_q16) : 32'sd0;
    // The position error and ramp now both retain Q16 count precision.
    wire signed [31:0] arm_pri_mag_q16_raw = arm_pri_over_q16 >>> ARM_PRI_SLOPE_SHIFT;
    wire signed [31:0] arm_pri_mag_q16 = (arm_pri_mag_q16_raw > ARM_PRI_DUTY_Q16) ?
        ARM_PRI_DUTY_Q16 : arm_pri_mag_q16_raw;
    wire signed [63:0] arm_pri_q16 = (arm_pri_err_counts_q16 >= 0) ?
        $signed({32'sd0, arm_pri_mag_q16}) : -$signed({32'sd0, arm_pri_mag_q16});
    wire signed [63:0] arm_loop_q16 = ARM_LOOP_ENABLE ?
        (arm_loop_p_pos_q16 + arm_loop_d_pos_q16 + arm_loop_i_q16 + arm_pri_q16) :
        64'sd0;   // P + clamped D + anchor + position ramp (ARM_LOOP_ENABLE=0 时整体清零)
    // Split the arm loop before balance-headroom limiting. This adds one 50 MHz
    // cycle to the complete arm term (20 ns), while preserving per-cycle updates;
    // the controller and trajectory still advance only on their 3 ms valid pulse.
    reg signed [63:0] arm_loop_prepriority_q16_r;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) arm_loop_prepriority_q16_r <= 64'sd0;
        else arm_loop_prepriority_q16_r <= arm_loop_q16;
    end
    // Position control may use only actuator headroom left by the pendulum
    // controller. This keeps the balance command from being clipped by arm
    // tracking while a move is in progress. The rod loop itself remains first
    // in the sum; outside BALANCE, preserve the existing manual-PD behavior.
    wire signed [63:0] pendulum_abs_q16 = pendulum_loop_q16[63] ?
        -pendulum_loop_q16 : pendulum_loop_q16;
    wire signed [63:0] balance_arm_headroom_q16 =
        (pendulum_abs_q16 < umax_wide) ? (umax_wide - pendulum_abs_q16) : 64'sd0;
    wire signed [63:0] arm_loop_priority_q16 = (state == STATE_BALANCE) ?
        ((arm_loop_prepriority_q16_r > balance_arm_headroom_q16) ? balance_arm_headroom_q16 :
         (arm_loop_prepriority_q16_r < -balance_arm_headroom_q16) ? -balance_arm_headroom_q16 : arm_loop_prepriority_q16_r) :
        arm_loop_prepriority_q16_r;
// Second arm-term pipeline register applies the priority-limited result.
    reg signed [63:0] arm_loop_q16_r;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) arm_loop_q16_r <= 64'sd0;
        else arm_loop_q16_r <= arm_loop_priority_q16;
    end
    wire signed [63:0] control_path_u_q16 = pendulum_loop_q16 + arm_loop_q16_r;
    wire signed [63:0] control_path_sat_comb = control_path_u_q16 > umax_wide ? umax_wide :
        control_path_u_q16 < -umax_wide ? -umax_wide : control_path_u_q16;
    reg signed [32:0] control_path_sat_q16;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) control_path_sat_q16 <= 33'sd0;
        else control_path_sat_q16 <= control_path_sat_comb[32:0];
    end

// Stage 3: |u|>>>16 -> global scale -> dead zone -> cap -> signed command.
    wire signed [32:0] control_path_offset_mag = (control_path_sat_q16[32] ?
        -control_path_sat_q16 : control_path_sat_q16) >>> 16;
    wire signed [32:0] control_path_mod_mag = control_path_offset_mag >>> DUTY_SCALE_SHIFT;
    wire signed [32:0] control_path_duty = (control_path_offset_mag == 33'sd0) ? 33'sd0 :
        ((DUTY_DZ + control_path_mod_mag) > $signed({1'b0, DUTY_MAX_CTRL}) ?
         $signed({1'b0, DUTY_MAX_CTRL}) : (DUTY_DZ + control_path_mod_mag));
    wire signed [31:0] control_path_cmd_comb = (control_path_duty == 33'sd0) ? 32'sd0 :
        (control_path_sat_q16[32] ? -control_path_duty[31:0] : control_path_duty[31:0]);
    // task-34: capture soft start on the BALANCE control command only (Q16 multiply by the
    // ramp gain; manual PD and the swing-up path are untouched). SAMPLES=0 keeps the exact
    // legacy expression (bit-for-bit compat). The command magnitude is bounded by
    // DUTY_MAX_CTRL (<=220, fits 9 signed bits) and the gain by 65536 (18 signed bits), so the
    // product is exact; a narrow 9x18 multiplier plus one extra command register keeps the
    // 50 MHz path short (a full 32x32 LUT multiplier failed timing by -62 ns).
    reg signed [31:0] bal_cmd_d1;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) bal_cmd_d1 <= 32'sd0;
        else bal_cmd_d1 <= control_path_cmd_comb;
    end
    wire signed [8:0] bal_cmd_n = bal_cmd_d1[8:0];
    // Exact signed product cmd*gain with two small LUT multipliers: split the 9-bit two's
    // complement command into 5-bit + 4-bit unsigned parts (value = lo + hi*32, i.e. cmd mod 512)
    // and subtract 512*gain for negative commands. gain is 0..65536 (17 unsigned bits).
    wire [4:0] bal_cmd_lo_u = bal_cmd_d1[4:0];
    wire [3:0] bal_cmd_hi_u = bal_cmd_d1[8:5];
    wire [16:0] bal_gain_u = bal_soft_gain_q16[16:0];
    wire [21:0] bal_prod_lo_u = bal_cmd_lo_u * bal_gain_u;   // 5x17
    wire [20:0] bal_prod_hi_u = bal_cmd_hi_u * bal_gain_u;   // 4x17
    wire [25:0] bal_prod_mod = {4'd0, bal_prod_lo_u} + ({5'd0, bal_prod_hi_u} << 5);
    wire [25:0] bal_prod_offset = bal_cmd_d1[8] ? ({9'd0, bal_gain_u} << 9) : 26'd0;
    wire [25:0] bal_prod_s = bal_prod_mod - bal_prod_offset;
    wire signed [63:0] bal_cmd_scaled_q16 = {{38{bal_prod_s[25]}}, bal_prod_s} >>> 16;
    reg signed [31:0] bal_cmd_scaled_r;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) bal_cmd_scaled_r <= 32'sd0;
        else bal_cmd_scaled_r <= bal_cmd_scaled_q16[31:0];
    end
    wire signed [31:0] bal_cmd_final = (BAL_SOFT_START_SAMPLES != 0 && balance_enabled) ?
        bal_cmd_scaled_r : control_path_cmd_comb;
    reg signed [31:0] control_cmd_r;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) control_cmd_r <= 32'sd0;
        else control_cmd_r <= control_path_enabled ? bal_cmd_final : 32'sd0;
    end

    wire signed [31:0] cmd = control_path_enabled ? control_cmd_r :
        auto_swing_enabled ? auto_cmd : 32'sd0;
    wire signed [63:0] cmd_wide = cmd;
    wire [63:0] cmd_magnitude = cmd_wide[63] ? -cmd_wide : cmd_wide;
    wire [9:0] duty_out = cmd_magnitude > 64'd255 ? 10'd255 : cmd_magnitude[9:0];
    // task-40: while coasting in AUTO after the initial segment, short-brake the motor (TB6612
    // mode 11: AIN1=AIN2=1) so the arm stops and the rod slides on a fixed pivot. Only the AUTO
    // coast phase is affected; the swing-up push, BALANCE, manual PD and OFF are unchanged, and
    // SWINGUP_COAST_BRAKE=0 reproduces the old behavior bit-for-bit.
    wire swingup_coasting = (state == STATE_AUTO_SWINGUP) && !swingup_initial_active && swingup_ready;
    wire coast_brake = (SWINGUP_COAST_BRAKE != 0) && swingup_coasting;
    // task-42: optional short brake during the inter-half pause (cmd is 0 during the pause).
    wire half_pause_brake = (SWINGUP_HALF_DELAY_BRAKE != 0) && (state == STATE_AUTO_SWINGUP) &&
        swingup_pause_active && swingup_ready;
    wire [1:0] motor_mode = (coast_brake || half_pause_brake) ? 2'b11 :
        (cmd == 0 ? 2'b00 : cmd[31] ? 2'b10 : 2'b01);

// TB6612 outputs (both channels driven identically).
    motor_ctrl u_motor_a(
        .clk_50m(clk), .rst_n(rst_n_sys), .motor_a_sel(motor_mode), .duty_out(duty_out),
        .ain1(ain1), .ain2(ain2), .pwm(pwma));
    motor_ctrl u_motor_b(
        .clk_50m(clk), .rst_n(rst_n_sys), .motor_a_sel(motor_mode), .duty_out(duty_out),
        .ain1(bn1), .ain2(bn2), .pwm(pwmb));

// Telemetry event flags: [4]=MANUAL_WAIT [3]=manual PD [2]=fault
// [1]=auto swing-up [0]=balance.
    wire [4:0] event_flags = {state == STATE_MANUAL_WAIT, manual_pd_enabled,
        state == STATE_FAULT, auto_swing_enabled, balance_enabled};

    wire telemetry_overrun;
    telemetry_3ms u_telemetry(
        .clk(clk), .rst_n(rst_n_sys), .sample_valid(sample_valid), .pos(pos), .theta(theta),
        .adc_raw(adc_data_in), .duty(duty_out[7:0]), .ain1(ain1), .ain2(ain2), .pwm(pwma),
        .event_flags(event_flags),
        .tx(uart_tx), .overrun(telemetry_overrun));

    // Screen values are signed whole degrees (rates in deg/s) and PWM percent.
    // The arm display uses encoder position relative to FPGA power-on, while
    // the balancing loop continues to use its own captured reference.
    wire signed [31:0] hmi_arm_absolute_counts = $signed(pos) - ARM_ZERO_COUNTS;
    wire signed [63:0] hmi_arm_absolute_product =
        $signed(hmi_arm_absolute_counts) * $signed(ARM_RAD_PER_COUNT_Q30);
    // Q16 radians to Q32 degrees: round(180/pi * 2^16).
    // A 22-bit serial multiply feeds a separate rounding cycle; the critical
    // accumulator path no longer contains a constant divide.
    localparam [21:0] Q16_RAD_TO_DEG = 22'd3754936;
    reg [1:0] hmi_conv_channel;
    reg [4:0] hmi_conv_bit;
    reg hmi_conv_busy;
    reg hmi_conv_round_pending;
    reg signed [53:0] hmi_conv_accum;
    reg signed [53:0] hmi_conv_multiplicand;
    reg signed [53:0] hmi_conv_result;
    reg [21:0] hmi_conv_coefficient;
    // Reduce the absolute arm display angle a full revolution at a time.
    // This is sequential to avoid a wide modulo/divider on the timing path.
    reg hmi_angle_wrap_busy;
    reg signed [31:0] hmi_angle_wrap_value;
    wire signed [53:0] hmi_conv_sum = hmi_conv_coefficient[0]
        ? hmi_conv_accum + hmi_conv_multiplicand : hmi_conv_accum;
    wire signed [21:0] hmi_conv_integer = hmi_conv_result[53:32];
    wire [31:0] hmi_conv_fraction = hmi_conv_result[31:0];
    // Round to nearest, ties away from zero, without a divider.
    wire hmi_conv_round_up = hmi_conv_result[53]
        ? (hmi_conv_fraction[31] && (|hmi_conv_fraction[30:0]))
        : hmi_conv_fraction[31];
    wire signed [21:0] hmi_conv_rounded = hmi_conv_integer + hmi_conv_round_up;
    wire signed [31:0] hmi_conv_rounded_deg =
        {{10{hmi_conv_rounded[21]}}, hmi_conv_rounded};

    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            hmi_arm_absolute_q16 <= 32'sd0;
            hmi_arm_angle_deg <= 32'sd0;
            hmi_pend_angle_deg <= 32'sd0;
            hmi_arm_rate_deg <= 32'sd0;
            hmi_pend_rate_deg <= 32'sd0;
            hmi_conv_channel <= 2'd0;
            hmi_conv_bit <= 5'd0;
            hmi_conv_busy <= 1'b0;
            hmi_conv_round_pending <= 1'b0;
            hmi_conv_accum <= 54'sd0;
            hmi_conv_multiplicand <= 54'sd0;
            hmi_conv_result <= 54'sd0;
            hmi_conv_coefficient <= 22'd0;
            hmi_angle_wrap_busy <= 1'b0;
            hmi_angle_wrap_value <= 32'sd0;
        end else begin
            if (sample_valid)
                hmi_arm_absolute_q16 <= hmi_arm_absolute_product >>> 14;
            if (hmi_angle_wrap_busy) begin
                if (hmi_angle_wrap_value > 32'sd180) begin
                    hmi_angle_wrap_value <= hmi_angle_wrap_value - 32'sd360;
                end else if (hmi_angle_wrap_value < -32'sd180) begin
                    hmi_angle_wrap_value <= hmi_angle_wrap_value + 32'sd360;
                end else begin
                    hmi_arm_angle_deg <= hmi_angle_wrap_value;
                    hmi_conv_channel <= 2'd1;
                    hmi_angle_wrap_busy <= 1'b0;
                end
            end else if (hmi_conv_round_pending) begin
                case (hmi_conv_channel)
                    2'd0: begin
                        hmi_angle_wrap_value <= hmi_conv_rounded_deg;
                        hmi_angle_wrap_busy <= 1'b1;
                    end
                    2'd1: hmi_pend_angle_deg <= hmi_conv_rounded_deg;
                    // The raw encoder difference is quantized at about 115 deg/s
                    // per count for a 3 ms sample. Reuse the existing averaged
                    // and low-pass filtered rate estimate for a smoother display.
                    2'd2: hmi_arm_rate_deg <= hmi_conv_rounded_deg;
                    default: hmi_pend_rate_deg <= hmi_conv_rounded_deg;
                endcase
                if (hmi_conv_channel != 2'd0)
                    hmi_conv_channel <= hmi_conv_channel + 1'b1;
                hmi_conv_round_pending <= 1'b0;
            end else if (!hmi_conv_busy && !hmi_angle_wrap_busy) begin
                hmi_conv_accum <= 54'sd0;
                hmi_conv_coefficient <= Q16_RAD_TO_DEG;
                hmi_conv_bit <= 5'd0;
                hmi_conv_busy <= 1'b1;
                case (hmi_conv_channel)
                    2'd0: hmi_conv_multiplicand <=
                        {{22{hmi_arm_absolute_q16[31]}}, hmi_arm_absolute_q16};
                    2'd1: hmi_conv_multiplicand <= {{22{pend_angle[31]}}, pend_angle};
                    2'd2: hmi_conv_multiplicand <=
                        {{22{arm_rate_dfilt_q16[31]}}, arm_rate_dfilt_q16};
                    default: hmi_conv_multiplicand <= {{22{pend_rate[31]}}, pend_rate};
                endcase
            end else begin
                hmi_conv_accum <= hmi_conv_sum;
                hmi_conv_multiplicand <= hmi_conv_multiplicand <<< 1;
                hmi_conv_coefficient <= hmi_conv_coefficient >> 1;
                if (hmi_conv_bit == 5'd21) begin
                    hmi_conv_result <= hmi_conv_sum;
                    hmi_conv_round_pending <= 1'b1;
                    hmi_conv_busy <= 1'b0;
                end else begin
                    hmi_conv_bit <= hmi_conv_bit + 1'b1;
                end
            end
        end
    end
    reg [7:0] hmi_pwm_duty_stage;
    reg signed [31:0] hmi_pwm_percent;
    wire [11:0] hmi_pwm_scaled = (hmi_pwm_duty_stage * 12'd10) + 12'd13;
    wire [6:0] hmi_pwm_percent_calc = hmi_pwm_scaled / 12'd27;
    always @(posedge clk or negedge rst_n_sys) begin
        if (!rst_n_sys) begin
            hmi_pwm_duty_stage <= 8'd0;
            hmi_pwm_percent <= 32'sd0;
        end else begin
            hmi_pwm_duty_stage <= duty_out[7:0];
            hmi_pwm_percent <= {25'd0, hmi_pwm_percent_calc};
        end
    end
    // TJC Xfloat fields display an integer value with vvs1=2. Convert the
    // runtime Q16.16 gains to hundredths with round-to-nearest: 950.00 is sent
    // as 95000, 0.10 is sent as 10.
    wire signed [63:0] pid_kp_wide = {{32{pid_kp_runtime_q16[31]}}, pid_kp_runtime_q16};
    wire signed [63:0] pid_ki_wide = {{32{pid_ki_runtime_q16[31]}}, pid_ki_runtime_q16};
    wire signed [63:0] pid_kd_wide = {{32{pid_kd_runtime_q16[31]}}, pid_kd_runtime_q16};
    // 100 = 64 + 32 + 4; the shift-add form avoids allocating another DSP
    // multiplier merely for the display conversion.
    wire signed [63:0] pid_kp_centi_product = (pid_kp_wide <<< 6) +
                                               (pid_kp_wide <<< 5) +
                                               (pid_kp_wide <<< 2);
    wire signed [63:0] pid_ki_centi_product = (pid_ki_wide <<< 6) +
                                               (pid_ki_wide <<< 5) +
                                               (pid_ki_wide <<< 2);
    wire signed [63:0] pid_kd_centi_product = (pid_kd_wide <<< 6) +
                                               (pid_kd_wide <<< 5) +
                                               (pid_kd_wide <<< 2);
    wire signed [31:0] hmi_pid_kp_centi =
        (pid_kp_centi_product + 64'sd32768) >>> 16;
    wire signed [31:0] hmi_pid_ki_centi =
        (pid_ki_centi_product + 64'sd32768) >>> 16;
    wire signed [31:0] hmi_pid_kd_centi =
        (pid_kd_centi_product + 64'sd32768) >>> 16;
    // ARM-loop display: P/D are shown as their EFFECTIVE magnitude (negate the
    // stored sign convention), I is shown as the raw shift value.
    wire signed [63:0] arm_kp_wide = -$signed({{32{arm_loop_kp_runtime_q16[31]}},
                                                 arm_loop_kp_runtime_q16});
    wire signed [63:0] arm_kd_wide = -$signed({{32{arm_loop_kd_runtime_q16[31]}},
                                                 arm_loop_kd_runtime_q16});
    wire signed [63:0] arm_kp_centi_product = (arm_kp_wide <<< 6) +
                                               (arm_kp_wide <<< 5) +
                                               (arm_kp_wide <<< 2);
    wire signed [63:0] arm_kd_centi_product = (arm_kd_wide <<< 6) +
                                               (arm_kd_wide <<< 5) +
                                               (arm_kd_wide <<< 2);
    wire signed [31:0] hmi_arm_kp_centi =
        (arm_kp_centi_product + 64'sd32768) >>> 16;
    wire signed [31:0] hmi_arm_kd_centi =
        (arm_kd_centi_product + 64'sd32768) >>> 16;
    wire signed [31:0] hmi_arm_li_shift =
        $signed({28'd0, arm_loop_i_shift_runtime});
    hmi_display8 u_hmi_display (
        .clk(clk), .rst_n(rst_n_sys),
        .value0(hmi_arm_angle_deg),
        .value1(hmi_arm_angle_deg),
        .value2(hmi_pend_angle_deg),
        .value3(hmi_arm_rate_deg),
        .value4(hmi_pend_rate_deg),
        .value5(hmi_pwm_percent),
        .value6({{16{target_position_deg[15]}}, target_position_deg}),
        .value7({{16{target_position_deg[15]}}, target_position_deg}),
        .value8(hmi_pid_kp_centi),
        .value9(hmi_pid_ki_centi),
        .value10(hmi_pid_kd_centi),
        .value11(hmi_arm_kp_centi),
        .value12(hmi_arm_li_shift),
        .value13(hmi_arm_kd_centi),
        .tx(hmi_tx));

    wire motor_enabled = (motor_mode != 2'b00);
    assign led = {~state[2], ~state[1], ~state[0], ~motor_enabled};
endmodule

