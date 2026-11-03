`timescale 1ns/1ps

// Low-resource sine position reference. The 32-bit phase represents one turn;
// a CORDIC computes the Q16.16 encoder-count offset once per control sample.
module arm_sine_reference (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,
    input  wire               balance_active,
    input  wire               motion_allowed,
    input  wire               start_request,
    input  wire               start_allowed,
    input  wire               stop_request,
    input  wire               clear_request,
    input  wire               period_valid,
    input  wire [7:0]         period_seconds,
    input  wire signed [31:0] safety_origin_counts_q16,
    input  wire signed [31:0] base_reference_counts_q16,
    input  wire signed [31:0] actual_counts_q16,
    output wire signed [31:0] reference_counts_q16,
    output reg                running,
    output reg [7:0]          period_seconds_active
);
    localparam [23:0] PHASE_NUMERATOR = 24'd12884902;
    localparam [31:0] CORDIC_X0_Q16 = 32'sd10347202; // 260.000 counts * CORDIC gain inverse
    localparam signed [31:0] BASE_SLEW_STEP_Q16 = 32'sd13107; // 0.2 encoder count per 3 ms sample

    reg [31:0] phase;
    reg [31:0] phase_step;
    reg signed [31:0] center_counts_q16;
    reg signed [31:0] held_reference_q16;
    reg signed [31:0] sine_offset_q16;
    reg signed [31:0] base_reference_applied_q16;
    reg hold_valid;
    // Keep an early S command until the balance safety gate opens.  The
    // command is still rejected outside BALANCE and is cleared by stop/reset.
    reg start_pending;

    wire signed [31:0] base_reference_error_q16 =
        base_reference_counts_q16 - base_reference_applied_q16;
    wire signed [31:0] base_reference_step_q16 =
        (base_reference_error_q16 > BASE_SLEW_STEP_Q16) ? BASE_SLEW_STEP_Q16 :
        (base_reference_error_q16 < -BASE_SLEW_STEP_Q16) ? -BASE_SLEW_STEP_Q16 :
        base_reference_error_q16;
    localparam signed [31:0] SINE_CENTER_LIMIT_Q16 = 32'sd39321600; // 600 counts
    localparam signed [31:0] HELD_REFERENCE_LIMIT_Q16 = 32'sd58916864; // 899 counts
    wire signed [31:0] center_min_q16 = safety_origin_counts_q16 - SINE_CENTER_LIMIT_Q16;
    wire signed [31:0] center_max_q16 = safety_origin_counts_q16 + SINE_CENTER_LIMIT_Q16;
    wire signed [31:0] center_upper_guard_q16 = center_max_q16 - BASE_SLEW_STEP_Q16;
    wire signed [31:0] center_lower_guard_q16 = center_min_q16 + BASE_SLEW_STEP_Q16;
    wire signed [31:0] held_min_q16 = safety_origin_counts_q16 - HELD_REFERENCE_LIMIT_Q16;
    wire signed [31:0] held_max_q16 = safety_origin_counts_q16 + HELD_REFERENCE_LIMIT_Q16;
    wire signed [31:0] held_upper_guard_q16 = held_max_q16 - BASE_SLEW_STEP_Q16;
    wire signed [31:0] held_lower_guard_q16 = held_min_q16 + BASE_SLEW_STEP_Q16;
    wire signed [31:0] applied_min_q16 = safety_origin_counts_q16 - HELD_REFERENCE_LIMIT_Q16;
    wire signed [31:0] applied_max_q16 = safety_origin_counts_q16 + HELD_REFERENCE_LIMIT_Q16;
    wire signed [31:0] applied_upper_guard_q16 = applied_max_q16 - BASE_SLEW_STEP_Q16;
    wire signed [31:0] applied_lower_guard_q16 = applied_min_q16 + BASE_SLEW_STEP_Q16;
    wire signed [31:0] center_reanchor_raw_q16 = actual_counts_q16 - sine_offset_q16;
    wire signed [31:0] center_reanchor_q16 =
        (center_reanchor_raw_q16 > center_max_q16) ? center_max_q16 :
        (center_reanchor_raw_q16 < center_min_q16) ? center_min_q16 :
        center_reanchor_raw_q16;
    wire signed [31:0] sine_reference_min_q16 = safety_origin_counts_q16 - 32'sd56360960;
    wire signed [31:0] sine_reference_max_q16 = safety_origin_counts_q16 + 32'sd56360960;
    wire reanchor_in_range =
        (center_reanchor_raw_q16 >= center_min_q16) &&
        (center_reanchor_raw_q16 <= center_max_q16) &&
        (actual_counts_q16 >= sine_reference_min_q16) &&
        (actual_counts_q16 <= sine_reference_max_q16);
    wire signed [31:0] held_actual_q16 =
        (actual_counts_q16 > held_max_q16) ? held_max_q16 :
        (actual_counts_q16 < held_min_q16) ? held_min_q16 : actual_counts_q16;
    wire signed [31:0] stop_reference_q16 = running ? running_reference_q16 : reference_counts_q16;
    wire signed [31:0] stop_reference_clamped_q16 =
        (stop_reference_q16 > held_max_q16) ? held_max_q16 :
        (stop_reference_q16 < held_min_q16) ? held_min_q16 : stop_reference_q16;

    // Split the target-to-step calculation and the bounded reference update
    // across clocks. Samples are 3 ms apart, so a one-clock pipeline is invisible
    // to the motion while preventing a long combinational path into the balance loop.
    reg slew_pending;
    reg slew_pending_running;
    reg signed [31:0] slew_step_pending_q16;
    wire center_step_blocked =
        ((slew_step_pending_q16 > 32'sd0) && (center_counts_q16 >= center_upper_guard_q16)) ||
        ((slew_step_pending_q16 < 32'sd0) && (center_counts_q16 <= center_lower_guard_q16));
    wire held_step_blocked =
        ((slew_step_pending_q16 > 32'sd0) && (held_reference_q16 >= held_upper_guard_q16)) ||
        ((slew_step_pending_q16 < 32'sd0) && (held_reference_q16 <= held_lower_guard_q16));
    wire applied_step_blocked =
        ((slew_step_pending_q16 > 32'sd0) && (base_reference_applied_q16 >= applied_upper_guard_q16)) ||
        ((slew_step_pending_q16 < 32'sd0) && (base_reference_applied_q16 <= applied_lower_guard_q16));
    wire signed [31:0] slew_step_applied_q16 =
        (slew_pending_running ? center_step_blocked :
         hold_valid ? held_step_blocked : applied_step_blocked) ?
        32'sd0 : slew_step_pending_q16;

    wire signed [31:0] running_reference_q16 = center_counts_q16 + sine_offset_q16;
    wire signed [31:0] reference_counts_q16_comb = running ? running_reference_q16 :
        hold_valid ? held_reference_q16 :
        balance_active ? base_reference_applied_q16 : base_reference_counts_q16;
    reg signed [31:0] reference_counts_q16_reg;
    assign reference_counts_q16 = reference_counts_q16_reg;

    // Compute the programmable phase increment only when the period changes.
    // Restoring division takes 24 clocks and does not put a divider on the
    // 50 MHz control path.
    reg divider_busy;
    reg [4:0] divider_bit;
    reg [23:0] divider_numerator;
    reg [23:0] divider_quotient;
    reg [7:0] divider_remainder;
    reg [6:0] divider_divisor;
    wire [8:0] divider_trial = {divider_remainder, divider_numerator[divider_bit]};
    wire divider_take = divider_trial >= {2'b00, divider_divisor};
    wire [8:0] divider_difference = divider_trial - {2'b00, divider_divisor};
    wire [7:0] divider_next_remainder = divider_take ?
        divider_difference[7:0] : divider_trial[7:0];
    wire [23:0] divider_next_quotient = divider_quotient |
        (divider_take ? (24'd1 << divider_bit) : 24'd0);
    wire divider_round_up = ({divider_next_remainder, 1'b0} >= {2'b00, divider_divisor});

    // CORDIC rotation constants in binary-angle measurement (one turn = 2^32).
    function [31:0] cordic_atan;
        input [4:0] index;
        begin
            case (index)
                5'd0: cordic_atan = 32'd536870912;
                5'd1: cordic_atan = 32'd316933406;
                5'd2: cordic_atan = 32'd167458907;
                5'd3: cordic_atan = 32'd85004756;
                5'd4: cordic_atan = 32'd42667331;
                5'd5: cordic_atan = 32'd21354465;
                5'd6: cordic_atan = 32'd10679838;
                5'd7: cordic_atan = 32'd5340245;
                5'd8: cordic_atan = 32'd2670163;
                5'd9: cordic_atan = 32'd1335087;
                5'd10: cordic_atan = 32'd667544;
                5'd11: cordic_atan = 32'd333772;
                5'd12: cordic_atan = 32'd166886;
                5'd13: cordic_atan = 32'd83443;
                5'd14: cordic_atan = 32'd41722;
                5'd15: cordic_atan = 32'd20861;
                5'd16: cordic_atan = 32'd10430;
                5'd17: cordic_atan = 32'd5215;
                5'd18: cordic_atan = 32'd2608;
                5'd19: cordic_atan = 32'd1304;
                default: cordic_atan = 32'd0;
            endcase
        end
    endfunction

    reg [4:0] cordic_iteration;
    reg signed [31:0] cordic_x, cordic_y, cordic_z;
    reg cordic_negative;
    reg cordic_busy;
    wire [31:0] phase_next = phase + phase_step;
    wire signed [31:0] cordic_start_z = phase_next[30] ?
        $signed(32'h40000000 - {2'b00, phase_next[29:0]}) :
        $signed({2'b00, phase_next[29:0]});
    wire signed [31:0] cordic_x_shift = cordic_x >>> cordic_iteration;
    wire signed [31:0] cordic_y_shift = cordic_y >>> cordic_iteration;
    wire signed [31:0] cordic_x_next = cordic_z[31] ?
        cordic_x + cordic_y_shift : cordic_x - cordic_y_shift;
    wire signed [31:0] cordic_y_next = cordic_z[31] ?
        cordic_y - cordic_x_shift : cordic_y + cordic_x_shift;
    wire signed [31:0] cordic_z_next = cordic_z[31] ?
        cordic_z + $signed(cordic_atan(cordic_iteration)) :
        cordic_z - $signed(cordic_atan(cordic_iteration));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= 32'd0;
            phase_step <= 32'd429497; // T = 30 s
            center_counts_q16 <= 32'sd0;
            held_reference_q16 <= 32'sd0;
            sine_offset_q16 <= 32'sd0;
            base_reference_applied_q16 <= 32'sd0;
            reference_counts_q16_reg <= 32'sd0;
            hold_valid <= 1'b0;
            start_pending <= 1'b0;
            slew_pending <= 1'b0;
            slew_pending_running <= 1'b0;
            slew_step_pending_q16 <= 32'sd0;
            running <= 1'b0;
            period_seconds_active <= 8'd30;
            divider_busy <= 1'b0;
            divider_bit <= 5'd0;
            divider_numerator <= PHASE_NUMERATOR;
            divider_quotient <= 24'd0;
            divider_remainder <= 8'd0;
            divider_divisor <= 7'd30;
            cordic_iteration <= 5'd0;
            cordic_x <= 32'sd0;
            cordic_y <= 32'sd0;
            cordic_z <= 32'sd0;
            cordic_negative <= 1'b0;
            cordic_busy <= 1'b0;
        end else begin
            reference_counts_q16_reg <= reference_counts_q16_comb;
            if (period_valid && (period_seconds >= 8'd10) && (period_seconds <= 8'd120)) begin
                period_seconds_active <= period_seconds;
                divider_busy <= 1'b1;
                divider_bit <= 5'd23;
                divider_numerator <= PHASE_NUMERATOR;
                divider_quotient <= 24'd0;
                divider_remainder <= 8'd0;
                divider_divisor <= period_seconds[6:0];
            end else if (divider_busy) begin
                divider_quotient <= divider_next_quotient;
                divider_remainder <= divider_next_remainder;
                if (divider_bit == 5'd0) begin
                    phase_step <= divider_next_quotient + divider_round_up;
                    divider_busy <= 1'b0;
                end else begin
                    divider_bit <= divider_bit - 1'b1;
                end
            end

            if (clear_request) begin
                // A new run may start at any absolute arm angle.  Re-anchor
                // the applied reference to the measured position here, so
                // the first AUTO sample cannot pull toward a stale power-on
                // target.  After BALANCE capture, main.v changes the base
                // reference to the committed target and the normal bounded
                // slew below moves there only after the posture gate opens.
                running <= 1'b0;
                hold_valid <= 1'b0;
                start_pending <= 1'b0;
                phase <= 32'd0;
                sine_offset_q16 <= 32'sd0;
                base_reference_applied_q16 <= actual_counts_q16;
                held_reference_q16 <= actual_counts_q16;
                cordic_busy <= 1'b0;
                slew_pending <= 1'b0;
            end else if (!balance_active && !running && !hold_valid && sample_valid) begin
                start_pending <= 1'b0;
                // During AUTO/MANUAL pre-capture the balance reference is not
                // driving the motor, so keep the saved reference at the actual
                // arm position.  If capture occurs away from the start point,
                // the first BALANCE sample therefore begins from the measured
                // capture position; the normal motion gate can then slew to
                // the committed target without a stale-reference jump.
                base_reference_applied_q16 <= actual_counts_q16;
                held_reference_q16 <= actual_counts_q16;
            end else if (stop_request || (running && !balance_active)) begin
                start_pending <= 1'b0;
                held_reference_q16 <= stop_reference_clamped_q16;
                hold_valid <= 1'b1;
                running <= 1'b0;
                base_reference_applied_q16 <= base_reference_counts_q16;
                cordic_busy <= 1'b0;
                slew_pending <= 1'b0;
            end else if (start_request && !start_allowed && balance_active && !running) begin
                // Do not lose a valid user request just because the rod is
                // still settling at the moment the button is pressed.
                start_pending <= 1'b1;
            end else if ((start_request || start_pending) && start_allowed) begin
                start_pending <= 1'b0;
                center_counts_q16 <= actual_counts_q16;
                sine_offset_q16 <= 32'sd0;
                base_reference_applied_q16 <= base_reference_counts_q16;
                held_reference_q16 <= actual_counts_q16;
                hold_valid <= 1'b0;
                phase <= 32'd0;
                running <= 1'b1;
                cordic_busy <= 1'b0;
                slew_pending <= 1'b0;
            end else if (running && !motion_allowed && sample_valid) begin
                // Re-anchor the center while safety pauses, keeping the output
                // at the measured position while the nominal sine envelope fits.
                cordic_busy <= 1'b0;
                slew_pending <= 1'b0;
                if (reanchor_in_range) begin
                    center_counts_q16 <= center_reanchor_q16;
                end else begin
                    // If the measured position can no longer fit the +/-860
                    // count envelope, stop the wave and hold an inward-safe ref.
                    running <= 1'b0;
                    hold_valid <= 1'b1;
                    held_reference_q16 <= held_actual_q16;
                    phase <= 32'd0;
                    sine_offset_q16 <= 32'sd0;
                end
            end else begin
                if (slew_pending) begin
                    if (motion_allowed && balance_active && slew_pending_running) begin
                        center_counts_q16 <= center_counts_q16 + slew_step_applied_q16;
                        base_reference_applied_q16 <=
                            base_reference_applied_q16 + slew_step_applied_q16;
                    end else if (motion_allowed && balance_active && hold_valid) begin
                        held_reference_q16 <= held_reference_q16 + slew_step_applied_q16;
                        base_reference_applied_q16 <=
                            base_reference_applied_q16 + slew_step_applied_q16;
                    end else if (motion_allowed && balance_active) begin
                        base_reference_applied_q16 <=
                            base_reference_applied_q16 + slew_step_applied_q16;
                    end
                    slew_pending <= 1'b0;
                end
                if (sample_valid) begin
                    if (running && motion_allowed) begin
                        // Keep this run's center fixed. The base reference can
                        // switch between the SW1 anchor and committed target as
                        // the posture gate opens/closes; following that input
                        // would turn a target-reference handoff into continuous
                        // arm travel. Target changes are applied after Q.
                        slew_pending_running <= 1'b0;
                        slew_pending <= 1'b0;
                    end else if (motion_allowed && balance_active && hold_valid) begin
                        // After Q, preserve the frozen trajectory point. Later
                        // committed target changes translate it at the same rate.
                        slew_step_pending_q16 <= base_reference_step_q16;
                        slew_pending_running <= 1'b0;
                        slew_pending <= 1'b1;
                    end else if (!running && !hold_valid && balance_active && motion_allowed) begin
                        // Ordinary balance-mode target changes use the same
                        // bounded reference slew as running and held sine modes.
                        slew_step_pending_q16 <= base_reference_step_q16;
                        slew_pending_running <= 1'b0;
                        slew_pending <= 1'b1;
                    end
                end
                if (cordic_busy) begin
                    cordic_x <= cordic_x_next;
                    cordic_y <= cordic_y_next;
                    cordic_z <= cordic_z_next;
                    if (cordic_iteration == 5'd19) begin
                        sine_offset_q16 <= cordic_negative ? -cordic_y_next : cordic_y_next;
                        cordic_busy <= 1'b0;
                    end else begin
                        cordic_iteration <= cordic_iteration + 1'b1;
                    end
                end else if (running && motion_allowed && sample_valid) begin
                    phase <= phase_next;
                    cordic_iteration <= 5'd0;
                    cordic_x <= CORDIC_X0_Q16;
                    cordic_y <= 32'sd0;
                    cordic_z <= cordic_start_z;
                    cordic_negative <= phase_next[31];
                    cordic_busy <= 1'b1;
                end
            end
        end
    end
endmodule
