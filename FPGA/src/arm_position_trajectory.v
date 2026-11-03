`timescale 1ns/1ps

// Acceleration-limited position reference for the horizontal arm.
// Position is encoder-count Q16; the angle output is radian Q16.
// The requested velocity is proportional to remaining distance and capped,
// then its change is limited once per valid 3 ms state sample.
module arm_position_trajectory #(
    parameter signed [31:0] MAX_SPEED_COUNTS_Q16 = 32'sd13107, // 0.20 count/sample ~= 23 deg/s
    parameter signed [31:0] ACCEL_COUNTS_Q16 = 32'sd64,       // max velocity change per sample
    parameter integer ERROR_TO_SPEED_SHIFT = 8,
    parameter signed [31:0] ARM_RAD_PER_COUNT_Q30 = 32'sd6116518
)(
    input  wire clk,
    input  wire rst_n,
    input  wire sample_valid,
    input  wire enable,
    input  wire reanchor,
    input  wire signed [31:0] actual_counts_q16,
    input  wire signed [31:0] actual_angle_q16,
    input  wire signed [31:0] target_counts_q16,
    input  wire signed [31:0] target_angle_q16,
    output reg  signed [31:0] reference_counts_q16,
    output reg  signed [31:0] reference_angle_q16,
    output reg  signed [31:0] velocity_counts_q16
);
    // Low 30 bits of the Q46 radian numerator, retained between samples so
    // sub-LSB angle increments are not lost while the count reference slews.
    reg [29:0] angle_fraction_q46_low;

    // Q16.16 in 32 bits spans +/-32768 encoder counts, comfortably beyond
    // the machine's +/-1000-count cable limit while keeping this fast control
    // path out of unnecessary 48-bit add/subtract hardware.
    wire signed [31:0] position_error_q16 = target_counts_q16 - reference_counts_q16;
    wire [31:0] position_error_abs_q16 = position_error_q16[31] ?
        -position_error_q16 : position_error_q16;
    wire signed [31:0] requested_velocity_wide =
        position_error_q16 >>> ERROR_TO_SPEED_SHIFT;
    wire signed [31:0] max_velocity_wide = MAX_SPEED_COUNTS_Q16;
    wire signed [31:0] requested_velocity_limited =
        (requested_velocity_wide > max_velocity_wide) ? MAX_SPEED_COUNTS_Q16 :
        (requested_velocity_wide < -max_velocity_wide) ? -MAX_SPEED_COUNTS_Q16 :
        requested_velocity_wide[31:0];
    wire signed [31:0] requested_velocity_nonzero =
        (position_error_q16 != 32'sd0 && requested_velocity_limited == 32'sd0) ?
            (position_error_q16[31] ? -32'sd1 : 32'sd1) : requested_velocity_limited;
    reg signed [31:0] requested_velocity_q;
    reg [31:0] position_error_abs_q;
    reg position_error_negative_q;
    reg position_error_zero_q;
    reg signed [31:0] sampled_target_counts_q16;
    reg signed [31:0] sampled_target_angle_q16;

    // The trajectory is only advanced once per 3 ms sample, so use otherwise
    // idle 50 MHz cycles to pipeline the error-to-velocity and acceleration
    // calculations. This keeps the new position feature from adding a long
    // combinational path to the existing balance controller.
    reg request_pending;
    reg update_pending;
    reg signed [31:0] pending_next_velocity;
    reg pending_reaches_goal;
    reg pending_error_zero;
    reg signed [31:0] pending_target_counts_q16;
    reg signed [31:0] pending_target_angle_q16;
    wire signed [32:0] velocity_difference =
        $signed(requested_velocity_q) - $signed(velocity_counts_q16);
    wire signed [32:0] acceleration_wide = {1'b0, ACCEL_COUNTS_Q16};
    wire signed [31:0] next_velocity =
        (velocity_difference > acceleration_wide) ?
            velocity_counts_q16 + ACCEL_COUNTS_Q16 :
        (velocity_difference < -acceleration_wide) ?
            velocity_counts_q16 - ACCEL_COUNTS_Q16 : requested_velocity_q;

    wire signed [63:0] angle_step_product_q46 =
        $signed(pending_next_velocity) * $signed(ARM_RAD_PER_COUNT_Q30);
    wire signed [63:0] angle_accum_q46 = angle_step_product_q46 +
        $signed({34'd0, angle_fraction_q46_low});
    wire signed [63:0] angle_step_q16 = angle_accum_q46 >>> 30;
    wire signed [63:0] angle_fraction_next_q46 =
        angle_accum_q46 - (angle_step_q16 <<< 30);
    wire [31:0] next_velocity_abs = next_velocity[31] ? -next_velocity : next_velocity;
    wire [31:0] next_velocity_abs_q16 = next_velocity_abs;
    wire motion_reaches_goal = (next_velocity != 32'sd0) &&
        (position_error_negative_q == next_velocity[31]) &&
        (next_velocity_abs_q16 >= position_error_abs_q);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reference_counts_q16 <= 32'sd0;
            reference_angle_q16 <= 32'sd0;
            velocity_counts_q16 <= 32'sd0;
            angle_fraction_q46_low <= 30'd0;
            requested_velocity_q <= 32'sd0;
            position_error_abs_q <= 32'd0;
            position_error_negative_q <= 1'b0;
            position_error_zero_q <= 1'b1;
            sampled_target_counts_q16 <= 32'sd0;
            sampled_target_angle_q16 <= 32'sd0;
            request_pending <= 1'b0;
            update_pending <= 1'b0;
            pending_next_velocity <= 32'sd0;
            pending_reaches_goal <= 1'b0;
            pending_error_zero <= 1'b1;
            pending_target_counts_q16 <= 32'sd0;
            pending_target_angle_q16 <= 32'sd0;
        end else if (reanchor) begin
            reference_counts_q16 <= actual_counts_q16;
            reference_angle_q16 <= actual_angle_q16;
            velocity_counts_q16 <= 32'sd0;
            angle_fraction_q46_low <= 30'd0;
            request_pending <= 1'b0;
            update_pending <= 1'b0;
        end else if (!enable) begin
            // While inactive or while the pendulum is outside the safe-motion
            // window, follow the measured arm position. This prevents the
            // position loop from pulling toward a stale pre-pause reference.
            // The final target is retained; motion resumes from the current
            // position after the safety gate re-opens.
            if (sample_valid) begin
                reference_counts_q16 <= actual_counts_q16;
                reference_angle_q16 <= actual_angle_q16;
                angle_fraction_q46_low <= 30'd0;
            end
            velocity_counts_q16 <= 32'sd0;
            request_pending <= 1'b0;
            update_pending <= 1'b0;
        end else begin
            update_pending <= request_pending;
            if (request_pending) begin
                pending_next_velocity <= next_velocity;
                pending_reaches_goal <= motion_reaches_goal;
                pending_error_zero <= position_error_zero_q;
                pending_target_counts_q16 <= sampled_target_counts_q16;
                pending_target_angle_q16 <= sampled_target_angle_q16;
            end
            if (update_pending) begin
                if (pending_error_zero) begin
                    velocity_counts_q16 <= 32'sd0;
                end else if (pending_reaches_goal) begin
                    // Finish exactly at the sampled setpoint; the last step
                    // cannot exceed the bounded reference velocity.
                    reference_counts_q16 <= pending_target_counts_q16;
                    reference_angle_q16 <= pending_target_angle_q16;
                    velocity_counts_q16 <= 32'sd0;
                    angle_fraction_q46_low <= 30'd0;
                end else begin
                    reference_counts_q16 <= reference_counts_q16 + $signed(pending_next_velocity);
                    reference_angle_q16 <= reference_angle_q16 + angle_step_q16[31:0];
                    velocity_counts_q16 <= pending_next_velocity;
                    angle_fraction_q46_low <= angle_fraction_next_q46[29:0];
                end
            end
            request_pending <= sample_valid;
            if (sample_valid) begin
                requested_velocity_q <= requested_velocity_nonzero;
                position_error_abs_q <= position_error_abs_q16;
                position_error_negative_q <= position_error_q16[31];
                position_error_zero_q <= (position_error_q16 == 32'sd0);
                sampled_target_counts_q16 <= target_counts_q16;
                sampled_target_angle_q16 <= target_angle_q16;
            end
        end
    end
endmodule
