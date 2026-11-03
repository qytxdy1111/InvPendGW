`timescale 1ns/1ps
// 3 ms backward differences; first sample establishes history. Q30 slopes, Q16 states.
module state_q16 #(
    parameter signed [31:0] ARM_ZERO_COUNTS = 0,
    parameter signed [31:0] ARM_RAD_PER_COUNT_Q30 = 6116518,
    parameter signed [31:0] PEND_ZERO_Q16 = 134217728,
    parameter signed [31:0] PEND_RAD_PER_COUNT_Q30 = 1589138,
    parameter signed [31:0] ARM_RATE_PER_COUNT_Q16 = 131979,
    parameter signed [31:0] PEND_RATE_PER_COUNT_Q16 = 32331
)(
    input wire clk, rst_n, sample_valid,
    input wire [31:0] pos,
    input wire signed [31:0] arm_reference_counts_q16,
    input wire [11:0] theta,
    output reg signed [31:0] arm_angle, arm_rate, pend_angle, pend_rate,
    output reg out_valid
);
    reg [31:0] previous_pos;
    reg [11:0] previous_theta;
    reg have_previous;
    // The arm reference is Q16.16 counts so smooth generated references retain
    // sub-encoder-count resolution. Velocity still uses raw encoder differences.
    wire signed [31:0] arm_reference_integer = arm_reference_counts_q16 >>> 16;
    wire signed [31:0] relative_pos_counts = $signed(pos) - arm_reference_integer;
    wire signed [63:0] relative_pos_q16 =
        ($signed({{32{relative_pos_counts[31]}}, relative_pos_counts}) <<< 16) -
        $signed({48'd0, arm_reference_counts_q16[15:0]});
    wire signed [63:0] arm_offset = relative_pos_q16 - (ARM_ZERO_COUNTS <<< 16);
    wire signed [63:0] pend_offset = ($signed({1'b0,theta}) * 64'sd65536) - PEND_ZERO_Q16;
    wire signed [31:0] delta_pos = $signed(pos - previous_pos);
    wire signed [31:0] delta_theta = $signed({1'b0,theta}) - $signed({1'b0,previous_theta});
    // Pipeline sensor conversion over otherwise-idle 50 MHz cycles. A sample
    // still produces one coherent set of Q16 outputs and one out_valid pulse;
    // only the sensor-to-control latency grows by a few clock cycles (far
    // below the 3 ms sample interval). This breaks the encoder-difference,
    // rate-multiply and saturation path into timing-safe stages.
    reg signed [63:0] arm_offset_q, pend_offset_q;
    reg signed [31:0] delta_pos_q, delta_theta_q;
    reg sample_stage1_valid, sample_stage1_out_valid;
    reg signed [63:0] arm_product_q, pend_product_q;
    reg signed [63:0] arm_speed_q, pend_speed_q;
    reg sample_stage2_valid, sample_stage2_out_valid;
    wire signed [63:0] arm_product = arm_offset_q * ARM_RAD_PER_COUNT_Q30;
    wire signed [63:0] pend_product = pend_offset_q * PEND_RAD_PER_COUNT_Q30;
    wire signed [63:0] arm_speed = delta_pos_q * ARM_RATE_PER_COUNT_Q16;
    wire signed [63:0] pend_speed = delta_theta_q * PEND_RATE_PER_COUNT_Q16;
    function signed [31:0] sat32(input signed [63:0] v);
        begin
            if(v > 64'sd2147483647) sat32 = 32'sh7fffffff;
            else if(v < -64'sd2147483648) sat32 = 32'sh80000000;
            else sat32 = v[31:0];
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            previous_pos <= 0; previous_theta <= 0; have_previous <= 0;
            arm_offset_q <= 0; pend_offset_q <= 0;
            delta_pos_q <= 0; delta_theta_q <= 0;
            sample_stage1_valid <= 0; sample_stage1_out_valid <= 0;
            arm_product_q <= 0; pend_product_q <= 0;
            arm_speed_q <= 0; pend_speed_q <= 0;
            sample_stage2_valid <= 0; sample_stage2_out_valid <= 0;
            arm_angle <= 0; pend_angle <= 0; arm_rate <= 0; pend_rate <= 0; out_valid <= 0;
        end else begin
            out_valid <= 0;
            sample_stage2_valid <= sample_stage1_valid;
            if (sample_stage1_valid) begin
                arm_product_q <= arm_product;
                pend_product_q <= pend_product;
                arm_speed_q <= arm_speed;
                pend_speed_q <= pend_speed;
            end
            sample_stage2_out_valid <= sample_stage1_out_valid;
            if (sample_stage2_valid) begin
                arm_angle <= sat32(arm_product_q >>> 30);
                pend_angle <= sat32(pend_product_q >>> 30);
                arm_rate <= sample_stage2_out_valid ? sat32(arm_speed_q) : 32'sd0;
                pend_rate <= sample_stage2_out_valid ? sat32(pend_speed_q) : 32'sd0;
                out_valid <= sample_stage2_out_valid;
            end
            sample_stage1_valid <= sample_valid;
            if(sample_valid) begin
                arm_offset_q <= arm_offset;
                pend_offset_q <= pend_offset;
                delta_pos_q <= delta_pos;
                delta_theta_q <= delta_theta;
                sample_stage1_out_valid <= have_previous;
                previous_pos <= pos; previous_theta <= theta;
                have_previous <= 1;
            end
        end
    end
endmodule
