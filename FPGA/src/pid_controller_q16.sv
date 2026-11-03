`timescale 1ns/1ps
// PID on pendulum angle, sampled every 3 ms. Gains use Q16.
// Preserves the reference arithmetic order; only /1000 is multi-cycle.
module pid_controller_q16 #(
    parameter signed [31:0] KP_Q16 = 0,
    parameter signed [31:0] KI_Q16 = 0,
    parameter signed [31:0] KD_Q16 = 0,
    parameter signed [31:0] UMAX_Q16 = 32'sd1966080,
    parameter signed [31:0] I_LIMIT_Q16 = 32'sd327680,
    parameter integer SAMPLE_PERIOD_MS = 20
)(
    input wire clk, input wire rst_n, input wire in_valid,
    input wire signed [31:0] pend_angle_q16,
    input wire signed [31:0] pend_rate_q16,
    input wire signed [31:0] kp_runtime_q16,
    input wire signed [31:0] ki_runtime_q16,
    input wire signed [31:0] kd_runtime_q16,
    output reg out_valid,
    output reg signed [31:0] control_u_q16,
    output reg signed [31:0] integral_q16
);
    localparam [3:0] ST_IDLE=4'd0, ST_PERIOD=4'd1, ST_DIV_INIT=4'd2,
        ST_DIV_RUN=4'd3, ST_CAND=4'd4, ST_MUL_CAND=4'd5,
        ST_SELECT=4'd6, ST_MUL_SELECTED=4'd7, ST_OUTPUT=4'd8;
    reg [3:0] state;

    reg signed [63:0] p_product, d_product;
    reg signed [63:0] period_product;
    reg dividend_negative;
    reg [63:0] dividend_shift, quotient;
    reg [10:0] remainder;
    reg [5:0] divide_count;
    reg signed [31:0] i_candidate, i_selected;
    reg signed [63:0] i_candidate_product, i_change_product;
    reg signed [63:0] i_selected_product;
    // Gains are captured together at transaction start. A screen command can
    // arrive while the multi-cycle integral calculation is in progress without
    // mixing old and new coefficients in one PID result.
    reg signed [31:0] kp_latched_q16, ki_latched_q16, kd_latched_q16;
    reg signed [31:0] pend_angle_latched_q16, pend_rate_latched_q16;

    wire [11:0] remainder_trial = {remainder, dividend_shift[63]};
    wire trial_ge_divisor = remainder_trial >= 12'd1000;
    wire [10:0] remainder_next = trial_ge_divisor ?
        remainder_trial - 12'd1000 : remainder_trial[10:0];
    wire [63:0] quotient_next = {quotient[62:0], trial_ge_divisor};
    wire signed [63:0] signed_quotient = dividend_negative ?
        -$signed(quotient) : $signed(quotient);

    wire signed [63:0] i_candidate_wide =
        $signed(integral_q16) + $signed(signed_quotient);
    wire signed [63:0] i_limit_wide = $signed(I_LIMIT_Q16);
    wire signed [31:0] i_candidate_sat =
        i_candidate_wide > i_limit_wide ? I_LIMIT_Q16 :
        i_candidate_wide < -i_limit_wide ? -I_LIMIT_Q16 :
        i_candidate_wide[31:0];

    wire signed [63:0] p_term = $signed(p_product) >>> 16;
    wire signed [63:0] d_term = $signed(d_product) >>> 16;
    wire signed [63:0] i_candidate_term = $signed(i_candidate_product) >>> 16;
    wire signed [63:0] i_control_delta = -($signed(i_change_product) >>> 16);
    wire signed [63:0] u_candidate =
        -($signed(p_term) + $signed(i_candidate_term) + $signed(d_term));
    wire reject_integral =
        (u_candidate > $signed(UMAX_Q16) && i_control_delta > 0) ||
        (u_candidate < -$signed(UMAX_Q16) && i_control_delta < 0);
    wire signed [63:0] i_selected_term = $signed(i_selected_product) >>> 16;
    wire signed [63:0] u_unclamped =
        -($signed(p_term) + $signed(i_selected_term) + $signed(d_term));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; out_valid <= 1'b0;
            control_u_q16 <= 0; integral_q16 <= 0;
            p_product <= 0; d_product <= 0; period_product <= 0;
            dividend_negative <= 0; dividend_shift <= 0; quotient <= 0;
            remainder <= 0; divide_count <= 0;
            i_candidate <= 0; i_selected <= 0;
            i_candidate_product <= 0; i_change_product <= 0;
            i_selected_product <= 0;
            kp_latched_q16 <= KP_Q16;
            ki_latched_q16 <= KI_Q16;
            kd_latched_q16 <= KD_Q16;
            pend_angle_latched_q16 <= 32'sd0;
            pend_rate_latched_q16 <= 32'sd0;
        end else begin
            out_valid <= 1'b0;
            case (state)
                ST_IDLE: if (in_valid) begin
                    kp_latched_q16 <= kp_runtime_q16;
                    ki_latched_q16 <= ki_runtime_q16;
                    kd_latched_q16 <= kd_runtime_q16;
                    pend_angle_latched_q16 <= pend_angle_q16;
                    pend_rate_latched_q16 <= pend_rate_q16;
                    state <= ST_PERIOD;
                end
                ST_PERIOD: begin
                    p_product <= $signed(kp_latched_q16) * $signed(pend_angle_latched_q16);
                    d_product <= $signed(kd_latched_q16) * $signed(pend_rate_latched_q16);
                    period_product <= $signed(pend_angle_latched_q16) * SAMPLE_PERIOD_MS;
                    state <= ST_DIV_INIT;
                end
                ST_DIV_INIT: begin
                    dividend_negative <= period_product[63];
                    dividend_shift <= period_product[63] ? -period_product : period_product;
                    quotient <= 0; remainder <= 0; divide_count <= 0;
                    state <= ST_DIV_RUN;
                end
                ST_DIV_RUN: begin
                    dividend_shift <= {dividend_shift[62:0],1'b0};
                    quotient <= quotient_next;
                    remainder <= remainder_next;
                    if (divide_count == 6'd63) state <= ST_CAND;
                    else divide_count <= divide_count + 1'b1;
                end
                ST_CAND: begin
                    i_candidate <= i_candidate_sat;
                    state <= ST_MUL_CAND;
                end
                ST_MUL_CAND: begin
                    i_candidate_product <= $signed(ki_latched_q16) * $signed(i_candidate);
                    i_change_product <= $signed(ki_latched_q16) *
                        ($signed(i_candidate) - $signed(integral_q16));
                    state <= ST_SELECT;
                end
                ST_SELECT: begin
                    i_selected <= reject_integral ? integral_q16 : i_candidate;
                    state <= ST_MUL_SELECTED;
                end
                ST_MUL_SELECTED: begin
                    i_selected_product <= $signed(ki_latched_q16) * $signed(i_selected);
                    state <= ST_OUTPUT;
                end
                ST_OUTPUT: begin
                    integral_q16 <= i_selected;
                    if (u_unclamped > $signed(UMAX_Q16)) control_u_q16 <= UMAX_Q16;
                    else if (u_unclamped < -$signed(UMAX_Q16)) control_u_q16 <= -UMAX_Q16;
                    else control_u_q16 <= u_unclamped[31:0];
                    out_valid <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
