`timescale 1ns / 1ps

// Parallel 10-bit ADC interface. The ADC clock is 25 MHz. The subsamples
// inside each control sample are smoothed by a one-pole (first-order) IIR
// filter instead of a boxcar average.
module adc(
    input  wire        clk_50m,
    input  wire        rst_n,
    output wire        adc_clk,
    output wire        adc_oe_n,
    output wire        adc_stby_n,
    output wire        adc_otr,
    input  wire [9:0]  adc_data_in,
    output wire [11:0] theta_out,
    output reg         sample_valid
);
    // One-pole filter: y <= y + ((x - y) >>> ADC_FILT_SHIFT), updated on every
    // subsample.  Time constant ~= 2^SHIFT x subsample period (SHIFT=3, subsample
    // 2 ms -> ~16 ms).  Raise SHIFT for more smoothing, lower for faster response.
    parameter integer ADC_FILT_SHIFT = 4;
    reg        clk_div;
    reg [11:0] theta_out_reg;
    reg signed [19:0] theta_filt;   // Q12.8: 12 integer + 8 fractional bits
    reg [17:0] cnt_3ms;
    reg [13:0] cnt_187us;
    // ------------------------------------------------------------------
    // Bus-level two-out-of-three vote (task-16, sensor integrity).
    // The parallel bus is sampled freely every 9375 clk_50m cycles (4687.5 ADC
    // clocks -- not an integer), so a read can land on the ADC's update edge and
    // return a garbage value. Bench run 15 measured 196 strictly isolated spikes
    // in 148 s (1.31/s) that were independent of motor power, plus one
    // 888 -> 390 -> 888 round trip (498 counts = 42.4 deg inside 3 ms, which
    // physically cannot happen) that false-triggered the 30 deg fault.
    // Three consecutive 50 MHz samples span 40 ns = exactly one 25 MHz ADC
    // period, so at most ONE of them can sit on a data transition: the
    // per-bit majority is therefore always a valid bus level, and this needs no
    // knowledge of the ADC edge polarity and no phase alignment.
    reg [9:0] adc_s0, adc_s1, adc_s2;
    wire [9:0] adc_maj = (adc_s0 & adc_s1) | (adc_s1 & adc_s2) | (adc_s0 & adc_s2);
    wire [11:0] theta_12bit = {adc_maj, 2'b00};

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            adc_s0 <= 10'd0;
            adc_s1 <= 10'd0;
            adc_s2 <= 10'd0;
        end else begin
            adc_s0 <= adc_data_in;
            adc_s1 <= adc_s0;
            adc_s2 <= adc_s1;
        end
    end

    assign adc_clk = clk_div;
    assign adc_oe_n = 1'b0;
    assign adc_stby_n = 1'b0;
    assign adc_otr = 1'b0;
    assign theta_out = theta_out_reg;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            clk_div <= 1'b0;
            cnt_3ms <= 18'd0;
            cnt_187us <= 14'd0;
            theta_filt <= 20'sd0;
            theta_out_reg <= 12'd0;
            sample_valid <= 1'b0;
        end else begin
            clk_div <= ~clk_div;
            sample_valid <= 1'b0;

            if (cnt_3ms == 18'd1000000)//改为50hz
                cnt_3ms <= 18'd0;
            else
                cnt_3ms <= cnt_3ms + 1'b1;

            if (cnt_187us == 14'd100000)//10次子采样
                cnt_187us <= 14'd0;
            else
                cnt_187us <= cnt_187us + 1'b1;

            if (cnt_187us == 14'd100000) begin
                // First-order IIR on the 12-bit sample (kept in Q12.8 so the
                // shift does not lose the small deltas between subsamples).
                theta_filt <= theta_filt +
                    ((($signed({8'd0, theta_12bit}) <<< 8) - theta_filt)
                        >>> ADC_FILT_SHIFT);
            end else if (cnt_3ms == 18'd1000000) begin
                theta_out_reg <= theta_filt[19:8];
                sample_valid <= 1'b1;
            end
        end
    end
endmodule
