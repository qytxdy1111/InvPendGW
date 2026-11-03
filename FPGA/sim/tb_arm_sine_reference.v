`timescale 1ns/1ps

module tb_arm_sine_reference;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg sample_valid = 1'b0;
    reg balance_active = 1'b1;
    reg motion_allowed = 1'b1;
    reg start_request = 1'b0;
    reg start_allowed = 1'b0;
    reg stop_request = 1'b0;
    reg clear_request = 1'b0;
    reg period_valid = 1'b0;
    reg [7:0] period_seconds = 8'd30;
    reg signed [31:0] safety_origin_counts_q16 = 32'sd0;
    reg signed [31:0] base_reference_counts_q16 = 32'sd0;
    reg signed [31:0] actual_counts_q16 = 32'sd6553600;
    wire signed [31:0] reference_counts_q16;
    wire running;
    wire [7:0] period_seconds_active;

    localparam signed [31:0] CENTER_Q16 = 32'sd6553600;
    localparam signed [31:0] PEAK_Q16 = 32'sd17039360;
    localparam signed [31:0] BASE_STEP_Q16 = 32'sd13107;

    always #10 clk = ~clk;

    arm_sine_reference dut (
        .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
        .balance_active(balance_active), .motion_allowed(motion_allowed),
        .start_request(start_request), .start_allowed(start_allowed),
        .stop_request(stop_request), .clear_request(clear_request),
        .period_valid(period_valid), .period_seconds(period_seconds),
        .safety_origin_counts_q16(safety_origin_counts_q16),
        .base_reference_counts_q16(base_reference_counts_q16),
        .actual_counts_q16(actual_counts_q16),
        .reference_counts_q16(reference_counts_q16), .running(running),
        .period_seconds_active(period_seconds_active)
    );

    task sample_tick;
        begin
            @(negedge clk);
            sample_valid = 1'b1;
            @(posedge clk);
            #1 sample_valid = 1'b0;
            repeat (21) @(posedge clk);
            #1;
        end
    endtask

    task set_period;
        input [7:0] seconds;
        begin
            @(negedge clk);
            period_seconds = seconds;
            period_valid = 1'b1;
            @(negedge clk);
            period_valid = 1'b0;
            repeat (26) @(posedge clk);
            #1;
        end
    endtask

    task sample_at_phase;
        input [31:0] requested_phase;
        begin
            @(negedge clk);
            dut.phase = requested_phase - dut.phase_step;
            sample_tick();
        end
    endtask

    task check_offset;
        input signed [31:0] expected_offset_q16;
        reg signed [63:0] error_q16;
        begin
            error_q16 = $signed(reference_counts_q16) - CENTER_Q16 - expected_offset_q16;
            if (error_q16 < 0) error_q16 = -error_q16;
            if (error_q16 > 64'sd131072)
                $fatal(1, "offset mismatch: got %0d Q16, expected %0d Q16",
                    $signed(reference_counts_q16) - CENTER_Q16, expected_offset_q16);
        end
    endtask

    reg signed [31:0] prior_center_q16;
    reg signed [31:0] frozen_reference_q16;
    reg [31:0] frozen_phase;
    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        set_period(8'd30);
        if (dut.phase_step != 32'd429497 || period_seconds_active != 8'd30)
            $fatal(1, "30-second phase step mismatch");

        @(negedge clk);
        start_allowed = 1'b1;
        start_request = 1'b1;
        @(posedge clk);
        #1 start_request = 1'b0;
        if (!running) $fatal(1, "sine start was not accepted");

        sample_at_phase(32'h00000000);
        check_offset(32'sd0);
        sample_at_phase(32'h40000000);
        check_offset(PEAK_Q16);
        sample_at_phase(32'h80000000);
        check_offset(32'sd0);
        sample_at_phase(32'hC0000000);
        check_offset(-PEAK_Q16);

        set_period(8'd120);
        if (dut.phase_step != 32'd107374 || period_seconds_active != 8'd120)
            $fatal(1, "120-second phase step mismatch");

        prior_center_q16 = dut.center_counts_q16;
        base_reference_counts_q16 = 32'sd655360;
        sample_tick();
        if (dut.center_counts_q16 != prior_center_q16)
            $fatal(1, "running center drifted when the base reference changed");

        @(negedge clk);
        stop_request = 1'b1;
        @(posedge clk);
        #1 stop_request = 1'b0;
        frozen_reference_q16 = reference_counts_q16;
        if (running || !dut.hold_valid || dut.held_reference_q16 != frozen_reference_q16)
            $fatal(1, "Q did not freeze the current reference");

        base_reference_counts_q16 = 32'sd1310720;
        motion_allowed = 1'b0;
        frozen_reference_q16 = dut.held_reference_q16;
        sample_tick();
        if (dut.held_reference_q16 != frozen_reference_q16)
            $fatal(1, "held reference moved while the motion safety gate was closed");
        motion_allowed = 1'b1;
        sample_tick();
        if ((dut.held_reference_q16 - frozen_reference_q16) != BASE_STEP_Q16)
            $fatal(1, "held reference did not slew by 0.2 count after safety recovery");

        @(negedge clk);
        clear_request = 1'b1;
        @(posedge clk);
        #1 clear_request = 1'b0;
        @(posedge clk);
        #1;
        if (reference_counts_q16 != base_reference_counts_q16)
            $fatal(1, "clear did not return to the base reference");

        actual_counts_q16 = 32'sd39256064;
        @(negedge clk);
        start_request = 1'b1;
        @(posedge clk);
        #1 start_request = 1'b0;
        base_reference_counts_q16 = 32'sd1966080;
        sample_tick();
        if (dut.center_counts_q16 > 32'sd39321600)
            $fatal(1, "sine center exceeded the 600-count safety limit");
        sample_at_phase(32'h40000000);
        if (reference_counts_q16 > 32'sd56360960)
            $fatal(1, "sine reference exceeded the 860-count safety margin");

        actual_counts_q16 = 32'sd32768000;
        motion_allowed = 1'b0;
        frozen_phase = dut.phase;
        sample_tick();
        if (dut.phase != frozen_phase)
            $fatal(1, "sine phase advanced while the motion safety gate was closed");
        if ((dut.center_counts_q16 > 32'sd39321600) ||
            (dut.center_counts_q16 < -32'sd39321600))
            $fatal(1, "safety re-anchor exceeded the 600-count center limit");
        if ((reference_counts_q16 > 32'sd56360960) ||
            (reference_counts_q16 < -32'sd56360960))
            $fatal(1, "safety re-anchor exceeded the 860-count reference limit");

        actual_counts_q16 = 32'sd62259200;
        sample_tick();
        if (running || !dut.hold_valid || (dut.held_reference_q16 != 32'sd58916864))
            $fatal(1, "out-of-envelope pause did not stop and clamp the held reference");

        @(negedge clk);
        start_request = 1'b1;
        actual_counts_q16 = 32'sd62259200;
        motion_allowed = 1'b1;
        @(posedge clk);
        #1 start_request = 1'b0;
        @(negedge clk);
        stop_request = 1'b1;
        @(posedge clk);
        #1 stop_request = 1'b0;
        if (dut.held_reference_q16 != 32'sd58916864)
            $fatal(1, "Q did not clamp its held reference to the 899-count limit");

        @(negedge clk);
        clear_request = 1'b1;
        @(posedge clk);
        #1 clear_request = 1'b0;
        repeat (2) @(posedge clk);
        #1;
        base_reference_counts_q16 = 32'sd6553600;
        prior_center_q16 = dut.base_reference_applied_q16;
        sample_tick();
        if ((dut.base_reference_applied_q16 - prior_center_q16) != BASE_STEP_Q16)
            $fatal(1, "ordinary balance reference did not slew by 0.2 count");
        if ((reference_counts_q16 - prior_center_q16) > BASE_STEP_Q16 ||
            (reference_counts_q16 - prior_center_q16) < -BASE_STEP_Q16)
            $fatal(1, "ordinary balance reference jumped instead of slewing");

        $display("PASS: quadrants, period, gated hold, idle slew, bounded re-anchor, and travel limits");
        $finish;
    end
endmodule
