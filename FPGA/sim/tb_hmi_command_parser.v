`timescale 1ns/1ps

// Parser-only testbench: exercises every ASCII and binary command, the new
// ARM-loop (L) commands, CRLF tolerance, and cross-talk between the rod-loop
// and arm-loop gain outputs.
//
// Timing contract with hmi_command_parser: every output is default-cleared on
// each posedge (parser lines 132-140), so a valid pulse is visible only between
// the posedge that sets it and the next posedge. send_byte deasserts rx_valid
// on the negedge AFTER the byte's sampling posedge and returns there, so the
// pulse set by that sampling posedge is still high: check IMMEDIATELY after
// send_byte returns, with NO extra wait. The single exception is D+<digits>:
// S_DELTA_REDUCE fires delta_valid one posedge LATER (on the first posedge
// with rx_valid=0 after the LF), so that check keeps one @(negedge clk).
module tb_hmi_command_parser;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg rx_valid = 1'b0;
    reg [7:0] rx_data = 8'd0;

    wire mode_valid;
    wire [1:0] mode_value;
    wire delta_valid;
    wire signed [15:0] delta_degrees;
    wire delta_apply_valid;
    wire delta_cancel_valid;
    wire sine_period_valid;
    wire [7:0] sine_period_seconds;
    wire sine_start_valid;
    wire sine_stop_valid;
    wire pid_gain_valid;
    wire [1:0] pid_gain_index;
    wire pid_gain_increment;
    wire arm_gain_valid;
    wire [1:0] arm_gain_index;
    wire arm_gain_increment;

    always #10 clk = ~clk;

    hmi_command_parser dut (
        .clk(clk), .rst_n(rst_n), .rx_valid(rx_valid), .rx_data(rx_data),
        .mode_valid(mode_valid), .mode_value(mode_value),
        .delta_valid(delta_valid), .delta_degrees(delta_degrees),
        .delta_apply_valid(delta_apply_valid),
        .delta_cancel_valid(delta_cancel_valid),
        .sine_period_valid(sine_period_valid),
        .sine_period_seconds(sine_period_seconds),
        .sine_start_valid(sine_start_valid),
        .sine_stop_valid(sine_stop_valid),
        .pid_gain_valid(pid_gain_valid), .pid_gain_index(pid_gain_index),
        .pid_gain_increment(pid_gain_increment),
        .arm_gain_valid(arm_gain_valid), .arm_gain_index(arm_gain_index),
        .arm_gain_increment(arm_gain_increment)
    );

    integer errors = 0;

    task send_byte;
        input [7:0] b;
        begin
            @(negedge clk);
            rx_data = b;
            rx_valid = 1'b1;
            @(negedge clk);
            rx_valid = 1'b0;
        end
    endtask

    task fail;
        input [640:1] msg;
        begin
            $display("FAIL: %0s", msg);
            errors = errors + 1;
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // --- rod-loop ASCII gains (existing protocol regression) ---
        send_byte(8'h4B); send_byte(8'h50); send_byte(8'h2B); send_byte(8'h0A);
        if (!pid_gain_valid) fail("KP+ missing pulse");
        if (pid_gain_index != 2'd0) fail("KP+ wrong index");
        if (!pid_gain_increment) fail("KP+ wrong increment");
        if (arm_gain_valid) fail("KP+ cross-talk to arm output");

        send_byte(8'h4B); send_byte(8'h49); send_byte(8'h2D); send_byte(8'h0A);
        if (!pid_gain_valid) fail("KI- missing pulse");
        if (pid_gain_index != 2'd1) fail("KI- wrong index");
        if (pid_gain_increment) fail("KI- wrong increment");

        send_byte(8'h4B); send_byte(8'h44); send_byte(8'h2B); send_byte(8'h0A);
        if (!pid_gain_valid || pid_gain_index != 2'd2 || !pid_gain_increment)
            fail("KD+ wrong pulse");

        // CRLF terminator tolerance
        send_byte(8'h4B); send_byte(8'h50); send_byte(8'h2D); send_byte(8'h0D);
        send_byte(8'h0A);
        if (!pid_gain_valid || pid_gain_index != 2'd0 || pid_gain_increment)
            fail("KP- CRLF wrong pulse");

        // --- rod-loop binary gains ---
        send_byte(8'h02);
        if (!pid_gain_valid || pid_gain_index != 2'd0 || pid_gain_increment)
            fail("binary 02 wrong");
        send_byte(8'h05);
        if (!pid_gain_valid || pid_gain_index != 2'd2 || !pid_gain_increment)
            fail("binary 05 wrong");
        send_byte(8'h09);
        if (pid_gain_valid || arm_gain_valid || mode_valid || delta_valid ||
            delta_apply_valid || delta_cancel_valid || sine_period_valid ||
            sine_start_valid || sine_stop_valid)
            fail("binary 09 must be ignored");

        // --- NEW arm-loop ASCII gains ---
        send_byte(8'h4C); send_byte(8'h50); send_byte(8'h2B); send_byte(8'h0A);
        if (!arm_gain_valid) fail("LP+ missing pulse");
        if (arm_gain_index != 2'd0) fail("LP+ wrong index");
        if (!arm_gain_increment) fail("LP+ wrong increment");
        if (pid_gain_valid) fail("LP+ cross-talk to rod output");

        send_byte(8'h4C); send_byte(8'h49); send_byte(8'h2B); send_byte(8'h0A);
        if (!arm_gain_valid || arm_gain_index != 2'd1 || !arm_gain_increment)
            fail("LI+ wrong pulse");

        send_byte(8'h4C); send_byte(8'h44); send_byte(8'h2D); send_byte(8'h0A);
        if (!arm_gain_valid || arm_gain_index != 2'd2 || arm_gain_increment)
            fail("LD- wrong pulse");

        // --- NEW arm-loop binary gains ---
        send_byte(8'h11);
        if (!arm_gain_valid || arm_gain_index != 2'd0 || !arm_gain_increment)
            fail("binary 11 wrong");
        send_byte(8'h13);
        if (!arm_gain_valid || arm_gain_index != 2'd1 || !arm_gain_increment)
            fail("binary 13 wrong");
        send_byte(8'h16);
        if (!arm_gain_valid || arm_gain_index != 2'd2 || arm_gain_increment)
            fail("binary 16 wrong");
        send_byte(8'h10);
        if (pid_gain_valid || arm_gain_valid)
            fail("binary 10 must be ignored");

        // lowercase 'l' is not a command
        send_byte(8'h6C); send_byte(8'h70); send_byte(8'h2B); send_byte(8'h0A);
        if (pid_gain_valid || arm_gain_valid)
            fail("lowercase lp+ must be ignored");

        // --- existing command regression ---
        send_byte(8'h4D); send_byte(8'h31); send_byte(8'h0A);
        if (!mode_valid || mode_value != 2'd1) fail("M1 wrong");
        send_byte(8'h4D); send_byte(8'h30); send_byte(8'h0A);
        if (!mode_valid || mode_value != 2'd0) fail("M0 wrong");

        send_byte(8'h44); send_byte(8'h2B); send_byte(8'h31); send_byte(8'h30);
        send_byte(8'h0A);
        @(negedge clk);   // S_DELTA_REDUCE fires one posedge after the LF sample
        if (!delta_valid || delta_degrees != 16'sd10) fail("D+10 wrong");

        send_byte(8'h41); send_byte(8'h0A);
        if (!delta_apply_valid) fail("A wrong");
        send_byte(8'h58); send_byte(8'h0A);
        if (!delta_cancel_valid) fail("X wrong");
        send_byte(8'h53); send_byte(8'h0A);
        if (!sine_start_valid) fail("S wrong");
        send_byte(8'h51); send_byte(8'h0A);
        if (!sine_stop_valid) fail("Q wrong");

        send_byte(8'h50); send_byte(8'h33); send_byte(8'h30); send_byte(8'h0A);
        if (!sine_period_valid || sine_period_seconds != 8'd30) fail("P30 wrong");

        if (errors == 0) begin
            $display("PASS: ASCII/binary rod+arm gain commands, CRLF, cross-talk, M/D/A/X/S/Q/P regression");
            $finish;
        end else begin
            $fatal(1, "%0d failures", errors);
        end
    end
endmodule
