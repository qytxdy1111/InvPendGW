`timescale 1ns/1ps

// Top-level integration testbench: drives real 115200 8N1 UART bytes into
// main's hmi_rx pin and checks the runtime arm-loop/rod-loop gain registers.
//
// Expected values are derived from the DUT's own synthesis parameters, so the
// test tracks whichever defaults main.v currently carries (the rod gains were
// zeroed during tuning; the arm defaults are -30/-50/shift 4).
// uart_send_cmd sends the payload MSB-first: {"LP+", 8'h00} with len=3 puts
// 'L' in cmd[31:24], 'P' in cmd[23:16], '+' in cmd[15:8], so the extraction
// cmd[(len-i)*8 +: 8] transmits 'L','P','+' in order.
module tb_main_gains;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg enc_a = 1'b0;
    reg enc_b = 1'b0;
    reg [9:0] adc_data_in = 10'd0;
    reg mode_button_n = 1'b1;
    reg hmi_rx = 1'b1;
    wire adc_clk, adc_oe_n, adc_stby_n, uart_tx, hmi_tx;
    wire ain1, ain2, pwma, bn1, bn2, pwmb;
    wire [3:0] led;

    localparam integer BIT_CLKS = 50_000_000 / 115200;  // 434

    always #10 clk = ~clk;

    main dut (
        .clk(clk), .rst_n(rst_n),
        .enc_a(enc_a), .enc_b(enc_b), .adc_data_in(adc_data_in),
        .mode_button_n(mode_button_n), .hmi_rx(hmi_rx),
        .adc_clk(adc_clk), .adc_oe_n(adc_oe_n), .adc_stby_n(adc_stby_n),
        .uart_tx(uart_tx), .hmi_tx(hmi_tx),
        .ain1(ain1), .ain2(ain2), .pwma(pwma),
        .bn1(bn1), .bn2(bn2), .pwmb(pwmb), .led(led)
    );

    integer errors = 0;

    task uart_send_byte;
        input [7:0] b;
        integer i;
        begin
            hmi_rx = 1'b0;                       // start bit
            repeat (BIT_CLKS) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                hmi_rx = b[i];
                repeat (BIT_CLKS) @(posedge clk);
            end
            hmi_rx = 1'b1;                       // stop bit
            repeat (BIT_CLKS) @(posedge clk);
        end
    endtask

    task uart_send_cmd;   // len payload bytes (MSB-first) + LF
        input [31:0] cmd;
        input [2:0]  len;
        integer i;
        begin
            for (i = 0; i < len; i = i + 1)
                uart_send_byte(cmd[(len-i)*8 +: 8]);
            uart_send_byte(8'h0A);
        end
    endtask

    task fail;
        input [640:1] msg;
        begin
            $display("FAIL: %0s", msg);
            errors = errors + 1;
        end
    endtask

    task settle;
        begin
            repeat (20) @(posedge clk);
        end
    endtask

    initial begin
        integer kp0, ki0, kd0, akp0, akd0, ishift0, steps;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (40000) @(posedge clk);   // past the 32768-cycle power-on reset stretch

        // ---- power-on defaults: runtime regs must reset to the parameters ----
        if (dut.arm_loop_kp_runtime_q16 != dut.ARM_LOOP_KP_Q16) fail("reset arm KP");
        if (dut.arm_loop_kd_runtime_q16 != dut.ARM_LOOP_KD_Q16) fail("reset arm KD");
        if (dut.arm_loop_i_shift_runtime != dut.ARM_LOOP_I_SHIFT[3:0]) fail("reset arm I shift");
        if (dut.pid_kp_runtime_q16 != dut.KP_Q16) fail("reset rod KP");
        if (dut.pid_ki_runtime_q16 != dut.KI_Q16) fail("reset rod KI");
        if (dut.pid_kd_runtime_q16 != dut.KD_Q16) fail("reset rod KD");
        if (dut.arm_i_acc_clamp != ($signed(dut.ARM_LOOP_I_CLAMP_Q16) <<< dut.arm_loop_i_shift_runtime))
            fail("reset arm I clamp");

        kp0  = $signed(dut.pid_kp_runtime_q16);
        ki0  = $signed(dut.pid_ki_runtime_q16);
        kd0  = $signed(dut.pid_kd_runtime_q16);
        akp0 = $signed(dut.arm_loop_kp_runtime_q16);
        akd0 = $signed(dut.arm_loop_kd_runtime_q16);
        ishift0 = dut.arm_loop_i_shift_runtime;

        // ---- NEW arm-loop ASCII commands ----
        uart_send_cmd({"LP+", 8'h00}, 3);
        settle;
        if ($signed(dut.arm_loop_kp_runtime_q16) != akp0 - dut.ARM_LOOP_KP_STEP_Q16)
            fail("LP+ did not step KP by 1.00");
        if ($signed(dut.pid_kp_runtime_q16) != kp0) fail("LP+ cross-talk rod KP");

        uart_send_cmd({"LP-", 8'h00}, 3);
        settle;
        if ($signed(dut.arm_loop_kp_runtime_q16) != akp0) fail("LP- did not restore KP");

        uart_send_byte(8'h11);   // binary LP+
        settle;
        if ($signed(dut.arm_loop_kp_runtime_q16) != akp0 - dut.ARM_LOOP_KP_STEP_Q16)
            fail("binary 11 did not step KP");

        uart_send_cmd({"LI+", 8'h00}, 3);
        settle;
        if (dut.arm_loop_i_shift_runtime != ishift0 - 1) fail("LI+ did not shift down");
        if (dut.arm_i_acc_clamp != ($signed(dut.ARM_LOOP_I_CLAMP_Q16) <<< (ishift0 - 1)))
            fail("LI+ did not shrink the I clamp");

        uart_send_byte(8'h14);   // binary LI-
        settle;
        if (dut.arm_loop_i_shift_runtime != ishift0) fail("binary 14 did not shift up");

        uart_send_byte(8'h13);   // binary LI+
        uart_send_byte(8'h13);
        uart_send_byte(8'h13);
        settle;
        if (dut.arm_loop_i_shift_runtime != ishift0 - 3)
            fail("three LI+ did not reach shift 1");

        // Drive down to MIN (0) from whatever shift the design starts with.
        steps = 0;
        while ((dut.arm_loop_i_shift_runtime != 4'd0) && (steps < 16)) begin
            uart_send_byte(8'h13);
            settle;
            steps = steps + 1;
        end
        if (dut.arm_loop_i_shift_runtime != 4'd0)
            fail("LI+ never reached the shift MIN");
        if (dut.arm_i_acc_clamp != $signed(dut.ARM_LOOP_I_CLAMP_Q16))
            fail("I clamp at shift MIN is wrong");

        uart_send_byte(8'h13);   // extra LI+ stays clamped at MIN
        settle;
        if (dut.arm_loop_i_shift_runtime != 4'd0) fail("LI+ escaped the shift MIN");

        // Back up to the reset shift.
        steps = 0;
        while ((dut.arm_loop_i_shift_runtime != ishift0) && (steps < 32)) begin
            uart_send_byte(8'h14);
            settle;
            steps = steps + 1;
        end
        if (dut.arm_loop_i_shift_runtime != ishift0) fail("LI- did not restore the start shift");

        uart_send_cmd({"LD+", 8'h00}, 3);
        settle;
        if ($signed(dut.arm_loop_kd_runtime_q16) != akd0 - dut.ARM_LOOP_KD_STEP_Q16)
            fail("LD+ did not step KD");

        uart_send_byte(8'h16);   // binary LD-
        settle;
        if ($signed(dut.arm_loop_kd_runtime_q16) != akd0)
            fail("binary 16 did not restore KD");

        // ---- existing rod-loop commands still work ----
        uart_send_cmd({"KP+", 8'h00}, 3);
        settle;
        if ($signed(dut.pid_kp_runtime_q16) != kp0 + dut.PID_KP_STEP_Q16)
            fail("KP+ did not step rod KP");
        if ($signed(dut.arm_loop_kp_runtime_q16) != akp0 - 65536)
            fail("KP+ cross-talk arm KP");
        uart_send_byte(8'h02);   // binary KP-
        settle;
        if ($signed(dut.pid_kp_runtime_q16) != kp0)
            fail("binary 02 did not restore rod KP");

        uart_send_cmd({"KI+", 8'h00}, 3);
        settle;
        if ($signed(dut.pid_ki_runtime_q16) != ki0 + dut.PID_KI_STEP_Q16)
            fail("KI+ did not step rod KI");
        uart_send_byte(8'h04);   // binary KI-
        settle;
        if ($signed(dut.pid_ki_runtime_q16) != ki0)
            fail("binary 04 did not restore rod KI");

        uart_send_cmd({"KD+", 8'h00}, 3);
        settle;
        if ($signed(dut.pid_kd_runtime_q16) != kd0 + dut.PID_KD_STEP_Q16)
            fail("KD+ did not step rod KD");
        uart_send_byte(8'h06);   // binary KD-
        settle;
        if ($signed(dut.pid_kd_runtime_q16) != kd0)
            fail("binary 06 did not restore rod KD");

        // ---- the runtime gains feed the position-loop arithmetic ----
        // arm_loop_p_pos_q16 = (-kp_runtime) * arm_angle_p >>> 16
        force dut.arm_angle_p = 32'sd65536;   // 1.0 rad
        #1;
        if (dut.arm_loop_p_pos_q16 !=
            -$signed({{32{dut.arm_loop_kp_runtime_q16[31]}}, dut.arm_loop_kp_runtime_q16}))
            fail("P term does not use the runtime KP");
        if ((dut.ARM_LOOP_ENABLE == 1'b0) && (dut.arm_loop_q16 != 64'sd0))
            fail("ARM_LOOP_ENABLE=0 did not zero the loop");
        release dut.arm_angle_p;
        force dut.arm_rate_dfilt_q16 = 32'sd65536;   // 1.0 rad/s
        #1;
        if (dut.arm_loop_d_pos_q16 !=
            -$signed({{32{dut.arm_loop_kd_runtime_q16[31]}}, dut.arm_loop_kd_runtime_q16}))
            fail("D term does not use the runtime KD");
        release dut.arm_rate_dfilt_q16;
        #1;

        if (dut.arm_loop_i_q16 != 32'sd0) fail("I term nonzero with empty accumulator");

        if (errors == 0) begin
            $display("PASS: runtime arm-loop LP/LI/LD gains, binary 11-16, clamping, rod-loop regression, loop arithmetic");
            $finish;
        end else begin
            $fatal(1, "%0d failures", errors);
        end
    end
endmodule
