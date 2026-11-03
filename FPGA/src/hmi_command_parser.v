`timescale 1ns/1ps

// ASCII command parser for the TJC screen link.
//
// Commands are terminated by LF; a CR immediately before LF is optional:
//   M0       stop/clear fault
//   M1       manual swing-up
//   M2       automatic swing-up
//   D+10     accumulate target position +10 degrees (not applied yet)
//   D-10     accumulate target position -10 degrees (not applied yet)
//   A        apply the buffered target-position change
//   X        discard the buffered target-position change
//   P10..P120 set the sinusoidal arm-motion period in seconds
//   S        start sinusoidal arm motion
//   Q        stop sinusoidal arm motion
//   KP+ / KP- change the runtime rod-loop P gain (step size preset in the FPGA)
//   KI+ / KI- change the runtime rod-loop I gain (step size preset in the FPGA)
//   KD+ / KD- change the runtime rod-loop D gain (step size preset in the FPGA)
//   LP+ / LP- change the runtime ARM-loop P gain (step size preset in the FPGA)
//   LI+ / LI- change the runtime ARM-loop I gain (step size preset in the FPGA)
//   LD+ / LD- change the runtime ARM-loop D gain (step size preset in the FPGA)
//   Binary 01..06 are equivalent one-byte commands for screen buttons:
//     01=KP+, 02=KP-, 03=KI+, 04=KI-, 05=KD+, 06=KD-
//   Binary 11..16 are the ARM-loop equivalents:
//     11=LP+, 12=LP-, 13=LI+, 14=LI-, 15=LD+, 16=LD-
//
// The outputs are one-clock pulses.  Digits are accumulated as they arrive;
// no whole-line multiply/parse is performed when the terminator arrives.
module hmi_command_parser (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               rx_valid,
    input  wire [7:0]         rx_data,
    output reg                mode_valid,
    output reg [1:0]          mode_value,
    output reg                delta_valid,
    output reg signed [15:0]  delta_degrees,
    output reg                delta_apply_valid,
    output reg                delta_cancel_valid,
    output reg                sine_period_valid,
    output reg [7:0]           sine_period_seconds,
    output reg                sine_start_valid,
    output reg                sine_stop_valid,
    output reg                pid_gain_valid,
    output reg [1:0]          pid_gain_index,
    output reg                pid_gain_increment,
    output reg                arm_gain_valid,
    output reg [1:0]          arm_gain_index,
    output reg                arm_gain_increment
);
    localparam [3:0] S_IDLE      = 4'd0;
    localparam [3:0] S_MODE      = 4'd1;
    localparam [3:0] S_DELTA_SIGN = 4'd2;
    localparam [3:0] S_DELTA_DIGITS = 4'd3;
    localparam [3:0] S_WAIT_EOL  = 4'd4;
    localparam [3:0] S_ACTION_EOL = 4'd5;
    localparam [3:0] S_DELTA_REDUCE = 4'd6;
    localparam [3:0] S_PERIOD_DIGITS = 4'd7;
    localparam [3:0] S_GAIN_PARAM = 4'd8;
    localparam [3:0] S_GAIN_SIGN = 4'd9;
    localparam [3:0] S_GAIN_EOL = 4'd10;
    localparam [3:0] S_ARM_GAIN_PARAM = 4'd11;

    reg [3:0] state;
    reg [1:0] mode_pending;
    reg       mode_pending_valid;
    reg       delta_negative;
    reg [16:0] delta_acc;
    reg [3:0]  delta_digit_count;
    reg [2:0] action_pending;
    reg [9:0] period_acc;
    reg [3:0] period_digit_count;
    reg [1:0] gain_index_pending;
    reg gain_increment_pending;
    reg gain_is_arm;

    wire is_cr = (rx_data == 8'h0d);
    wire is_lf = (rx_data == 8'h0a);
    wire is_digit = (rx_data >= 8'h30) && (rx_data <= 8'h39);
    // For ASCII '0'..'9', the low nibble is already the numeric digit.
    wire [3:0] digit_value = rx_data[3:0];

    // Once the magnitude has been reduced below 360, fold it into the
    // principal angle interval. Reduction itself is iterative (one subtraction
    // per clock) so a long command does not add a divider to the timing path.
    wire [8:0] delta_remainder = delta_acc[8:0];
    wire signed [16:0] delta_remainder_wrapped =
        (delta_remainder > 9'd180) ?
            ($signed({1'b0, delta_remainder}) - 17'sd360) :
            $signed({1'b0, delta_remainder});
    wire signed [16:0] delta_normalized = delta_negative ?
        -delta_remainder_wrapped : delta_remainder_wrapped;

    task emit_pending;
        begin
            if (mode_pending_valid) begin
                mode_valid <= 1'b1;
                mode_value <= mode_pending;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            mode_pending      <= 2'd0;
            mode_pending_valid<= 1'b0;
            delta_negative    <= 1'b0;
            delta_acc         <= 17'd0;
            delta_digit_count <= 4'd0;
            action_pending    <= 2'd0;
            period_acc       <= 10'd0;
            period_digit_count <= 4'd0;
            mode_valid        <= 1'b0;
            mode_value        <= 2'd0;
            delta_valid       <= 1'b0;
            delta_degrees     <= 16'sd0;
            delta_apply_valid <= 1'b0;
            delta_cancel_valid<= 1'b0;
            sine_period_seconds <= 8'd30;
            sine_period_valid <= 1'b0;
            sine_start_valid <= 1'b0;
            sine_stop_valid <= 1'b0;
            pid_gain_valid <= 1'b0;
            pid_gain_index <= 2'd0;
            pid_gain_increment <= 1'b0;
            arm_gain_valid <= 1'b0;
            arm_gain_index <= 2'd0;
            arm_gain_increment <= 1'b0;
            gain_is_arm <= 1'b0;
        end else begin
            mode_valid  <= 1'b0;
            delta_valid <= 1'b0;
            delta_apply_valid <= 1'b0;
            delta_cancel_valid<= 1'b0;
            sine_period_valid <= 1'b0;
            sine_start_valid <= 1'b0;
            sine_stop_valid <= 1'b0;
            pid_gain_valid <= 1'b0;
            arm_gain_valid <= 1'b0;

            if ((state == S_DELTA_REDUCE) && !rx_valid) begin
                if (delta_acc >= 17'd360) begin
                    delta_acc <= delta_acc - 17'd360;
                end else begin
                    if (delta_acc != 17'd0) begin
                        delta_valid <= 1'b1;
                        delta_degrees <= delta_normalized[15:0];
                    end
                    state <= S_IDLE;
                end
            end else if (rx_valid) begin
                case (state)
                    S_IDLE: begin
                        mode_pending_valid <= 1'b0;
                        delta_digit_count  <= 4'd0;
                        if (rx_data >= 8'h01 && rx_data <= 8'h06) begin
                            pid_gain_valid <= 1'b1;
                            pid_gain_index <= (rx_data <= 8'h02) ? 2'd0 :
                                              (rx_data <= 8'h04) ? 2'd1 : 2'd2;
                            pid_gain_increment <= (rx_data == 8'h01) ||
                                                  (rx_data == 8'h03) ||
                                                  (rx_data == 8'h05);
                            state <= S_IDLE;
                        end else if (rx_data >= 8'h11 && rx_data <= 8'h16) begin
                            arm_gain_valid <= 1'b1;
                            arm_gain_index <= (rx_data <= 8'h12) ? 2'd0 :
                                              (rx_data <= 8'h14) ? 2'd1 : 2'd2;
                            arm_gain_increment <= (rx_data == 8'h11) ||
                                                  (rx_data == 8'h13) ||
                                                  (rx_data == 8'h15);
                            state <= S_IDLE;
                        end else if (rx_data == 8'h4d) begin // M
                            state <= S_MODE;
                        end else if (rx_data == 8'h44) begin // D
                            state <= S_DELTA_SIGN;
                            delta_negative <= 1'b0;
                            delta_acc <= 17'd0;
                        end else if (rx_data == 8'h41) begin // A: apply buffered delta
                            action_pending <= 2'd1;
                            state <= S_ACTION_EOL;
                        end else if (rx_data == 8'h58) begin // X: discard buffered delta
                            action_pending <= 3'd2;
                            state <= S_ACTION_EOL;
                        end else if (rx_data == 8'h53) begin // S: start sine motion
                            action_pending <= 3'd3;
                            state <= S_ACTION_EOL;
                        end else if (rx_data == 8'h51) begin // Q: stop sine motion
                            action_pending <= 3'd4;
                            state <= S_ACTION_EOL;
                        end else if (rx_data == 8'h50) begin // P: set sine period
                            period_acc <= 10'd0;
                            period_digit_count <= 4'd0;
                            state <= S_PERIOD_DIGITS;
                        end else if (rx_data == 8'h4b) begin // K: set a PID gain step command
                            gain_is_arm <= 1'b0;
                            state <= S_GAIN_PARAM;
                        end else if (rx_data == 8'h4c) begin // L: set an ARM-loop gain step command
                            gain_is_arm <= 1'b1;
                            state <= S_ARM_GAIN_PARAM;
                        end else begin
                            state <= S_IDLE;
                        end
                    end

                    S_MODE: begin
                        if ((rx_data >= 8'h30) && (rx_data <= 8'h32)) begin
                            mode_pending <= rx_data[1:0];
                            mode_pending_valid <= 1'b1;
                            state <= S_WAIT_EOL;
                        end else if (is_lf) begin
                            state <= S_IDLE;
                            mode_pending_valid <= 1'b0;
                        end else begin
                            state <= S_IDLE;
                            mode_pending_valid <= 1'b0;
                        end
                    end

                    S_DELTA_SIGN: begin
                        if (rx_data == 8'h2d || rx_data == 8'h2b) begin // - / +
                            delta_negative <= (rx_data == 8'h2d);
                            delta_digit_count <= 4'd0;
                            delta_acc <= 17'd0;
                            state <= S_DELTA_DIGITS;
                        end else if (is_digit) begin
                            delta_negative <= 1'b0;
                            delta_acc <= digit_value;
                            delta_digit_count <= 4'd1;
                            state <= S_DELTA_DIGITS;
                        end else begin
                            state <= S_IDLE;
                        end
                    end

                    S_DELTA_DIGITS: begin
                        if (is_digit && (delta_digit_count < 4'd5)) begin
                            // This expression is evaluated once per UART byte,
                            // keeping the terminator path short and predictable.
                            delta_acc <= (delta_acc << 3) + (delta_acc << 1) + digit_value;
                            delta_digit_count <= delta_digit_count + 1'b1;
                        end else if (is_lf) begin
                            if (delta_digit_count != 4'd0)
                                state <= S_DELTA_REDUCE;
                            else
                                state <= S_IDLE;
                            mode_pending_valid <= 1'b0;
                        end else if (is_cr) begin
                            // Hold through CR and emit once on the LF of CRLF.
                            state <= S_DELTA_DIGITS;
                        end else begin
                            state <= S_IDLE;
                            mode_pending_valid <= 1'b0;
                        end
                    end

                    S_WAIT_EOL: begin
                        if (is_lf) begin
                            emit_pending();
                            state <= S_IDLE;
                            mode_pending_valid <= 1'b0;
                        end else if (is_cr) begin
                            state <= S_WAIT_EOL;
                        end else begin
                            // A new non-terminator starts a fresh command.
                            state <= S_IDLE;
                            mode_pending_valid <= 1'b0;
                        end
                    end

                    S_ACTION_EOL: begin
                        if (is_lf) begin
                            if (action_pending == 3'd1)
                                delta_apply_valid <= 1'b1;
                            else if (action_pending == 3'd2)
                                delta_cancel_valid <= 1'b1;
                            else if (action_pending == 3'd3)
                                sine_start_valid <= 1'b1;
                            else if (action_pending == 3'd4)
                                sine_stop_valid <= 1'b1;
                            action_pending <= 3'd0;
                            state <= S_IDLE;
                        end else if (is_cr) begin
                            state <= S_ACTION_EOL;
                        end else begin
                            action_pending <= 3'd0;
                            state <= S_IDLE;
                        end
                    end

                    S_PERIOD_DIGITS: begin
                        if (is_digit && (period_digit_count < 4'd3)) begin
                            period_acc <= (period_acc << 3) + (period_acc << 1) + digit_value;
                            period_digit_count <= period_digit_count + 1'b1;
                        end else if (is_lf) begin
                            if ((period_digit_count != 4'd0) &&
                                (period_acc >= 8'd10) && (period_acc <= 8'd120)) begin
                                sine_period_valid <= 1'b1;
                                sine_period_seconds <= period_acc[7:0];
                            end
                            state <= S_IDLE;
                        end else if (is_cr) begin
                            state <= S_PERIOD_DIGITS;
                        end else begin
                            state <= S_IDLE;
                        end
                    end

                    S_GAIN_PARAM: begin
                        if (rx_data == 8'h50) begin       // KP
                            gain_index_pending <= 2'd0;
                            state <= S_GAIN_SIGN;
                        end else if (rx_data == 8'h49) begin // KI
                            gain_index_pending <= 2'd1;
                            state <= S_GAIN_SIGN;
                        end else if (rx_data == 8'h44) begin // KD
                            gain_index_pending <= 2'd2;
                            state <= S_GAIN_SIGN;
                        end else begin
                            state <= S_IDLE;
                        end
                    end

                    S_ARM_GAIN_PARAM: begin
                        if (rx_data == 8'h50) begin       // LP
                            gain_index_pending <= 2'd0;
                            state <= S_GAIN_SIGN;
                        end else if (rx_data == 8'h49) begin // LI
                            gain_index_pending <= 2'd1;
                            state <= S_GAIN_SIGN;
                        end else if (rx_data == 8'h44) begin // LD
                            gain_index_pending <= 2'd2;
                            state <= S_GAIN_SIGN;
                        end else begin
                            state <= S_IDLE;
                        end
                    end

                    S_GAIN_SIGN: begin
                        if (rx_data == 8'h2b || rx_data == 8'h2d) begin
                            gain_increment_pending <= (rx_data == 8'h2b);
                            state <= S_GAIN_EOL;
                        end else begin
                            state <= S_IDLE;
                        end
                    end

                    S_GAIN_EOL: begin
                        if (is_lf) begin
                            if (gain_is_arm) begin
                                arm_gain_valid <= 1'b1;
                                arm_gain_index <= gain_index_pending;
                                arm_gain_increment <= gain_increment_pending;
                            end else begin
                                pid_gain_valid <= 1'b1;
                                pid_gain_index <= gain_index_pending;
                                pid_gain_increment <= gain_increment_pending;
                            end
                            state <= S_IDLE;
                        end else if (is_cr) begin
                            state <= S_GAIN_EOL;
                        end else begin
                            state <= S_IDLE;
                        end
                    end

                    default: begin
                        state <= S_IDLE;
                        mode_pending_valid <= 1'b0;
                    end
                endcase
            end
        end
    end
endmodule
