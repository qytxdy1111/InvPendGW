`timescale 1ns / 1ps

// Periodically writes five signed integer values to TJC/Nextion number fields
// n0..n4. Commands use the form "n0.val=-123" followed by FF FF FF.
module hmi_display #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD_RATE = 115200,
    parameter integer REFRESH_MS = 100
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] value0,
    input  wire signed [31:0] value1,
    input  wire signed [31:0] value2,
    input  wire signed [31:0] value3,
    input  wire signed [31:0] value4,
    output wire               tx
);
    localparam integer REFRESH_CYCLES = (CLK_FREQ / 1000) * REFRESH_MS;
    localparam integer REFRESH_W = $clog2(REFRESH_CYCLES + 1);

    localparam [2:0] WAIT_REFRESH = 3'd0;
    localparam [2:0] PREP_VALUE   = 3'd1;
    localparam [2:0] CONVERT      = 3'd2;
    localparam [2:0] LOAD_DIGITS  = 3'd3;
    localparam [2:0] SEND_BYTE    = 3'd4;
    localparam [2:0] WAIT_BYTE    = 3'd5;

    reg [2:0] state;
    reg [REFRESH_W-1:0] refresh_count;
    reg [2:0] field_index;
    reg [4:0] byte_index;
    reg [3:0] digit_count;
    reg [5:0] convert_count;
    reg       negative;
    reg [31:0] binary_shift;
    reg [39:0] bcd_value;
    reg [39:0] bcd_adjusted;
    reg [7:0] digits [0:9];

    reg       tx_start;
    reg [7:0] tx_data;
    wire      tx_busy;
    wire      tx_done;

    integer i;
    integer j;

    // One double-dabble step: adjust each BCD digit before shifting in one bit.
    always @(*) begin
        bcd_adjusted = bcd_value;
        for (j = 0; j < 10; j = j + 1) begin
            if (bcd_value[j*4 +: 4] >= 4'd5)
                bcd_adjusted[j*4 +: 4] = bcd_value[j*4 +: 4] + 4'd3;
        end
    end

    function signed [31:0] select_value;
        input [2:0] index;
        begin
            case (index)
                3'd0: select_value = value0;
                3'd1: select_value = value1;
                3'd2: select_value = value2;
                3'd3: select_value = value3;
                default: select_value = value4;
            endcase
        end
    endfunction

    // Prefix occupies bytes 0..6. Decimal digits are stored least-significant first.
    function [7:0] command_byte;
        input [4:0] index;
        integer digit_pos;
        begin
            case (index)
                5'd0: command_byte = 8'h6e; // n
                5'd1: command_byte = 8'h30 + field_index;
                5'd2: command_byte = 8'h2e; // .
                5'd3: command_byte = 8'h76; // v
                5'd4: command_byte = 8'h61; // a
                5'd5: command_byte = 8'h6c; // l
                5'd6: command_byte = 8'h3d; // =
                default: begin
                    if (negative && index == 5'd7)
                        command_byte = 8'h2d;
                    else if (index < 5'd7 + negative + digit_count) begin
                        digit_pos = digit_count - 1 - (index - 7 - negative);
                        command_byte = digits[digit_pos];
                    end else
                        command_byte = 8'hff;
                end
            endcase
        end
    endfunction

    wire [4:0] command_length = 5'd10 + negative + digit_count;
    wire signed [31:0] current_value = select_value(field_index);

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_hmi_uart_tx (
        .clk(clk),
        .rst_n(rst_n),
        .tx_start(tx_start),
        .tx_data(tx_data),
        .tx(tx),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= WAIT_REFRESH;
            refresh_count <= {REFRESH_W{1'b0}};
            field_index <= 3'd0;
            byte_index <= 5'd0;
            digit_count <= 4'd0;
            convert_count <= 6'd0;
            negative <= 1'b0;
            binary_shift <= 32'd0;
            bcd_value <= 40'd0;
            tx_start <= 1'b0;
            tx_data <= 8'h00;
            for (i = 0; i < 10; i = i + 1)
                digits[i] <= 8'h30;
        end else begin
            tx_start <= 1'b0;
            case (state)
                WAIT_REFRESH: begin
                    if (refresh_count == REFRESH_CYCLES - 1) begin
                        refresh_count <= {REFRESH_W{1'b0}};
                        field_index <= 3'd0;
                        state <= PREP_VALUE;
                    end else begin
                        refresh_count <= refresh_count + 1'b1;
                    end
                end

                PREP_VALUE: begin
                    negative <= current_value[31];
                    binary_shift <= current_value[31] ? (~current_value + 1'b1) : current_value;
                    bcd_value <= 40'd0;
                    digit_count <= 4'd0;
                    convert_count <= 6'd0;
                    state <= CONVERT;
                end

                CONVERT: begin
                    bcd_value <= {bcd_adjusted[38:0], binary_shift[31]};
                    binary_shift <= {binary_shift[30:0], 1'b0};
                    if (convert_count == 6'd31)
                        state <= LOAD_DIGITS;
                    else
                        convert_count <= convert_count + 1'b1;
                end

                LOAD_DIGITS: begin
                    for (i = 0; i < 10; i = i + 1)
                        digits[i] <= 8'h30 + bcd_value[i*4 +: 4];
                    if (bcd_value[39:36] != 0)      digit_count <= 4'd10;
                    else if (bcd_value[35:32] != 0) digit_count <= 4'd9;
                    else if (bcd_value[31:28] != 0) digit_count <= 4'd8;
                    else if (bcd_value[27:24] != 0) digit_count <= 4'd7;
                    else if (bcd_value[23:20] != 0) digit_count <= 4'd6;
                    else if (bcd_value[19:16] != 0) digit_count <= 4'd5;
                    else if (bcd_value[15:12] != 0) digit_count <= 4'd4;
                    else if (bcd_value[11:8] != 0)  digit_count <= 4'd3;
                    else if (bcd_value[7:4] != 0)   digit_count <= 4'd2;
                    else                            digit_count <= 4'd1;
                    byte_index <= 5'd0;
                    state <= SEND_BYTE;
                end

                SEND_BYTE: begin
                    if (!tx_busy) begin
                        tx_data <= command_byte(byte_index);
                        tx_start <= 1'b1;
                        state <= WAIT_BYTE;
                    end
                end

                WAIT_BYTE: begin
                    if (tx_done) begin
                        if (byte_index == command_length - 1) begin
                            if (field_index == 3'd4) begin
                                state <= WAIT_REFRESH;
                            end else begin
                                field_index <= field_index + 1'b1;
                                state <= PREP_VALUE;
                            end
                        end else begin
                            byte_index <= byte_index + 1'b1;
                            state <= SEND_BYTE;
                        end
                    end
                end

                default: state <= WAIT_REFRESH;
            endcase
        end
    end
endmodule
