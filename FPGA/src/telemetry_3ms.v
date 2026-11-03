`timescale 1ns / 1ps
// ADC-synchronous binary telemetry, 28 bytes, little endian, UART 8N1.
// CRC16/CCITT-FALSE covers bytes 2..25; bytes 26..27 carry CRC low first.
// Each interval includes its closing sample edge. Both interval counters reset
// even when a frame is dropped; sequence numbers expose those dropped samples.
// flags bit 0=UART overrun, bit 1=first sample after reset, bit 2=ADC rail,
// bit 3=balance active, bit 4=auto swing-up active, bit 5=latched fault,
// bit 6=manual PD active, bit 7=button press accepted / waiting for the
// long-press threshold (STATE_MANUAL_WAIT diagnostic). Frame length stays
// 28 bytes and frame[2] stays 1.
module telemetry_3ms #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD_RATE = 115200
)(
    input wire clk, rst_n, sample_valid,
    input wire signed [31:0] pos,
    input wire [11:0] theta,
    input wire [9:0] adc_raw,
    input wire [7:0] duty,
    input wire ain1, ain2, pwm,
    input wire [4:0] event_flags,
    output wire tx,
    output reg overrun
);
    localparam IDLE=0, CRC=1, SEND=2, WAIT_BYTE=3;
    reg [1:0] state;
    reg [7:0] frame [0:27];
    reg [31:0] sequence_number, interval_ticks;
    reg signed [31:0] pwm_ticks;
    reg first_interval, rail_seen;
    reg [4:0] byte_index, crc_byte;
    reg [2:0] crc_bit;
    reg [15:0] crc;
    reg tx_start;
    reg [7:0] tx_data;
    reg signed [31:0] pos_q;
    reg [11:0] theta_q;
    reg [9:0] adc_raw_q;
    reg [7:0] duty_q;
    reg ain1_q, ain2_q, pwm_q;
    reg [4:0] event_flags_q;
    wire tx_done;
    wire [7:0] mode = ain1_q ? (ain2_q ? 8'd3 : 8'd1) : (ain2_q ? 8'd2 : 8'd0);
    wire rail_now = (adc_raw_q == 10'd0) || (adc_raw_q == 10'd1023);
    wire signed [31:0] pwm_step = !pwm_q ? 32'sd0 :
        ((mode == 1) ? 32'sd1 : ((mode == 2) ? -32'sd1 : 32'sd0));
    wire signed [31:0] pwm_total = pwm_ticks + pwm_step;
    wire [31:0] period_total = interval_ticks + 32'd1;
    wire [15:0] crc_next = {crc[14:0],1'b0} ^
        ((crc[15] ^ frame[crc_byte][crc_bit]) ? 16'h1021 : 16'h0000);
    integer i;

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) uart (
        .clk(clk), .rst_n(rst_n), .tx_start(tx_start), .tx_data(tx_data),
        .tx(tx), .tx_busy(), .tx_done(tx_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            sequence_number <= 0;
            interval_ticks <= 0;
            pwm_ticks <= 0;
            first_interval <= 1;
            rail_seen <= 0;
            overrun <= 0;
            tx_start <= 0;
            tx_data <= 0;
            pos_q <= 0;
            theta_q <= 0;
            adc_raw_q <= 0;
            duty_q <= 0;
            ain1_q <= 0;
            ain2_q <= 0;
            pwm_q <= 0;
            event_flags_q <= 0;
            byte_index <= 0;
            crc_byte <= 2;
            crc_bit <= 7;
            crc <= 16'hffff;
            for (i=0; i<28; i=i+1) frame[i] <= 0;
        end else begin
            tx_start <= 0;
            pos_q <= pos;
            theta_q <= theta;
            adc_raw_q <= adc_raw;
            duty_q <= duty;
            ain1_q <= ain1;
            ain2_q <= ain2;
            pwm_q <= pwm;
            event_flags_q <= event_flags;
            interval_ticks <= period_total;
            pwm_ticks <= pwm_total;
            rail_seen <= rail_seen | rail_now;
            if (sample_valid) begin
                interval_ticks <= 0;
                pwm_ticks <= 0;
                rail_seen <= 0;
                first_interval <= 0;
                sequence_number <= sequence_number + 1'b1;
                if (state != IDLE) begin
                    overrun <= 1;
                end else begin
                    frame[0] <= 8'ha5;
                    frame[1] <= 8'h5a;
                    frame[2] <= 8'd1;
                    frame[3] <= {event_flags_q[4], event_flags_q[3:0], (rail_seen | rail_now), first_interval, overrun};
                    for (i=0; i<4; i=i+1) begin
                        frame[4+i] <= sequence_number[8*i +: 8];
                        frame[8+i] <= pos_q[8*i +: 8];
                        frame[16+i] <= pwm_total[8*i +: 8];
                        frame[20+i] <= period_total[8*i +: 8];
                    end
                    frame[12] <= theta_q[7:0];
                    frame[13] <= {4'b0,theta_q[11:8]};
                    frame[14] <= duty_q;
                    frame[15] <= mode;
                    frame[24] <= adc_raw_q[7:0];
                    frame[25] <= {6'b0,adc_raw_q[9:8]};
                    crc <= 16'hffff;
                    crc_byte <= 2;
                    crc_bit <= 7;
                    state <= CRC;
                end
            end
            case (state)
                CRC: begin
                    crc <= crc_next;
                    if (crc_bit == 0) begin
                        crc_bit <= 7;
                        if (crc_byte == 25) begin
                            frame[26] <= crc_next[7:0];
                            frame[27] <= crc_next[15:8];
                            byte_index <= 0;
                            state <= SEND;
                        end else crc_byte <= crc_byte + 1'b1;
                    end else crc_bit <= crc_bit - 1'b1;
                end
                SEND: begin
                    tx_data <= frame[byte_index];
                    tx_start <= 1;
                    state <= WAIT_BYTE;
                end
                WAIT_BYTE: if (tx_done) begin
                    if (byte_index == 27) state <= IDLE;
                    else begin
                        byte_index <= byte_index + 1'b1;
                        state <= SEND;
                    end
                end
                default: ;
            endcase
        end
    end
endmodule
