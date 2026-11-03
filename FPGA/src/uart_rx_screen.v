`timescale 1ns/1ps

// 115200 8N1 UART receiver for the TJC screen link.
module uart_rx_screen #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD_RATE = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire rx,
    output reg [7:0] data,
    output reg valid
);
    localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    localparam integer HALF_BIT = CLKS_PER_BIT / 2;
    localparam integer CNT_W = $clog2(CLKS_PER_BIT + 1);

    reg [CNT_W-1:0] baud_count;
    reg [3:0] bit_index;
    reg [7:0] shift_data;
    reg busy;
    reg rx_meta, rx_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_count <= 0;
            bit_index <= 0;
            shift_data <= 0;
            data <= 0;
            valid <= 0;
            busy <= 0;
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            valid <= 1'b0;
            if (!busy) begin
                if (!rx_sync) begin
                    busy <= 1'b1;
                    bit_index <= 0;
                    baud_count <= HALF_BIT - 1;
                end
            end else if (baud_count != 0) begin
                baud_count <= baud_count - 1'b1;
            end else begin
                baud_count <= CLKS_PER_BIT - 1;
                if (bit_index == 0) begin
                    if (rx_sync) busy <= 1'b0;
                    else bit_index <= 1;
                end else if (bit_index <= 8) begin
                    shift_data[bit_index - 1'b1] <= rx_sync;
                    bit_index <= bit_index + 1'b1;
                end else begin
                    busy <= 1'b0;
                    if (rx_sync) begin
                        data <= shift_data;
                        valid <= 1'b1;
                    end
                end
            end
        end
    end
endmodule
