`timescale 1ns/1ps

// Periodically updates fourteen TJC widgets with signed integer values.
// Each assignment is followed by the required FF FF FF terminator.
// Fields 8..10 are the runtime rod-loop PID gains in hundredths (Q16 gain * 100)
// and go to p3.n0..p3.n2. Fields 11..13 are the runtime ARM-loop gains LP/LI/LD
// (LI = raw integral shift) and go to p5.n0..p5.n2. Configure the gain widgets
// as TJC Xfloat with vvs1=2 so, for example, the transmitted value 95000 is
// rendered as 950.00 (the LI shift widget should use vvs1=0, integer display).
module hmi_display8 #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD_RATE = 115200,
    parameter integer REFRESH_MS = 100
)(
    input wire clk,
    input wire rst_n,
    input wire signed [31:0] value0,
    input wire signed [31:0] value1,
    input wire signed [31:0] value2,
    input wire signed [31:0] value3,
    input wire signed [31:0] value4,
    input wire signed [31:0] value5,
    input wire signed [31:0] value6,
    input wire signed [31:0] value7,
    input wire signed [31:0] value8,
    input wire signed [31:0] value9,
    input wire signed [31:0] value10,
    input wire signed [31:0] value11,
    input wire signed [31:0] value12,
    input wire signed [31:0] value13,
    output wire tx
);
    localparam integer REFRESH_CYCLES = (CLK_FREQ / 1000) * REFRESH_MS;
    localparam integer REFRESH_W = $clog2(REFRESH_CYCLES + 1);
    localparam [2:0] WAIT_REFRESH=0, PREP_VALUE=1, CONVERT=2,
                     LOAD_DIGITS=3, SEND_BYTE=4, WAIT_BYTE=5;
    reg [2:0] state;
    reg [REFRESH_W-1:0] refresh_count;
    reg [3:0] field_index;
    reg [4:0] byte_index;
    reg [3:0] digit_count;
    reg [5:0] convert_count;
    reg negative;
    reg [31:0] binary_shift;
    reg [39:0] bcd_value, bcd_adjusted;
    reg [7:0] digits [0:9];
    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy, tx_done;
    integer i, j;

    always @(*) begin
        bcd_adjusted = bcd_value;
        for (j=0; j<10; j=j+1)
            if (bcd_value[j*4 +: 4] >= 5)
                bcd_adjusted[j*4 +: 4] = bcd_value[j*4 +: 4] + 4'd3;
    end

    function signed [31:0] select_value(input [3:0] index);
        begin
            case (index)
                0: select_value=value0; 1: select_value=value1;
                2: select_value=value2; 3: select_value=value3;
                4: select_value=value4; 5: select_value=value5;
                6: select_value=value6; 7: select_value=value7;
                8: select_value=value8; 9: select_value=value9;
                10: select_value=value10; 11: select_value=value11;
                12: select_value=value12;
                default: select_value=value13;
            endcase
        end
    endfunction

    function [7:0] prefix_byte(input [4:0] index, input [3:0] field);
        reg [7:0] page_digit, component_digit;
        begin
            if (field >= 11) begin
                page_digit = 8'h35; // p5: runtime ARM-loop gains (LP/LI/LD)
                component_digit = 8'h30 + (field - 11);
            end else if (field >= 8) begin
                page_digit = 8'h33; // p3: runtime rod PID gains
                component_digit = 8'h30 + (field - 8);
            end else begin
                page_digit = ((field==1) || (field==7)) ? 8'h31 : 8'h30;
                case (field)
                    0,1: component_digit=8'h30;
                    2: component_digit=8'h31;
                    3: component_digit=8'h32;
                    4: component_digit=8'h33;
                    5: component_digit=8'h34;
                    6: component_digit=8'h35;
                    default: component_digit=8'h32;
                endcase
            end
            case (index)
                0: prefix_byte=8'h70; // p
                1: prefix_byte=page_digit;
                2: prefix_byte=8'h2e; // .
                3: prefix_byte=8'h6e; // n
                4: prefix_byte=component_digit;
                5: prefix_byte=8'h2e;
                6: prefix_byte=8'h76; // v
                7: prefix_byte=8'h61; // a
                8: prefix_byte=8'h6c; // l
                9: prefix_byte=8'h3d; // =
                default: prefix_byte=8'h00;
            endcase
        end
    endfunction

    function [7:0] command_byte(input [4:0] index);
        integer digit_pos;
        begin
            if (index < 10) command_byte = prefix_byte(index, field_index);
            else if (negative && index == 10) command_byte = 8'h2d;
            else if ((index >= 10 + negative) &&
                     (index < 10 + negative + digit_count)) begin
                digit_pos = digit_count - 1 - (index - 10 - negative);
                command_byte = digits[digit_pos];
            end else command_byte = 8'hff;
        end
    endfunction

    wire [5:0] command_length = 6'd13 + (negative ? 6'd1 : 6'd0) + digit_count;
    wire signed [31:0] current_value = select_value(field_index);

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) u_tx (
        .clk(clk), .rst_n(rst_n), .tx_start(tx_start), .tx_data(tx_data),
        .tx(tx), .tx_busy(tx_busy), .tx_done(tx_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=WAIT_REFRESH; refresh_count<=0; field_index<=0; byte_index<=0;
            digit_count<=0; convert_count<=0; negative<=0; binary_shift<=0;
            bcd_value<=0; tx_start<=0; tx_data<=0;
            for (i=0; i<10; i=i+1) digits[i]<=8'h30;
        end else begin
            tx_start<=0;
            case (state)
                WAIT_REFRESH: begin
                    if (refresh_count == REFRESH_CYCLES-1) begin
                        refresh_count<=0; field_index<=0; state<=PREP_VALUE;
                    end else refresh_count<=refresh_count+1'b1;
                end
                PREP_VALUE: begin
                    negative<=current_value[31];
                    binary_shift<=current_value[31] ? (~current_value+1'b1) : current_value;
                    bcd_value<=0; digit_count<=0; convert_count<=0; state<=CONVERT;
                end
                CONVERT: begin
                    bcd_value<={bcd_adjusted[38:0],binary_shift[31]};
                    binary_shift<={binary_shift[30:0],1'b0};
                    if (convert_count==31) state<=LOAD_DIGITS;
                    else convert_count<=convert_count+1'b1;
                end
                LOAD_DIGITS: begin
                    for (i=0; i<10; i=i+1) digits[i]<=8'h30+bcd_value[i*4 +: 4];
                    if (bcd_value[39:36]!=0) digit_count<=10;
                    else if (bcd_value[35:32]!=0) digit_count<=9;
                    else if (bcd_value[31:28]!=0) digit_count<=8;
                    else if (bcd_value[27:24]!=0) digit_count<=7;
                    else if (bcd_value[23:20]!=0) digit_count<=6;
                    else if (bcd_value[19:16]!=0) digit_count<=5;
                    else if (bcd_value[15:12]!=0) digit_count<=4;
                    else if (bcd_value[11:8]!=0) digit_count<=3;
                    else if (bcd_value[7:4]!=0) digit_count<=2;
                    else digit_count<=1;
                    byte_index<=0; state<=SEND_BYTE;
                end
                SEND_BYTE: begin
                    if (!tx_busy) begin tx_data<=command_byte(byte_index); tx_start<=1; state<=WAIT_BYTE; end
                end
                WAIT_BYTE: begin
                    if (tx_done) begin
                        if ({1'b0,byte_index} == command_length-1'b1) begin
                            if (field_index==13) state<=WAIT_REFRESH;
                            else begin field_index<=field_index+1'b1; state<=PREP_VALUE; end
                        end else begin byte_index<=byte_index+1'b1; state<=SEND_BYTE; end
                    end
                end
                default: state<=WAIT_REFRESH;
            endcase
        end
    end
endmodule
