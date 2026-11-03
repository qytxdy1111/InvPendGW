// ============================================================================
// uart_tx.v —— UART 发送模块（8N1，115200 波特率）
//
// 作用：把 8 bit 并行数据按 UART 协议一位一位串行发出去。
//       一帧 = 起始位(0) + 8 数据位(LSB 先发) + 停止位(1)
//
// 用法：tx_data 准备好后，给 tx_start 一个高电平脉冲（一拍），
//       模块自动把 tx_data 锁存并发完一帧。发送期间 tx_busy=1，
//       发完给 tx_done 一个脉冲。
//
// 波特率：115200 = 50_000_000 / 434，BAUD_DIV = 434
// 结构参考 Gowin 官方 uart 例程，改为 115200 并补了中文注释
// ============================================================================
`timescale 1ns / 1ps

module uart_tx #(
    parameter integer CLK_FREQ  = 50_000_000,   // 系统时钟 50MHz
    parameter integer BAUD_RATE = 115200         // 波特率
)(
    input  wire        clk,
    input  wire        rst_n,          // 低有效复位
    input  wire        tx_start,       // 高电平脉冲：启动发送
    input  wire [7:0]  tx_data,        // 待发送的 8 位数据
    output reg         tx,             // 串行输出（空闲为高电平）
    output wire        tx_busy,        // 1 = 正在发送
    output wire        tx_done         // 发完一帧的脉冲（一拍）
);
    localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE;   // 434
    localparam integer BAUD_W   = $clog2(BAUD_DIV);        // 9

    reg [BAUD_W-1:0] baud_cnt;   // 波特率分频计数器
    reg              bit_flag;   // 波特率边界脉冲（每 434 拍一次）
    reg [3:0]        bit_cnt;    // 当前位：0 起始 … 8 数据 … 9 停止
    reg              work_en;    // 发送使能（= tx_busy）
    reg [7:0]        data_lat;   // 锁存待发数据

    assign tx_busy = work_en;

    // 锁存数据：tx_start 有效且空闲时，把 tx_data 存下来
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            data_lat <= 8'd0;
        else if (tx_start && !work_en)
            data_lat <= tx_data;
    end

    // work_en：启动发送，发完停止位（bit_cnt==10）后才拉低
    // 保证停止位保持满一个位周期，符合 UART 协议（否则下一个字节会截断停止位）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            work_en <= 1'b0;
        else if (tx_start)
            work_en <= 1'b1;
        else if (bit_flag && bit_cnt == 4'd10)
            work_en <= 1'b0;
    end

    // 波特率计数器：0 → BAUD_DIV-1 → 0
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            baud_cnt <= {BAUD_W{1'b0}};
        else if ((baud_cnt == BAUD_DIV - 1) || !work_en)
            baud_cnt <= {BAUD_W{1'b0}};
        else
            baud_cnt <= baud_cnt + 1'b1;
    end

    // bit_flag：baud_cnt 计数到 1 时拉高一拍
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bit_flag <= 1'b0;
        else if (baud_cnt == 1)
            bit_flag <= 1'b1;
        else
            bit_flag <= 1'b0;
    end

    // bit_cnt：0 起始 / 1..8 数据 / 9..10 停止（10 让停止位满一个位周期）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bit_cnt <= 4'd0;
        else if (bit_flag && bit_cnt == 4'd10)
            bit_cnt <= 4'd0;
        else if (bit_flag && work_en)
            bit_cnt <= bit_cnt + 1'b1;
    end

    // tx 输出：按 bit_cnt 一位一位发
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tx <= 1'b1;
        else if (bit_flag) begin
            case (bit_cnt)
                4'd0:    tx <= 1'b0;        // 起始位
                4'd1:    tx <= data_lat[0];
                4'd2:    tx <= data_lat[1];
                4'd3:    tx <= data_lat[2];
                4'd4:    tx <= data_lat[3];
                4'd5:    tx <= data_lat[4];
                4'd6:    tx <= data_lat[5];
                4'd7:    tx <= data_lat[6];
                4'd8:    tx <= data_lat[7];
                4'd9:    tx <= 1'b1;        // 停止位
                4'd10:   tx <= 1'b1;        // 停止位保持
                default: tx <= 1'b1;
            endcase
        end
    end

    // tx_done：work_en 下降沿脉冲（发完一帧）
    reg work_en_r1, work_en_r2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            work_en_r1 <= 1'b0;
            work_en_r2 <= 1'b0;
        end else begin
            work_en_r1 <= work_en;
            work_en_r2 <= work_en_r1;
        end
    end
    assign tx_done = ~work_en_r1 & work_en_r2;
endmodule
