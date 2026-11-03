// ============================================================================
// encoder.v -- 正交编码器 4 倍频解码
//
// 作用：解码 AB 相正交编码器，输出累计位置 pos 与方向 dir
//
// 约定：A 超前 B 90°（电机正转）→ pos 增加，dir = 1
// 编码器装在电机轴：4 倍频后计数/转 = 线数 × 4
//   减速比 1:20 → 输出轴理论 1040 计数/转（13×4×20）；实测均值 ≈1103（1084/1117/1108），
//   main.v 的标定常数按 1103 计数/转 计算
//
// 引脚：A 相 = Y20, B 相 = AA20（见 pin.cst）
// ============================================================================
`timescale 1ns / 1ps

module encoder #(
    parameter integer CNT_W = 32     // 位置计数器位宽
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             a,       // 编码器 A 相（1A = Y20）
    input  wire             b,       // 编码器 B 相（1B = AA20）
    output reg  [CNT_W-1:0] pos,     // 累计位置（正转增加）
    output wire             dir      // 1 = A 超前 B（正转）
);
    // 两级同步，消除亚稳态（外部信号必须先打两拍再用）
    reg a1, a2, b1, b2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a1 <= 1'b0; a2 <= 1'b0;
            b1 <= 1'b0; b2 <= 1'b0;
        end else begin
            a1 <= a;  a2 <= a1;
            b1 <= b;  b2 <= b1;
        end
    end

    wire [1:0] ab_cur = {a2, b2};   // 当前状态 {A,B}
    reg  [1:0] ab_prev;             // 上一拍状态
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ab_prev <= 2'b0;
        else        ab_prev <= ab_cur;
    end

    reg dir_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos     <= 0;
            dir_reg <= 1'b0;
        end else begin
            case ({ab_prev, ab_cur})   // {上一状态, 当前状态} = 4 位
                // A 超前 B（正转）：00→10→11→01→00 → 计数加
                4'b0010, 4'b1011, 4'b1101, 4'b0100: begin
                    pos     <= pos + 1'b1;
                    dir_reg <= 1'b1;
                end
                // B 超前 A（反转）：00→01→11→10→00 → 计数减
                4'b0001, 4'b0111, 4'b1110, 4'b1000: begin
                    pos     <= pos - 1'b1;
                    dir_reg <= 1'b0;
                end
                default: ;   // 状态没变化，保持
            endcase
        end
    end

    assign dir = dir_reg;
endmodule

