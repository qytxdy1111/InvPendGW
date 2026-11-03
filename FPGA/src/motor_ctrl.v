// ============================================================================
// motor_ctrl.v —— TB6612 电机控制（照搬 Gowin 官方 Interface_test 例程）
//
// 作用：根据 motor_a_sel（模式选择）+ duty_out（占空比 0~255），
//       输出 TB6612 所需的 AIN1/AIN2（方向）与 PWM（调速）
//
// 模式：00=停止  01=正转(CW)  10=反转(CCW)  11=短接刹车
// PWM：PWM_PERIOD=5400（约 9.26kHz），cmp = duty_out * 20
//
// 适配说明：与官方例程一致，仅补充 `timescale（仿真需要）
//           顶层 main.v 里用胶合逻辑把有符号 speed 转成 motor_a_sel+duty_out
// ============================================================================
`timescale 1ns / 1ps

// DUTY_PERMILLE=1: duty_out is 0..999, duty/1000; 100 means exactly10%.
// Default0 preserves the existing LQR duty/270 input convention.
module  motor_ctrl #(parameter integer DUTY_PERMILLE = 0)(

    input				clk_50m 	,	//时钟输入
    input				rst_n   	,	//复位输入

    // Motor A Control Command
    input       [1:0]   motor_a_sel ,   // A Motor Mode Selection：00:stop 01:CW 10:CCW 11:Short brake
    input       [9:0]   duty_out	,   // 0..999 permille, or legacy0..255

    // TB6612FNG Hardware Output Pin
    output	reg			ain1        ,
    output	reg			ain2        ,
    output              pwm

    );

//==================== 10kHz PWM Carrier Generation ====================
// 50MHz / 5000 = 10kHz
reg [24:0] pwm_cnt;
localparam [24:0] PWM_PERIOD = 25'd5400;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
        pwm_cnt <= 25'd0;
    else if(pwm_cnt >= PWM_PERIOD - 25'd1)
        pwm_cnt <= 25'd0;
    else
        pwm_cnt <= pwm_cnt + 25'd1;
end

// Occupancy Ratio Mapping：0~255 -> 0~5000
wire [9:0] duty_limited = duty_out > 999 ? 10'd999 : duty_out;
wire [31:0] cmp_a = duty_out * 10'd20;
// count/5400 < duty/1000 simplifies to count*5 < duty*27.
// Avoid a long combinational divider. Fractional pulse lengths round up one clock.
// The experiment's100/200/300/400 have exact integer pulse lengths.
wire [31:0] count_times5 = {3'b0,pwm_cnt[12:0]} * 16'd5;
wire [31:0] duty_times27 = duty_limited * 16'd27;

//==================== Motor A Directional Logic ====================
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        ain1 <= 1'b0;
        ain2 <= 1'b0;
    end else begin
        case(motor_a_sel)
            2'b00 : begin ain1 <= 1'b0; ain2 <= 1'b0; end  // Stop
            2'b01 : begin ain1 <= 1'b1; ain2 <= 1'b0; end  // CW
            2'b10 : begin ain1 <= 1'b0; ain2 <= 1'b1; end  // CCW
            2'b11 : begin ain1 <= 1'b1; ain2 <= 1'b1; end  // Short brake
        endcase
    end
end

// PWM
assign pwm = DUTY_PERMILLE ? (count_times5 < duty_times27) : (pwm_cnt < cmp_a);

endmodule
