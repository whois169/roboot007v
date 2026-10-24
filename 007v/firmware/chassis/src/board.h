// 底盘 ESP32-S3-DevKitC-1 的接线和硬件常数（引脚和 007v/wiring.py 的引脚表一致）。
// 标 [到货] 的数要等东西到了实测再填，别的都按厂家资料定好了。
#pragma once
#include <cstdint>

// ---- 引脚
constexpr int PIN_CAN_TX = 4, PIN_CAN_RX = 5;           // Adafruit CAN Pal（TJA1051）
constexpr int PIN_SDA = 8, PIN_SCL = 9;                  // TCA9548A、INA226
constexpr int PIN_HIP_RPWM[4] = {10, 12, 14, 47};        // 前左 前右 后左 后右
constexpr int PIN_HIP_LPWM[4] = {11, 13, 21, 48};
constexpr int PIN_HIP_EN = 42;                           // 4 块 BTS7960 的使能并在一起，上电默认低
constexpr int PIN_HIP_IS[4] = {1, 2, 6, 7};              // 电流检测（ADC1）：硬件接了，保护逻辑等到货实测比例后再加
constexpr int PIN_ENC_A[4] = {15, 17, 38, 40}, PIN_ENC_B[4] = {16, 18, 39, 41};   // 蜗轮电机霍尔编码器（备用，现在用 AS5600 测角度）
constexpr int PIN_ESTOP = 43;                            // 急停按钮常开触点：按下 = 接地 = 低
constexpr int PIN_LED = 44;                              // WS2812 灯链（经 5V 电平转换）

// ---- I2C 设备（TCA9548A 分路：AS5600 和 VL53L1X 地址固定，只能一个通道一个）
constexpr uint8_t ADDR_TCA = 0x70, ADDR_AS5600 = 0x36, ADDR_VL53 = 0x29, ADDR_INA226 = 0x40;
constexpr int TCA_AS5600[4] = {0, 1, 2, 3};              // 前左 前右 后左 后右 的 AS5600 在 TCA 的第几路
constexpr int TCA_VL53[4] = {4, 5, 6, 7};                // 前左 前右 后左 后右 的台阶测距
constexpr uint32_t I2C_HZ = 400000;

// ---- 髋（AS5600 量髋轴角度，BTS7960 驱动蜗轮电机）
constexpr float HIP_SIGN[4] = {1, 1, 1, 1};              // [到货] AS5600 读数方向：摆臂往下摆，角度要变大；反了改 -1
constexpr float HIP_PWM_SIGN[4] = {1, 1, 1, 1};          // [到货] PWM 方向：正的占空比让角度变大；反了改 -1
constexpr int HIP_PWM_HZ = 20000, HIP_PWM_BITS = 10;
constexpr float HIP_KP = 0.08f;                          // [到货] 位置环：每差 1° 给 8% 占空比
constexpr float HIP_KV = 0.02f;                          // [到货] 速度前馈：每 1°/秒 给 2%
constexpr float HIP_HOLD_DEG = 0.5f;                     // 误差小于这个、又不要求转：断开 PWM（蜗轮自锁，不耗电）
constexpr float HIP_DUTY_MAX = 0.95f;

// ---- 轮子（SteadyWin GIM4310-10 + GDZ34 驱动，“自定义 CAN 协议 V3.07”，ref/gim4310/gdz_docs/）
constexpr uint32_t CAN_BAUD = 1000000;                   // 驱动出厂 1 Mbps
constexpr uint8_t WHEEL_ADDR[4] = {1, 2, 3, 4};          // [到货] 出厂都是 1，要用厂家上位机（USB 转 RS485）改成 1–4
constexpr float WHEEL_DIR[4] = {1, -1, 1, -1};           // [到货] 左边轮子正转 = 车往前；右边装反了所以取反；实测不对就改
constexpr float WHEEL_RPM_PER_RPM = 1.0f;                // [到货] 速度指令是输出轴转速（1.0）还是转子转速（=减速比 10）：发 60 RPM 看轮子是不是 1 秒 1 圈
constexpr float WHEEL_KT = 0.0f;                         // 力矩常数 N·m/A：开机用 0xB0 从驱动读，读不到就是 0（载重估计不工作）
constexpr int WHEEL_LOST_MS = 100;                       // 多久没回应算“轮子电机没回应”

// ---- 灯（WS2812 一条链：腰线左、腰线右、尾灯左、尾灯右、电机端面光环 ×4、前白灯 ×2）
// [到货] 灯珠型号还没定，数量都先写 0（不点灯）；定了型号按实际颗数填
constexpr int LED_WAIST = 0, LED_TAIL = 0, LED_RING = 0, LED_FRONT = 0;
constexpr int LED_TOTAL = 2 * LED_WAIST + 2 * LED_TAIL + 4 * LED_RING + 2 * LED_FRONT;

// ---- 电池（INA226 + 分流电阻串在电池负极）
constexpr float INA_SHUNT_OHM = 0.002f;                  // [到货] 分流电阻阻值（2 mΩ）
constexpr float INA_CUR_SIGN = 1.0f;                     // [到货] 充电时电流要是正的；反了改 -1
