// 硬件层：把传感器读成 Sensors、把 Output 发给电机和灯。controller 不知道硬件长什么样，全在这里。
#pragma once
#include "controller.h"

bool hw_begin();                                  // 开机：I2C、CAN、PWM、灯；返回 false = 有东西没找到（会在 hw_problem 里说明）
const char* hw_problem();                         // 开机发现的问题（空字符串 = 都好）
void hw_read(Sensors& s);                         // 每 5 ms：最新的传感器读数
void hw_apply(const Output& o, State st);         // 每 5 ms：髋、轮子、灯
void hw_calib_hips();                             // 趴平时调用：把现在的 4 个 AS5600 读数记成“0°”，存进 Flash
void hw_clear_wheel_faults();                     // 复位时清掉轮子驱动的故障
// 轮子电机的问题：返回 0 = 没问题；否则 code / text 填好（WHEEL_LOST 没回应 / WHEEL_FAULT 驱动报故障）
int hw_wheel_problem(const char** code, char* text, int n);
