// 大脑 ↔ 底盘的“一行一条 JSON”：解析指令（和 007v/chassis/protocol.py 的 check_cmd 一样严格）、把事件和状态写成一行。
#pragma once
#include <cstddef>

#include "controller.h"

// 解析一行指令。不合法时 kind = Bad（带了 id 的话 has_id = true，主程序回 ack why=BAD）
Cmd parse_cmd(const char* line, size_t len);

// 写成一行 JSON（不带换行），返回长度；buf 不够时返回 0
size_t write_event(const Event& e, char* buf, size_t n);
size_t write_status(const Status& s, char* buf, size_t n);
