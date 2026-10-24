// 机械狗007v 底盘 ESP32-S3 固件：主循环。
// 每 5 ms：收大脑的指令 → 读传感器 → 底盘程序算一步（controller）→ 发给电机和灯 → 把事件、状态发回大脑。
// 和大脑之间是 USB 串口，一行一条 JSON（格式见 007v/chassis/protocol.py）。
#include <Arduino.h>
#include <esp_task_wdt.h>

#include <cstdio>

#include "board.h"
#include "controller.h"
#include "hw.h"
#include "protocol.h"

static Controller* ctl = nullptr;
static char line[512];
static size_t line_n = 0;
static bool line_over = false;
static char out[640];

static void send_line(size_t n) {
    if (n == 0) return;
    out[n] = '\n';
    Serial.write((const uint8_t*)out, n + 1);
}

static void read_serial() {
    while (Serial.available()) {
        int ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (line_n > 0 && !line_over) {
                Cmd c = parse_cmd(line, line_n);
                if (c.kind == CmdKind::Reset) hw_clear_wheel_faults();   // 复位时也清掉轮子驱动的故障
                ctl->command(c);
            }
            line_n = 0;
            line_over = false;
        } else if (line_n < sizeof line - 1) {
            line[line_n++] = (char)ch;
        } else {
            line_over = true;                                       // 太长的一行整行丢掉
        }
    }
}

void setup() {
    Serial.begin(115200);                                           // USB 串口（波特率其实不起作用）
    bool ok = hw_begin();
    Sensors s;
    hw_read(s);
    ctl = new Controller(s.hip, 0.0f);
    if (!ok) {                                                      // 开机发现的问题：当成提醒发给大脑（不影响别的功能）
        Event e{EvKind::Warn};
        e.code = "HW_CHECK";
        std::snprintf(e.text, sizeof e.text, "%s", hw_problem());
        ctl->q[ctl->qn++] = e;
    }
    esp_task_wdt_init(1, true);                                     // 主循环 1 秒没动静：重启（重启后轮子先松开）
    esp_task_wdt_add(nullptr);
}

void loop() {
    static uint32_t next = micros();
    const uint32_t period = 1000000 / RATE;
    read_serial();
    Sensors s;
    hw_read(s);
    Output o = ctl->update(1.0f / RATE, s);
    Status st = ctl->status(s.hip);
    if (st.st != State::Estop) {                                    // 急停时电机电源被继电器断了，没回应是正常的
        const char* code;
        char text[72];
        if (hw_wheel_problem(&code, text, sizeof text)) ctl->external_fault(code, text);
    }
    if (ctl->calib_hip_req) {
        hw_calib_hips();
        ctl->calib_hip_req = false;
    }
    hw_apply(o, st.st);
    for (int i = 0; i < ctl->qn; i++) send_line(write_event(ctl->q[i], out, sizeof out - 1));
    ctl->qn = 0;
    if (ctl->status_due) {
        send_line(write_status(st, out, sizeof out - 1));
        ctl->status_due = false;
    }
    esp_task_wdt_reset();
    next += period;                                                 // 固定 200 Hz（算得慢了就别补，免得连着追）
    int32_t wait = (int32_t)(next - micros());
    if (wait > 0) delayMicroseconds(wait);
    else next = micros();
}
