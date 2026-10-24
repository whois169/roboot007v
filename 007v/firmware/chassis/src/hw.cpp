// 硬件层（Arduino-ESP32 2.x）。
//   I2C（AS5600 ×4、VL53L1X ×4、INA226）在核心 0 上单独一个任务里读，控制循环（核心 1，200 Hz）只拿最新值，不等 I2C；
//   CAN（轮子）和 PWM（髋）在控制循环里直接做。
#include "hw.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Preferences.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <cmath>
#include <cstring>
#include <driver/twai.h>

#include "board.h"

static char problem[96] = "";
static void note(const char* s) {
    if (!problem[0]) std::snprintf(problem, sizeof problem, "%s", s);
}
const char* hw_problem() { return problem; }

// ================================================================ I2C 任务（核心 0）
struct I2CData {
    float hip_raw[4];          // AS5600 读到的角度（度，0–360）
    bool hip_ok[4];
    float step[4];             // mm；-1 = 测不到地面
    uint32_t step_ms[4];       // 最后一次读到的时间
    float vbat, ibat;
};
static I2CData shared;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static VL53L1X vl[4];
static bool vl_ok[4];

static bool tca(int ch) {
    Wire.beginTransmission(ADDR_TCA);
    Wire.write(1 << ch);
    return Wire.endTransmission() == 0;
}

static bool reg_read(uint8_t addr, uint8_t reg, uint8_t* out, int n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, n) != n) return false;
    for (int i = 0; i < n; i++) out[i] = Wire.read();
    return true;
}

static bool reg_write16(uint8_t addr, uint8_t reg, uint16_t v) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(v >> 8);
    Wire.write(v & 0xFF);
    return Wire.endTransmission() == 0;
}

static void i2c_task(void*) {
    int k = 0, vi = 0;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        I2CData d;
        portENTER_CRITICAL(&mux);
        d = shared;
        portEXIT_CRITICAL(&mux);
        for (int i = 0; i < 4; i++) {                              // AS5600：RAW ANGLE 寄存器 0x0C，12 位
            uint8_t b[2];
            d.hip_ok[i] = tca(TCA_AS5600[i]) && reg_read(ADDR_AS5600, 0x0C, b, 2);
            if (d.hip_ok[i]) d.hip_raw[i] = (((b[0] & 0x0F) << 8) | b[1]) * 360.0f / 4096.0f;
        }
        for (int n = 0; n < 2; n++, vi = (vi + 1) % 4) {           // 台阶测距：每次看两个，有新数据就读（各自 50 Hz）
            if (!vl_ok[vi] || !tca(TCA_VL53[vi]) || !vl[vi].dataReady()) continue;
            uint16_t mm = vl[vi].read(false);
            bool valid = vl[vi].ranging_data.range_status == VL53L1X::RangeValid;
            d.step[vi] = valid ? (float)mm : -1.0f;                 // 没有有效回波 = 测不到地面
            d.step_ms[vi] = millis();
        }
        if (++k % 20 == 0) {                                       // INA226：每 0.1 秒
            uint8_t b[2];
            if (reg_read(ADDR_INA226, 0x02, b, 2)) d.vbat = ((b[0] << 8) | b[1]) * 1.25e-3f;
            if (reg_read(ADDR_INA226, 0x01, b, 2)) d.ibat = INA_CUR_SIGN * (int16_t)((b[0] << 8) | b[1]) * 2.5e-6f / INA_SHUNT_OHM;
        }
        portENTER_CRITICAL(&mux);
        shared = d;
        portEXIT_CRITICAL(&mux);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    }
}

// ================================================================ 髋（AS5600 零点 + BTS7960）
static float hip_off[4];
static Preferences prefs;

static float hip_deg(int i, float raw) {
    float a = HIP_SIGN[i] * (raw - hip_off[i]);
    while (a >= 180) a -= 360;
    while (a < -180) a += 360;
    return a;
}

void hw_calib_hips() {
    I2CData d;
    portENTER_CRITICAL(&mux);
    d = shared;
    portEXIT_CRITICAL(&mux);
    prefs.begin("hip", false);
    for (int i = 0; i < 4; i++) {
        hip_off[i] = d.hip_raw[i] - HIP_SIGN[i] * FOLD_DEG;      // 趴平时的读数 = FOLD_DEG
        char key[8];
        std::snprintf(key, sizeof key, "off%d", i);
        prefs.putFloat(key, hip_off[i]);
    }
    prefs.end();
}

static void hip_pwm(int i, float duty) {
    duty = HIP_PWM_SIGN[i] * duty;
    if (duty > HIP_DUTY_MAX) duty = HIP_DUTY_MAX;
    if (duty < -HIP_DUTY_MAX) duty = -HIP_DUTY_MAX;
    uint32_t full = (1u << HIP_PWM_BITS) - 1;
    ledcWrite(2 * i, duty > 0 ? (uint32_t)(duty * full) : 0);         // RPWM
    ledcWrite(2 * i + 1, duty < 0 ? (uint32_t)(-duty * full) : 0);    // LPWM
}

// ================================================================ 轮子（GDZ34 自定义 CAN 协议）
static float wheel_w[4], wheel_t[4], wheel_kt[4];
static uint32_t wheel_seen[4];
static uint8_t wheel_fault[4];
static bool wheel_free_now = false;
static int poll_i = 0;

static void can_send(uint8_t addr, const uint8_t* data, uint8_t len) {
    twai_message_t m = {};
    m.identifier = 0x100 | addr;                                   // 主机发的帧用 0x100|地址，回复的帧 ID 就是地址
    m.data_length_code = len;
    std::memcpy(m.data, data, len);
    twai_transmit(&m, 0);                                          // 不等：发不出去就算了，下一个 5 ms 再发
}

static void can_cmd(uint8_t addr, uint8_t cmd) { can_send(addr, &cmd, 1); }

static void can_speed(uint8_t addr, float rpm) {
    int32_t v = (int32_t)std::lround(rpm * 100.0f);                // 单位 0.01 RPM
    uint8_t d[5] = {0xC1, (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    can_send(addr, d, 5);
}

static int wheel_index(uint32_t id) {
    for (int i = 0; i < 4; i++)
        if (WHEEL_ADDR[i] == id) return i;
    return -1;
}

static float rpm_to_w(int i, float rpm) { return WHEEL_DIR[i] * rpm / WHEEL_RPM_PER_RPM * 2 * PI_F / 60.0f; }

static void can_poll() {
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        int i = wheel_index(m.identifier);
        if (i < 0 || m.data_length_code < 1) continue;
        const uint8_t* b = m.data;
        wheel_seen[i] = millis();
        switch (b[0]) {
        case 0xC1: case 0xA2:                                      // 速度：4s，0.01 RPM
            if (m.data_length_code >= 5) wheel_w[i] = rpm_to_w(i, (int32_t)(b[1] | b[2] << 8 | b[3] << 16 | b[4] << 24) / 100.0f);
            break;
        case 0xA4:                                                 // 温度、Q 轴电流（2s，0.001 A）、速度（2s，0.01 RPM）、角度
            if (m.data_length_code >= 6) {
                float iq = (int16_t)(b[2] | b[3] << 8) / 1000.0f;
                wheel_t[i] = WHEEL_DIR[i] * wheel_kt[i] * iq;      // 力矩 = 力矩常数 × Q 轴电流（厂家手册）
                wheel_w[i] = rpm_to_w(i, (int16_t)(b[4] | b[5] << 8) / 100.0f);
            }
            break;
        case 0xAE: case 0xCF:                                      // 状态：最后一个字节是故障码（有故障时驱动每 200 ms 主动发）
            if (m.data_length_code >= 8) wheel_fault[i] = b[7];
            break;
        case 0xAF:
            if (m.data_length_code >= 2) wheel_fault[i] = b[1];
            break;
        case 0xB0:                                                 // 极对数、力矩常数（4f）、减速比
            if (m.data_length_code >= 7) {
                float kt;
                std::memcpy(&kt, b + 2, 4);
                if (std::isfinite(kt) && kt > 0 && kt < 10) wheel_kt[i] = kt;
            }
            break;
        }
    }
}

void hw_clear_wheel_faults() {
    for (int i = 0; i < 4; i++) {
        can_cmd(WHEEL_ADDR[i], 0xAF);
        wheel_fault[i] = 0;
    }
}

int hw_wheel_problem(const char** code, char* text, int n) {
    static const char* const NAME[4] = {"前左", "前右", "后左", "后右"};
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        if (now - wheel_seen[i] > (uint32_t)WHEEL_LOST_MS) {
            *code = "WHEEL_LOST";
            std::snprintf(text, n, "%s轮子电机 %d ms 没回应", NAME[i], WHEEL_LOST_MS);
            return 1;
        }
        if (wheel_fault[i]) {
            *code = "WHEEL_FAULT";
            std::snprintf(text, n, "%s轮子驱动报故障 0x%02X（电压/电流/温度/编码器）", NAME[i], wheel_fault[i]);
            return 1;
        }
    }
    return 0;
}

// ================================================================ 灯
static Adafruit_NeoPixel strip(LED_TOTAL > 0 ? LED_TOTAL : 1, PIN_LED, NEO_GRB + NEO_KHZ800);

static uint32_t color_of(Color c, float k) {
    uint8_t r = 0, g = 0, b = 0;
    switch (c) {
    case Color::Cyan: r = 0; g = 190; b = 255; break;
    case Color::Green: r = 40; g = 220; b = 90; break;
    case Color::Amber: r = 255; g = 150; b = 0; break;
    case Color::Red: r = 255; g = 10; b = 10; break;
    }
    return strip.Color(r * k, g * k, b * k);
}

static float fx_level(Fx fx, uint32_t ms) {
    switch (fx) {
    case Fx::Solid: return 1.0f;
    case Fx::Breath: return 0.25f + 0.75f * (0.5f + 0.5f * std::sin(ms * 2 * PI_F / 2500.0f));
    case Fx::Flash: return (ms / 125) % 2 ? 1.0f : 0.0f;        // 4 Hz
    case Fx::Blink: return (ms / 500) % 2 ? 1.0f : 0.0f;        // 1 Hz
    }
    return 1.0f;
}

static void lights_show(const Lights& L) {
    if (LED_TOTAL <= 0) return;                                    // 灯珠还没定（board.h 里数量是 0）
    static uint32_t last = 0;
    uint32_t ms = millis();
    if (ms - last < 20) return;                                    // 50 帧就够
    last = ms;
    float dim = L.dim / 100.0f;
    int p = 0;
    uint32_t waist = color_of(L.waist, dim * fx_level(L.waist_fx, ms));
    for (int i = 0; i < 2 * LED_WAIST; i++) strip.setPixelColor(p++, waist);
    uint32_t tail = color_of(Color::Red, dim * L.tail / 100.0f * fx_level(L.tail_fx, ms));
    for (int i = 0; i < 2 * LED_TAIL; i++) strip.setPixelColor(p++, tail);
    uint32_t ring = L.ring ? color_of(Color::Cyan, dim) : 0;
    for (int i = 0; i < 4 * LED_RING; i++) strip.setPixelColor(p++, ring);
    uint8_t w = 255 * dim * L.front / 100.0f;
    for (int i = 0; i < 2 * LED_FRONT; i++) strip.setPixelColor(p++, strip.Color(w, w, w));
    strip.show();
}

// ================================================================ 开机、每 5 ms
bool hw_begin() {
    pinMode(PIN_HIP_EN, OUTPUT);
    digitalWrite(PIN_HIP_EN, LOW);                                 // 先别让髋动
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    for (int i = 0; i < 4; i++) {
        ledcSetup(2 * i, HIP_PWM_HZ, HIP_PWM_BITS);
        ledcSetup(2 * i + 1, HIP_PWM_HZ, HIP_PWM_BITS);
        ledcAttachPin(PIN_HIP_RPWM[i], 2 * i);
        ledcAttachPin(PIN_HIP_LPWM[i], 2 * i + 1);
        hip_pwm(i, 0);
    }
    prefs.begin("hip", true);
    for (int i = 0; i < 4; i++) {
        char key[8];
        std::snprintf(key, sizeof key, "off%d", i);
        hip_off[i] = prefs.getFloat(key, 0.0f);
    }
    bool calibrated = prefs.isKey("off0");
    prefs.end();
    if (!calibrated) note("髋角度还没标定：趴平以后发 {\"calib\":\"hip\"}");

    Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
    Wire.setTimeOut(5);
    if (!tca(0)) note("找不到 TCA9548A（I2C 0x70）");
    for (int i = 0; i < 4; i++) {
        vl_ok[i] = tca(TCA_VL53[i]) && vl[i].init();
        if (vl_ok[i]) {
            vl[i].setTimeout(30);
            vl[i].setDistanceMode(VL53L1X::Short);                 // 0.3 m 左右，短距模式最准、不怕环境光
            vl[i].setMeasurementTimingBudget(20000);
            vl[i].startContinuous(20);
        } else {
            note("有台阶测距没找到（VL53L1X）");
        }
    }
    reg_write16(ADDR_INA226, 0x00, 0x4527);                        // 16 次平均
    for (int i = 0; i < 4; i++) shared.step_ms[i] = 0;
    xTaskCreatePinnedToCore(i2c_task, "i2c", 4096, nullptr, 5, nullptr, 0);

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
    g.rx_queue_len = 32;
    g.tx_queue_len = 16;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) note("CAN 启动失败");
    for (int i = 0; i < 4; i++) {
        can_cmd(WHEEL_ADDR[i], 0xCF);                              // 先松开（上电本来就是松开的）
        can_cmd(WHEEL_ADDR[i], 0xAF);                              // 清故障
        can_cmd(WHEEL_ADDR[i], 0xB0);                              // 读力矩常数
        wheel_seen[i] = millis();
    }
    if (LED_TOTAL > 0) {
        strip.begin();
        strip.clear();
        strip.show();
    }
    delay(50);                                                     // 等 I2C 任务读到第一轮
    return problem[0] == 0;
}

void hw_read(Sensors& s) {
    I2CData d;
    portENTER_CRITICAL(&mux);
    d = shared;
    portEXIT_CRITICAL(&mux);
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        s.hip[i] = hip_deg(i, d.hip_raw[i]);
        s.wv[i] = wheel_w[i];
        s.wt[i] = wheel_t[i];
        s.step[i] = d.step[i];
        s.step_ok[i] = vl_ok[i] && now - d.step_ms[i] < 500;       // 0.5 秒没新数据 = 没回应
    }
    s.has_wt = wheel_kt[0] > 0 && wheel_kt[1] > 0 && wheel_kt[2] > 0 && wheel_kt[3] > 0;
    s.has_step = true;
    s.estop = digitalRead(PIN_ESTOP) == LOW;
    s.vbat = d.vbat;
    s.ibat = d.ibat;
}

void hw_apply(const Output& o, State st) {
    can_poll();
    // ---- 髋：位置环 + 速度前馈；误差很小又不要求转时断开（蜗轮自锁）
    bool en = st != State::Estop && st != State::Fault;
    digitalWrite(PIN_HIP_EN, en ? HIGH : LOW);
    I2CData d;
    portENTER_CRITICAL(&mux);
    d = shared;
    portEXIT_CRITICAL(&mux);
    for (int i = 0; i < 4; i++) {
        float err = o.hip_ref[i] - hip_deg(i, d.hip_raw[i]);
        float duty = HIP_KP * err + HIP_KV * o.hip_vel;
        if (!en || !d.hip_ok[i] || (std::fabs(err) < HIP_HOLD_DEG && std::fabs(o.hip_vel) < 0.1f)) duty = 0;
        hip_pwm(i, duty);
    }
    // ---- 轮子：松开 / 速度环
    if (o.wheel_free) {
        if (!wheel_free_now)
            for (int i = 0; i < 4; i++) can_cmd(WHEEL_ADDR[i], 0xCF);
        wheel_free_now = true;
        can_cmd(WHEEL_ADDR[poll_i], 0xA4);                         // 松开时也要知道转速（里程计）
    } else {
        wheel_free_now = false;
        for (int i = 0; i < 4; i++) {
            float rpm = WHEEL_DIR[i] * o.wheel_w[i] * 60.0f / (2 * PI_F) * WHEEL_RPM_PER_RPM;
            can_speed(WHEEL_ADDR[i], rpm);
        }
        static int k = 0;
        if (++k % 2 == 0) can_cmd(WHEEL_ADDR[poll_i], 0xA4);       // 每 10 ms 问一个电机的电流（算载重）
    }
    poll_i = (poll_i + 1) % 4;
    lights_show(o.light);
}
