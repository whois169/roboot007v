// 底盘程序：007v/chassis/controller.py 一行行翻过来的 C++（逻辑完全一样，Python 那边是测过的参考）。
// 这一层不碰硬件、不用 Arduino，电脑上也能编译测试；硬件的收发在 hw_*.cpp。
#pragma once
#include <cstdint>

#include "params.h"

// ---- 大脑发来的一条指令（已经过 protocol.cpp 检查）
enum class CmdKind : uint8_t { Drive, Move, Pose, Stop, Estop, Reset, Limit, Hb, Calib, Light, Bad };
struct Cmd {
    CmdKind kind = CmdKind::Bad;
    bool has_id = false;
    int32_t id = 0;
    float drive[3] = {0, 0, 0};
    float dx = 0, dy = 0, dth = 0, v = MOVE_V;  // move
    bool stand = true;                           // pose：true = stand，false = fold
    bool limit_null = true;                      // limit：null = 取消
    float limit_v = 0;
    bool calib_hip = false;                      // calib：false = step 台阶测距，true = hip 髋角度零点
    bool has_front = false, has_dim = false;     // light
    int front = 0, dim = 100;
};

// ---- 每 5 ms 的传感器读数
struct Sensors {
    float hip[4] = {0, 0, 0, 0};     // 髋角度（度，AS5600）
    float wv[4] = {0, 0, 0, 0};      // 轮子转速（rad/s）
    float wt[4] = {0, 0, 0, 0};      // 轮子扭矩（N·m）
    bool has_wt = false;
    bool estop = false;              // 急停按钮按下
    float vbat = 0;                  // 电池电压
    float ibat = 0;                  // 电池电流（往电池里流为正）
    bool has_step = false;           // 有没有台阶测距
    float step[4] = {0, 0, 0, 0};    // mm；-1 = 测不到地面
    bool step_ok[4] = {true, true, true, true};   // false = 测距板没回应（Python 里的 None）
};

// ---- 灯
enum class Fx : uint8_t { Solid, Breath, Flash, Blink };
enum class Color : uint8_t { Cyan, Green, Amber, Red };
struct Lights {
    Fx waist_fx = Fx::Solid;
    Color waist = Color::Cyan;
    Fx tail_fx = Fx::Solid;
    int tail = 30;                   // 尾灯亮度 %
    bool ring = true;                // 电机端面光环
    int front = 0, dim = 100;
};

// ---- 给电机的输出
struct Output {
    float hip_ref[4] = {0, 0, 0, 0};
    float hip_vel = 0;
    bool wheel_free = false;         // true = 松开（站起来、趴下的过程中）
    float wheel_w[4] = {0, 0, 0, 0}; // rad/s
    Lights light;
};

// ---- 发给大脑的事件
enum class EvKind : uint8_t { Hello, Status, Ack, Done, Warn, Fault };
enum class State : uint8_t { Folded, Unfolding, Standing, Folding, Estop, Fault };
struct Event {
    explicit Event(EvKind k = EvKind::Hello) : kind(k) {}
    EvKind kind;
    int32_t id = 0;
    bool ok = true;
    const char* why = "";        // ack / done 失败的原因
    const char* code = "";       // warn / fault
    char text[72] = "";
    const char* where = nullptr; // CLIFF / STEP / STEP_SENSOR
    int h = -1;                  // STEP 估的高度 mm
};
struct Status {
    State st;
    float v[3], odo[3], hip[4];
    int bat;  bool has_bat;  float vbat;
    float lim;  bool has_lim;
    float load; bool has_load;
    int32_t busy; bool has_busy;
    float step[4]; bool step_ok[4]; bool has_step;
    const char* cliff;           // nullptr = 没锁
    bool chg;
    int front, dim;
};

const char* state_name(State s);
int bat_pct(float v);

class Controller {
public:
    explicit Controller(const float hips_deg[4], float now = 0.0f);
    void command(const Cmd& c);
    Output update(float dt, const Sensors& s);
    Status status(const float hips[4]) const;
    Lights lights() const;
    float vlim() const;              // <=0 表示不限速
    void external_fault(const char* code, const char* text);   // 硬件层发现的故障（轮子电机没回应 / 驱动报故障）
    bool calib_hip_req = false;      // 收到合法的 calib hip：主程序去存零点，然后清掉

    // 事件队列（主循环每次取走发出去）
    static constexpr int QN = 32;
    Event q[QN];
    int qn = 0;
    bool status_due = false;         // 该发状态了
    float now;

private:
    State state;
    float hip_ref[4];
    float hip_vel = 0;
    bool has_goal = false;
    float hip_goal = 0;
    float v_cmd[3] = {0, 0, 0}, v_tgt[3] = {0, 0, 0}, v_meas[3] = {0, 0, 0};
    float odo[3] = {0, 0, 0};
    float drive_t = -9, rx_t;
    bool link_ok = true;
    enum class Mode : uint8_t { Idle, Drive, Move, Pose, Stop } mode = Mode::Idle;
    struct Task { bool active = false; bool has_id = false; int32_t id = 0; bool is_move = true;
                  float goal[3] = {0, 0, 0}; float v = 0; bool stand = true; float deadline = 0; } task;
    float limit = 0; bool has_limit = false;
    const char* fault = nullptr;
    bool estop_soft = false;
    float estop_t = 0;
    State before_estop;
    float stall_t = 0;
    float vbat = 0; bool bat_warned = false;
    int n = 0;
    float ls[5];                     // 载重估计：n, Σa, Σa², ΣF, ΣaF
    float load = 0; bool has_load = false;
    float still_t = 0, a_f = 0, f_f = 0, vx_prev = 0;
    float step[4] = {0, 0, 0, 0}; bool step_ok[4] = {false, false, false, false}; bool has_step = false;
    float step_cal[4] = {0, 0, 0, 0};
    const char* cliff = nullptr;
    float clear_t = 0;
    float step_lim = 0; bool has_step_lim = false;
    float sensor_lim = 0; bool has_sensor_lim = false;
    float step_t = -9;
    bool hard_brake = false, chg = false, braking = false;
    int light_front = 0, light_dim = 100;
    float vabs_prev = 0;
    float warn_t[8];                 // _warn_once 的上次时间（按 code 分）

    void push(const Event& e);
    void ack(bool has, int32_t id, bool ok, const char* why = "");
    void done(int32_t id, bool ok, const char* why = "");
    void warn(const char* code, const char* text, const char* where = nullptr, int h = -1);
    void warn_once(int slot, const char* code, const char* text, const char* where = nullptr);
    const char* blocked(CmdKind k);
    void clamp(float v[3]) const;
    void wheel_xy(const float hips[4], float xc[4]) const;
    void wheel_speeds(const float v[3], const float hips[4], float out[4]) const;
    void body_speed(const float wv[4], const float hips[4], float out[3]) const;
    bool moving() const;
    void shape(float dt);
    void move_step();
    void pose_step(float dt, const float hips[4]);
    void stall_check(const float hips[4], float dt);
    void load_reset();
    void load_step(const Sensors& s, float dt);
    float step_geo(int i) const;
    void step_check(const Sensors& s, float dt);
    void enter_estop(const char* code, const char* text);
    void reset();
    State rest_state() const;
    void cancel(const char* why);
    void finish(bool ok, const char* why = "");
};
