// 007v/chassis/controller.py 的 C++ 版。每个函数都对着 Python 同名函数翻，注释只写和 Python 不一样的地方。
#include "controller.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static float signf(float x) { return x < 0 ? -1.0f : 1.0f; }
static float rad(float d) { return d * PI_F / 180.0f; }
static float wrap(float a) {
    a = std::fmod(a + PI_F, 2 * PI_F);
    if (a < 0) a += 2 * PI_F;
    return a - PI_F;
}

const char* state_name(State s) {
    static const char* const N[] = {"folded", "unfolding", "standing", "folding", "estop", "fault"};
    return N[(int)s];
}

int bat_pct(float v) {
    static const float pts[][2] = {{18.0f, 0}, {20.4f, 5}, {21.6f, 20}, {22.2f, 40}, {22.8f, 60}, {23.7f, 80}, {25.2f, 100}};
    if (v <= pts[0][0]) return 0;
    for (int i = 0; i < 6; i++)
        if (v <= pts[i + 1][0])
            return (int)std::lround(pts[i][1] + (pts[i + 1][1] - pts[i][1]) * (v - pts[i][0]) / (pts[i + 1][0] - pts[i][0]));
    return 100;
}

// 3×3 线性方程（高斯消元，选主元）
static void solve3(float A[3][3], const float b[3], float x[3]) {
    float M[3][4];
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) M[i][k] = A[i][k];
        M[i][3] = b[i];
    }
    for (int c = 0; c < 3; c++) {
        int p = c;
        for (int r = c; r < 3; r++)
            if (std::fabs(M[r][c]) > std::fabs(M[p][c])) p = r;
        for (int k = 0; k < 4; k++) { float t = M[c][k]; M[c][k] = M[p][k]; M[p][k] = t; }
        for (int r = 0; r < 3; r++)
            if (r != c) {
                float f = M[r][c] / M[c][c];
                for (int k = 0; k < 4; k++) M[r][k] -= f * M[c][k];
            }
    }
    for (int i = 0; i < 3; i++) x[i] = M[i][3] / M[i][i];
}

Controller::Controller(const float hips_deg[4], float now_) : now(now_), rx_t(now_) {
    state = hips_deg[0] < TOUCH_DEG ? State::Folded : State::Standing;
    for (int i = 0; i < 4; i++) hip_ref[i] = hips_deg[i];
    before_estop = state;
    for (float& t : warn_t) t = -9;
    load_reset();
    Event e{EvKind::Hello};
    push(e);
}

// ------------------------------------------------------------ 事件
void Controller::push(const Event& e) {
    if (qn < QN) q[qn++] = e;                 // 满了就丢（主循环每 5 ms 取一次，正常不会满）
}
void Controller::ack(bool has, int32_t id, bool ok, const char* why) {
    if (!has) return;
    Event e{EvKind::Ack};
    e.id = id; e.ok = ok; e.why = ok ? "" : why;
    push(e);
}
void Controller::done(int32_t id, bool ok, const char* why) {
    Event e{EvKind::Done};
    e.id = id; e.ok = ok; e.why = ok ? "" : why;
    push(e);
}
void Controller::warn(const char* code, const char* text, const char* where, int h) {
    Event e{EvKind::Warn};
    e.code = code; e.where = where; e.h = h;
    std::snprintf(e.text, sizeof e.text, "%s", text);
    push(e);
}
void Controller::warn_once(int slot, const char* code, const char* text, const char* where) {
    if (now - warn_t[slot] > 1.0f) {
        warn_t[slot] = now;
        warn(code, text, where);
    }
}

// ------------------------------------------------------------ 收指令
void Controller::command(const Cmd& c) {
    rx_t = now;
    if (c.kind == CmdKind::Bad) { ack(c.has_id, c.id, false, "BAD"); return; }
    switch (c.kind) {
    case CmdKind::Hb: return;
    case CmdKind::Reset: reset(); ack(c.has_id, c.id, true); return;
    case CmdKind::Estop: enter_estop("ESTOP_CMD", "大脑发了急停"); ack(c.has_id, c.id, true); return;
    case CmdKind::Limit:
        has_limit = !c.limit_null; limit = c.limit_v;
        ack(c.has_id, c.id, true); return;
    case CmdKind::Light:
        if (c.has_front) light_front = c.front;
        if (c.has_dim) light_dim = c.dim;
        ack(c.has_id, c.id, true); return;
    case CmdKind::Calib:
        if (c.calib_hip) {                    // 髋角度零点：只能趴平、停稳时做（主程序看到 calib_hip_req 去存）
            if (state != State::Folded || moving()) { ack(c.has_id, c.id, false, state != State::Folded ? "NOT_FOLDED" : "MOVING"); return; }
            calib_hip_req = true;
            ack(c.has_id, c.id, true); return;
        }
        if (state != State::Standing || moving()) { ack(c.has_id, c.id, false, state != State::Standing ? "FOLDED" : "MOVING"); return; }
        for (int i = 0; i < 4; i++)
            if (!step_ok[i] || step[i] < 0 || !has_step) { ack(c.has_id, c.id, false, "NODATA"); return; }
        for (int i = 0; i < 4; i++) step_cal[i] = step[i] - step_geo(i);
        ack(c.has_id, c.id, true); return;
    case CmdKind::Stop:
        cancel("STOPPED");
        for (float& x : v_tgt) x = 0;
        if (mode == Mode::Drive || mode == Mode::Move) mode = Mode::Stop;
        ack(c.has_id, c.id, true); return;
    default: break;
    }
    const char* b = blocked(c.kind);
    if (*b) {
        if (c.has_id) ack(true, c.id, false, b);
        else if (c.kind == CmdKind::Drive) {
            char t[64];
            std::snprintf(t, sizeof t, "现在不能走（%s）", b);
            if (std::strcmp(b, "FOLDED") == 0) warn_once(0, "FOLDED", t);
            else warn_once(1, "LIMIT", t);
        }
        return;
    }
    if (c.kind == CmdKind::Drive) {
        cancel("STOPPED");
        mode = Mode::Drive;
        drive_t = now;
        for (int i = 0; i < 3; i++) v_tgt[i] = c.drive[i];
        clamp(v_tgt);
        ack(c.has_id, c.id, true);
    } else if (c.kind == CmdKind::Move) {
        cancel("STOPPED");
        float th0 = odo[2];
        float gx = odo[0] + c.dx * std::cos(th0) - c.dy * std::sin(th0);
        float gy = odo[1] + c.dx * std::sin(th0) + c.dy * std::cos(th0);
        float vm = std::fmin(c.v, V_MAX);
        float dist = std::hypot(c.dx, c.dy);
        float ang = std::fabs(rad(c.dth));
        float vl = vlim() > 0 ? vlim() : 9.0f;
        float t_est = dist / std::fmax(std::fmin(vm, vl), 0.05f) + ang / MOVE_WZ + 2.0f;
        task = Task();
        task.active = true; task.has_id = c.has_id; task.id = c.id; task.is_move = true;
        task.goal[0] = gx; task.goal[1] = gy; task.goal[2] = th0 + rad(c.dth);
        task.v = vm; task.deadline = now + 2 * t_est + 3;
        mode = Mode::Move;
        ack(c.has_id, c.id, true);
    } else if (c.kind == CmdKind::Pose) {
        if ((c.stand && state == State::Standing) || (!c.stand && state == State::Folded)) {
            ack(c.has_id, c.id, true);
            if (c.has_id) done(c.id, true);
            return;
        }
        cancel("STOPPED");
        task = Task();
        task.active = true; task.has_id = c.has_id; task.id = c.id; task.is_move = false; task.stand = c.stand;
        task.deadline = now + 15;
        state = c.stand ? State::Unfolding : State::Folding;
        has_goal = true;
        hip_goal = c.stand ? STANCE_DEG : FOLD_DEG;
        mode = Mode::Pose;
        ack(c.has_id, c.id, true);
    }
}

const char* Controller::blocked(CmdKind k) {
    if (fault) return "FAULT";
    if (state == State::Estop) return "ESTOP";
    if (task.active && !task.is_move) return "BUSY";
    if ((k == CmdKind::Drive || k == CmdKind::Move) && state != State::Standing) return "FOLDED";
    if (k == CmdKind::Pose && moving()) return "MOVING";
    return "";
}

float Controller::vlim() const {
    float m = 0;
    bool any = false;
    const float vals[3] = {limit, step_lim, sensor_lim};
    const bool has[3] = {has_limit, has_step_lim, has_sensor_lim};
    for (int i = 0; i < 3; i++)
        if (has[i] && vals[i] > 0) { m = any ? std::fmin(m, vals[i]) : vals[i]; any = true; }
    return any ? m : 0.0f;
}

void Controller::clamp(float v[3]) const {
    float vmax = vlim() > 0 ? std::fmin(V_MAX, vlim()) : V_MAX;
    float vy = std::fmin(VY_MAX, vmax);
    v[0] = clampf(v[0], -vmax, vmax);
    v[1] = clampf(v[1], -vy, vy);
    v[2] = clampf(v[2], -WZ_MAX, WZ_MAX);
    if (cliff && std::strcmp(cliff, "front") == 0) v[0] = std::fmin(0.0f, v[0]);
    else if (cliff && std::strcmp(cliff, "rear") == 0) v[0] = std::fmax(0.0f, v[0]);
    float w[4];
    wheel_speeds(v, hip_ref, w);
    float wm = 0;
    for (float x : w) wm = std::fmax(wm, std::fabs(x));
    if (wm > W_MAX)
        for (int i = 0; i < 3; i++) v[i] *= W_MAX / wm;
}

// ------------------------------------------------------------ 运动学
void Controller::wheel_xy(const float hips[4], float xc[4]) const {
    for (int i = 0; i < 4; i++) xc[i] = LEG_FX[i] * (HIP_X / 1000 + ARM / 1000 * std::cos(rad(hips[i])));
}
void Controller::wheel_speeds(const float v[3], const float hips[4], float out[4]) const {
    float xc[4];
    wheel_xy(hips, xc);
    for (int i = 0; i < 4; i++) out[i] = (v[0] + S_ROLL[i] * v[1] + v[2] * (S_ROLL[i] * xc[i] - YW[i])) / R;
}
void Controller::body_speed(const float wv[4], const float hips[4], float out[3]) const {
    float xc[4], rows[4][3];
    wheel_xy(hips, xc);
    for (int i = 0; i < 4; i++) { rows[i][0] = 1.0f; rows[i][1] = (float)S_ROLL[i]; rows[i][2] = S_ROLL[i] * xc[i] - YW[i]; }
    float AtA[3][3], Atb[3];
    for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
            AtA[a][b] = 0;
            for (int i = 0; i < 4; i++) AtA[a][b] += rows[i][a] * rows[i][b];
        }
        Atb[a] = 0;
        for (int i = 0; i < 4; i++) Atb[a] += rows[i][a] * wv[i] * R;
    }
    solve3(AtA, Atb, out);
}
bool Controller::moving() const {
    float m = std::fmax(std::fmax(std::fabs(v_meas[0]), std::fabs(v_meas[1])), std::fabs(v_meas[2]) * 0.3f);
    if (m > 0.03f) return true;
    for (float x : v_cmd)
        if (std::fabs(x) > 1e-3f) return true;
    return false;
}

// ------------------------------------------------------------ 每 5 ms
Output Controller::update(float dt, const Sensors& s) {
    now += dt;
    n += 1;
    vbat = s.vbat;
    body_speed(s.wv, s.hip, v_meas);
    float th = odo[2];
    odo[0] += (v_meas[0] * std::cos(th) - v_meas[1] * std::sin(th)) * dt;
    odo[1] += (v_meas[0] * std::sin(th) + v_meas[1] * std::cos(th)) * dt;
    odo[2] = wrap(th + v_meas[2] * dt);
    if (s.estop && state != State::Estop) enter_estop("ESTOP_BTN", "急停按钮被按下");
    if (now - rx_t > LINK_TIMEOUT) {
        if (link_ok && moving()) warn("LINK_LOST", "2 秒没收到大脑的消息，自己停下");
        if (link_ok) {
            cancel("STOPPED");
            for (float& x : v_tgt) x = 0;
            if (mode == Mode::Drive || mode == Mode::Move) mode = Mode::Stop;
        }
        link_ok = false;
    } else {
        link_ok = true;
    }
    if (vbat > 0 && bat_pct(vbat) < BAT_LOW && !bat_warned) {
        bat_warned = true;
        char t[32];
        std::snprintf(t, sizeof t, "电量 %d%%", bat_pct(vbat));
        warn("BAT_LOW", t);
    }
    chg = s.ibat > CHG_A;
    if (s.has_step) step_check(s, dt);
    if (mode == Mode::Drive && now - drive_t > DRIVE_TIMEOUT) {
        for (float& x : v_tgt) x = 0;
        mode = Mode::Stop;
    }
    if (mode == Mode::Move) move_step();
    if (task.active && now > task.deadline) {
        cancel("TIMEOUT");
        for (float& x : v_tgt) x = 0;
        mode = Mode::Stop;
    }
    if (mode == Mode::Pose) pose_step(dt, s.hip);
    shape(dt);
    stall_check(s.hip, dt);
    load_step(s, dt);
    if (mode == Mode::Stop && !moving()) mode = Mode::Idle;
    float vabs = std::fabs(v_cmd[0]);
    braking = hard_brake || (vabs > 0.0213f && vabs < vabs_prev - BRAKE_A * dt);
    vabs_prev = vabs;
    Output o;
    for (int i = 0; i < 4; i++) o.hip_ref[i] = hip_ref[i];
    o.hip_vel = hip_vel;
    o.wheel_free = state == State::Unfolding || state == State::Folding;
    wheel_speeds(v_cmd, s.hip, o.wheel_w);
    o.light = lights();
    if (n % STATUS_EVERY == 0) status_due = true;
    return o;
}

void Controller::shape(float dt) {
    float tgt[3] = {0, 0, 0};
    if (state == State::Standing) {
        for (int i = 0; i < 3; i++) tgt[i] = v_tgt[i];
        clamp(tgt);
    }
    const float acc0[3] = {A_ACC, A_Y, A_W}, dec0[3] = {A_BRAKE, A_Y * 2, A_W * 2};
    for (int i = 0; i < 3; i++) {
        float acc = acc0[i], dec = dec0[i];
        if (state == State::Estop) {
            float k = std::fmin(1.0f, (now - estop_t) / ESTOP_RAMP);
            acc = dec = i < 2 ? (A_BRAKE + (A_ESTOP - A_BRAKE) * k) : A_W * 4;
        }
        if (i == 0 && hard_brake) dec = A_ESTOP;
        float cur = v_cmd[i], t = tgt[i];
        float a = (t * cur >= 0 && std::fabs(t) > std::fabs(cur)) ? acc : dec;
        float st = a * dt;
        v_cmd[i] = cur + clampf(t - cur, -st, st);
    }
    if (hard_brake && std::fabs(v_cmd[0]) < 1e-6f) hard_brake = false;
}

static float prof(float e, float vmax, float a, float tol) {
    if (std::fabs(e) < tol * 0.5f) return 0.0f;
    float m = std::fmin(std::fmin(vmax, std::sqrt(2 * a * std::fabs(e)) * 0.9f), 4 * std::fabs(e) + 0.03f);
    return std::copysign(m, e);
}

void Controller::move_step() {
    float gx = task.goal[0], gy = task.goal[1], gth = task.goal[2];
    float x = odo[0], y = odo[1], th = odo[2];
    float exw = gx - x, eyw = gy - y;
    float ex = exw * std::cos(th) + eyw * std::sin(th);
    float ey = -exw * std::sin(th) + eyw * std::cos(th);
    float eth = wrap(gth - th);
    float vl = std::fmin(task.v, vlim() > 0 ? vlim() : V_MAX);
    v_tgt[0] = prof(ex, vl, MOVE_A, POS_TOL);
    v_tgt[1] = prof(ey, std::fmin(vl, MOVE_VY), MOVE_A, POS_TOL);
    v_tgt[2] = prof(eth, MOVE_WZ, rad(90), ANG_TOL);
    if (std::fabs(ex) < POS_TOL && std::fabs(ey) < POS_TOL && std::fabs(eth) < ANG_TOL && !moving()) {
        for (float& v : v_tgt) v = 0;
        mode = Mode::Idle;
        finish(true);
    }
}

void Controller::pose_step(float dt, const float hips[4]) {
    float goal = hip_goal, cur = hip_ref[0];
    float d = goal - cur;
    if (std::fabs(d) < 0.05f) {
        for (float& h : hip_ref) h = goal;
        hip_vel = 0;
        float err = 0;
        for (int i = 0; i < 4; i++) err = std::fmax(err, std::fabs(hips[i] - goal));
        if (err > 3.0f) return;
        state = goal == STANCE_DEG ? State::Standing : State::Folded;
        mode = Mode::Idle;
        finish(true);
        return;
    }
    float sgn = d > 0 ? 1.0f : -1.0f;
    float lo = TOUCH_DEG - TOUCH_BAND, hi = TOUCH_DEG + TOUCH_BAND;
    float vmax = (lo <= cur && cur <= hi) ? HIP_SLOW : HIP_SPEED;
    float entry = sgn > 0 ? lo : hi;
    if ((entry - cur) * sgn > 0) vmax = std::fmin(vmax, std::sqrt(HIP_SLOW * HIP_SLOW + 2 * HIP_ACC * (entry - cur) * sgn));
    vmax = std::fmin(vmax, std::sqrt(2 * HIP_ACC * std::fabs(d)));
    float v = std::fabs(hip_vel);
    v = v < vmax ? std::fmin(v + HIP_ACC * dt, vmax) : std::fmax(v - HIP_ACC * dt, vmax);
    v = std::fmax(v, 2.0f);
    float st = std::fmin(std::fabs(d), v * dt);
    hip_vel = sgn * v;
    for (float& h : hip_ref) h = cur + sgn * st;
}

void Controller::stall_check(const float hips[4], float dt) {
    float err = 0;
    for (int i = 0; i < 4; i++) err = std::fmax(err, std::fabs(hip_ref[i] - hips[i]));
    stall_t = err > HIP_STALL_DEG ? stall_t + dt : 0.0f;
    if (stall_t > HIP_STALL_S && !fault) {
        fault = "HIP_STALL";
        state = State::Fault;
        cancel("FAULT");
        for (int i = 0; i < 4; i++) hip_ref[i] = hips[i];
        hip_vel = 0;
        for (float& x : v_tgt) x = 0;
        mode = Mode::Idle;
        Event e{EvKind::Fault};
        e.code = "HIP_STALL";
        std::snprintf(e.text, sizeof e.text, "髋关节跟不上（差 %.0f°），停下", err);
        push(e);
    }
}

void Controller::external_fault(const char* code, const char* text) {
    if (fault) return;
    fault = code;
    state = State::Fault;
    cancel("FAULT");
    hip_vel = 0;
    for (float& x : v_tgt) x = 0;
    mode = Mode::Idle;
    Event e{EvKind::Fault};
    e.code = code;
    std::snprintf(e.text, sizeof e.text, "%s", text);
    push(e);
}

// ------------------------------------------------------------ 载重估计
void Controller::load_reset() {
    for (float& x : ls) x = 0;
    has_load = false;
}

void Controller::load_step(const Sensors& s, float dt) {
    if (!s.has_wt || state != State::Standing) return;
    float vx = v_cmd[0];
    float a = (vx - vx_prev) / dt;
    vx_prev = vx;
    float k = std::fmin(1.0f, dt / 0.05f);
    a_f += (a - a_f) * k;
    f_f += ((s.wt[0] + s.wt[1] + s.wt[2] + s.wt[3]) / R - f_f) * k;
    still_t = moving() ? 0.0f : still_t + dt;
    if (still_t > LOAD_RESET_S && has_load) load_reset();
    bool straight = std::fabs(v_cmd[1]) < 0.02f && std::fabs(v_cmd[2]) < 0.05f;
    if (!straight || std::fabs(a_f) < 0.15f) return;
    float F = f_f;
    ls[0] += 1; ls[1] += a_f; ls[2] += a_f * a_f; ls[3] += F; ls[4] += a_f * F;
    float nn = ls[0];
    float var = ls[2] / nn - (ls[1] / nn) * (ls[1] / nn);
    if (nn >= LOAD_MIN_N && var > LOAD_MIN_VAR) {
        float m = (ls[4] / nn - ls[1] / nn * ls[3] / nn) / var;
        load = std::fmax(0.0f, std::round((m - M_EMPTY) * 2) / 2);
        has_load = true;
    }
}

// ------------------------------------------------------------ 台阶测距
float Controller::step_geo(int i) const {
    float h = std::fmax(SKID_H, WHEEL_D / 2 + ARM * std::sin(rad(STANCE_DEG)) - HIP_Z);
    return (h + STEP_Z[i]) / STEP_SIN;
}

void Controller::step_check(const Sensors& s, float dt) {
    has_step = true;
    int bad = -1;
    for (int i = 0; i < 4; i++) {
        step[i] = s.step[i];
        step_ok[i] = s.step_ok[i];
        if (!s.step_ok[i] && bad < 0) bad = i;
    }
    if (bad >= 0) {
        has_sensor_lim = true; sensor_lim = STEP_SENSOR_V;
        char t[72];
        std::snprintf(t, sizeof t, "%s 台阶测距没回应，限速 %.1f", STEP_NAMES[bad], STEP_SENSOR_V);
        warn_once(2, "STEP_SENSOR", t, STEP_NAMES[bad]);
    } else {
        has_sensor_lim = false;
    }
    if (state != State::Standing) {
        cliff = nullptr;
        has_step_lim = false;
        return;
    }
    bool drop_any = false;
    for (int side = 0; side < 2; side++) {
        const char* name = side == 0 ? "front" : "rear";
        int i0 = side * 2;
        float sgn = side == 0 ? 1.0f : -1.0f;
        bool drop = false;
        float rise = 0;
        for (int i = i0; i < i0 + 2; i++) {
            float ex = step_geo(i) + step_cal[i];
            if (!s.step_ok[i]) continue;
            if (s.step[i] == -1 || s.step[i] - ex > CLIFF_MM) drop = true;
            if (s.step[i] >= 0) rise = std::fmax(rise, ex - s.step[i]);
        }
        bool toward = v_cmd[0] * sgn > 0.01f || v_tgt[0] * sgn > 0.01f;
        if (drop) {
            drop_any = true;
            clear_t = 0;
            if (cliff != name) {
                cliff = name;
                if (toward) {
                    hard_brake = true;
                    v_tgt[0] = 0;
                    if (task.active && task.is_move) {
                        cancel("CLIFF");
                        mode = Mode::Stop;
                    }
                }
                warn("CLIFF", side == 0 ? "前面地面没了，已刹车，只能往反方向走" : "后面地面没了，已刹车，只能往反方向走", name);
            }
        }
        if (rise > STEP_MM && toward) {
            if (!has_step_lim) {
                int h = (int)std::lround(rise * STEP_SIN);
                char t[72];
                std::snprintf(t, sizeof t, "%s面有约 %d mm 的坎，限速 %.1f", side == 0 ? "前" : "后", h, STEP_V);
                warn("STEP", t, name, h);
            }
            has_step_lim = true; step_lim = STEP_V; step_t = now;
        }
    }
    if (cliff && !drop_any) {
        clear_t += dt;
        if (clear_t >= CLIFF_CLEAR_S - EPS_T) cliff = nullptr;
    }
    if (has_step_lim && now - step_t > STEP_HOLD_S) has_step_lim = false;
}

// ------------------------------------------------------------ 灯
Lights Controller::lights() const {
    Lights L;
    bool bad = state == State::Estop || state == State::Fault;
    int pct = vbat > 0 ? bat_pct(vbat) : 100;
    if (bad) { L.waist_fx = Fx::Flash; L.waist = Color::Red; }
    else if (chg) { L.waist_fx = Fx::Breath; L.waist = Color::Green; }
    else if (pct < 20) { L.waist_fx = Fx::Solid; L.waist = Color::Amber; }
    else { L.waist_fx = Fx::Solid; L.waist = Color::Cyan; }
    if (bad) { L.tail_fx = Fx::Flash; L.tail = 100; }
    else if (braking) { L.tail_fx = Fx::Solid; L.tail = 100; }
    else if (v_cmd[0] < -0.0213f) { L.tail_fx = Fx::Blink; L.tail = 100; }
    else { L.tail_fx = Fx::Solid; L.tail = 30; }
    L.ring = !bad;
    L.front = light_front;
    L.dim = light_dim;
    return L;
}

// ------------------------------------------------------------ 急停、故障、收尾
void Controller::enter_estop(const char* code, const char* text) {
    if (state != State::Estop) {
        before_estop = (state == State::Standing || state == State::Folded) ? state : rest_state();
        estop_t = now;
    }
    state = State::Estop;
    estop_soft = std::strcmp(code, "ESTOP_CMD") == 0;
    cancel("ESTOP");
    for (float& x : v_tgt) x = 0;
    hip_vel = 0;
    mode = Mode::Idle;
    Event e{EvKind::Fault};
    e.code = code;
    std::snprintf(e.text, sizeof e.text, "%s", text);
    push(e);
}

void Controller::reset() {
    if (state == State::Estop) state = before_estop;
    if (fault) {
        fault = nullptr;
        state = rest_state();
    }
    stall_t = 0;
}

State Controller::rest_state() const {
    return std::fabs(hip_ref[0] - STANCE_DEG) < 3 ? State::Standing : State::Folded;
}

void Controller::cancel(const char* why) {
    if (!task.active) return;
    if (!task.is_move) {
        hip_vel = 0;
        if (state != State::Estop && state != State::Fault) state = rest_state();
    }
    finish(false, why);
}

void Controller::finish(bool ok, const char* why) {
    Task t = task;
    task = Task();
    if (t.active && t.has_id) done(t.id, ok, why);
}

Status Controller::status(const float hips[4]) const {
    Status s{};
    s.st = state;
    for (int i = 0; i < 3; i++) { s.v[i] = v_meas[i]; s.odo[i] = odo[i]; }
    s.odo[2] = odo[2] * 180.0f / PI_F;
    for (int i = 0; i < 4; i++) s.hip[i] = hips[i];
    s.has_bat = vbat > 0; s.bat = s.has_bat ? bat_pct(vbat) : 0; s.vbat = vbat;
    s.lim = vlim(); s.has_lim = s.lim > 0;
    s.load = load; s.has_load = has_load;
    s.busy = task.id; s.has_busy = task.active && task.has_id;
    s.has_step = has_step;
    for (int i = 0; i < 4; i++) { s.step[i] = step[i]; s.step_ok[i] = step_ok[i]; }
    s.cliff = cliff;
    s.chg = chg;
    s.front = light_front; s.dim = light_dim;
    return s;
}
