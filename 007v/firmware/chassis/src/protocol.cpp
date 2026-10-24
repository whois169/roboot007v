// 007v/chassis/protocol.py 的 C++ 版：check_cmd 的每一条规则都照搬。
#include "protocol.h"

#include <ArduinoJson.h>
#include <cmath>
#include <cstring>

// Python 的 _num：数字（不是 true/false），在范围里
static bool num(JsonVariantConst v, float lo, float hi) {
    if (v.is<bool>() || !(v.is<float>() || v.is<int>() || v.is<long>() || v.is<double>())) return false;
    float x = v.as<float>();
    return std::isfinite(x) && x >= lo && x <= hi;
}

static bool is_true(JsonVariantConst v) {
    return (v.is<bool>() && v.as<bool>()) || (v.is<int>() && !v.is<bool>() && v.as<int>() == 1);
}

Cmd parse_cmd(const char* line, size_t len) {
    Cmd c;
    JsonDocument doc;
    if (deserializeJson(doc, line, len)) return c;                 // 不是 JSON
    if (!doc.is<JsonObject>()) return c;
    JsonObject o = doc.as<JsonObject>();
    if (o["id"].is<JsonVariantConst>() && !o["id"].isNull()) {
        JsonVariantConst id = o["id"];
        if (id.is<bool>() || !id.is<long>() || id.as<long>() < 0 || id.as<long>() >= 2147483647L) return c;
        c.has_id = true;
        c.id = (int32_t)id.as<long>();
    }
    const char* key = nullptr;
    int nkeys = 0;
    for (JsonPair kv : o) {
        if (std::strcmp(kv.key().c_str(), "id") == 0) continue;
        key = kv.key().c_str();
        nkeys++;
    }
    if (nkeys != 1) return c;                                      // 一条只能有一个指令
    JsonVariantConst v = o[key];
    if (!std::strcmp(key, "drive")) {
        if (!v.is<JsonArrayConst>() || v.size() != 3 || !num(v[0], -1.1f, 1.1f) || !num(v[1], -0.5f, 0.5f) || !num(v[2], -1.6f, 1.6f)) return c;
        for (int i = 0; i < 3; i++) c.drive[i] = v[i].as<float>();
        c.kind = CmdKind::Drive;
    } else if (!std::strcmp(key, "move")) {
        if (!v.is<JsonObjectConst>()) return c;
        for (JsonPairConst kv : v.as<JsonObjectConst>()) {
            const char* a = kv.key().c_str();
            JsonVariantConst x = kv.value();
            if (!std::strcmp(a, "dx")) { if (!num(x, -5.0f, 5.0f)) return c; c.dx = x.as<float>(); }
            else if (!std::strcmp(a, "dy")) { if (!num(x, -2.0f, 2.0f)) return c; c.dy = x.as<float>(); }
            else if (!std::strcmp(a, "dth")) { if (!num(x, -360.0f, 360.0f)) return c; c.dth = x.as<float>(); }
            else if (!std::strcmp(a, "v")) { if (!num(x, 0.05f, 1.1f)) return c; c.v = x.as<float>(); }
            else return c;                                         // move 只有 dx dy dth v
        }
        c.kind = CmdKind::Move;
    } else if (!std::strcmp(key, "pose")) {
        if (!v.is<const char*>()) return c;
        if (!std::strcmp(v.as<const char*>(), "stand")) c.stand = true;
        else if (!std::strcmp(v.as<const char*>(), "fold")) c.stand = false;
        else return c;
        c.kind = CmdKind::Pose;
    } else if (!std::strcmp(key, "stop") || !std::strcmp(key, "estop") || !std::strcmp(key, "reset") || !std::strcmp(key, "hb")) {
        if (!is_true(v)) return c;
        c.kind = !std::strcmp(key, "stop") ? CmdKind::Stop : !std::strcmp(key, "estop") ? CmdKind::Estop
               : !std::strcmp(key, "reset") ? CmdKind::Reset : CmdKind::Hb;
    } else if (!std::strcmp(key, "limit")) {
        if (!v.is<JsonObjectConst>() || v.size() != 1 || !v["v"].is<JsonVariantConst>()) return c;
        JsonVariantConst x = v["v"];
        if (x.isNull()) c.limit_null = true;
        else if (num(x, 0.05f, 1.1f)) { c.limit_null = false; c.limit_v = x.as<float>(); }
        else return c;
        c.kind = CmdKind::Limit;
    } else if (!std::strcmp(key, "calib")) {
        if (!v.is<const char*>()) return c;
        if (!std::strcmp(v.as<const char*>(), "step")) c.calib_hip = false;
        else if (!std::strcmp(v.as<const char*>(), "hip")) c.calib_hip = true;
        else return c;
        c.kind = CmdKind::Calib;
    } else if (!std::strcmp(key, "light")) {
        if (!v.is<JsonObjectConst>() || v.size() == 0) return c;
        for (JsonPairConst kv : v.as<JsonObjectConst>()) {
            const char* a = kv.key().c_str();
            if (!num(kv.value(), 0, 100)) return c;
            if (!std::strcmp(a, "front")) { c.has_front = true; c.front = (int)std::lround(kv.value().as<float>()); }
            else if (!std::strcmp(a, "dim")) { c.has_dim = true; c.dim = (int)std::lround(kv.value().as<float>()); }
            else return c;
        }
        c.kind = CmdKind::Light;
    }
    return c;                                                      // 不认识的指令：Bad
}

static float r(float x, float k) { return std::round(x * k) / k; }

size_t write_event(const Event& e, char* buf, size_t n) {
    JsonDocument d;
    switch (e.kind) {
    case EvKind::Hello: {
        d["ev"] = "hello";
        d["fw"] = FW;
        JsonArray caps = d["caps"].to<JsonArray>();
        for (const char* s : {"mecanum", "hip4", "odom", "load", "estop", "step", "light"}) caps.add(s);
        break;
    }
    case EvKind::Ack:
    case EvKind::Done:
        d["ev"] = e.kind == EvKind::Ack ? "ack" : "done";
        d["id"] = e.id;
        d["ok"] = e.ok;
        if (!e.ok) d["why"] = e.why;
        break;
    case EvKind::Warn:
    case EvKind::Fault:
        d["ev"] = e.kind == EvKind::Warn ? "warn" : "fault";
        d["code"] = e.code;
        d["text"] = e.text;
        if (e.where) d["where"] = e.where;
        if (e.h >= 0) d["h"] = e.h;
        break;
    default:
        return 0;
    }
    size_t len = serializeJson(d, buf, n);
    return len < n ? len : 0;
}

size_t write_status(const Status& s, char* buf, size_t n) {
    JsonDocument d;
    d["ev"] = "chassis";
    d["st"] = state_name(s.st);
    JsonArray v = d["v"].to<JsonArray>();
    for (int i = 0; i < 3; i++) v.add(r(s.v[i], 1000));
    JsonArray odo = d["odo"].to<JsonArray>();
    odo.add(r(s.odo[0], 1000)); odo.add(r(s.odo[1], 1000)); odo.add(r(s.odo[2], 10));
    JsonArray hip = d["hip"].to<JsonArray>();
    for (int i = 0; i < 4; i++) hip.add(r(s.hip[i], 10));
    if (s.has_bat) { d["bat"] = s.bat; d["vbat"] = r(s.vbat, 100); } else { d["bat"] = nullptr; d["vbat"] = nullptr; }
    if (s.has_lim) d["lim"] = s.lim; else d["lim"] = nullptr;
    if (s.has_load) d["load"] = s.load; else d["load"] = nullptr;
    if (s.has_busy) d["busy"] = s.busy; else d["busy"] = nullptr;
    JsonArray st = d["step"].to<JsonArray>();
    for (int i = 0; i < 4; i++) {
        if (s.has_step && s.step_ok[i]) st.add((int)std::nearbyint(s.step[i]));   // 和 Python 的 round 一样：.5 取偶数
        else st.add(nullptr);
    }
    if (s.cliff) d["cliff"] = s.cliff; else d["cliff"] = nullptr;
    d["chg"] = s.chg;
    JsonObject li = d["light"].to<JsonObject>();
    li["front"] = s.front;
    li["dim"] = s.dim;
    size_t len = serializeJson(d, buf, n);
    return len < n ? len : 0;
}
