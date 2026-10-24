// 电脑上跑的对比程序：把 Python 底盘程序记下来的“每 5 ms 的传感器 + 指令”原样喂给 C++ 版，打印每一步的输出和事件。
// 由 test/compare.py 编译、运行、和 Python 的结果逐行比较。不上板子。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "../src/controller.h"
#include "../src/protocol.h"

static const char* FX[] = {"solid", "breath", "flash", "blink"};
static const char* COL[] = {"cyan", "green", "amber", "red"};

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "用法：replay 场景文件\n"); return 2; }
    FILE* f = std::fopen(argv[1], "rb");
    if (!f) return 2;
    static char line[4096], buf[1024];
    Controller* c = nullptr;
    float last_hip[4] = {0, 0, 0, 0};
    auto flush = [&]() {
        for (int i = 0; i < c->qn; i++) {
            size_t n = write_event(c->q[i], buf, sizeof buf);
            std::printf("E %.*s\n", (int)n, buf);
        }
        c->qn = 0;
    };
    while (std::fgets(line, sizeof line, f)) {
        size_t len = std::strlen(line);
        while (len && (line[len - 1] == '\n' || line[len - 1] == '\r')) line[--len] = 0;
        if (line[0] == 'I') {                                  // I h0 h1 h2 h3：开机时的髋角度
            float h[4];
            std::sscanf(line + 2, "%f %f %f %f", &h[0], &h[1], &h[2], &h[3]);
            c = new Controller(h, 0.0f);
            flush();
        } else if (line[0] == 'C') {                           // C <一行 JSON 指令>
            c->command(parse_cmd(line + 2, len - 2));
            flush();
        } else if (line[0] == 'F') {                           // F code text：硬件层报的故障
            char code[32], text[96];
            std::sscanf(line + 2, "%31s %95[^\n]", code, text);
            static std::string keep[16];
            static int k = 0;
            keep[k % 16] = code;
            c->external_fault(keep[k++ % 16].c_str(), text);
            flush();
        } else if (line[0] == 'U') {                           // U dt hip×4 wv×4 wt×4 has_wt estop vbat ibat has_step step×4 ok×4
            Sensors s;
            float dt;
            int has_wt, estop, has_step, ok[4];
            char* p = line + 2;
            dt = std::strtof(p, &p);
            for (float& x : s.hip) x = std::strtof(p, &p);
            for (float& x : s.wv) x = std::strtof(p, &p);
            for (float& x : s.wt) x = std::strtof(p, &p);
            has_wt = (int)std::strtol(p, &p, 10);
            estop = (int)std::strtol(p, &p, 10);
            s.vbat = std::strtof(p, &p);
            s.ibat = std::strtof(p, &p);
            has_step = (int)std::strtol(p, &p, 10);
            for (float& x : s.step) x = std::strtof(p, &p);
            for (int& x : ok) x = (int)std::strtol(p, &p, 10);
            s.has_wt = has_wt; s.estop = estop; s.has_step = has_step;
            for (int i = 0; i < 4; i++) s.step_ok[i] = ok[i];
            for (int i = 0; i < 4; i++) last_hip[i] = s.hip[i];
            Output o = c->update(dt, s);
            std::printf("O %.6f %.6f %.6f %.6f %.6f %d %.6f %.6f %.6f %.6f %s %s %s %d %d %d %d\n",
                        o.hip_ref[0], o.hip_ref[1], o.hip_ref[2], o.hip_ref[3], o.hip_vel, o.wheel_free ? 1 : 0,
                        o.wheel_w[0], o.wheel_w[1], o.wheel_w[2], o.wheel_w[3],
                        FX[(int)o.light.waist_fx], COL[(int)o.light.waist], FX[(int)o.light.tail_fx], o.light.tail,
                        o.light.ring ? 1 : 0, o.light.front, o.light.dim);
            flush();
            if (c->status_due) {
                size_t n = write_status(c->status(last_hip), buf, sizeof buf);
                std::printf("E %.*s\n", (int)n, buf);
                c->status_due = false;
            }
        }
    }
    return 0;
}
