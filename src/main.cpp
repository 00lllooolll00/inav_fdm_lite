// inav_sitl_bridge：INAV SITL 无头 6DOF 飞行动力学后端 + MAVLink 闭环测试桥。
// 链路 1（UDP）：X-Plane RREF/DREF 协议，50Hz 注入传感器、读电机推演位姿。
// 链路 2（TCP）：MAVLink v2，收遥测、发测试指令、可选转发 QGC。

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

#include "config.hpp"
#include "ctl_server.hpp"
#include "fdm.hpp"
#include "mavlink_link.hpp"
#include "xplane_link.hpp"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }
}  // namespace

int main(int argc, char** argv) {
    BridgeConfig cfg;
    bool help_only = false;
    if (parse_args(argc, argv, cfg, help_only) != 0) return 2;
    if (help_only) {
        print_usage(argv[0]);
        return 0;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    auto log = [&](const std::string& m) {
        if (cfg.verbose) std::printf("[UAV %u] %s\n", cfg.sys_id, m.c_str());
        std::fflush(stdout);
    };

    // 动力学
    QuadFdm fdm(cfg.mass_kg, cfg.max_thrust_n, cfg.arm_alt_m);
    fdm.reset(cfg.init_lat, cfg.init_lon, cfg.init_alt);

    // UDP 链路
    XplaneLink xp(cfg.listen_port, cfg.sitl_ip, cfg.simport);
    if (!xp.start([&](const std::string& m) { log("[XP] " + m); })) {
        std::fprintf(stderr, "[UAV %u] UDP 监听 %u 失败\n", cfg.sys_id, cfg.listen_port);
        return 1;
    }

    // MAVLink 链路
    MavlinkLink mav(cfg.mav_tcp_ip, cfg.mav_tcp_port, cfg.sys_id, cfg.gcs_udp_port);
    mav.start([&](const std::string& m) { log("[MAV] " + m); });

    // 控制面
    CtlServer::Handlers handlers;
    handlers.reposition = [&](double lat, double lon, float alt, uint8_t frame) {
        mav.send_reposition(static_cast<int32_t>(lat * 1e7), static_cast<int32_t>(lon * 1e7), alt, frame);
        return "OK REPOSITION sent";
    };
    handlers.override = [&](const uint16_t ch[18]) {
        mav.send_rc_override(ch);
        return "OK OVERRIDE sent (held @50Hz)";
    };
    handlers.clear = [&]() {
        mav.clear_rc_override();
        return "OK OVERRIDE cleared";
    };
    handlers.cmdint = [&](uint16_t cmd, uint8_t frame, float p1, float p2, float p3, float p4,
                          int32_t x, int32_t y, float z) {
        mav.send_command_int(cmd, frame, p1, p2, p3, p4, x, y, z);
        return "OK CMDINT sent";
    };
    handlers.cmdlong = [&](uint16_t cmd, float p1, float p2, float p3, float p4, float p5, float p6,
                           float p7) {
        mav.send_command_long(cmd, p1, p2, p3, p4, p5, p6, p7);
        return "OK CMDLONG sent";
    };
    handlers.arm = [&](bool arm) {
        mav.send_arm(arm);
        return arm ? "OK ARM sent" : "OK DISARM sent";
    };
    CtlServer ctl(cfg.ctl_tcp_port, handlers);
    if (cfg.ctl_tcp_port != 0 && !ctl.start()) {
        std::fprintf(stderr, "[UAV %u] 控制面端口 %u 监听失败\n", cfg.sys_id, cfg.ctl_tcp_port);
        return 1;
    }

    std::printf("[UAV %u] 启动: FDM %.1fHz @(%f,%f,%.1fm) XP-udp=%u MAVLink=%s:%u%s%s\n", cfg.sys_id,
                cfg.rate_hz, cfg.init_lat, cfg.init_lon, cfg.init_alt, cfg.listen_port,
                cfg.mav_tcp_ip.c_str(), cfg.mav_tcp_port,
                cfg.gcs_udp_port ? (" GCS-fwd=" + std::to_string(cfg.gcs_udp_port)).c_str() : "",
                cfg.ctl_tcp_port ? (" CTL=" + std::to_string(cfg.ctl_tcp_port)).c_str() : "");
    std::fflush(stdout);

    // 主循环：定频步进
    const auto period = std::chrono::duration<double>(1.0 / cfg.rate_hz);
    auto next = std::chrono::steady_clock::now() + period;
    auto last_status = std::chrono::steady_clock::now();
    std::array<double, 4> motors{0, 0, 0, 0};

    while (!g_stop) {
        std::this_thread::sleep_until(next);
        next += period;
        // 掉队保护：落后超过 5 周期则追平
        auto now = std::chrono::steady_clock::now();
        if (now - next > period * 5) next = now + period;

        std::array<double, 4> m;
        if (xp.get_motors(m)) motors = m;
        SensorSnapshot s = fdm.step(motors, period.count());
        xp.send_sensors(s);

        // 1Hz 状态行
        if (now - last_status >= std::chrono::seconds(1)) {
            last_status = now;
            mav::Telemetry t = mav.telemetry();
            const char* mode = t.has_heartbeat ? mav::copter_mode_name(t.custom_mode) : "--";
            const char* armed = t.has_heartbeat ? (t.armed ? "YES" : "NO") : "--";
            double alt = t.has_gpi ? t.rel_alt_mm / 1000.0 : (s.alt_m - cfg.arm_alt_m);
            int sats = t.has_gps_raw ? t.sats : -1;
            // INAV 用 i16 打包 u16 油门：未解锁/无 RC 时为负（回绕成 65xxx），显示钳位
            int thr_raw = t.has_vfr ? static_cast<int16_t>(t.throttle) : static_cast<int>(fdm.avg_throttle() * 100);
            int thr = thr_raw < 0 ? 0 : (thr_raw > 100 ? 100 : thr_raw);
            const char* mavst = mav.connected() ? "CONNECTED" : "WAIT";
            const char* xpst = xp.connected() ? "CONNECTED" : "WAIT";
            char satsbuf[16];
            if (sats < 0)
                std::snprintf(satsbuf, sizeof(satsbuf), "--");
            else
                std::snprintf(satsbuf, sizeof(satsbuf), "%d", sats);
            std::printf("[UAV %u] Mode: %s | Armed: %s | Alt: %.1fm | Sats: %s | Throttle: %d%% | XP: %s | MAVLink: %s\n",
                        cfg.sys_id, mode, armed, alt, satsbuf, thr, xpst, mavst);
            if (cfg.verbose && t.has_rc) {
                std::printf("  RC回读: %u %u %u %u %u %u %u %u\n", t.rc_ch[0], t.rc_ch[1], t.rc_ch[2],
                            t.rc_ch[3], t.rc_ch[4], t.rc_ch[5], t.rc_ch[6], t.rc_ch[7]);
            }
            if (cfg.verbose) {
                std::printf("  FDM: mot=%.2f %.2f %.2f %.2f alt=%.1f agl=%.1f vn=%.1f ve=%.1f vd=%.1f\n",
                            motors[0], motors[1], motors[2], motors[3], fdm.state().alt_m,
                            s.agl_m, fdm.state().vn, fdm.state().ve, fdm.state().vd);
                std::printf("  POS: lat=%.7f lon=%.7f fc=%d,%d\n", fdm.state().lat_deg,
                            fdm.state().lon_deg, t.lat_e7, t.lon_e7);
            }
            std::fflush(stdout);
        }
    }

    std::printf("\n[UAV %u] 退出\n", cfg.sys_id);
    ctl.stop();
    mav.stop();
    xp.stop();
    return 0;
}
