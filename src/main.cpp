// inav_sitl_bridge：通过 X-Plane UDP 为 INAV SITL 注入传感器并读取电机输出，驱动本地 FDM。

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "config.hpp"
#include "fdm.hpp"
#include "xplane_link.hpp"

namespace
{
std::atomic<bool> g_stop{ false };
void on_signal(int)
{
    g_stop = true;
}
} // namespace

int main(int argc, char **argv)
{
    BridgeConfig cfg;
    bool help_only = false;
    if (parse_args(argc, argv, cfg, help_only) != 0) return 2;
    if (help_only)
    {
        print_usage(argv[0]);
        return 0;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    auto log = [&](const std::string &m)
    {
        if (cfg.verbose) std::printf("[UAV %u] %s\n", cfg.instance_id, m.c_str());
        std::fflush(stdout);
    };

    // 动力学
    QuadFdm fdm(cfg.mass_kg, cfg.max_thrust_n, cfg.arm_alt_m);
    fdm.reset(cfg.init_lat, cfg.init_lon, cfg.init_alt);

    // UDP 链路
    XplaneLink xp(cfg.listen_port, cfg.sitl_ip, cfg.simport);
    if (!xp.start([&](const std::string &m) { log("[XP] " + m); }))
    {
        std::fprintf(stderr, "[UAV %u] UDP 监听 %u 失败\n", cfg.instance_id, cfg.listen_port);
        return 1;
    }

    std::printf("[UAV %u] 启动: FDM %.1fHz @(%f,%f,%.1fm) XP-udp=%u\n",
                cfg.instance_id,
                cfg.rate_hz,
                cfg.init_lat,
                cfg.init_lon,
                cfg.init_alt,
                cfg.listen_port);
    std::fflush(stdout);

    // 主循环：定频步进
    const auto period = std::chrono::duration<double>(1.0 / cfg.rate_hz);
    auto next = std::chrono::steady_clock::now() + period;
    auto last_status = std::chrono::steady_clock::now();
    std::array<double, 4> motors{ 0, 0, 0, 0 };

    while (!g_stop)
    {
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
        if (now - last_status >= std::chrono::seconds(1))
        {
            last_status = now;
            const char *xpst = xp.connected() ? "CONNECTED" : "WAIT";
            std::printf("[UAV %u] Alt: %.1fm | Motor: %.0f%% | XP: %s\n",
                        cfg.instance_id,
                        fdm.state().alt_m,
                        fdm.avg_throttle() * 100.0,
                        xpst);
            if (cfg.verbose)
            {
                std::printf("  FDM: mot=%.2f %.2f %.2f %.2f agl=%.1f vn=%.1f ve=%.1f vd=%.1f\n",
                            motors[0],
                            motors[1],
                            motors[2],
                            motors[3],
                            s.agl_m,
                            fdm.state().vn,
                            fdm.state().ve,
                            fdm.state().vd);
                std::printf("  POS: lat=%.7f lon=%.7f\n", fdm.state().lat_deg, fdm.state().lon_deg);
            }
            std::fflush(stdout);
        }
    }

    std::printf("\n[UAV %u] 退出\n", cfg.instance_id);
    xp.stop();
    return 0;
}
