// inav_sitl_bridge：通过 X-Plane UDP 为 INAV SITL 注入传感器并读取电机输出，驱动本地 FDM。

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <thread>

#include "config.hpp"
#include "fdm.hpp"
#include "msp_link.hpp"
#include "xplane_link.hpp"

namespace
{
std::atomic<bool> g_stop{ false };
void on_signal(int)
{
    g_stop = true;
}

// 与 临时 MSP 工具 的约定：该标志文件存在（且不太旧）时让出 MSP 口。
// SITL 每个 UART 只服务一个客户端，桥长期占用会饿死其它工具（见 msp_link.hpp）。
bool mspYieldRequested(uint16_t msp_port)
{
    char path[64];
    std::snprintf(path, sizeof(path), "/tmp/maplrs_msp_yield_%u", static_cast<unsigned>(msp_port));
    struct stat st{};
    if (::stat(path, &st) != 0) return false;
    return (std::time(nullptr) - st.st_mtime) <= 60; // 60 秒 TTL，防工具被强杀后卡死
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
    MspLink msp(cfg.sitl_ip, cfg.msp_port);
    unsigned msp_tick = 0;
    // 解锁状态监测：seen 锁存本段出现过的原因（恢复正常后清零重新计）
    uint32_t arming_seen = 0U;
    bool arming_first_ok = false;
    bool armed_prev = false;

    while (!g_stop)
    {
        std::this_thread::sleep_until(next);
        next += period;
        // 掉队保护：落后超过 5 周期则追平
        auto now = std::chrono::steady_clock::now();
        if (now - next > period * 5) next = now + period;

        std::array<double, 4> m;
        if (cfg.msp_port != 0)
        {
            // MSP 模式：50Hz 轮询电机真值（TCP 往返不宜每周期做），其余周期沿用上次结果。
            // X-Plane 通道只有总油门 + 舵面，多旋翼姿态差动传不出来，只能走 MSP。
            ++msp_tick;
            if (msp_tick % 100U == 0) // 5Hz 查让位标志（临时 MSP 工具 等工具占用时）
            {
                const bool yield = mspYieldRequested(cfg.msp_port);
                if (yield != msp.paused())
                {
                    std::printf("[UAV %u] MSP %s\n", cfg.instance_id,
                                yield ? "让位：暂停轮询，把口让给工具" : "恢复轮询");
                    std::fflush(stdout);
                    msp.set_paused(yield);
                }
            }
            if (msp_tick % 10U == 0 && msp.poll_motors(m)) motors = mspMotorsToFdmOrder(m);
            // 解锁状态：位 6 起是阻止解锁原因（CAL/RX/OVRLD/NAV…），位 2 是 ARMED。
            // 与电机轮询错开半拍轮询同一连接，逐位锁存——瞬时原因（等 RC 帧、校准）也不会漏。
            if (msp_tick % 10U == 5U)
            {
                uint32_t flags = 0;
                if (msp.poll_status(flags))
                {
                    const uint32_t blocks = flags & MSP_ARMING_BLOCK_MASK;
                    const bool armed = (flags & MSP_ARMING_ARMED) != 0U;
                    if (armed != armed_prev)
                    {
                        armed_prev = armed;
                        std::printf("[UAV %u] %s\n", cfg.instance_id, armed ? "ARMED" : "DISARMED");
                        std::fflush(stdout);
                    }
                    if ((blocks & ~arming_seen) != 0U)
                    {
                        arming_seen |= blocks;
                        std::printf("[UAV %u] 解锁受阻: %s\n", cfg.instance_id, mspArmingBlockNames(blocks).c_str());
                        std::fflush(stdout);
                    }
                    else if (blocks == 0U && arming_seen != 0U)
                    {
                        std::printf("[UAV %u] 解锁条件恢复（曾受阻: %s）\n", cfg.instance_id,
                                    mspArmingBlockNames(arming_seen).c_str());
                        std::fflush(stdout);
                        arming_seen = 0U;
                    }
                    if (blocks == 0U && !arming_first_ok)
                    {
                        std::printf("[UAV %u] 解锁状态: 正常\n", cfg.instance_id);
                        std::fflush(stdout);
                    }
                    arming_first_ok = true;
                }
            }
        }
        else if (xp.get_motors(m))
        {
            motors = m; // X-Plane 回退路径（总油门 + 舵面重构）
        }
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
