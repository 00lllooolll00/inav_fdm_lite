// bridge_selftest：零依赖自检（断言式）。
// 覆盖：FDM 姿态/运动与命令行配置解析。

#include <array>
#include <cmath>
#include <cstdio>

#include "config.hpp"
#include "fdm.hpp"
#include "xplane_link.hpp"

namespace
{
int g_fail = 0;
#define CHECK(cond, msg)                                              \
    do                                                                \
    {                                                                 \
        if (!(cond))                                                  \
        {                                                             \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
            ++g_fail;                                                 \
        }                                                             \
    } while (0)

// --- FDM ---
void test_fdm_hover()
{
    QuadFdm fdm(1.2, 30.0, 10.0);
    fdm.reset(30.0, 120.0, 10.0);
    // 悬停油门 = m*g/Tmax ≈ 0.392
    const double hov = 1.2 * 9.80665 / 30.0;
    std::array<double, 4> m{ hov, hov, hov, hov };
    SensorSnapshot s;
    for (int i = 0; i < 200; ++i) s = fdm.step(m, 0.02); // 4 秒
    CHECK(std::fabs(s.alt_m - 10.0) < 1.0, "hover alt");
    CHECK(std::fabs(s.g_nrml - 1.0) < 0.15, "hover load");
    CHECK(std::fabs(s.g_axil) < 0.05 && std::fabs(s.g_side) < 0.05, "hover level");
    CHECK(s.fix_type == 2 && s.num_sats == 16, "gps default");
}

void test_fdm_gyroscopic_coupling()
{
    QuadFdm fdm(1.2, 30.0, 10.0);
    fdm.reset(30.0, 120.0, 100.0);
    const std::array<double, 4> motors{ 0.292, 0.692, 0.292, 0.292 };
    fdm.step(motors, 0.1);
    const SensorSnapshot s = fdm.step(motors, 0.1);
    CHECK(s.gyro_q_dps - s.gyro_p_dps > 1.0, "rigid-body roll/pitch gyroscopic coupling direction");
}

void test_fdm_climb_and_ground()
{
    QuadFdm fdm(1.2, 30.0, 10.0);
    fdm.reset(30.0, 120.0, 10.0);
    std::array<double, 4> full{ 1, 1, 1, 1 };
    SensorSnapshot s;
    for (int i = 0; i < 50; ++i) s = fdm.step(full, 0.02); // 1 秒全油门
    CHECK(s.alt_m > 11.0, "climb");
    CHECK(s.local_vy > 0.0f, "climb velocity X-Plane sign");
    // 断油下坠，最终贴地不穿地
    std::array<double, 4> zero{ 0, 0, 0, 0 };
    for (int i = 0; i < 100; ++i) s = fdm.step(zero, 0.02);
    CHECK(s.local_vy < 0.0f, "descent velocity X-Plane sign");
    for (int i = 0; i < 400; ++i) s = fdm.step(zero, 0.02);
    CHECK(std::fabs(s.alt_m - 10.0) < 1e-6, "ground clamp");
    CHECK(s.agl_m == 0, "agl zero");
    CHECK(std::fabs(s.g_nrml - 1.0) < 0.05, "ground load 1g");
}

void test_fdm_attitude_response()
{
    QuadFdm fdm(1.2, 30.0, 10.0);
    fdm.reset(30.0, 120.0, 10.0, 90.0);
    // 右侧电机升力较大时左滚，roll 和 P 角速度应为负。
    std::array<double, 4> m{ 0.6, 0.4, 0.4, 0.6 };
    SensorSnapshot s;
    for (int i = 0; i < 10; ++i) s = fdm.step(m, 0.02);
    CHECK(s.phi_deg < -0.5, "right-side lift rolls left");
    CHECK(s.gyro_p_dps < -1.0, "left roll gyro sign");
    CHECK(std::fabs(s.psi_deg - 90.0) < 1e-6, "yaw remains at initial heading");
    const double mag_norm = std::sqrt(s.mag_x * s.mag_x + s.mag_y * s.mag_y + s.mag_z * s.mag_z);
    CHECK(std::fabs(mag_norm - 1.0) < 1e-6, "quaternion rotation remains normalized");
}

// --- 配置 ---
void test_config_defaults()
{
    char arg0[] = "prog";
    char *argv[] = { arg0 };
    BridgeConfig c;
    bool help = false;
    CHECK(parse_args(1, argv, c, help) == 0 && !help, "cfg default rc");
    CHECK(c.simport == 49000 && c.listen_port == 49000, "cfg ports");
    CHECK(std::fabs(c.rate_hz - 500.0) < 1e-9, "cfg default rate");
    CHECK(c.instance_id == 1, "cfg instance id");
    CHECK(std::fabs(c.arm_alt_m - c.init_alt) < 1e-9, "cfg arm-alt follow");
}

void test_config_multi()
{
    char a0[] = "prog", a1[] = "--id", a2[] = "2", a3[] = "-p", a4[] = "49003", a5[] = "--arm-alt", a6[] = "5.0",
         a7[] = "--lat", a8[] = "31.5";
    char *argv[] = { a0, a1, a2, a3, a4, a5, a6, a7, a8 };
    BridgeConfig c;
    bool help = false;
    CHECK(parse_args(9, argv, c, help) == 0, "cfg multi rc");
    CHECK(c.instance_id == 2 && c.simport == 49003 && c.listen_port == 49003, "cfg multi ports");
    CHECK(std::fabs(c.arm_alt_m - 5.0) < 1e-9, "cfg arm-alt explicit");
    CHECK(std::fabs(c.init_lat - 31.5) < 1e-9, "cfg lat");
}
} // namespace

int main()
{
    test_fdm_hover();
    test_fdm_gyroscopic_coupling();
    test_fdm_climb_and_ground();
    test_fdm_attitude_response();
    test_config_defaults();
    test_config_multi();
    if (g_fail == 0)
    {
        std::printf("SELFTEST PASS\n");
        return 0;
    }
    std::printf("SELFTEST FAIL: %d\n", g_fail);
    return 1;
}
