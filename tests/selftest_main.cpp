// bridge_selftest：零依赖自检（断言式）。
// 覆盖：FDM 姿态/运动与命令行配置解析。

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "config.hpp"
#include "fdm.hpp"
#include "msp_link.hpp"
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

// 水平平移：后侧电机推力略大 → 低头 → 机头朝北（yaw=0）时向北更新经纬度
void test_fdm_horizontal_translation()
{
    QuadFdm fdm(1.2, 30.0, 10.0);
    fdm.reset(30.0, 120.0, 100.0, 0.0);
    const double hov = 1.2 * 9.80665 / 30.0;
    const std::array<double, 4> m{ hov - 0.02, hov - 0.02, hov + 0.02, hov + 0.02 };
    for (int i = 0; i < 30; ++i) fdm.step(m, 0.02); // 0.6 秒
    CHECK(fdm.state().pitch_deg < 0.0, "rear thrust bias pitches nose down");
    CHECK(fdm.state().vn > 0.05, "nose-down gives north velocity");
    CHECK(fdm.state().lat_deg > 30.0, "latitude integrated north");
    CHECK(std::fabs(fdm.state().lon_deg - 120.0) < 1e-9, "no east drift on yaw=0");
}

// --- MSP ---
void test_msp_motor_frame()
{
    // MSP v2 响应：$X> + flags + cmd(u16) + len(u16=16) + 8×PWM + crc（解析不校验 crc）
    const uint8_t frame[] = {
        '$', 'X', '>',
        0,       // flags
        104, 0,  // cmd = 104（MSP_MOTOR）
        16, 0,   // len = 16（8 路，只取前 4 路）
        0xE8, 0x03, // 1000 → 0.00
        0xE8, 0x07, // 2000 → 1.00
        0xF4, 0x06, // 1780 → 0.78
        0xD0, 0x07, // 2000 → 1.00
        0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, // 后 4 路 1000
        0x00,       // crc
    };
    std::array<double, 4> m{};
    CHECK(mspParseMotorFrame(frame, sizeof(frame), m, nullptr), "msp frame parsed");
    CHECK(std::fabs(m[0] - 0.00) < 1e-9, "msp motor0 1000");
    CHECK(std::fabs(m[1] - 1.00) < 1e-9, "msp motor1 2000");
    CHECK(std::fabs(m[2] - 0.78) < 1e-9, "msp motor2 1780");
    CHECK(std::fabs(m[3] - 1.00) < 1e-9, "msp motor3 2000");

    std::array<double, 4> m2{};
    CHECK(!mspParseMotorFrame(frame, 8, m2, nullptr), "msp partial frame needs more bytes");

    uint8_t noisy[sizeof(frame) + 3] = { 0xAA, 0xBB, 0xCC };
    std::memcpy(noisy + 3, frame, sizeof(frame));
    std::array<double, 4> m3{};
    size_t used = 0;
    CHECK(mspParseMotorFrame(noisy, sizeof(noisy), m3, &used), "msp frame after noise");
    CHECK(std::fabs(m3[2] - 0.78) < 1e-9, "msp noisy motor2");
    CHECK(used == 3 + sizeof(frame), "msp consumed bytes");
}

// 电机顺序重排：INAV 序号 [右后,右前,左后,左前] → FDM [右前,左前,左后,右后]。
// 用实测 eeprom 的 Quad X 混控表生成电机输出（已含 INAV 对 yaw 的取反），重排后喂 FDM：
// 纯滚转/纯俯仰/纯偏航只能驱动对应轴。错序时纯滚转会变成纯俯仰——空中表现为绕圈。
void test_msp_motor_order()
{
    // 每行 = INAV 电机序号 M1..M4（右后/右前/左后/左前）对三轴单位命令的响应增益
    const double gain[4][3] = {
        { -1, +1, +1 }, // M1 右后
        { -1, -1, -1 }, // M2 右前
        { +1, +1, -1 }, // M3 左后
        { +1, -1, +1 }, // M4 左前
    };
    const double expect_sign[3] = { +1, -1, +1 }; // 右滚为正 / 后侧加力低头为负 / 右偏航为正
    for (size_t axis = 0; axis < 3; ++axis)
    {
        std::array<double, 4> msp;
        for (size_t i = 0; i < 4; ++i) msp[i] = 0.5 + 0.1 * gain[i][axis];
        QuadFdm fdm(1.2, 30.0, 10.0);
        fdm.reset(30.0, 120.0, 10.0);
        fdm.step(mspMotorsToFdmOrder(msp), 0.02);
        const SensorSnapshot s = fdm.step(mspMotorsToFdmOrder(msp), 0.02);
        const double rate[3] = { s.gyro_p_dps, s.gyro_q_dps, s.gyro_r_dps };
        CHECK(rate[axis] * expect_sign[axis] > 1.0, "msp order drives its own axis");
        for (size_t other = 0; other < 3; ++other)
        {
            if (other == axis) continue;
            CHECK(std::fabs(rate[other]) < 0.05 * std::fabs(rate[axis]), "msp order no cross-axis coupling");
        }
    }
}

// 解锁位图：位 6..30 是阻止原因（RX=18 在高位，MSP_STATUS_EX 的 u16 会把它丢掉），位 2 是 ARMED。
void test_arming_flags()
{
    CHECK(mspArmingBlockNames(0U).empty(), "no block -> empty");
    CHECK(mspArmingBlockNames(1U << 18) == "RX", "high block RX");
    CHECK(mspArmingBlockNames((1U << 9) | (1U << 18)) == "CAL,RX", "block names joined");
    CHECK((0x00040a20U & MSP_ARMING_BLOCK_MASK) == ((1U << 9) | (1U << 11) | (1U << 18)), "mask keeps 6..30");
    CHECK((0x00000004U & MSP_ARMING_ARMED) != 0U, "armed status bit");

    // MSP2_INAV_STATUS 响应：payload 里 9..12 是 armingFlags（小端）；解析必须按 cmd 过滤
    uint8_t frame[25] = { '$', 'X', '>', 0, 0x00, 0x20, 16, 0 };
    const uint32_t flags = 0x00040820U;
    for (int i = 0; i < 4; ++i) frame[8 + 9 + i] = (uint8_t)((flags >> (8 * i)) & 0xFF);
    const uint8_t *payload = nullptr;
    size_t plen = 0;
    CHECK(mspParseFrame(frame, sizeof(frame), 0x2000, &payload, &plen, nullptr), "status frame found");
    CHECK(plen == 16 && payload != nullptr && payload[11] == 0x04, "arming flags high byte");
    CHECK(!mspParseFrame(frame, sizeof(frame), 104, &payload, &plen, nullptr), "cmd filter rejects other query");
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
    test_fdm_horizontal_translation();
    test_msp_motor_frame();
    test_arming_flags();
    test_msp_motor_order();
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
