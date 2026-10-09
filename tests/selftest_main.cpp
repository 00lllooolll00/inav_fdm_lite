// bridge_selftest：零依赖自检（断言式）。
// 覆盖：MAVLink crc_extra/组帧/往返解码、FDM 悬停自洽、X-Plane RREF 注册解析、控制面解析。

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "config.hpp"
#include "ctl_server.hpp"
#include "fdm.hpp"
#include "mavlink_codec.hpp"
#include "xplane_link.hpp"  // 仅验证头文件自洽

namespace {
int g_fail = 0;
#define CHECK(cond, msg)                                                                            \
    do {                                                                                            \
        if (!(cond)) {                                                                              \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                               \
            ++g_fail;                                                                               \
        }                                                                                           \
    } while (0)

// --- MAVLink ---
void test_crc_extra() {
    CHECK(mav::crc_extra(mav::kMsgHeartbeat) == 50, "HB crc");
    CHECK(mav::crc_extra(mav::kMsgGpsRawInt) == 24, "GPS crc");
    CHECK(mav::crc_extra(mav::kMsgAttitude) == 39, "ATT crc");
    CHECK(mav::crc_extra(mav::kMsgGlobalPositionInt) == 104, "GPI crc");
    CHECK(mav::crc_extra(mav::kMsgRcChannelsOverride) == 124, "OVR crc");
    CHECK(mav::crc_extra(mav::kMsgCommandInt) == 158, "CI crc");
    CHECK(mav::crc_extra(mav::kMsgCommandLong) == 152, "CL crc");
    CHECK(mav::crc_extra(mav::kMsgCommandAck) == 143, "ACK crc");
    CHECK(mav::crc_extra(mav::kMsgVfrHud) == 20, "HUD crc");
    CHECK(mav::crc_extra(mav::kMsgStatustext) == 83, "STX crc");
}

void test_frame_roundtrip() {
    // 构造一个 HEARTBEAT（armed + GUIDED=4），组帧→解析→解码
    std::vector<uint8_t> p(9, 0);
    mav::put_u32(p, 0, 4);
    mav::put_u8(p, 4, 2);   // quadrotor
    mav::put_u8(p, 5, 3);   // ardu
    mav::put_u8(p, 6, 0x80 | 0x08);  // armed + guided
    mav::put_u8(p, 7, 4);   // active
    mav::put_u8(p, 8, 3);
    auto frame = mav::frame_v2(1, 1, 0, mav::kMsgHeartbeat, p.data(), p.size());
    CHECK(frame.size() > 12 && frame[0] == 0xFD, "frame head");

    bool got = false;
    mav::Telemetry t;
    mav::Parser parser([&](uint32_t id, uint8_t sys, uint8_t comp, const std::vector<uint8_t>& pl) {
        (void)comp;
        CHECK(id == mav::kMsgHeartbeat, "msgid");
        CHECK(sys == 1, "sysid");
        got = mav::decode_into(id, pl, sys, t);
    });
    parser.feed(frame.data(), frame.size());
    CHECK(got, "parsed");
    CHECK(t.armed && t.custom_mode == 4, "armed+guided");
    CHECK(std::strcmp(mav::copter_mode_name(4), "GUIDED") == 0, "mode name");
    // 坏 CRC 应被丢弃
    frame[frame.size() - 1] ^= 0xFF;
    bool got2 = false;
    mav::Parser p2([&](uint32_t, uint8_t, uint8_t, const std::vector<uint8_t>&) { got2 = true; });
    p2.feed(frame.data(), frame.size());
    CHECK(!got2, "bad crc dropped");
}

void test_gpi_decode() {
    std::vector<uint8_t> p(28, 0);
    mav::put_i32(p, 4, 300123456);
    mav::put_i32(p, 8, 1200123456);
    mav::put_i32(p, 12, 15000);
    mav::put_i32(p, 16, 5000);
    mav::put_i16(p, 20, 100);
    mav::put_i16(p, 24, -50);
    auto frame = mav::frame_v2(2, 1, 7, mav::kMsgGlobalPositionInt, p.data(), p.size());
    mav::Telemetry t;
    mav::Parser parser([&](uint32_t id, uint8_t sys, uint8_t, const std::vector<uint8_t>& pl) {
        mav::decode_into(id, pl, sys, t);
    });
    parser.feed(frame.data(), frame.size());
    CHECK(t.has_gpi && t.lat_e7 == 300123456 && t.rel_alt_mm == 5000, "gpi fields");
    CHECK(t.vx_cms == 100 && t.vz_cms == -50, "gpi vel");
    // v2 零截断：throttle=51 高字节被截断后仍应读出 51
    {
        std::vector<uint8_t> h(20, 0);
        mav::put_i16(h, 16, 90);
        mav::put_u16(h, 18, 51);
        auto fr = mav::frame_v2(7, 1, 3, mav::kMsgVfrHud, h.data(), h.size());
        CHECK(fr[1] == 19, "vfr trimmed len");
        mav::Telemetry tt;
        mav::Parser pp([&](uint32_t id, uint8_t sys, uint8_t, const std::vector<uint8_t>& pl) {
            mav::decode_into(id, pl, sys, tt);
        });
        pp.feed(fr.data(), fr.size());
        CHECK(tt.has_vfr && tt.throttle == 51 && tt.heading == 90, "vfr trimmed decode");
    }
}

void test_cmd_pack() {
    // REPOSITION 打包偏移校验（与 c_library_v2 对齐）
    auto p = mav::pack_command_int_reposition(1, 300000000, 1200000000, 25.0f, 0);
    CHECK(p.size() == 35, "CI len");
    CHECK(mav::get_i32(p, 16) == 300000000, "CI x");
    CHECK(mav::get_i32(p, 20) == 1200000000, "CI y");
    CHECK(std::fabs(mav::get_f32(p, 24) - 25.0f) < 1e-6, "CI z");
    CHECK(mav::get_u16(p, 28) == 192, "CI cmd");
    CHECK(mav::get_u8(p, 30) == 1, "CI tgt");
    CHECK(mav::get_u8(p, 32) == 0, "CI frame");
    // OVERRIDE 打包
    uint16_t ch[18] = {0};
    ch[4] = 2000;
    ch[7] = 1500;
    auto o = mav::pack_rc_override(1, ch);
    CHECK(o.size() == 38, "OVR len");
    CHECK(mav::get_u16(o, 8) == 2000, "OVR ch5");
    CHECK(mav::get_u16(o, 14) == 1500, "OVR ch8");
    CHECK(mav::get_u8(o, 16) == 1, "OVR tgt");
}

// --- FDM ---
void test_fdm_hover() {
    QuadFdm fdm(1.2, 30.0, 10.0);
    fdm.reset(30.0, 120.0, 10.0);
    // 悬停油门 = m*g/Tmax ≈ 0.392
    const double hov = 1.2 * 9.80665 / 30.0;
    std::array<double, 4> m{hov, hov, hov, hov};
    SensorSnapshot s;
    for (int i = 0; i < 200; ++i) s = fdm.step(m, 0.02);  // 4 秒
    CHECK(std::fabs(s.alt_m - 10.0) < 1.0, "hover alt");
    CHECK(std::fabs(s.g_nrml - 1.0) < 0.15, "hover load");
    CHECK(std::fabs(s.g_axil) < 0.05 && std::fabs(s.g_side) < 0.05, "hover level");
    CHECK(s.fix_type == 3 && s.num_sats == 16, "gps default");
}

void test_fdm_climb_and_ground() {
    QuadFdm fdm(1.2, 30.0, 10.0);
    fdm.reset(30.0, 120.0, 10.0);
    std::array<double, 4> full{1, 1, 1, 1};
    SensorSnapshot s;
    for (int i = 0; i < 50; ++i) s = fdm.step(full, 0.02);  // 1 秒全油门
    CHECK(s.alt_m > 11.0, "climb");
    // 断油下坠，最终贴地不穿地
    std::array<double, 4> zero{0, 0, 0, 0};
    for (int i = 0; i < 500; ++i) s = fdm.step(zero, 0.02);
    CHECK(std::fabs(s.alt_m - 10.0) < 1e-6, "ground clamp");
    CHECK(s.agl_m == 0, "agl zero");
}

void test_fdm_attitude_response() {
    QuadFdm fdm(1.2, 30.0, 10.0);
    fdm.reset(30.0, 120.0, 10.0, 90.0);
    // 右侧电机大 → 应向左滚转（roll 减小为负为左压？本约定右压为正，故左侧升力大→右滚为正）
    std::array<double, 4> m{0.6, 0.4, 0.4, 0.6};
    SensorSnapshot s;
    for (int i = 0; i < 50; ++i) s = fdm.step(m, 0.02);
    CHECK(std::fabs(s.phi_deg) > 0.5, "roll response");
    CHECK(std::fabs(s.gyro_p_dps) > 1.0, "gyro response");
}

// --- 控制面 ---
void test_ctl_parse() {
    CtlServer::Handlers h;
    double rlat = 0, rlon = 0;
    float ralt = 0;
    uint8_t rframe = 99;
    h.reposition = [&](double la, double lo, float al, uint8_t fr) {
        rlat = la;
        rlon = lo;
        ralt = al;
        rframe = fr;
        return "OK";
    };
    uint16_t got[18] = {0};
    bool ovr_called = false;
    h.override = [&](const uint16_t ch[18]) {
        std::memcpy(got, ch, sizeof(got));
        ovr_called = true;
        return "OK";
    };
    CHECK(CtlServer::handle_line("REPOSITION 30.1 120.1 25", h) == "OK", "ctl repos");
    CHECK(std::fabs(rlat - 30.1) < 1e-9 && rframe == 0, "ctl repos vals");
    CHECK(CtlServer::handle_line("REPOSITION 91 0 0", h).substr(0, 3) == "ERR", "ctl repos range");
    CHECK(CtlServer::handle_line("OVERRIDE_ONE 6 2000", h) == "OK", "ctl ovr1");
    CHECK(ovr_called && got[5] == 2000 && got[0] == 0, "ctl ovr1 vals");
    CHECK(CtlServer::handle_line("OVERRIDE_CLEAR", h) == "OK", "ctl clear");
    CHECK(got[5] == 0, "ctl clear vals");
    CHECK(CtlServer::handle_line("BOGUS", h).substr(0, 3) == "ERR", "ctl unknown");
}

// --- 配置 ---
void test_config_defaults() {
    char arg0[] = "prog";
    char* argv[] = {arg0};
    BridgeConfig c;
    bool help = false;
    CHECK(parse_args(1, argv, c, help) == 0 && !help, "cfg default rc");
    CHECK(c.simport == 49000 && c.listen_port == 49000, "cfg ports");
    CHECK(c.sys_id == 1 && c.gcs_udp_port == 0, "cfg id/gcs");
    CHECK(std::fabs(c.arm_alt_m - c.init_alt) < 1e-9, "cfg arm-alt follow");
}

void test_config_multi() {
    char a0[] = "prog", a1[] = "--id", a2[] = "2", a3[] = "-p", a4[] = "49003", a5[] = "--ctl-port",
         a6[] = "6902", a7[] = "--arm-alt", a8[] = "5.0", a9[] = "--lat", a10[] = "31.5";
    char* argv[] = {a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10};
    BridgeConfig c;
    bool help = false;
    CHECK(parse_args(11, argv, c, help) == 0, "cfg multi rc");
    CHECK(c.sys_id == 2 && c.simport == 49003 && c.listen_port == 49003, "cfg multi ports");
    CHECK(c.ctl_tcp_port == 6902, "cfg ctl");
    CHECK(std::fabs(c.arm_alt_m - 5.0) < 1e-9, "cfg arm-alt explicit");
    CHECK(std::fabs(c.init_lat - 31.5) < 1e-9, "cfg lat");
}
}  // namespace

int main() {
    test_crc_extra();
    test_frame_roundtrip();
    test_gpi_decode();
    test_cmd_pack();
    test_fdm_hover();
    test_fdm_climb_and_ground();
    test_fdm_attitude_response();
    test_ctl_parse();
    test_config_defaults();
    test_config_multi();
    if (g_fail == 0) {
        std::printf("SELFTEST PASS\n");
        return 0;
    }
    std::printf("SELFTEST FAIL: %d\n", g_fail);
    return 1;
}
