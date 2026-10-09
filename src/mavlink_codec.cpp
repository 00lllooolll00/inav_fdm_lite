#include "mavlink_codec.hpp"

#include <cstddef>

namespace mav {
namespace {
// X.25 CRC（MAVLink 御用）
uint16_t crc_acc(uint8_t b, uint16_t crc) {
    uint8_t tmp = static_cast<uint8_t>(b ^ (crc & 0xff));
    tmp ^= static_cast<uint8_t>(tmp << 4);
    return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                                 (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
}

// v2 载荷零截断：去掉尾部 0 字节（INAV/c_library 通行做法）
size_t trim_len(const uint8_t* p, size_t n) {
    while (n > 1 && p[n - 1] == 0) --n;
    return n;
}
}  // namespace

uint8_t crc_extra(uint32_t msgid) {
    switch (msgid) {
        case kMsgHeartbeat:
            return 50;
        case kMsgGpsRawInt:
            return 24;
        case kMsgAttitude:
            return 39;
        case kMsgGlobalPositionInt:
            return 104;
        case kMsgRcChannelsScaled:
            return 237;
        case kMsgRcChannels:
            return 118;
        case kMsgVfrHud:
            return 20;
        case kMsgCommandInt:
            return 158;
        case kMsgCommandLong:
            return 152;
        case kMsgCommandAck:
            return 143;
        case kMsgRcChannelsOverride:
            return 124;
        case kMsgStatustext:
            return 83;
        default:
            return 0;
    }
}

std::vector<uint8_t> frame_v2(uint8_t sysid, uint8_t compid, uint8_t seq, uint32_t msgid,
                              const uint8_t* payload, size_t len) {
    len = trim_len(payload, len);
    std::vector<uint8_t> f;
    f.reserve(len + 12);
    f.push_back(0xFD);
    f.push_back(static_cast<uint8_t>(len));
    f.push_back(0);  // incompat flags
    f.push_back(0);  // compat flags
    f.push_back(seq);
    f.push_back(sysid);
    f.push_back(compid);
    f.push_back(static_cast<uint8_t>(msgid & 0xff));
    f.push_back(static_cast<uint8_t>((msgid >> 8) & 0xff));
    f.push_back(static_cast<uint8_t>((msgid >> 16) & 0xff));
    f.insert(f.end(), payload, payload + len);
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < f.size(); ++i) crc = crc_acc(f[i], crc);
    crc = crc_acc(crc_extra(msgid), crc);
    f.push_back(static_cast<uint8_t>(crc & 0xff));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    return f;
}

void Parser::feed(const uint8_t* data, size_t n) {
    buf_.insert(buf_.end(), data, data + n);
    // 找帧头 0xFD（v2）/ 0xFE（v1，仅跳过不解析）
    while (true) {
        size_t start = 0;
        while (start < buf_.size() && buf_[start] != 0xFD && buf_[start] != 0xFE) ++start;
        if (start > 0) buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(start));
        if (buf_.empty()) return;
        if (buf_[0] == 0xFE) {
            // v1：len 在 byte1，总长 = len+8
            if (buf_.size() < 2) return;
            const size_t need = static_cast<size_t>(buf_[1]) + 8;
            if (buf_.size() < need) return;
            buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(need));  // 跳过 v1 帧
            continue;
        }
        // v2：header 10 字节
        if (buf_.size() < 10) return;
        const size_t plen = buf_[1];
        const bool signed_msg = (buf_[2] & 0x01) != 0;
        const size_t need = 10 + plen + 2 + (signed_msg ? 13 : 0);
        if (buf_.size() < need) return;
        // 校验 CRC
        uint16_t crc = 0xFFFF;
        for (size_t i = 1; i < 10 + plen; ++i) crc = crc_acc(buf_[i], crc);
        const uint32_t msgid = static_cast<uint32_t>(buf_[7]) | (static_cast<uint32_t>(buf_[8]) << 8) |
                               (static_cast<uint32_t>(buf_[9]) << 16);
        crc = crc_acc(crc_extra(msgid), crc);
        const uint16_t recvd = static_cast<uint16_t>(buf_[10 + plen]) |
                               (static_cast<uint16_t>(buf_[10 + plen + 1]) << 8);
        std::vector<uint8_t> payload;
        if (recvd == crc) {
            payload.assign(buf_.begin() + 10, buf_.begin() + static_cast<std::ptrdiff_t>(10 + plen));
            const uint8_t sysid = buf_[5], compid = buf_[6];
            buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(need));
            if (cb_) cb_(msgid, sysid, compid, payload);
        } else {
            buf_.erase(buf_.begin());  // CRC 错：滑窗 1 字节继续找
        }
    }
}

void put_u8(std::vector<uint8_t>& p, size_t off, uint8_t v) {
    if (off < p.size()) p[off] = v;
}
void put_u16(std::vector<uint8_t>& p, size_t off, uint16_t v) {
    if (off + 1 < p.size()) {
        p[off] = static_cast<uint8_t>(v);
        p[off + 1] = static_cast<uint8_t>(v >> 8);
    }
}
void put_i16(std::vector<uint8_t>& p, size_t off, int16_t v) { put_u16(p, off, static_cast<uint16_t>(v)); }
void put_u32(std::vector<uint8_t>& p, size_t off, uint32_t v) {
    if (off + 3 < p.size()) {
        p[off] = static_cast<uint8_t>(v);
        p[off + 1] = static_cast<uint8_t>(v >> 8);
        p[off + 2] = static_cast<uint8_t>(v >> 16);
        p[off + 3] = static_cast<uint8_t>(v >> 24);
    }
}
void put_i32(std::vector<uint8_t>& p, size_t off, int32_t v) { put_u32(p, off, static_cast<uint32_t>(v)); }
void put_f32(std::vector<uint8_t>& p, size_t off, float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    put_u32(p, off, u);
}

uint8_t get_u8(const std::vector<uint8_t>& p, size_t off) { return off < p.size() ? p[off] : 0; }
uint16_t get_u16(const std::vector<uint8_t>& p, size_t off) {
    // v2 零截断语义：缺失尾字节视为 0，允许部分读取
    uint16_t v = 0;
    if (off < p.size()) v |= p[off];
    if (off + 1 < p.size()) v = static_cast<uint16_t>(v | (static_cast<uint16_t>(p[off + 1]) << 8));
    return v;
}
int16_t get_i16(const std::vector<uint8_t>& p, size_t off) { return static_cast<int16_t>(get_u16(p, off)); }
uint32_t get_u32(const std::vector<uint8_t>& p, size_t off) {
    uint32_t v = 0;
    if (off < p.size()) v |= p[off];
    if (off + 1 < p.size()) v |= static_cast<uint32_t>(p[off + 1]) << 8;
    if (off + 2 < p.size()) v |= static_cast<uint32_t>(p[off + 2]) << 16;
    if (off + 3 < p.size()) v |= static_cast<uint32_t>(p[off + 3]) << 24;
    return v;
}
int32_t get_i32(const std::vector<uint8_t>& p, size_t off) { return static_cast<int32_t>(get_u32(p, off)); }
float get_f32(const std::vector<uint8_t>& p, size_t off) {
    uint32_t u = get_u32(p, off);
    float v;
    std::memcpy(&v, &u, 4);
    return v;
}

// --- 上行打包（偏移取自 c_library_v2 _mav_put_ 实测） ---

std::vector<uint8_t> pack_heartbeat_gcs() {
    std::vector<uint8_t> p(9, 0);
    put_u32(p, 0, 0);     // custom_mode
    put_u8(p, 4, 6);      // type = GCS
    put_u8(p, 5, 3);      // autopilot = INVALID(用 3 占位无妨，GCS 端忽略)
    put_u8(p, 6, 0);      // base_mode
    put_u8(p, 7, 0);      // system_status
    put_u8(p, 8, 3);      // mavlink_version
    return p;
}

std::vector<uint8_t> pack_command_int(uint8_t target_sys, uint16_t command, uint8_t frame, float p1,
                                     float p2, float p3, float p4, int32_t x, int32_t y, float z) {
    std::vector<uint8_t> p(35, 0);
    put_f32(p, 0, p1);
    put_f32(p, 4, p2);
    put_f32(p, 8, p3);
    put_f32(p, 12, p4);
    put_i32(p, 16, x);
    put_i32(p, 20, y);
    put_f32(p, 24, z);
    put_u16(p, 28, command);
    put_u8(p, 30, target_sys);
    put_u8(p, 31, 0);     // target_component（0=全部）
    put_u8(p, 32, frame);
    put_u8(p, 33, 0);     // current
    put_u8(p, 34, 1);     // autocontinue
    return p;
}

std::vector<uint8_t> pack_command_int_reposition(uint8_t target_sys, int32_t lat_e7, int32_t lon_e7,
                                                float alt_m, uint8_t frame) {
    return pack_command_int(target_sys, kCmdDoReposition, frame, -1, 0, 0, 0, lat_e7, lon_e7, alt_m);
}

std::vector<uint8_t> pack_command_long(uint8_t target_sys, uint16_t command, float p1, float p2,
                                      float p3, float p4, float p5, float p6, float p7) {
    std::vector<uint8_t> p(33, 0);
    put_f32(p, 0, p1);
    put_f32(p, 4, p2);
    put_f32(p, 8, p3);
    put_f32(p, 12, p4);
    put_f32(p, 16, p5);
    put_f32(p, 20, p6);
    put_f32(p, 24, p7);
    put_u16(p, 28, command);
    put_u8(p, 30, target_sys);
    put_u8(p, 31, 0);  // target_component
    put_u8(p, 32, 0);  // confirmation
    return p;
}

std::vector<uint8_t> pack_rc_override(uint8_t target_sys, const uint16_t ch[18]) {
    std::vector<uint8_t> p(38, 0);
    for (int i = 0; i < 8; ++i) put_u16(p, static_cast<size_t>(i) * 2, ch[i]);
    put_u8(p, 16, target_sys);
    put_u8(p, 17, 0);  // target_component
    for (int i = 8; i < 18; ++i) put_u16(p, static_cast<size_t>(i) * 2 + 2, ch[i]);
    return p;
}

// --- 下行解码 ---

bool decode_into(uint32_t msgid, const std::vector<uint8_t>& payload, uint8_t src_sysid, Telemetry& t) {
    switch (msgid) {
        case kMsgHeartbeat:
            if (payload.size() < 9) return false;
            t.has_heartbeat = true;
            t.fc_sysid = src_sysid;
            t.custom_mode = get_u32(payload, 0);
            t.autopilot = get_u8(payload, 5);
            t.base_mode = get_u8(payload, 6);
            t.armed = (t.base_mode & 0x80) != 0;
            return true;
        case kMsgGlobalPositionInt:
            if (payload.size() < 26) return false;  // hdg 允许被零截断
            t.has_gpi = true;
            t.lat_e7 = get_i32(payload, 4);
            t.lon_e7 = get_i32(payload, 8);
            t.alt_mm = get_i32(payload, 12);
            t.rel_alt_mm = get_i32(payload, 16);
            t.vx_cms = get_i16(payload, 20);
            t.vy_cms = get_i16(payload, 22);
            t.vz_cms = get_i16(payload, 24);
            t.hdg_cdeg = get_u16(payload, 26);
            return true;
        case kMsgGpsRawInt:
            if (payload.size() < 30) return false;
            t.has_gps_raw = true;
            t.fix_type = get_u8(payload, 28);
            t.sats = get_u8(payload, 29);
            return true;
        case kMsgVfrHud:
            if (payload.size() < 18) return false;  // throttle 允许被截断
            t.has_vfr = true;
            t.airspeed = get_f32(payload, 0);
            t.groundspeed = get_f32(payload, 4);
            t.alt_msl = get_f32(payload, 8);
            t.climb = get_f32(payload, 12);
            t.heading = get_i16(payload, 16);
            t.throttle = get_u16(payload, 18);
            return true;
        case kMsgAttitude:
            if (payload.size() < 28) return false;
            t.has_attitude = true;
            t.roll = get_f32(payload, 4);
            t.pitch = get_f32(payload, 8);
            t.yaw = get_f32(payload, 12);
            return true;
        case kMsgCommandAck:
            if (payload.size() < 3) return false;
            t.has_cmd_ack = true;
            t.ack_cmd = get_u16(payload, 0);
            t.ack_result = get_u8(payload, 2);
            return true;
        case kMsgRcChannels:
            if (payload.size() < 22) return false;  // 至少前 9 通道
            t.has_rc = true;
            for (size_t i = 0; i < 18; ++i) t.rc_ch[i] = get_u16(payload, 4 + i * 2);
            return true;
        case kMsgRcChannelsScaled:
            // scaled(-10000..10000)→PWM 近似，仅当前 8 通道有意义
            if (payload.size() < 20) return false;
            t.has_rc = true;
            for (size_t i = 0; i < 8; ++i) {
                int32_t sc = get_i16(payload, 4 + i * 2);
                int32_t pwm = 1500 + sc / 20;  // ±10000 → ±500
                t.rc_ch[i] = static_cast<uint16_t>(pwm < 0 ? 0 : pwm);
            }
            return true;
        default:
            return false;
    }
}

const char* copter_mode_name(uint32_t custom_mode) {
    // ArduPilot Copter mode 编号（INAV 多旋翼 custom_mode 映射）
    switch (custom_mode) {
        case 0:
            return "STABILIZE";
        case 1:
            return "ACRO";
        case 2:
            return "ALT_HOLD";
        case 3:
            return "AUTO";
        case 4:
            return "GUIDED";
        case 5:
            return "LOITER";
        case 6:
            return "RTL";
        case 7:
            return "CIRCLE";
        case 9:
            return "LAND";
        case 13:
            return "SPORT";
        case 16:
            return "POSHOLD";
        case 17:
            return "BRAKE";
        case 18:
            return "THROW";
        default:
            return "UNKNOWN";
    }
}

}  // namespace mav
