#pragma once

// 最小 MAVLink v2 编解码（零依赖，CRC 种子取自 c_library_v2）。
// 只覆盖本桥需要的消息；新增消息按表追加即可。

#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

namespace mav {

// --- 消息 ID ---
constexpr uint32_t kMsgHeartbeat = 0;
constexpr uint32_t kMsgGpsRawInt = 24;
constexpr uint32_t kMsgAttitude = 30;
constexpr uint32_t kMsgGlobalPositionInt = 33;
constexpr uint32_t kMsgRcChannels = 65;
constexpr uint32_t kMsgVfrHud = 74;
constexpr uint32_t kMsgCommandInt = 75;
constexpr uint32_t kMsgCommandLong = 76;
constexpr uint32_t kMsgCommandAck = 77;
constexpr uint32_t kMsgRcChannelsOverride = 70;
constexpr uint32_t kMsgStatustext = 253;

// --- crc_extra（c_library_v2 实测） ---
uint8_t crc_extra(uint32_t msgid);  // 未知消息返回 0

// --- 常量 ---
constexpr uint16_t kCmdDoReposition = 192;
constexpr uint16_t kCmdComponentArmDisarm = 400;
constexpr uint8_t kFrameGlobal = 0;
constexpr uint8_t kFrameGlobalRelativeAlt = 3;
constexpr uint8_t kRcOverrideIgnore = 0;  // INAV 把 0 / UINT16_MAX 视为忽略
constexpr uint16_t kRcOverrideRelease = 0;

// MAV_RESULT
constexpr uint8_t kAckAccepted = 0;
constexpr uint8_t kAckDenied = 2;
constexpr uint8_t kAckUnsupported = 3;

// 解码后的遥测快照（INAV 发什么存什么）
struct Telemetry {
    bool has_heartbeat = false;
    uint32_t custom_mode = 0;
    uint8_t base_mode = 0;  // bit7 = armed
    bool armed = false;
    uint8_t autopilot = 0;  // 3=ArduPilot 映射时用于区分机型

    bool has_gpi = false;
    int32_t lat_e7 = 0, lon_e7 = 0, alt_mm = 0, rel_alt_mm = 0;
    int16_t vx_cms = 0, vy_cms = 0, vz_cms = 0;
    uint16_t hdg_cdeg = 0;

    bool has_gps_raw = false;
    uint8_t fix_type = 0, sats = 0;

    bool has_vfr = false;
    float airspeed = 0, groundspeed = 0, alt_msl = 0, climb = 0;
    int16_t heading = 0;
    uint16_t throttle = 0;  // 0..100

    bool has_attitude = false;
    float roll = 0, pitch = 0, yaw = 0;

    bool has_cmd_ack = false;
    uint16_t ack_cmd = 0;
    uint8_t ack_result = 255;

    uint8_t fc_sysid = 0;  // 对端飞控 sysid
};

// v2 组帧：payload 裸包 → 完整帧（含 seq 自增由调用方维护）
std::vector<uint8_t> frame_v2(uint8_t sysid, uint8_t compid, uint8_t seq, uint32_t msgid,
                              const uint8_t* payload, size_t len);

// 逐字节解析器：喂流，完整帧回调（msg_id, sysid, compid, payload）
class Parser {
  public:
    using FrameCb = std::function<void(uint32_t, uint8_t, uint8_t, const std::vector<uint8_t>&)>;
    explicit Parser(FrameCb cb) : cb_(std::move(cb)) {}
    void feed(const uint8_t* data, size_t n);

  private:
    FrameCb cb_;
    std::vector<uint8_t> buf_;
};

// payload 打包 helpers（小端）
void put_u8(std::vector<uint8_t>& p, size_t off, uint8_t v);
void put_u16(std::vector<uint8_t>& p, size_t off, uint16_t v);
void put_i16(std::vector<uint8_t>& p, size_t off, int16_t v);
void put_u32(std::vector<uint8_t>& p, size_t off, uint32_t v);
void put_i32(std::vector<uint8_t>& p, size_t off, int32_t v);
void put_f32(std::vector<uint8_t>& p, size_t off, float v);
uint8_t get_u8(const std::vector<uint8_t>& p, size_t off);
uint16_t get_u16(const std::vector<uint8_t>& p, size_t off);
int16_t get_i16(const std::vector<uint8_t>& p, size_t off);
uint32_t get_u32(const std::vector<uint8_t>& p, size_t off);
int32_t get_i32(const std::vector<uint8_t>& p, size_t off);
float get_f32(const std::vector<uint8_t>& p, size_t off);

// 上行指令 payload 构造
std::vector<uint8_t> pack_heartbeat_gcs();  // GCS 心跳（sysid 由 frame_v2 定）
std::vector<uint8_t> pack_command_int_reposition(uint8_t target_sys, int32_t lat_e7, int32_t lon_e7,
                                                float alt_m, uint8_t frame);
// 通用 COMMAND_INT（透传 param1..4/x/y/z/command/frame）
std::vector<uint8_t> pack_command_int(uint8_t target_sys, uint16_t command, uint8_t frame,
                                     float p1, float p2, float p3, float p4,
                                     int32_t x, int32_t y, float z);
std::vector<uint8_t> pack_command_long(uint8_t target_sys, uint16_t command, float p1, float p2,
                                      float p3, float p4, float p5, float p6, float p7);
// 18 通道 override（0=该通道忽略/释放）
std::vector<uint8_t> pack_rc_override(uint8_t target_sys, const uint16_t ch[18]);

// 下行解析 → Telemetry（只处理认识的消息）
bool decode_into(uint32_t msgid, const std::vector<uint8_t>& payload, uint8_t src_sysid, Telemetry& t);

// custom_mode → 可读模式名（INAV 多旋翼映射到 ArduPilot Copter mode）
const char* copter_mode_name(uint32_t custom_mode);

}  // namespace mav
