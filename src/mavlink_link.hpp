#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "mavlink_codec.hpp"

// MAVLink TCP 链路：主动连接 SITL 的串口映射 TCP 端口。
// 职责：维持 GCS 心跳、解析飞控遥测、发送测试指令、可选转发给 QGC。
class MavlinkLink {
  public:
    using LogFn = std::function<void(const std::string&)>;
    using RawRxFn = std::function<void(const uint8_t*, size_t)>;  // 原始字节（供 GCS 转发）

    MavlinkLink(const std::string& ip, uint16_t port, uint8_t gcs_sysid, uint16_t gcs_fwd_port);
    ~MavlinkLink();

    MavlinkLink(const MavlinkLink&) = delete;
    MavlinkLink& operator=(const MavlinkLink&) = delete;

    bool start(LogFn log = nullptr);
    void stop();
    bool connected() const { return connected_.load(); }
    mav::Telemetry telemetry() const;

    void set_raw_rx(RawRxFn fn);

    // --- 上行指令（线程安全，断线时静默丢弃） ---
    void send_reposition(int32_t lat_e7, int32_t lon_e7, float alt_m, uint8_t frame);
    void send_command_int(uint16_t command, uint8_t frame, float p1, float p2, float p3, float p4,
                          int32_t x, int32_t y, float z);
    void send_command_long(uint16_t command, float p1, float p2, float p3, float p4, float p5,
                           float p6, float p7);
    // ch[18]：PWM 1000..2000，0=该通道忽略
    void send_rc_override(const uint16_t ch[18]);
    void send_arm(bool arm);  // COMMAND_LONG 400（INAV 未实现则回 UNSUPPORTED，仅透传不断言）

  private:
    void worker();
    bool try_connect();
    void send_frame(uint32_t msgid, const std::vector<uint8_t>& payload);

    std::string ip_;
    uint16_t port_;
    uint8_t gcs_sysid_;
    uint16_t gcs_fwd_port_;
    LogFn log_;

    std::thread th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    mutable std::mutex mu_;
    int fd_ = -1;
    int gcs_fd_ = -1;  // GCS 转发 UDP socket（-1=关闭）
    uint8_t seq_ = 0;
    mav::Telemetry telem_{};
    RawRxFn raw_rx_;
};
