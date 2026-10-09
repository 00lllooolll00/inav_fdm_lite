#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "fdm.hpp"

// X-Plane UDP 链路（对标 fl2sitl 行为）：
//   - 被动监听：读 SITL 发来的 RREF 注册请求，按 dref 名建 id 映射表；
//   - 读 DREF：解析 SITL 发来的电机/舵面 DREF 包，提炼 4 路电机油门；
//   - 写 RREF：按映射表把 SensorSnapshot 打包回传（id,value 对）。
// 电机语义（与 --chanmap 对应）：
//   - throttle_ratio_all：多旋翼总油门（--chanmap=M01-01... 时 motorValue 由首个 M 通道决定，
//     典型配法 4 路电机同值时即总油门）；
//   - yoke_roll/pitch/yaw_ratio：姿态控制量 -1..1；
//   - 本桥按“总油门 + 姿态差动”重构 4 路电机，还原 X 构型混控。
class XplaneLink {
  public:
    using LogFn = std::function<void(const std::string&)>;

    XplaneLink(uint16_t listen_port, const std::string& sitl_ip, uint16_t simport);
    ~XplaneLink();

    XplaneLink(const XplaneLink&) = delete;
    XplaneLink& operator=(const XplaneLink&) = delete;

    bool start(LogFn log = nullptr);
    void stop();

    // 发送一帧传感器数据（50Hz 主循环调用）；未建联时静默丢弃。
    void send_sensors(const SensorSnapshot& s);
    // 取最新电机油门（0..1）；返回 false 表示从未收到过电机包。
    bool get_motors(std::array<double, 4>& out) const;
    bool connected() const { return connected_.load(); }
    // 已注册 dref 名 → id（供自检）。
    std::map<std::string, uint32_t> dref_map() const;

  private:
    void rx_loop();
    void handle_rref_request(const uint8_t* buf, size_t n);
    void handle_dref(const uint8_t* buf, size_t n);

    uint16_t listen_port_;
    std::string sitl_ip_;
    uint16_t simport_;
    LogFn log_;

    int fd_ = -1;
    std::thread rx_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    mutable std::mutex mu_;
    std::map<std::string, uint32_t> dref_ids_;  // 短名 → RREF id
    // 对端地址（从首个 RREF/DREF 包学习；sendto 回包用）
    struct SockAddr {
        uint8_t bytes[32] = {0};
        uint32_t len = 0;
    };
    SockAddr peer_{};
    // 电机状态
    std::array<double, 4> motors_{0, 0, 0, 0};
    bool motors_valid_ = false;
    double thr_all_ = 0, yoke_r_ = 0, yoke_p_ = 0, yoke_y_ = 0;
};
