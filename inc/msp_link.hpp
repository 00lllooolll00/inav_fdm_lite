#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

// MSP v2 只读客户端：轮询 INAV SITL 的 MSP 口（TCP）取 4 路电机输出。
// 动机：INAV 的 X-Plane 后端只发总油门 + 舵面（yoke 取自 servo[]），多旋翼的姿态差动
// 只存在于 motor[] 里，原生 dref 通道取不到 —— 不接 MSP 时仿真里飞机永远不倾斜。
// 注意必须用 MSP v2 帧（$X</$X>）：INAV 10 的 SITL 不响应 v1（$M<）请求。
//
// 从字节流里找一帧 MSP_MOTOR 响应并解析前 4 路（PWM 1000..2000 → 0..1）。
// 帧未收全或不足 4 路时返回 false（调用方继续收）；consumed 非空时回写本帧字节数。
bool mspParseMotorFrame(const uint8_t *data, size_t len, std::array<double, 4> &out, size_t *consumed);

class MspLink
{
public:
    MspLink(const std::string &host, uint16_t port);
    ~MspLink();

    MspLink(const MspLink &) = delete;
    MspLink &operator=(const MspLink &) = delete;

    // 读一次 MSP_MOTOR(104)，成功返回 true 并写入 0..1 归一化的 4 路电机值。
    // 连续失败约 2 秒会断开让位（SITL 每个 UART 只服务一个客户端，见 note_failure）。
    bool poll_motors(std::array<double, 4> &out);

    // 让位开关：外部工具（arm_status.py）通过标志文件请求时置位——暂停轮询并断开
    // 连接，把 SITL 的唯一客户端位让出去；复位后重连，等对方退出自动接管。
    void set_paused(bool paused);
    bool paused() const
    {
        return paused_;
    }

private:
    bool ensure_connected();
    bool note_failure();

    std::string host_;
    uint16_t port_ = 0;
    int fd_ = -1;
    uint8_t buf_[512] = { 0 };
    size_t buf_len_ = 0;
    bool paused_ = false;
    // 让位机制：SITL 每个 UART 只服务一个客户端。连续读不到响应说明端口被别的
    // 临时客户端（如 arm_status.py）占了——断开让位并退避，稍后重连自动接管。
    int fail_run_ = 0;
    std::chrono::steady_clock::time_point retry_after_{};
};
