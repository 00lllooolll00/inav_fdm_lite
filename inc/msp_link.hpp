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

// 在字节流里找一帧指定 cmd 的 MSP v2 响应（$X> + flags + cmd(u16) + len(u16) + payload + crc）。
// 同一连接上会话式交错着多种查询的响应，必须按 cmd 过滤，否则会把别的响应当电机包解析。
// 返回 false 表示帧未收全或流里没有该 cmd 的帧。
bool mspParseFrame(const uint8_t *data, size_t len, uint16_t want_cmd, const uint8_t **payload, size_t *plen,
                   size_t *consumed);

// armingFlags（MSP2_INAV_STATUS 的 32 位字段）的位图约定：
//   位 2..5：状态（ARMED / WAS_EVER_ARMED / SIMULATOR_MODE_*）；位 6..30：阻止解锁原因。
// 名字与 INAV runtime_config.c 的 armingDisableFlagNames[] 一一对应。
constexpr uint32_t MSP_ARMING_BLOCK_MASK = 0x7FFFFFC0u;
constexpr uint32_t MSP_ARMING_ARMED = 1u << 2;

// 把阻止位解码成逗号分隔的简短名字（如 "CAL,RX"）；无阻止位返回空串。
std::string mspArmingBlockNames(uint32_t arming_flags);

// MSP_MOTOR 的 4 路电机按 INAV 电机序号排列，Quad X 是 [右后, 右前, 左后, 左前]；
// FDM 的 step() 期望 [右前, 左前, 左后, 右后]（见 fdm.cpp 注释）。
// 直接透传会让滚转/俯仰/偏航三轴循环错位耦合，仿真里表现为绕着目标点转圈。
inline std::array<double, 4> mspMotorsToFdmOrder(const std::array<double, 4> &m)
{
    return { m[1], m[3], m[2], m[0] };
}

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

    // 读一次 MSP2_INAV_STATUS(0x2000) 的 armingFlags（32 位，含高位阻止原因）。
    // 不能用 MSP_STATUS_EX：那个字段是 u16，位 16 以上的原因（RX/THR/OSD…）会被截断。
    bool poll_status(uint32_t &arming_flags);

    // 让位开关：外部工具（临时 MSP 工具）通过标志文件请求时置位——暂停轮询并断开
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
    // 临时客户端（如 临时 MSP 工具）占了——断开让位并退避，稍后重连自动接管。
    int fail_run_ = 0;
    std::chrono::steady_clock::time_point retry_after_{};
};
