#include "msp_link.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>

namespace
{
constexpr uint8_t kMspMotor = 104; // MSP_MOTOR：各电机 PWM（1000..2000）
constexpr int kRecvTimeoutUs = 20000; // 单次收超时，避免拖住 500Hz 主循环
constexpr int kFailRunLimit = 100; // 连续失败 ~2 秒（50Hz 轮询）后断开让位
constexpr std::chrono::milliseconds kRetryBackoff{ 1000 }; // 让位后的重连退避

uint8_t crc8_dvb_s2(const uint8_t *data, size_t len)
{
    uint8_t c = 0;
    for (size_t i = 0; i < len; ++i)
    {
        c ^= data[i];
        for (int b = 0; b < 8; ++b)
        {
            c = static_cast<uint8_t>((c & 0x80) != 0 ? (c << 1) ^ 0xD5 : c << 1);
        }
    }
    return c;
}

double pwm_to_unit(uint16_t pwm)
{
    if (pwm <= 1000) return 0.0;
    if (pwm >= 2000) return 1.0;
    return static_cast<double>(pwm - 1000) / 1000.0;
}
} // namespace

bool mspParseMotorFrame(const uint8_t *data, size_t len, std::array<double, 4> &out, size_t *consumed)
{
    // MSP v2 响应：$X> + flags(1) + cmd(2) + len(2) + payload + crc(1)
    for (size_t i = 0; i + 9 <= len; ++i)
    {
        if (!(data[i] == '$' && data[i + 1] == 'X' && data[i + 2] == '>'))
        {
            continue;
        }
        const size_t plen =
            static_cast<size_t>(data[i + 6] | (static_cast<uint16_t>(data[i + 7]) << 8));
        if (i + 9 + plen > len) return false; // 帧未收全
        if (plen < 8) return false;           // 少于 4 路电机
        const uint8_t *payload = data + i + 8;
        for (size_t m = 0; m < 4; ++m)
        {
            const uint16_t pwm = static_cast<uint16_t>(
                payload[m * 2] | (static_cast<uint16_t>(payload[m * 2 + 1]) << 8));
            out[m] = pwm_to_unit(pwm);
        }
        if (consumed) *consumed = i + 9 + plen;
        return true;
    }
    return false;
}

MspLink::MspLink(const std::string &host, uint16_t port)
: host_(host)
, port_(port)
{
}

MspLink::~MspLink()
{
    if (fd_ >= 0) ::close(fd_);
}

bool MspLink::ensure_connected()
{
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1 ||
        ::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
    {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    timeval tv{};
    tv.tv_usec = kRecvTimeoutUs;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return true;
}

void MspLink::set_paused(bool paused)
{
    paused_ = paused;
    if (paused_ && fd_ >= 0)
    {
        // 立即断开，把 SITL 的"活跃客户端"位让给工具；失败计数清零
        ::close(fd_);
        fd_ = -1;
        fail_run_ = 0;
    }
}

bool MspLink::poll_motors(std::array<double, 4> &out)
{
    if (paused_) return false;

    if (std::chrono::steady_clock::now() < retry_after_) return false; // 让位退避中

    if (fd_ < 0 && !ensure_connected()) return note_failure();

    // MSP v2 请求：$X< + flags(0) + cmd(u16) + len(u16) + crc8(flags..len)
    uint8_t req[9] = { '$', 'X', '<', 0, kMspMotor, 0, 0, 0, 0 };
    req[8] = crc8_dvb_s2(&req[3], 5);
    if (::send(fd_, req, sizeof(req), 0) != static_cast<ssize_t>(sizeof(req)))
    {
        ::close(fd_);
        fd_ = -1;
        return note_failure();
    }

    // 收响应（TCP 可能分片）：$X> + flags + cmd(u16) + len(u16) + payload + crc8
    buf_len_ = 0;
    for (int tries = 0; tries < 16; ++tries)
    {
        const ssize_t n = ::recv(fd_, buf_ + buf_len_, sizeof(buf_) - buf_len_, 0);
        if (n == 0)
        {
            // 对端关闭：下次轮询重连
            ::close(fd_);
            fd_ = -1;
            return note_failure();
        }
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                // 收超时（SITL 启动期较常见）：保留连接下轮重试，累积到阈值才让位
                return note_failure();
            }
            ::close(fd_);
            fd_ = -1;
            return note_failure();
        }
        buf_len_ += static_cast<size_t>(n);
        if (mspParseMotorFrame(buf_, buf_len_, out, nullptr))
        {
            fail_run_ = 0;
            return true;
        }
    }
    return note_failure();
}

bool MspLink::note_failure()
{
    if (++fail_run_ < kFailRunLimit) return false;

    // 连续读不到响应约 2 秒：多半是本端口被别的临时客户端（arm_status.py 等）占了。
    // 断开让出“活跃客户端”位并退避；重连若排队也无害，对方退出后会被自动接管。
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
    retry_after_ = std::chrono::steady_clock::now() + kRetryBackoff;
    fail_run_ = 0;
    return false;
}
