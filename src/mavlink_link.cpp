#include "mavlink_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace {
constexpr uint8_t kGcsCompId = 191;  // QGC 常用 compid
constexpr uint8_t kFcTargetComp = 0;

bool set_nonblock(int fd) {
    int f = ::fcntl(fd, F_GETFL, 0);
    return ::fcntl(fd, F_SETFL, f | O_NONBLOCK) == 0;
}

// 返回：0=成功，1=背压丢帧（不断线），2=真错误（需重连）
int send_all(int fd, const uint8_t* p, size_t n) {
    while (n > 0) {
        ssize_t w = ::send(fd, p, n, MSG_NOSIGNAL);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 1;
            return 2;
        }
        if (w == 0) return 2;
        p += w;
        n -= static_cast<size_t>(w);
    }
    return 0;
}
}  // namespace

MavlinkLink::MavlinkLink(const std::string& ip, uint16_t port, uint8_t gcs_sysid, uint16_t gcs_fwd_port)
    : ip_(ip), port_(port), gcs_sysid_(gcs_sysid), gcs_fwd_port_(gcs_fwd_port) {}

MavlinkLink::~MavlinkLink() { stop(); }

bool MavlinkLink::start(LogFn log) {
    log_ = std::move(log);
    if (gcs_fwd_port_ != 0) {
        gcs_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        // GCS 转发失败不致命：QGC 没开时 UDP 照发不误
    }
    running_ = true;
    th_ = std::thread(&MavlinkLink::worker, this);
    return true;
}

void MavlinkLink::stop() {
    running_ = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (fd_ >= 0) {
            ::shutdown(fd_, SHUT_RDWR);
            ::close(fd_);
            fd_ = -1;
        }
    }
    if (th_.joinable()) th_.join();
    if (gcs_fd_ >= 0) {
        ::close(gcs_fd_);
        gcs_fd_ = -1;
    }
    connected_ = false;
}

mav::Telemetry MavlinkLink::telemetry() const {
    std::lock_guard<std::mutex> lk(mu_);
    return telem_;
}

void MavlinkLink::set_raw_rx(RawRxFn fn) {
    std::lock_guard<std::mutex> lk(mu_);
    raw_rx_ = std::move(fn);
}

bool MavlinkLink::try_connect() {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (::inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr) != 1) {
        ::close(fd);
        return false;
    }
    // 非阻塞 connect + 超时（2 秒）
    set_nonblock(fd);
    int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (rc != 0 && errno != EINPROGRESS) {
        ::close(fd);
        return false;
    }
    fd_set wf;
    FD_ZERO(&wf);
    FD_SET(fd, &wf);
    timeval tv{2, 0};
    rc = ::select(fd + 1, nullptr, &wf, nullptr, &tv);
    if (rc <= 0) {
        ::close(fd);
        return false;
    }
    int err = 0;
    socklen_t elen = sizeof(err);
    ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &elen);
    if (err != 0) {
        ::close(fd);
        return false;
    }
    // 保持非阻塞：发送背压时丢帧（50Hz 冗余足），绝不阻塞 worker 节拍；
    // 读循环用 select 驱动，无需超时兜底。
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (fd_ >= 0) ::close(fd_);
        fd_ = fd;
    }
    return true;
}

void MavlinkLink::worker() {
    mav::Parser parser([this](uint32_t msgid, uint8_t sysid, uint8_t compid,
                              const std::vector<uint8_t>& payload) {
        (void)compid;
        std::lock_guard<std::mutex> lk(mu_);
        mav::decode_into(msgid, payload, sysid, telem_);
    });
    uint8_t buf[4096];
    auto last_hb = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    int backoff_ms = 500;

    while (running_) {
        if (!connected_) {
            if (try_connect()) {
                connected_ = true;
                backoff_ms = 500;
                if (log_) log_("MAVLink TCP 已连接 " + ip_ + ":" + std::to_string(port_));
                // 建联即发心跳，宣告 GCS 在线
                send_frame(mav::kMsgHeartbeat, mav::pack_heartbeat_gcs());
                last_hb = std::chrono::steady_clock::now();
            } else {
                if (log_ && backoff_ms == 500) log_("MAVLink TCP 等待 SITL " + ip_ + ":" + std::to_string(port_));
                std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
                backoff_ms = std::min(backoff_ms * 2, 5000);
                continue;
            }
        }
        // 1Hz 心跳维持 + 50Hz override 保持（INAV rxSignalTimeout=100ms，必须 ≥10Hz，留余量）
        auto now = std::chrono::steady_clock::now();
        if (now - last_hb > std::chrono::seconds(1)) {
            send_frame(mav::kMsgHeartbeat, mav::pack_heartbeat_gcs());
            last_hb = now;
        }
        // 50Hz override 保持
        {
            bool active = false;
            uint16_t ch[18] = {0};
            uint8_t tgt = 0;
            {
                std::lock_guard<std::mutex> lk(mu_);
                active = ovr_active_;
                if (active) {
                    std::memcpy(ch, ovr_ch_, sizeof(ch));
                    tgt = telem_.fc_sysid != 0 ? telem_.fc_sysid : gcs_sysid_;
                }
            }
            static auto last_ovr = std::chrono::steady_clock::now() - std::chrono::seconds(1);
            if (active && now - last_ovr > std::chrono::milliseconds(20)) {
                last_ovr = now;
                send_frame(mav::kMsgRcChannelsOverride, mav::pack_rc_override(tgt, ch));
                static int ovr_count = 0;
                static auto ovr_win = std::chrono::steady_clock::now();
                if (++ovr_count >= 50) {
                    auto dt = std::chrono::steady_clock::now() - ovr_win;
                    if (log_) {
                        char buf[96];
                        std::snprintf(buf, sizeof(buf), "OVR 实际帧率: %.1f Hz",
                                        50.0 / std::chrono::duration<double>(dt).count());
                        log_(buf);
                    }
                    ovr_count = 0;
                    ovr_win = std::chrono::steady_clock::now();
                }
            }
        }
        // 读（select 20ms 超时：发送节拍 50Hz，不能被读阻塞拖慢）
        int fd;
        {
            std::lock_guard<std::mutex> lk(mu_);
            fd = fd_;
        }
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(fd, &rf);
        timeval tv{0, 20000};
        int rc = ::select(fd + 1, &rf, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR) continue;
            connected_ = false;
            continue;
        }
        if (rc == 0) continue;
        ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
            if (log_) log_("MAVLink TCP 断开，重连中…");
            connected_ = false;
            std::lock_guard<std::mutex> lk(mu_);
            if (fd_ >= 0) {
                ::close(fd_);
                fd_ = -1;
            }
            continue;
        }
        parser.feed(buf, static_cast<size_t>(n));
        // GCS 转发：原始字节直转 UDP（QGC 监听该端口）
        RawRxFn raw;
        int gfd = -1;
        {
            std::lock_guard<std::mutex> lk(mu_);
            raw = raw_rx_;
            gfd = gcs_fd_;
        }
        if (raw) raw(buf, static_cast<size_t>(n));
        if (gfd >= 0) {
            sockaddr_in gcs{};
            gcs.sin_family = AF_INET;
            gcs.sin_port = htons(gcs_fwd_port_);
            gcs.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            ::sendto(gfd, buf, static_cast<size_t>(n), 0, reinterpret_cast<sockaddr*>(&gcs), sizeof(gcs));
        }
    }
}

void MavlinkLink::send_frame(uint32_t msgid, const std::vector<uint8_t>& payload) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!connected_ || fd_ < 0) return;
    auto frame = mav::frame_v2(gcs_sysid_, kGcsCompId, seq_++, msgid, payload.data(), payload.size());
    if (send_all(fd_, frame.data(), frame.size()) == 2) {
        // 真错误：标记断线，worker 负责重连（背压丢帧不重连）
        ::shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        fd_ = -1;
        connected_ = false;
    }
}

void MavlinkLink::send_reposition(int32_t lat_e7, int32_t lon_e7, float alt_m, uint8_t frame) {
    uint8_t tgt;
    {
        std::lock_guard<std::mutex> lk(mu_);
        tgt = telem_.fc_sysid != 0 ? telem_.fc_sysid : gcs_sysid_;
    }
    send_frame(mav::kMsgCommandInt, mav::pack_command_int_reposition(tgt, lat_e7, lon_e7, alt_m, frame));
}

void MavlinkLink::send_command_int(uint16_t command, uint8_t frame, float p1, float p2, float p3,
                                  float p4, int32_t x, int32_t y, float z) {
    uint8_t tgt;
    {
        std::lock_guard<std::mutex> lk(mu_);
        tgt = telem_.fc_sysid != 0 ? telem_.fc_sysid : gcs_sysid_;
    }
    send_frame(mav::kMsgCommandInt, mav::pack_command_int(tgt, command, frame, p1, p2, p3, p4, x, y, z));
}

void MavlinkLink::send_command_long(uint16_t command, float p1, float p2, float p3, float p4, float p5,
                                   float p6, float p7) {
    uint8_t tgt;
    {
        std::lock_guard<std::mutex> lk(mu_);
        tgt = telem_.fc_sysid != 0 ? telem_.fc_sysid : gcs_sysid_;
    }
    (void)kFcTargetComp;
    send_frame(mav::kMsgCommandLong, mav::pack_command_long(tgt, command, p1, p2, p3, p4, p5, p6, p7));
}

void MavlinkLink::send_rc_override(const uint16_t ch[18]) {
    uint8_t tgt;
    {
        std::lock_guard<std::mutex> lk(mu_);
        std::memcpy(ovr_ch_, ch, sizeof(ovr_ch_));
        ovr_active_ = true;
        tgt = telem_.fc_sysid != 0 ? telem_.fc_sysid : gcs_sysid_;
    }
    if (log_) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "OVR 发 ch1..8=%u %u %u %u %u %u %u %u tgt=%u", ch[0],
                        ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7], tgt);
        log_(buf);
    }
    send_frame(mav::kMsgRcChannelsOverride, mav::pack_rc_override(tgt, ch));
}

void MavlinkLink::clear_rc_override() {
    uint8_t tgt;
    {
        std::lock_guard<std::mutex> lk(mu_);
        ovr_active_ = false;
        std::memset(ovr_ch_, 0, sizeof(ovr_ch_));
        tgt = telem_.fc_sysid != 0 ? telem_.fc_sysid : gcs_sysid_;
    }
    uint16_t zero[18] = {0};
    send_frame(mav::kMsgRcChannelsOverride, mav::pack_rc_override(tgt, zero));
}

void MavlinkLink::send_arm(bool arm) {
    send_command_long(mav::kCmdComponentArmDisarm, arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0);
}
