#include "xplane_link.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace {
// XITL 版本声明：必须 <2！=2 时 SITL 走 XITL 插件路径，不发 throttle_ratio_all（"INAV XITL sets throttle itself"），电机闭环断裂。=1 走 X-Plane 原生路径发总油门+yoke（GPS 星数/fix 用默认值 16/3D，电池固定 16.8V——对多旋翼闭环足够）。
constexpr int kXitlDrefVersion = 1;

double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// dref 全路径取短名（fl2sitl 同款：按 '/' 切最后一段）
std::string short_name(const std::string& full) {
    auto pos = full.find_last_of('/');
    return pos == std::string::npos ? full : full.substr(pos + 1);
}

void put_u32le(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}
void put_f32le(uint8_t* p, float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    put_u32le(p, u);
}
uint32_t get_u32le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
float get_f32le(const uint8_t* p) {
    uint32_t u = get_u32le(p);
    float v;
    std::memcpy(&v, &u, 4);
    return v;
}
}  // namespace

XplaneLink::XplaneLink(uint16_t listen_port, const std::string& sitl_ip, uint16_t simport)
    : listen_port_(listen_port), sitl_ip_(sitl_ip), simport_(simport) {}

XplaneLink::~XplaneLink() { stop(); }

bool XplaneLink::start(LogFn log) {
    log_ = std::move(log);
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    // SITL 背靠背突发 10 包/轮，加大缓冲防 RcvbufErrors 丢包
    int rcvbuf = 4 * 1024 * 1024, sndbuf = 1 * 1024 * 1024;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    ::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port_);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    // 预置对端（SITL 发包源端口为临时端口，之后从首包学习覆盖）
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(simport_);
    if (::inet_pton(AF_INET, sitl_ip_.c_str(), &peer.sin_addr) == 1) {
        std::lock_guard<std::mutex> lk(mu_);
        std::memcpy(peer_.bytes, &peer, sizeof(peer));
        peer_.len = sizeof(peer);
    }
    load_cache();
    running_ = true;
    rx_thread_ = std::thread(&XplaneLink::rx_loop, this);
    return true;
}

void XplaneLink::stop() {
    running_ = false;
    if (fd_ >= 0) {
        ::shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        fd_ = -1;
    }
    if (rx_thread_.joinable()) rx_thread_.join();
    connected_ = false;
}

void XplaneLink::rx_loop() {
    uint8_t buf[2048];
    long n_rref = 0, n_dref = 0, n_other = 0;
    auto last_rep = std::chrono::steady_clock::now();
    while (running_) {
        sockaddr_storage src{};
        socklen_t slen = sizeof(src);
        ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&src), &slen);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (!running_) break;
            continue;  // 非阻塞容错：错误不退出
        }
        if (n < 5) continue;
        {
            std::lock_guard<std::mutex> lk(mu_);
            const size_t cplen = slen <= sizeof(peer_.bytes) ? static_cast<size_t>(slen) : sizeof(peer_.bytes);
            std::memcpy(peer_.bytes, &src, cplen);
            peer_.len = slen;
        }
        connected_ = true;
        if (std::memcmp(buf, "RREF", 4) == 0) {
            ++n_rref;
            handle_rref_request(buf, static_cast<size_t>(n));
        } else if (std::memcmp(buf, "DREF", 4) == 0) {
            ++n_dref;
            handle_dref(buf, static_cast<size_t>(n));
        } else {
            ++n_other;
        }
        auto now = std::chrono::steady_clock::now();
        if (log_ && now - last_rep > std::chrono::seconds(5)) {
            char tmp[128];
            std::snprintf(tmp, sizeof(tmp), "UDP 收包: RREF=%ld DREF=%ld OTHER=%ld", n_rref, n_dref,
                            n_other);
            log_(tmp);
            last_rep = now;
        }
    }
}

void XplaneLink::handle_rref_request(const uint8_t* buf, size_t n) {
    // RREF 请求：'RREF'(4) + freq u32 + id u32 + dref 路径 NUL 结尾
    if (n < 13) return;
    const uint32_t freq = get_u32le(buf + 5);
    const uint32_t id = get_u32le(buf + 9);
    const char* path = reinterpret_cast<const char*>(buf + 13);
    const size_t maxlen = n - 13;
    const size_t plen = strnlen(path, maxlen);
    if (plen == 0 || plen >= maxlen) return;
    const std::string full(path, plen);
    const std::string name = short_name(full);
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (freq == 0) {
            dref_ids_.erase(name);  // freq=0 表示注销
        } else {
            dref_ids_[name] = id;
        }
        save_cache();
    }
    if (log_) log_("RREF 注册: " + full + " id=" + std::to_string(id) + " freq=" + std::to_string(freq));
    // xitlDrefVersion：一收到注册立刻回版本号，否则 SITL 卡在 CONNECTING
    if (name == "xitlDrefVersion" && freq != 0) {
        std::lock_guard<std::mutex> lk(mu_);
        uint8_t out[16] = {'R', 'R', 'E', 'F', 0};
        put_u32le(out + 5, id);
        put_f32le(out + 9, static_cast<float>(kXitlDrefVersion));
        ::sendto(fd_, out, 13, 0, reinterpret_cast<sockaddr*>(peer_.bytes), peer_.len);
    }
}

void XplaneLink::handle_dref(const uint8_t* buf, size_t n) {
    // DREF 包：'DREF'(4) + 0 + value f32 + 路径（500 字节，空格填充）
    if (n < 10) return;
    const float value = get_f32le(buf + 5);
    std::string full(reinterpret_cast<const char*>(buf + 9), n - 9);
    // 去尾部空格/NUL（注意 find_last_not_of 不接受含 \0 的 C 字符串，逐字符判断）
    size_t end = full.size();
    while (end > 0 && (full[end - 1] == ' ' || full[end - 1] == '\0')) --end;
    if (end == 0) return;
    full.resize(end);
    const std::string name = short_name(full);
    if (log_ && name == "throttle_ratio_all") {
        static int n = 0;
        if (++n % 50 == 1) log_("DREF 油门: " + std::to_string(value));
    }
    std::lock_guard<std::mutex> lk(mu_);
    if (name == "throttle_ratio_all") {
        thr_all_ = clamp01(value);
    } else if (name == "yoke_roll_ratio") {
        yoke_r_ = value;
    } else if (name == "yoke_pitch_ratio") {
        yoke_p_ = value;
    } else if (name == "yoke_heading_ratio") {
        yoke_y_ = value;
    } else {
        return;
    }
    // 总油门 + 姿态差动 → 4 路电机（X 构型混控正解）
    const double t = thr_all_;
    const double dr = yoke_r_ * 0.5, dp = yoke_p_ * 0.5, dy = yoke_y_ * 0.3;
    motors_[0] = clamp01(t - dr * 0.7071 - dp * 0.7071 - dy);  // 右前
    motors_[1] = clamp01(t + dr * 0.7071 - dp * 0.7071 + dy);  // 左前
    motors_[2] = clamp01(t + dr * 0.7071 + dp * 0.7071 - dy);  // 左后
    motors_[3] = clamp01(t - dr * 0.7071 + dp * 0.7071 + dy);  // 右后
    motors_valid_ = true;
}

void XplaneLink::send_sensors(const SensorSnapshot& s) {
    if (!connected_) return;
    std::lock_guard<std::mutex> lk(mu_);
    if (dref_ids_.empty() || peer_.len == 0) return;
    uint8_t out[2048] = {'R', 'R', 'E', 'F', 0};
    size_t pos = 5;
    auto emit = [&](const char* name, float v) {
        auto it = dref_ids_.find(name);
        if (it == dref_ids_.end()) return;
        if (pos + 8 > sizeof(out)) return;
        put_u32le(out + pos, it->second);
        put_f32le(out + pos + 4, v);
        pos += 8;
    };
    // 姿态/位置/速度（fl2sitl 同款集合 + X-Plane 本地速度三分量）
    emit("latitude", static_cast<float>(s.lat_deg));
    emit("longitude", static_cast<float>(s.lon_deg));
    emit("elevation", s.alt_m);
    emit("y_agl", s.agl_m);
    emit("local_vx", s.local_vx);
    emit("local_vy", s.local_vy);
    emit("local_vz", s.local_vz);
    emit("groundspeed", s.groundspeed);
    emit("true_airspeed", s.airspeed);
    emit("phi", s.phi_deg);
    emit("theta", -s.theta_deg);  // fl2sitl 锚点：INAV 侧 pitch 取反，此处预反
    emit("psi", s.psi_deg);
    emit("hpath", s.hpath_deg);
    // SITL 固定重力补偿为 +981 cm/s^2；该映射按静止读数匹配，动态符号仍需真实 SITL 飞行验证。
    emit("g_axil", -s.g_axil);
    emit("g_side", -s.g_side);
    emit("g_nrml", 2.0f - s.g_nrml);
    emit("P", s.gyro_p_dps);
    emit("Q", s.gyro_q_dps);
    emit("R", s.gyro_r_dps);
    emit("barometer_current_inhg", s.baro_inhg);
    // XITL 扩展
    emit("magnetometer[0]", s.mag_x);
    emit("magnetometer[1]", s.mag_y);
    emit("magnetometer[2]", s.mag_z);
    emit("numSats", static_cast<float>(s.num_sats));
    emit("fix", static_cast<float>(s.fix_type));
    emit("battery_voltage", s.batt_v);
    emit("battery_current", s.batt_a);
    emit("rssi", static_cast<float>(s.rssi));
    emit("failsafe", s.failsafe ? 1.0f : 0.0f);
    emit("rangefinder", static_cast<float>(s.rangefinder_cm));
    emit("airspeed", s.airspeed);
    // local velocities 数组别名（INAV XITL 注册 velocities[0..2]）
    emit("velocities[0]", s.local_vx);
    emit("velocities[1]", s.local_vy);
    emit("velocities[2]", s.local_vz);
    if (pos > 5) {
        ::sendto(fd_, out, pos, 0, reinterpret_cast<sockaddr*>(peer_.bytes), peer_.len);
    }
}

bool XplaneLink::get_motors(std::array<double, 4>& out) const {
    std::lock_guard<std::mutex> lk(mu_);
    out = motors_;
    return motors_valid_;
}

std::map<std::string, uint32_t> XplaneLink::dref_map() const {
    std::lock_guard<std::mutex> lk(mu_);
    return dref_ids_;
}

std::string XplaneLink::cache_path() const {
    char tmp[128];
    std::snprintf(tmp, sizeof(tmp), "/tmp/inav_bridge_dref_%u.cache", listen_port_);
    return tmp;
}

void XplaneLink::save_cache() {
    // 持锁调用。SITL 只在启动时发一次注册，桥重启后靠缓存恢复回包能力。
    std::ofstream f(cache_path(), std::ios::trunc);
    if (!f) return;
    for (const auto& kv : dref_ids_) f << kv.first << ' ' << kv.second << '\n';
}

void XplaneLink::load_cache() {
    std::ifstream f(cache_path());
    if (!f) return;
    std::map<std::string, uint32_t> m;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line);
        std::string name;
        uint32_t id;
        if (iss >> name >> id) m[name] = id;
    }
    if (!m.empty()) {
        std::lock_guard<std::mutex> lk(mu_);
        dref_ids_ = std::move(m);
        if (log_) log_("DREF 注册表从缓存恢复 " + std::to_string(dref_ids_.size()) + " 项");
    }
}
