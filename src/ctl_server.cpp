#include "ctl_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <vector>

CtlServer::CtlServer(uint16_t port, Handlers h) : port_(port), handlers_(std::move(h)) {}
CtlServer::~CtlServer() { stop(); }

bool CtlServer::start() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;
    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);  // 仅本机，避免误暴露
    addr.sin_port = htons(port_);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
        ::listen(listen_fd_, 4) != 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    running_ = true;
    th_ = std::thread(&CtlServer::accept_loop, this);
    return true;
}

void CtlServer::stop() {
    running_ = false;
    if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (th_.joinable()) th_.join();
}

void CtlServer::accept_loop() {
    while (running_) {
        int fd = ::accept(listen_fd_, nullptr, nullptr);
        if (fd < 0) {
            if (errno == EINTR) continue;
            if (!running_) break;
            continue;
        }
        serve_client(fd);
        ::close(fd);
    }
}

void CtlServer::serve_client(int fd) {
    // 行缓冲读：一次连接可发多条命令
    std::string acc;
    char buf[1024];
    while (running_) {
        ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        acc.append(buf, static_cast<size_t>(n));
        size_t pos;
        while ((pos = acc.find('\n')) != std::string::npos) {
            std::string line = acc.substr(0, pos);
            acc.erase(0, pos + 1);
            std::string resp = handle_line(line, handlers_) + "\n";
            ::send(fd, resp.c_str(), resp.size(), MSG_NOSIGNAL);
        }
        if (acc.size() > 4096) acc.clear();  // 防超长行撑爆内存
    }
}

namespace {
std::vector<std::string> split(const std::string& s) {
    std::istringstream iss(s);
    std::vector<std::string> out;
    std::string w;
    while (iss >> w) out.push_back(w);
    return out;
}
bool to_double(const std::string& s, double& v) {
    char* e = nullptr;
    v = std::strtod(s.c_str(), &e);
    return e && *e == '\0';
}
bool to_long(const std::string& s, long& v) {
    char* e = nullptr;
    v = std::strtol(s.c_str(), &e, 10);
    return e && *e == '\0';
}
}  // namespace

std::string CtlServer::handle_line(const std::string& line, const Handlers& h) {
    auto t = split(line);
    if (t.empty()) return "ERR empty";
    const std::string& cmd = t[0];

    if (cmd == "REPOSITION") {
        if (t.size() != 4 && t.size() != 5) return "ERR REPOSITION <lat> <lon> <alt_m> [frame]";
        double lat, lon, alt;
        if (!to_double(t[1], lat) || !to_double(t[2], lon) || !to_double(t[3], alt))
            return "ERR bad number";
        long frame = 0;
        if (t.size() == 5 && !to_long(t[4], frame)) return "ERR bad frame";
        if (lat < -90 || lat > 90 || lon < -180 || lon > 180) return "ERR lat/lon range";
        if (!h.reposition) return "ERR no handler";
        return h.reposition(lat, lon, static_cast<float>(alt), static_cast<uint8_t>(frame));
    }
    if (cmd == "OVERRIDE") {
        if (t.size() != 19) return "ERR OVERRIDE <18 x pwm|0>";
        uint16_t ch[18];
        for (size_t i = 0; i < 18; ++i) {
            long v;
            if (!to_long(t[1 + i], v) || v < 0 || v > 2200) return "ERR bad pwm";
            ch[i] = static_cast<uint16_t>(v);
        }
        if (!h.override) return "ERR no handler";
        return h.override(ch);
    }
    if (cmd == "CMDINT") {
        // CMDINT <cmd> <frame> <p1> <p2> <p3> <p4> <x> <y> <z>（z 为 float，REPOSITION 语义=米）
        if (t.size() != 10) return "ERR CMDINT <cmd> <frame> <p1> <p2> <p3> <p4> <x> <y> <z>";
        long cmdl, fr, x, y;
        double p1, p2, p3, p4, z;
        if (!to_long(t[1], cmdl) || !to_long(t[2], fr)) return "ERR bad cmd/frame";
        if (!to_double(t[3], p1) || !to_double(t[4], p2) || !to_double(t[5], p3) || !to_double(t[6], p4))
            return "ERR bad param";
        if (!to_long(t[7], x) || !to_long(t[8], y) || !to_double(t[9], z)) return "ERR bad xyz";
        if (!h.cmdint) return "ERR no handler";
        return h.cmdint(static_cast<uint16_t>(cmdl), static_cast<uint8_t>(fr), static_cast<float>(p1),
                        static_cast<float>(p2), static_cast<float>(p3), static_cast<float>(p4),
                        static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<float>(z));
    }
    if (cmd == "OVERRIDE_ONE") {
        if (t.size() != 3) return "ERR OVERRIDE_ONE <ch1..18> <pwm|0>";
        long chn, pwm;
        if (!to_long(t[1], chn) || chn < 1 || chn > 18) return "ERR bad ch";
        if (!to_long(t[2], pwm) || pwm < 0 || pwm > 2200) return "ERR bad pwm";
        uint16_t ch[18] = {0};
        ch[chn - 1] = static_cast<uint16_t>(pwm);
        if (!h.override) return "ERR no handler";
        return h.override(ch);
    }
    if (cmd == "OVERRIDE_CLEAR") {
        uint16_t ch[18] = {0};
        if (!h.override) return "ERR no handler";
        return h.override(ch);
    }
    if (cmd == "CMDLONG") {
        if (t.size() != 9) return "ERR CMDLONG <cmd> <p1..p7>";
        long cmdl;
        double p[7];
        if (!to_long(t[1], cmdl)) return "ERR bad cmd";
        for (size_t i = 0; i < 7; ++i)
            if (!to_double(t[2 + i], p[i])) return "ERR bad param";
        if (!h.cmdlong) return "ERR no handler";
        return h.cmdlong(static_cast<uint16_t>(cmdl), static_cast<float>(p[0]), static_cast<float>(p[1]),
                         static_cast<float>(p[2]), static_cast<float>(p[3]), static_cast<float>(p[4]),
                         static_cast<float>(p[5]), static_cast<float>(p[6]));
    }
    if (cmd == "ARM") {
        if (!h.arm) return "ERR no handler";
        return h.arm(true);
    }
    if (cmd == "DISARM") {
        if (!h.arm) return "ERR no handler";
        return h.arm(false);
    }
    return "ERR unknown cmd";
}
