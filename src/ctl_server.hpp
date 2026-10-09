#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

// 本地 TCP 控制面：测试指令注入（文本行协议，\n 结尾）。
//   REPOSITION <lat> <lon> <alt_m> [frame]  → COMMAND_INT DO_REPOSITION(192)
//   OVERRIDE <ch1> ... <ch18>                → RC_CHANNELS_OVERRIDE（0=忽略该通道）
//   OVERRIDE_ONE <ch1based> <pwm>             → 单通道 override，其余忽略
//   OVERRIDE_CLEAR                            → 18 通道全 0（释放）
//   CMDINT <cmd> <frame> <p1> <p2> <p3> <p4> <x> <y> <z>
//   CMDLONG <cmd> <p1>..<p7>                  → COMMAND_LONG 透传
//   ARM / DISARM                              → COMMAND_LONG 400（INAV 未实现则回 UNSUPPORTED）
// 回复：单行 "OK ..." 或 "ERR ..."。
class CtlServer {
  public:
    struct Handlers {
        std::function<std::string(double, double, float, uint8_t)> reposition;
        std::function<std::string(const uint16_t[18])> override;
        std::function<std::string(uint16_t, uint8_t, float, float, float, float, int32_t, int32_t, float)>
            cmdint;
        std::function<std::string(uint16_t, float, float, float, float, float, float, float)> cmdlong;
        std::function<std::string(bool)> arm;
    };

    CtlServer(uint16_t port, Handlers h);
    ~CtlServer();

    CtlServer(const CtlServer&) = delete;
    CtlServer& operator=(const CtlServer&) = delete;

    bool start();
    void stop();

    // 纯函数：解析一行命令并执行（供自检直接调用，无需 socket）。
    static std::string handle_line(const std::string& line, const Handlers& h);

  private:
    void accept_loop();
    void serve_client(int fd);

    uint16_t port_;
    Handlers handlers_;
    int listen_fd_ = -1;
    std::thread th_;
    std::atomic<bool> running_{false};
};
