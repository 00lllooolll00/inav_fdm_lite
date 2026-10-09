#include "config.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <getopt.h>

void print_usage(const char* prog) {
    std::printf(
        "用法: %s [选项]\n"
        "\n"
        "  X-Plane UDP 链路:\n"
        "    -i <ip>                 SITL 所在 IP（默认 127.0.0.1）\n"
        "    -p <port>               SITL 的 UDP 端口 simport（默认 49000）\n"
        "    -l <port>               本地监听 UDP 端口（默认与 -p 相同，单 socket 复用）\n"
        "    -r <hz>                 物理推演与注入刷新率（默认 50）\n"
        "\n"
        "  MAVLink 链路:\n"
        "    --mav-tcp-ip <ip>       SITL 串口映射 TCP 目标 IP（默认 127.0.0.1）\n"
        "    --mav-tcp-port <port>   SITL 串口映射 TCP 端口（默认 5762，对应 MAVLink UART）\n"
        "    --gcs-udp-port <port>   对外转发给地面站的 UDP 端口（0=关闭，默认 0）\n"
        "\n"
        "  实例与地理:\n"
        "    --lat <deg>             初始纬度（默认 30.0）\n"
        "    --lon <deg>             初始经度（默认 120.0）\n"
        "    --alt <m>               初始海拔 AMSL（默认 10.0）\n"
        "    --id <num>              无人机 System ID（默认 1）\n"
        "\n"
        "  控制面（测试指令注入）:\n"
        "    --ctl-port <port>       本地 TCP 控制端口（0=关闭；REPOSITION/OVERRIDE/ARM 等）\n"
        "\n"
        "  物理模型:\n"
        "    --mass <kg>             机体质量（默认 1.2）\n"
        "    --max-thrust <N>        全油门总推力（默认 30.0）\n"
        "    --arm-alt <m>           地面高度 AMSL（默认 = --alt，即地面起飞）\n"
        "    -v                      详细日志\n"
        "    -h, --help              显示本帮助\n"
        "\n"
        "示例（单机）:\n"
        "  SITL 端: inav_SITL --sim=xp --simip=127.0.0.1 --simport=49000 --path ./uav1.bin\n"
        "  桥接端: %s --id 1 --lat 30.0 --lon 120.0 --alt 10.0 \\\n"
        "           -p 49000 -l 49000 --mav-tcp-port 5762 -r 50 --ctl-port 6901\n",
        prog, prog);
}

namespace {

bool parse_u16(const char* s, uint16_t& out) {
    char* end = nullptr;
    long v = std::strtol(s, &end, 10);
    if (!end || *end != '\0' || v < 0 || v > 65535) return false;
    out = static_cast<uint16_t>(v);
    return true;
}

bool parse_u8(const char* s, uint8_t& out) {
    char* end = nullptr;
    long v = std::strtol(s, &end, 10);
    if (!end || *end != '\0' || v < 0 || v > 255) return false;
    out = static_cast<uint8_t>(v);
    return true;
}

bool parse_double(const char* s, double& out) {
    char* end = nullptr;
    double v = std::strtod(s, &end);
    if (!end || *end != '\0' || !std::isfinite(v)) return false;
    out = v;
    return true;
}

}  // namespace

int parse_args(int argc, char** argv, BridgeConfig& out, bool& help_only) {
    out = BridgeConfig{};
    help_only = false;
    bool listen_set = false;

    static const struct option kLongOpts[] = {
        {"mav-tcp-ip", required_argument, nullptr, 1000},
        {"mav-tcp-port", required_argument, nullptr, 1001},
        {"gcs-udp-port", required_argument, nullptr, 1002},
        {"lat", required_argument, nullptr, 1003},
        {"lon", required_argument, nullptr, 1004},
        {"alt", required_argument, nullptr, 1005},
        {"id", required_argument, nullptr, 1006},
        {"ctl-port", required_argument, nullptr, 1007},
        {"mass", required_argument, nullptr, 1008},
        {"max-thrust", required_argument, nullptr, 1009},
        {"arm-alt", required_argument, nullptr, 1010},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0},
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "i:p:l:r:vh", kLongOpts, nullptr)) != -1) {
        switch (opt) {
            case 'i':
                out.sitl_ip = optarg;
                break;
            case 'p':
                if (!parse_u16(optarg, out.simport)) {
                    std::fprintf(stderr, "非法 -p 端口: %s\n", optarg);
                    return 2;
                }
                break;
            case 'l':
                if (!parse_u16(optarg, out.listen_port)) {
                    std::fprintf(stderr, "非法 -l 端口: %s\n", optarg);
                    return 2;
                }
                listen_set = true;
                break;
            case 'r':
                if (!parse_double(optarg, out.rate_hz) || out.rate_hz <= 0 || out.rate_hz > 500) {
                    std::fprintf(stderr, "非法 -r 刷新率: %s（范围 0..500）\n", optarg);
                    return 2;
                }
                break;
            case 'v':
                out.verbose = true;
                break;
            case 'h':
                help_only = true;
                return 0;
            case 1000:
                out.mav_tcp_ip = optarg;
                break;
            case 1001:
                if (!parse_u16(optarg, out.mav_tcp_port)) {
                    std::fprintf(stderr, "非法 --mav-tcp-port: %s\n", optarg);
                    return 2;
                }
                break;
            case 1002:
                if (!parse_u16(optarg, out.gcs_udp_port)) {
                    std::fprintf(stderr, "非法 --gcs-udp-port: %s\n", optarg);
                    return 2;
                }
                break;
            case 1003:
                if (!parse_double(optarg, out.init_lat) || out.init_lat < -90 || out.init_lat > 90) {
                    std::fprintf(stderr, "非法 --lat: %s\n", optarg);
                    return 2;
                }
                break;
            case 1004:
                if (!parse_double(optarg, out.init_lon) || out.init_lon < -180 || out.init_lon > 180) {
                    std::fprintf(stderr, "非法 --lon: %s\n", optarg);
                    return 2;
                }
                break;
            case 1005:
                if (!parse_double(optarg, out.init_alt)) {
                    std::fprintf(stderr, "非法 --alt: %s\n", optarg);
                    return 2;
                }
                break;
            case 1006: {
                uint8_t id = 0;
                if (!parse_u8(optarg, id) || id == 0) {
                    std::fprintf(stderr, "非法 --id: %s（范围 1..255）\n", optarg);
                    return 2;
                }
                out.sys_id = id;
                break;
            }
            case 1007:
                if (!parse_u16(optarg, out.ctl_tcp_port)) {
                    std::fprintf(stderr, "非法 --ctl-port: %s\n", optarg);
                    return 2;
                }
                break;
            case 1008:
                if (!parse_double(optarg, out.mass_kg) || out.mass_kg <= 0) {
                    std::fprintf(stderr, "非法 --mass: %s\n", optarg);
                    return 2;
                }
                break;
            case 1009:
                if (!parse_double(optarg, out.max_thrust_n) || out.max_thrust_n <= 0) {
                    std::fprintf(stderr, "非法 --max-thrust: %s\n", optarg);
                    return 2;
                }
                break;
            case 1010:
                if (!parse_double(optarg, out.arm_alt_m)) {
                    std::fprintf(stderr, "非法 --arm-alt: %s\n", optarg);
                    return 2;
                }
                break;
            default:
                print_usage(argv[0]);
                return 2;
        }
    }
    if (optind < argc) {
        std::fprintf(stderr, "多余参数: %s\n", argv[optind]);
        print_usage(argv[0]);
        return 2;
    }
    if (!listen_set) out.listen_port = out.simport;
    // arm-alt 默认 = 初始高度（地面起飞）；显式传入（含 0）则尊重用户值。
    // 区分方式：argv 中出现过 --arm-alt 才覆盖，否则跟随 init_alt。
    bool arm_alt_given = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strncmp(argv[i], "--arm-alt", 9) == 0) arm_alt_given = true;
    }
    if (!arm_alt_given) out.arm_alt_m = out.init_alt;
    return 0;
}
