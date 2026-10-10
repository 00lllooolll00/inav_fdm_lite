#pragma once

#include <cstdint>
#include <string>

// 全局运行配置：所有网络接口与地理参数均由命令行传入，支持多实例多机。
struct BridgeConfig
{
    // --- X-Plane UDP 链路 ---
    std::string sitl_ip = "127.0.0.1"; // -i：SITL 所在 IP
    uint16_t simport = 49000; // -p：SITL 的 UDP 端口（RREF/DREF 对端）
    uint16_t listen_port = 49000; // -l：本地监听端口（默认与 -p 相同，单 socket 复用）
    double rate_hz = 500.0; // -r：物理推演与注入刷新率

    // --- 实例与地理 ---
    double init_lat = 30.0; // --lat
    double init_lon = 120.0; // --lon
    double init_alt = 10.0; // --alt（AMSL 米）
    uint8_t instance_id = 1; // --id（日志实例编号）

    // --- 物理模型微调 ---
    double mass_kg = 1.2; // --mass
    double max_thrust_n = 30.0; // --max-thrust（悬停油门≈mass*g/max_thrust）
    double arm_alt_m = 0.0; // --arm-alt（<0=水面起飞等特殊场景，默认地面=init_alt）
    bool verbose = false; // -v
};

// 解析失败返回非零并打印用法；--help 返回 0 且 help_only=true。
int parse_args(int argc, char **argv, BridgeConfig &out, bool &help_only);
void print_usage(const char *prog);
