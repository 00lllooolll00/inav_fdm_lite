# Repository Guidelines

## Project Overview
`inav_fdm_lite` 是一个 C++17、零第三方运行时依赖的无头四旋翼 FDM 后端。它通过 X-Plane RREF/DREF UDP 与 INAV SITL（`--sim=xp`）互通：接收 SITL 的电机控制量，推进本地 6-DOF 动力学，并把位置、速度、姿态和传感器数据回传给 SITL。

桥接程序只负责 X-Plane UDP 与本地 FDM，不连接 MAVLink TCP、不转发 QGC，也不提供控制面。SITL 的 TCP/UART/MAVLink 链路由外部模块单独管理。

## Architecture & Data Flow
入口是 [`src/main.cpp`](src/main.cpp)。启动流程为：

1. `parse_args()` 生成 `BridgeConfig`。
2. 注册 `SIGINT`/`SIGTERM`，创建并初始化 `QuadFdm`。
3. 创建 `XplaneLink`，绑定 UDP 监听端口并启动接收线程。
4. 主循环按 `1 / rate_hz` 调度：读取最新电机量 -> `QuadFdm::step()` -> `XplaneLink::send_sensors()`。
5. 每秒输出状态；`-v` 追加 FDM 和位置诊断信息。

SITL 到桥：

- `RREF` 注册包建立 DREF 短名到 ID 的映射。
- `DREF` 包读取 `throttle_ratio_all` 和 `yoke_*_ratio`，按总油门加姿态差动重构 X 构型四路电机。
- 对端地址从收到的 UDP 包学习；DREF 映射缓存写入 `/tmp/inav_bridge_dref_<listen_port>.cache`。

桥到 SITL：

- 以 `RREF` 数据包返回已注册的 `id,value` 对。
- 注入字段包括经纬度、AMSL/离地高度、速度、姿态、比力、陀螺、气压、磁力、卫星数、定位状态、电池、RSSI、失控保护和测距仪。
- 默认注入频率为 `500 Hz`。INAV SITL 默认加计滤波配置下，`50 Hz` 可能造成 POSHOLD 高度环振荡。

坐标约定必须保持一致：FDM 机体系为 FRD（前右下），世界系为 NED（北东地），`vd > 0` 表示下降。X-Plane 本地速度字段为 `vx=东`、`vy=上`、`vz=南`。`theta`、`g_axil`、`g_side` 在发送时有协议兼容性的符号预变换；修改前要同时检查 `src/fdm.cpp` 和 `src/xplane_link.cpp`。

`XplaneLink` 的 UDP 接收在线程中运行，共享映射、对端地址和电机状态由 `std::mutex` 保护；`running_` 和 `connected_` 使用原子变量。析构路径必须通过 `stop()` 完成 `shutdown`、`close` 和 `join`。

## Key Directories
- `inc/`：公共头文件，当前为 `config.hpp`、`fdm.hpp`、`xplane_link.hpp`。
- `src/`：生产实现；入口、参数解析、FDM 和 X-Plane UDP 链路。
- `tests/`：自写断言式单元自检。
- `scripts/`：`duo.sh` 双实例 SITL + bridge 演示脚本（`sitl_air.py` 串口联调随 SITL 资产移到主仓 `inav_sitl/`）。
- SITL 二进制与 EEPROM 基线不在本仓：主仓 `inav_sitl/`（`inav_10.0.0_SITL`、`eeprom/`、`run_3sitl.sh`）；运行测试时使用工作副本，避免修改基线。
- `build/`、`build-release/`：CMake/Ninja 生成物，已被 `.gitignore` 忽略。

## Development Commands
需要 C++17 编译器、CMake 3.20+、Ninja 和 `just`。项目只链接系统 `pthread`。

```bash
just build          # RelWithDebInfo 构建到 build/
just release        # Release/-O2 构建到 build-release/
just test           # 构建并运行 bridge_selftest
just test-release   # 构建并运行 Release 自检
just ctest          # 通过 CTest 运行自检
just run            # 使用默认参数启动单实例 bridge
just format         # 对 src/ 和 tests/ 执行 clang-format
just clean          # 删除 build/ 和 build-release/
```

单实例参数可覆盖：

```bash
just run id=1 lat=30.0 lon=120.0 alt=10.0 simport=49000
```

直接启动 SITL 时必须提供电机映射：

```bash
./inav_SITL --sim=xp --simip=127.0.0.1 --simport=49000 \
  --path ./uav1.bin --chanmap=M01-01,M02-02,M03-03,M04-04
```

## Code Conventions & Common Patterns
- 使用 C++17、无扩展编译；新代码必须通过 `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion`。
- 公共接口放在 `inc/`，实现放在 `src/`；包含名保持简单，例如 `#include "fdm.hpp"`。
- 代码使用 4 空格缩进、`snake_case` 变量/函数、`PascalCase` 类型；现有注释以中文说明协议和坐标语义。
- 文件局部常量和字节序/钳位辅助函数放在匿名命名空间。
- 参数解析统一使用 `parse_u16`、`parse_u8`、`parse_double`，检查完整字符串、有限值和范围；非法 CLI 返回 `2` 并打印错误。
- 新增 CLI 参数要同时更新 `inc/config.hpp` 默认值、`src/config.cpp` 的 getopt 解析、README 参数表和 `tests/selftest_main.cpp`。
- 新增传感器字段要沿着 `SensorSnapshot` -> `QuadFdm::step()` -> `XplaneLink::send_sensors()` 完整传递，并明确单位、坐标和符号。
- 不要把 `kXitlDrefVersion` 改为 `2`：当前 SITL 闭环依赖版本 `1` 才会提供 `throttle_ratio_all`。
- X-Plane DREF 路径是 500 字节空格填充；去尾必须同时处理空格和 NUL，不能只按单个字符使用 `find_last_not_of`。
- `--arm-alt` 未显式传入时跟随 `--alt`；显式传入 `0` 也必须保留，相关逻辑在 `parse_args()` 末尾。

## Important Files
- [`src/main.cpp`](src/main.cpp)：程序入口、定频循环、信号处理和状态日志。
- [`src/config.cpp`](src/config.cpp)、[`inc/config.hpp`](inc/config.hpp)：全部 CLI 参数、默认值和校验。
- [`src/fdm.cpp`](src/fdm.cpp)、[`inc/fdm.hpp`](inc/fdm.hpp)：FDM 状态、四元数姿态、动力学和传感器快照。
- [`src/xplane_link.cpp`](src/xplane_link.cpp)、[`inc/xplane_link.hpp`](inc/xplane_link.hpp)：RREF/DREF UDP 协议、接收线程、缓存和电机解析。
- [`tests/selftest_main.cpp`](tests/selftest_main.cpp)：唯一自动测试目标及 `CHECK` 宏。
- [`CMakeLists.txt`](CMakeLists.txt)：target、编译选项、include 路径和 CTest 注册。
- [`justfile`](justfile)：日常构建、测试、运行和格式化入口。
- [`scripts/duo.sh`](scripts/duo.sh)：双实例端口和工作目录规划。
- [`README.md`](README.md)：用户可见参数、SITL 使用方式和已知限制。

## Runtime/Tooling Preferences
- 生产程序使用标准 C++17、POSIX socket 和 `pthread`；不要为简单功能增加第三方库。
- 构建生成器优先使用 Ninja，并通过 `just` 调用 CMake。
- 参数、端口、IP、经纬度和高度保持命令行可配置；不要把单实例联调值写死到运行逻辑。
- `scripts/duo.sh` 的端口规划为 UDP `49000/49001`，外部 SITL TCP 为 `5760/5770`；bridge 不管理 TCP 链路。
- `--chanmap=M01-01,M02-02,M03-03,M04-04` 是有效电机 DREF 的必要运行约定。
- `just format` 当前覆盖 `src/` 和 `tests/`；修改 `inc/` 后需按同一 `.clang-format` 规则手动格式化或单独运行 clang-format。

## Testing & QA
自动测试不使用 gtest/catch2。`tests/selftest_main.cpp` 通过自写 `CHECK` 宏直接构造 FDM 和配置解析对象；失败返回 `1`，全过输出 `SELFTEST PASS`。

当前覆盖：

- 悬停、高度稳定、1g 比力和水平加速度。
- 陀螺耦合、姿态响应、磁力计归一化。
- 爬升、断油下坠、触地防穿透和 X-Plane 速度符号。
- 默认参数、`500 Hz` 默认刷新率、端口、多实例参数和 `--arm-alt` 语义。

推荐验证顺序：

```bash
just test
just ctest
```

涉及 FDM、坐标符号、UDP 协议或默认参数的改动，还应做真实 SITL 联调。`xplane_link` 的网络编解码、主循环时序、多实例行为和 `chanmap` 仍主要依赖手动 SITL 验证，CTest 没有网络集成用例。SITL EEPROM 应先复制到 `/tmp` 或独立工作目录，避免 `save` 污染主仓 `inav_sitl/eeprom/` 基线。

已知限制：X-Plane 多旋翼控制量不足以表达完整姿态差动，水平位置闭环受限；FDM 不包含电机转速动态、旋翼入流、气动系数、风场和完整起落架模型。
