# inav_fdm_lite

INAV SITL 的无头四旋翼 6-DOF 飞行动力学后端，通过 X-Plane RREF/DREF UDP 接口注入传感器并接收电机输出。C++17，零第三方依赖。

## 职责

```text
INAV SITL (--sim=xp)  <-- X-Plane RREF/DREF over UDP -->  inav_sitl_bridge
                                                       6-DOF FDM
```

桥接程序只管理 X-Plane UDP 与本地 FDM：监听 RREF 注册、响应 `xitlDrefVersion=1`、发送传感器数据，并解析 SITL 的电机 DREF。程序不连接 MAVLink TCP、不做 QGC 转发，也不提供 TCP 控制面。SITL 与用户外部模块之间的 MAVLink 通信由各自配置和进程管理，不属于本项目。

X-Plane 接口给出的多旋翼控制量不足以表达每个电机的差动，因此模型使用总油门推力，不能据此还原真实四路电机控制或实现完整水平位置闭环。

## 动力学范围

FDM 使用机体系 FRD 和局部地理系 NED，以简化 Newton-Euler 方程及归一化四元数推进姿态、速度和位置。电机命令线性映射到推力，转动惯量、反扭矩和阻尼为简化参数，地面使用防穿透钳位；没有电机转速动态、旋翼入流、气动系数、风场或起落架接触模型。坐标系与刚体运动方程参考 [JSBSim 坐标系说明](https://github.com/JSBSim-Team/jsbsim-reference-manual/blob/master/docs/user/concepts/frames-of-reference.md) 和 [运动方程](https://github.com/JSBSim-Team/jsbsim-reference-manual/blob/master/docs/formulation/equations-of-motion.md)，本项目不链接 JSBSim。

## 构建与测试

需要 C++17 编译器、CMake、Ninja 和 `just`。

```bash
just build   # 构建桥接程序和自检
just test    # 运行自检
just ctest   # 通过 CTest 运行自检
just clean   # 清理构建目录
```

编译启用 `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion`。

## 快速开始

启动 SITL，并映射电机输出通道：

```bash
./inav_SITL --sim=xp --simip=127.0.0.1 --simport=49000 \
  --path ./uav1.bin --chanmap=M01-01,M02-02,M03-03,M04-04
```

另一个终端启动桥接：

```bash
./build/inav_sitl_bridge --id 1 --lat 30.0 --lon 120.0 --alt 10.0 \
  -p 49000 -l 49000 -r 500
```

`-p` 是 SITL 的 UDP `simport`，`-l` 是桥接端监听端口，默认与 `-p` 相同。默认 `500 Hz` 与 INAV SITL 的加计滤波采样周期兼容；需要降低负载时可用 `-r` 调低，但可能增加高度环延迟。详细 FDM 日志使用 `-v`。桥接状态行示例：

```text
[UAV 1] Alt: 10.0m | Motor: 0% | XP: CONNECTED
```

双实例示例：`./scripts/duo.sh /path/to/inav_SITL`。脚本为每个实例分配独立 UDP 端口和 EEPROM 工作路径；其中 `--tcpbaseport` 只为外部模块保留，桥接不连接该端口。

## SITL 配置

- 启用 X-Plane 仿真模式 `--sim=xp`，并将 `--simport` 与桥接端口对应。
- 用 `--chanmap` 映射电机输出；缺少映射时桥接收不到有效电机 DREF。
- 仿真传感器应由 SITL 的 X-Plane 接口接收；无需为本桥配置 MAVLink UART、GCS 或控制面。
- 若同时运行 MAVLink 外部模块，请单独配置 SITL 的 TCP/UART 链路；桥接程序不会打开或转发该链路。

## 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `-i` | `127.0.0.1` | SITL 的 IP 地址 |
| `-p` | `49000` | SITL 的 X-Plane UDP 端口 |
| `-l` | 与 `-p` 相同 | 本地 UDP 监听端口 |
| `-r` | `500` | FDM 与传感器注入频率（Hz） |
| `--lat` / `--lon` / `--alt` | `30` / `120` / `10` | 初始纬度、经度和 AMSL 高度 |
| `--id` | `1` | 日志实例编号 |
| `--mass` | `1.2` | 机体质量（kg） |
| `--max-thrust` | `30` | 全油门总推力（N） |
| `--arm-alt` | 与 `--alt` 相同 | 地面高度 AMSL |
| `-v` | 关闭 | 输出详细 FDM 状态 |

## 验证与限制

- 自检覆盖 FDM 悬停、爬升、触地、姿态响应、陀螺耦合和多实例参数解析。
- 已用 INAV SITL 验证 X-Plane UDP 建联、电机 DREF 接收及传感器注入，并完成 ACRO 起飞/降落和模式切换联调。
- POSHOLD 高度环已在用户提供的 INAV SITL 固件上验证：默认 `500 Hz` 注入时最后 10 秒高度跨度约 `0.02 m`、垂向速度为 `0`。若使用 `50 Hz` 注入而保留 SITL 默认加计滤波配置，滤波延迟会导致高度环振荡；应优先保持 `500 Hz`，或在临时测试 EEPROM 中关闭 `acc_lpf_hz`。
- 当前 X-Plane 多旋翼输出缺少姿态差动输入，水平位置控制能力受限。

## 文件结构

```text
src/main.cpp              定频主循环
src/config.cpp            命令行参数解析
src/fdm.cpp               四旋翼6-DOF动力学与传感器合成
src/xplane_link.cpp       X-Plane RREF/DREF UDP链路
inc/*.hpp                 公共头文件
tests/selftest_main.cpp   FDM与配置自检
scripts/duo.sh            双实例启动示例
```
