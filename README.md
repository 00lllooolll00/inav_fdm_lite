# inav_fdm_lite

INAV SITL 无头 6-DOF 飞行动力学后端 + MAVLink 双向闭环测试桥（C++17，零第三方依赖，纯命令行）。

## 架构

```text
┌─────────────┐  UDP RREF/DREF   ┌──────────────────┐  TCP MAVLink v2  ┌─────────────┐
│  INAV SITL  │◄───────────────►│ inav_sitl_bridge │◄────────────────►│  SITL UART  │
│  --sim=xp   │  传感器注入/电机  │  6DOF + 遥测/控制 │  心跳/遥测/指令   │  (5760+N)   │
└─────────────┘                 └──────┬─────┬─────┘                  └─────────────┘
                                      │     │
                                 控制面 TCP  UDP 转发
                                 (690N)    (QGC 1455N)
```

两条并发链路：

1. **物理动力学链路（UDP）**：实现 X-Plane RREF/DREF 协议子集（参考 `fl2sitl` / `codeberg.org/stronnag/flightlog2kml` 的 `pkg/sitlgen`），被动监听 RREF 注册、回 `xitlDrefVersion=1`、按 id 回传传感器帧，并解析 DREF 电机量。
   四旋翼模型提供简化 Newton-Euler 刚体运动、姿态、位置、速度和传感器输出；模型范围与 JSBSim 的参考关系见下节。
2. **MAVLink 链路（TCP Client）**：连接 SITL 串口映射 TCP 端口，自研最小 MAVLink v2
   编解码（crc_extra 取自 `c_library_v2` 实测值，支持零截断帧）。收 `HEARTBEAT` /
   `GLOBAL_POSITION_INT` / `GPS_RAW_INT` / `VFR_HUD` / `ATTITUDE` / `COMMAND_ACK`；
   发 `COMMAND_INT(DO_REPOSITION)` / `RC_CHANNELS_OVERRIDE` / `COMMAND_LONG`。
   断线自动重连（指数退避），可选 UDP 转发给 QGC。


## 动力学范围与 JSBSim 参考

FDM 采用机体系 FRD、局部地理系 NED，按 `I*omega_dot = torque - omega x (I*omega)` 更新角速度，并用归一化四元数传播姿态。这些坐标和刚体运动方程参考 [JSBSim 坐标系说明](https://github.com/JSBSim-Team/jsbsim-reference-manual/blob/master/docs/user/concepts/frames-of-reference.md)、[运动方程](https://github.com/JSBSim-Team/jsbsim-reference-manual/blob/master/docs/formulation/equations-of-motion.md) 及 [官方实现](https://github.com/JSBSim-Team/jsbsim)。

本项目仍是轻量模型，不等同于 JSBSim：电机命令线性映射到推力，转动惯量/反扭矩/阻尼为简化参数，地面采用防穿透钳位；没有电机转速动态、旋翼入流、气动系数/风场或起落架接触动力学。X-Plane 链路只提供总油门和 yoke 控制量，缺少每个电机独立输入，因而不能从 SITL 输出还原真实四路电机差动。当前零第三方运行依赖的约束也意味着不直接链接 JSBSim。

## 构建（CMake + Ninja + just）

```bash
just build   # 配置 + 编译，产物 build/inav_sitl_bridge
just test    # 构建 + 自检（MAVLink 编解码/FDM/控制面/配置）
just ctest   # 同上，经 ctest
just clean   # 清理
```

编译器：g++ ≥ 9（C++17），`-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion` 零警告。

## 快速开始（单机）

SITL 端（UART3 配 MAVLink，见下）：

```bash
./inav_SITL --sim=xp --simip=127.0.0.1 --simport=49000 \
  --tcpbaseport=5760 --path ./uav1.bin \
  --chanmap=M01-01,M02-02,M03-03,M04-04
```

桥接端：

```bash
./build/inav_sitl_bridge --id 1 --lat 30.0 --lon 120.0 --alt 10.0 \
  -p 49000 -l 49000 --mav-tcp-port 5762 -r 50 \
  --ctl-port 6901 --gcs-udp-port 14551
```

控制台 1Hz 状态行：

```text
[UAV 1] Mode: GUIDED | Armed: YES | Alt: 12.4m | Sats: 16 | Throttle: 51% | XP: CONNECTED | MAVLink: CONNECTED
```

双机编队：`./scripts/duo.sh /path/to/inav_SITL`（端口规划见脚本头）。

## SITL 侧配置（关键）

1. 传感器全部选 **FAKE**（GPS/IMU/气压/磁罗盘等），数据才从仿真器来。
2. 启用 `feature TELEMETRY`，否则 SITL 不会绑定 MAVLink TCP 端口。将一个 UART 配为
   **MAVLink**；UART N 映射到 `tcpbaseport + N - 1`（如 UART1 对应 base，UART2 对应 base+1），
   桥接 `--mav-tcp-port` 必须指向该实际端口。
3. CLI：`set mavlink_sysid = <id>`（多机各不相同）；用 RC override 时，接收机选
   `MAVLink`（`SERIALRX_MAVLINK`），并在 Modes 页绑定 ARM / POSHOLD / GCS NAV / RTH。
4. 解锁先将 ARM 开关拉低并保持数秒，再拉高；若开关自锁未解除，再拨低后拨高一次。
5. `--chanmap` 映射电机输出；缺少它时桥收不到有效油门。

## 控制面（测试指令注入）

本地 TCP 文本协议（`--ctl-port` 开启，仅监听 127.0.0.1），一行一条，回复 `OK/ERR`：

```bash
# GUIDED 下打临时航点（目标位置读回 MSP WP 254，需先切 POSHOLD+GCS NAV）
echo 'REPOSITION 30.0005 120.0005 25' | nc 127.0.0.1 6901
# AUX 通道 override：无打杆解锁/模式切换（通道号按 Modes 页绑定改）
echo 'OVERRIDE_ONE 6 2000' | nc 127.0.0.1 6901   # CH6 打高
echo 'OVERRIDE_CLEAR'      | nc 127.0.0.1 6901   # 释放全部通道
# 通用透传
echo 'CMDINT 192 0 -1 0 0 0 300005000 1200005000 25' | nc 127.0.0.1 6901
echo 'CMDLONG 400 1 0 0 0 0 0 0' | nc 127.0.0.1 6901  # ARM（INAV 未实现则回 UNSUPPORTED）
```

注意（INAV 10.0.0 SITL 实测语义）：

- 本桥 `REPOSITION` 默认发送 `MAV_FRAME_GLOBAL(0)`；需已解锁、位置估计可信、GPS 原点有效，并处于有效 GCS NAV 的 3D POSHOLD。接受时回 `COMMAND_ACK=ACCEPTED`，更新导航目标。
- `MSP_WP(118)` 的 254 读回 GCS 目标位置，255 读回当前实际位置；不能用 WP255 判断目标是否写入。
- `RC_CHANNELS_OVERRIDE` 中 0 / 65535 表示该通道忽略；`COMMAND_LONG ARM(400)` 未实现，解锁请走 RC override 的 ARM 开关。
- SITL 的串口 TCP 实现存在重连风险：桥断开可能触发未屏蔽的 `SIGPIPE`，部分情况下重连后 MAVLink 下行停流；遇到此状态需重启 SITL。

## 命令行参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `-i` | 127.0.0.1 | SITL 所在 IP |
| `-p` | 49000 | SITL simport（UDP 对端） |
| `-l` | =`-p` | 本地 UDP 监听端口 |
| `-r` | 50 | 物理推演与注入刷新率 Hz |
| `--mav-tcp-ip/port` | 127.0.0.1:5762 | SITL MAVLink UART 的 TCP 映射 |
| `--gcs-udp-port` | 0 | QGC 转发 UDP 端口（0=关闭） |
| `--lat/--lon/--alt` | 30/120/10 | 初始经纬高 |
| `--id` | 1 | MAVLink sysid + 日志前缀 |
| `--ctl-port` | 0 | 控制面 TCP 端口（0=关闭） |
| `--mass/--max-thrust` | 1.2kg/30N | 悬停油门≈m·g/Tmax |
| `--arm-alt` | =`--alt` | 地面高度 AMSL |
| `-v` | off | 详细日志（RREF 注册/建联） |

## 文件结构

```text
src/  main.cpp  主循环（定频步进 + 1Hz 状态行）
      fdm.*     四旋翼 6DOF + 自洽传感器合成
      xplane_link.*  RREF/DREF UDP 链路
      mavlink_codec.*  最小 MAVLink v2 编解码
      mavlink_link.*   TCP 链路（重连/心跳/转发/指令）
      ctl_server.*     本地 TCP 控制面
      config.*         命令行解析
tests/selftest_main.cpp  零依赖自检
scripts/duo.sh  双机编队启动示例
```

## 验证记录

- `just test`：MAVLink 编解码、FDM 悬停/爬升/触地、陀螺耦合方向、姿态符号与航向保持、控制面解析和多实例配置——全过。
- 跨实现校验（pymavlink）：我方 HEARTBEAT 可被标准库解析；伪 SITL 发 HEARTBEAT+GPI+GPS+VFR 后状态行 `GUIDED/YES/16星/51%/CONNECTED`；控制面下发的 COMMAND_INT(192)/OVERRIDE 可被 pymavlink 正确解出字段。
- 隔离 EEPROM 副本静态联调：重力补偿后 `accZ=0`、`epv≈100`。进入 POSHOLD 后，高度超出目标时观察到 `corr=-219`、推力目标 `1081`，修正方向为减小推力；短测 FDM 仍上升至约 20 m，未证明高度最终收敛或稳定悬停。
- 隔离 EEPROM 副本 GUIDED 联调：`DO_REPOSITION` 收到 `COMMAND_ACK=ACCEPTED`；MSP 254 读回目标 `30.00049992, 120.00049992, 25 m`，MSP 255 仍为实际位置 `30.0, 120.0, 10 m`。水平位置不移动符合当前 SITL X-Plane 输出缺少姿态差动的限制；双实例连接及 QGC UDP 转发已验证。
- 原始 `inav_sitl/eeprom` 与用户固件均未修改。
