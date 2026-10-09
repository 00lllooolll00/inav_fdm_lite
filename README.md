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

1. **物理动力学链路（UDP）**：实现 X-Plane RREF/DREF 协议子集（对标 `fl2sitl` /
   `codeberg.org/stronnag/flightlog2kml` 的 `pkg/sitlgen` 行为，被动监听 RREF 注册、
   回 `xitlDrefVersion=2`、按 id 回传传感器帧；解析 DREF 电机包）。
   四旋翼 X 构型 6DOF：油门→推力/力矩→平动/转动积分→自洽传感器（姿态、角速度、
   经纬高、地速、比力过载、气压、地磁、GPS 星数等）。
2. **MAVLink 链路（TCP Client）**：连接 SITL 串口映射 TCP 端口，自研最小 MAVLink v2
   编解码（crc_extra 取自 `c_library_v2` 实测值，支持零截断帧）。收 `HEARTBEAT` /
   `GLOBAL_POSITION_INT` / `GPS_RAW_INT` / `VFR_HUD` / `ATTITUDE` / `COMMAND_ACK`；
   发 `COMMAND_INT(DO_REPOSITION)` / `RC_CHANNELS_OVERRIDE` / `COMMAND_LONG`。
   断线自动重连（指数退避），可选 UDP 转发给 QGC。

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
2. 某 UART（如 UART3）功能选 **MAVLink**，波特率任意（TCP 映射忽略波特率）；
   SITL 的 UART3 → TCP `tcpbaseport+2`（默认 5762），桥接 `--mav-tcp-port` 对准它。
3. CLI：`set mavlink_sysid = <--id>`（多机各不相同），`feature TELEMETRY` 按需开。
4. 用 RC override 做解锁/模式切换时，接收机类型选 `MAVLink`（SERIALRX_MAVLINK），
   并在 Modes 页把 ARM / POSHOLD / GCS NAV / RTH 绑到 AUX 通道。
5. `--chanmap` 把 4 路电机映射出来（多旋翼机型），否则桥收不到油门、飞机会一直贴地。

## 控制面（测试指令注入）

本地 TCP 文本协议（`--ctl-port` 开启，仅监听 127.0.0.1），一行一条，回复 `OK/ERR`：

```bash
# GUIDED 下打临时航点（Index 255，需先切 POSHOLD+GCS NAV）
echo 'REPOSITION 30.0005 120.0005 25' | nc 127.0.0.1 6901
# AUX 通道 override：无打杆解锁/模式切换（通道号按 Modes 页绑定改）
echo 'OVERRIDE_ONE 6 2000' | nc 127.0.0.1 6901   # CH6 打高
echo 'OVERRIDE_CLEAR'      | nc 127.0.0.1 6901   # 释放全部通道
# 通用透传
echo 'CMDINT 192 0 -1 0 0 0 300005000 1200005000 25' | nc 127.0.0.1 6901
echo 'CMDLONG 400 1 0 0 0 0 0 0' | nc 127.0.0.1 6901  # ARM（INAV 未实现则回 UNSUPPORTED）
```

注意（INAV 9.1/10.0-RC2 实测语义，见 `src/main/telemetry/mavlink.c`）：

- `DO_REPOSITION` 只接受 `MAV_FRAME_GLOBAL(0)`，且需 GCS NAV 有效，否则回 `DENIED`；
  航点写入 Index 255。
- `RC_CHANNELS_OVERRIDE` 中 0 / 65535 表示该通道忽略；INAV 不校验 target_system。
- `COMMAND_LONG ARM(400)` INAV 未实现，会回 `UNSUPPORTED`——解锁请走 RC override 的 ARM 开关。

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

- `bridge_selftest`：crc_extra（10 条消息）/ 组帧往返 / 坏 CRC 丢弃 / 零截断解码 /
  悬停±1m / 爬升 / 触地钳位 / 姿态响应 / 控制面解析 / 多实例配置——全过。
- 跨实现校验（pymavlink）：我方 HEARTBEAT 可被标准库解析；伪 SITL 发 HEARTBEAT+
  GPI+GPS+VFR 后状态行 `GUIDED/YES/16星/51%/CONNECTED`；控制面下发的
  COMMAND_INT(192)/OVERRIDE 可被 pymavlink 正确解出字段。
- 未覆盖：真实 INAV SITL 联调（待用户环境验证，重点看 EKF 是否收敛、模式切换链路）。
