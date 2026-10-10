#!/usr/bin/env bash
# 双机编队启动示例：两份 SITL + 两个 UDP/FDM 桥接实例。
# 用法：./scripts/duo.sh <inav_SITL路径> [基址目录]
# 端口规划（实例 N=1,2）：
#   X-Plane UDP: simport=49000+N-1
#   SITL TCP:    tcpbase=5760+(N-1)*10（仅供外部模块使用，本脚本不管理该链路）
set -euo pipefail
SITL_BIN="${1:?用法: $0 <inav_SITL可执行文件> [工作目录]}"
WORK="${2:-$PWD/sitl_run}"
BRIDGE="$PWD/build/inav_sitl_bridge"

mkdir -p "$WORK"
just build >/dev/null

# UAV1
"$SITL_BIN" --sim=xp --simip=127.0.0.1 --simport=49000 \
  --tcpbaseport=5760 --path "$WORK/uav1.bin" \
  --chanmap=M01-01,M02-02,M03-03,M04-04 >/tmp/sitl1.log 2>&1 &
SITL1=$!
"$BRIDGE" --id 1 --lat 30.0 --lon 120.0 --alt 10.0 \
  -p 49000 -l 49000 -r 500 >/tmp/bridge1.log 2>&1 &
BRG1=$!

# UAV2（向东 50 米，高度相同）
"$SITL_BIN" --sim=xp --simip=127.0.0.1 --simport=49001 \
  --tcpbaseport=5770 --path "$WORK/uav2.bin" \
  --chanmap=M01-01,M02-02,M03-03,M04-04 >/tmp/sitl2.log 2>&1 &
SITL2=$!
"$BRIDGE" --id 2 --lat 30.0 --lon 120.0005 --alt 10.0 \
  -p 49001 -l 49001 -r 500 >/tmp/bridge2.log 2>&1 &
BRG2=$!

trap 'kill $SITL1 $SITL2 $BRG1 $BRG2 2>/dev/null; exit 0' INT TERM
echo "双机运行中：SITL($SITL1,$SITL2) 桥接($BRG1,$BRG2)，Ctrl-C 退出"
echo "  tail -f /tmp/bridge1.log /tmp/bridge2.log"
wait
