#!/bin/bash
# 三实例 SITL 启动脚本：eeprom_0/1/2.bin 各跑一个实例，端口错开。
# 基线只读：拷贝到 /tmp 工作副本再跑，CLI save 不脏仓库。
# 用法：./run_3sitl.sh [start|stop|status]，缺省 start。
# 端口：实例 N 基址 5760+N*10，UART1/2 = 基址/基址+1（Configurator 连 TCP localhost:<基址+1>，因 UART1 已改 320）。
set -u
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$SCRIPT_DIR/inav_10.0.0_SITL"
EEPROM_DIR="$SCRIPT_DIR/eeprom"
WORK_DIR="/tmp/maplrs_sitl"
BASE_PORT=5760
PORT_STEP=10

start_one() {
    local idx=$1
    local eeprom="$EEPROM_DIR/eeprom_${idx}.bin"
    local work="$WORK_DIR/sitl_${idx}.bin"
    local port=$((BASE_PORT + idx * PORT_STEP))
    local log="$WORK_DIR/sitl_${idx}.log"
    local pidfile="$WORK_DIR/sitl_${idx}.pid"
    if [ ! -f "$eeprom" ]; then echo "缺基线 $eeprom，跳过实例 $idx"; return 1; fi
    if [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
        echo "实例 $idx 已在跑（pid $(cat "$pidfile")），跳过"
        return 0
    fi
    cp "$eeprom" "$work"
    "$BIN" --path="$work" --tcpbaseport="$port" >"$log" 2>&1 &
    echo $! >"$pidfile"
    echo "实例 $idx 启动：pid $!，eeprom $work，UART1/2 = $port/$((port + 1))，日志 $log"
}

stop_one() {
    local idx=$1
    local pidfile="$WORK_DIR/sitl_${idx}.pid"
    if [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
        kill "$(cat "$pidfile")" && echo "实例 $idx 已停"
    else
        echo "实例 $idx 没在跑"
    fi
    rm -f "$pidfile"
}

status_one() {
    local idx=$1
    local port=$((BASE_PORT + idx * PORT_STEP))
    local pidfile="$WORK_DIR/sitl_${idx}.pid"
    if [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
        echo "实例 $idx 运行中：pid $(cat "$pidfile")，UART1/2 = $port/$((port + 1))"
    else
        echo "实例 $idx 未运行"
    fi
}

cmd="${1:-start}"
mkdir -p "$WORK_DIR"
case "$cmd" in
    start) for i in 0 1 2; do start_one "$i"; done
        echo "---"; echo "Configurator 连法（TCP）：实例0 localhost:5761，实例1 localhost:5771，实例2 localhost:5781" ;;
    stop) for i in 0 1 2; do stop_one "$i"; done ;;
    status) for i in 0 1 2; do status_one "$i"; done ;;
    *) echo "用法：$0 [start|stop|status]"; exit 1 ;;
esac
