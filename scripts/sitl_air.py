#!/usr/bin/env python3
"""SITL + FDM + 串口桥，前台常驻：每组 <串口设备>=<eeprom> 起一套独立实例。

用法：
  ./sitl_air.py /dev/ttyUSB0=eeprom_1.bin
  ./sitl_air.py /dev/ttyUSB0=eeprom_0.bin /dev/ttyUSB1=eeprom_1.bin
  ./sitl_air.py /dev/ttyUSB0=eeprom_1.bin@30.6580,104.0670 /dev/ttyUSB1=eeprom_2.bin@30.6560,104.0650

每组参数：<串口设备>=<eeprom>[@lat,lon]
  eeprom：编号（1）、文件名（eeprom_1.bin）或任意路径（相对当前目录）；
  @lat,lon：可选初始位置，默认 30.6570,104.0660（成都天府广场）。

每组端口（i 从 0 起，与串口设备一一对应）：
  UART1 = TCP 5770+10i（AIR 模块的 MAVLink+RX 口）、UART2 = TCP 5771+10i（Configurator/MSP）、
  X-Plane UDP = 49000+i。

日志带 [组号] 前缀实时输出；串口桥断线自动重连；Ctrl-C 退出并回收全部子进程。
依赖 pyserial；首次使用先 `just build` 编译 bridge。
"""
import os
import shutil
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SITL_BIN = os.path.join(ROOT, "inav_sitl", "inav_10.0.0_SITL")
BRIDGE_BIN = os.path.join(ROOT, "build", "inav_sitl_bridge")
EEPROM_DIR = os.path.join(ROOT, "inav_sitl", "eeprom")
WORK_DIR = "/tmp/maplrs_sitl_air"
BAUD = 115200
DEFAULT_LAT, DEFAULT_LON = "30.6570", "104.0660"


class Uav:
    def __init__(self, idx, dev, eeprom, lat, lon):
        self.idx = idx
        self.dev = dev
        self.eeprom = eeprom
        self.lat = lat
        self.lon = lon
        self.tcp_base = 5770 + idx * 10      # UART1(MAVLink)；UART2 = +1
        self.sim_port = 49000 + idx


def resolve_eeprom(text):
    if os.path.exists(text):
        return os.path.abspath(text)
    name = f"eeprom_{text}.bin" if text.isdigit() else (text if text.endswith(".bin") else f"{text}.bin")
    path = os.path.join(EEPROM_DIR, name)
    if not os.path.exists(path):
        sys.exit(f"找不到 eeprom：{text}（也不是 {path}）")
    return path


def parse_spec(idx, text):
    if "=" not in text:
        sys.exit(f"参数格式应为 <串口设备>=<eeprom>[@lat,lon]，收到：{text}")
    dev, rest = text.split("=", 1)
    lat, lon = DEFAULT_LAT, DEFAULT_LON
    if "@" in rest:
        rest, pos = rest.split("@", 1)
        parts = pos.split(",")
        if len(parts) != 2:
            sys.exit(f"位置格式应为 @lat,lon，收到：@{pos}")
        lat, lon = parts
    return Uav(idx, dev, resolve_eeprom(rest), lat, lon)


_LOG_LOCK = threading.Lock()


def log(text):
    """多实例日志统一出口：单次 write，线程安全。"""
    with _LOG_LOCK:
        sys.stdout.write(text + "\n")
        sys.stdout.flush()


def pump(tag, stream, stop):
    """转发子进程输出，带组号前缀。"""
    try:
        for line in iter(stream.readline, b""):
            if stop.is_set():
                break
            text = line.decode("utf-8", "replace").rstrip()
            if text:
                log(f"[{tag}] {text}")
    except (OSError, ValueError):
        pass


def serial_bridge(uav, stop):
    """TCP(UART1) <-> 串口 双向透传，断线自动重连，每 10 秒报一次流量。"""
    import selectors
    import serial

    tag = uav.idx
    while not stop.is_set():
        try:
            sock = socket.create_connection(("127.0.0.1", uav.tcp_base), timeout=2)
        except OSError:
            stop.wait(1.0)
            continue
        try:
            ser = serial.Serial(uav.dev, BAUD, timeout=0)
        except Exception as err:                     # pylint: disable=broad-except
            log(f"[{tag}] 串口 {uav.dev} 打开失败：{err}，2 秒后重试")
            sock.close()
            stop.wait(2.0)
            continue

        log(f"[{tag}] {uav.dev} <-> SITL UART1(TCP {uav.tcp_base}) 已接通")
        up = down = 0
        last_report = time.time()
        reason = ""
        sel = selectors.DefaultSelector()
        sel.register(sock, selectors.EVENT_READ, "tcp")
        sel.register(ser, selectors.EVENT_READ, "ser")
        while not stop.is_set() and not reason:
            for key, _ in sel.select(timeout=0.5):
                if key.data == "tcp":
                    try:
                        data = sock.recv(4096)
                    except OSError:
                        reason = "TCP 读失败"
                        break
                    if not data:
                        reason = "SITL 侧断开"
                        break
                    ser.write(data)
                    down += len(data)
                else:
                    try:
                        data = ser.read(4096)
                    except (OSError, serial.SerialException):
                        reason = "串口读失败"
                        break
                    if data:
                        try:
                            sock.sendall(data)
                        except OSError:
                            reason = "TCP 写失败"
                            break
                        up += len(data)
            if time.time() - last_report >= 10.0:
                log(f"[{tag}] 串口 {uav.dev}：收 {up}B 发 {down}B")
                last_report = time.time()
        sel.close()
        ser.close()
        sock.close()
        if not stop.is_set():
            log(f"[{tag}] 连接结束（{reason}，收 {up}B 发 {down}B），重连中")
            stop.wait(1.0)


def shutdown(procs):
    for p in procs:
        if p.poll() is None:
            p.terminate()
    for p in procs:
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        log(__doc__.strip())
        sys.exit(0 if len(sys.argv) > 1 else 1)

    for path in (SITL_BIN, BRIDGE_BIN):
        if not os.path.exists(path):
            sys.exit(f"缺 {path}（bridge 先 `just build`）")
    try:
        import serial                                # noqa: F401
    except ImportError:
        sys.exit("缺 pyserial：pip install pyserial")

    uavs = [parse_spec(i, text) for i, text in enumerate(sys.argv[1:])]
    os.makedirs(WORK_DIR, exist_ok=True)

    stop = threading.Event()
    procs = []
    log("实例映射：")
    for u in uavs:
        log(f"  [{u.idx}] {u.dev} <-> {os.path.basename(u.eeprom)} @({u.lat},{u.lon})"
            f"  UART1=TCP {u.tcp_base} / UART2=TCP {u.tcp_base + 1} / XP-UDP {u.sim_port}")

    try:
        for u in uavs:
            work_eeprom = os.path.join(WORK_DIR, f"uav{u.idx}.bin")
            shutil.copyfile(u.eeprom, work_eeprom)   # 只读基线，SITL 写在 /tmp 副本
            sitl = subprocess.Popen(
                [SITL_BIN, f"--path={work_eeprom}", f"--tcpbaseport={u.tcp_base}",
                 "--sim=xp", "--simip=127.0.0.1", f"--simport={u.sim_port}",
                 "--chanmap=M01-01,M02-02,M03-03,M04-04"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, cwd=ROOT)
            bridge = subprocess.Popen(
                [BRIDGE_BIN, "--id", str(u.idx + 1), "--lat", u.lat, "--lon", u.lon,
                 "--alt", "10.0", "-p", str(u.sim_port), "-l", str(u.sim_port), "-r", "500"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, cwd=ROOT)
            procs += [sitl, bridge]
            for proc in (sitl, bridge):
                threading.Thread(target=pump, args=(u.idx, proc.stdout, stop), daemon=True).start()
            threading.Thread(target=serial_bridge, args=(u, stop), daemon=True).start()

        log("运行中，Ctrl-C 退出\n")
        while not stop.is_set():
            if all(p.poll() is not None for p in procs):
                log("全部子进程已退出")
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        log("\n收到 Ctrl-C，回收子进程…")
    finally:
        stop.set()
        shutdown(procs)
        log("已退出")


if __name__ == "__main__":
    main()
