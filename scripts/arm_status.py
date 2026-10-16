#!/usr/bin/env python3
"""读 INAV SITL 的解锁状态：armingFlags 解析 + RC 通道 + ARM 盒映射（MSP，默认 5771）。

用法：python3 arm_status.py [msp端口]
位定义来自 INAV src/main/fc/runtime_config.h；盒激活判定 (rc-900)/25 落在配置范围内。
SITL 每个 UART 只服务一个客户端：本工具先写标志文件请仿真桥让位，退出时自动删除。
"""
import os
import socket
import struct
import sys
import time

# armingFlags：前几个是状态位（非禁用原因），其余是禁用原因
STATUS_BITS = {
    2: "ARMED", 3: "WAS_EVER_ARMED", 4: "SIMULATOR_MODE_HITL", 5: "SIMULATOR_MODE_SITL",
}
DISABLE_BITS = {
    6: "GEOZONE", 7: "FAILSAFE_SYSTEM", 8: "NOT_LEVEL", 9: "SENSORS_CALIBRATING",
    10: "SYSTEM_OVERLOADED", 11: "NAVIGATION_UNSAFE", 12: "COMPASS_NOT_CALIBRATED",
    13: "ACCELEROMETER_NOT_CALIBRATED", 14: "ARM_SWITCH", 15: "HARDWARE_FAILURE",
    16: "BOXFAILSAFE", 18: "RC_LINK", 19: "THROTTLE", 20: "CLI", 21: "CMS_MENU",
    22: "OSD_MENU", 23: "ROLLPITCH_NOT_CENTERED", 24: "SERVO_AUTOTRIM", 25: "OOM",
    26: "INVALID_SETTING", 27: "PWM_OUTPUT_ERROR", 28: "NO_PREARM", 29: "DSHOT_BEEPER",
    30: "LANDING_DETECTED",
}


def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def req(cmd, payload=b""):
    head = b"\x24\x58\x3c" + bytes([0]) + struct.pack("<HH", cmd, len(payload))
    return head + payload + bytes([crc8(head[3:] + payload)])


def read_payload(sock, timeout=0.8):
    sock.settimeout(timeout)
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        try:
            chunk = sock.recv(2048)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        i = buf.find(b"$X>")
        if i >= 0 and len(buf) >= i + 9:
            size = struct.unpack("<H", buf[i + 6:i + 8])[0]
            if len(buf) >= i + 9 + size:
                return buf[i + 8:i + 8 + size]
    return None


def ask(sock, cmd):
    sock.sendall(req(cmd))
    time.sleep(0.25)
    return read_payload(sock)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5771
    # 请仿真桥让位：SITL 每个 UART 口只服务第一个客户端，桥长期占着会让本工具读不到数据
    flag = f"/tmp/maplrs_msp_yield_{port}"
    try:
        with open(flag, "w", encoding="ascii") as f:
            f.write(str(time.time()))
    except OSError:
        flag = None
    try:
        run(port)
    finally:
        if flag:
            try:
                os.remove(flag)
            except OSError:
                pass


def run(port):
    s = socket.create_connection(("127.0.0.1", port), timeout=2)

    # 等桥让位：最多 4 秒；期间的连接会排队，桥一断开即被 SITL 接管
    p = None
    deadline = time.time() + 4.0
    while p is None and time.time() < deadline:
        p = ask(s, 150)                              # MSP_STATUS_EX
        if p is None:
            time.sleep(0.4)
    arming = None
    if p and len(p) >= 16:
        arming = struct.unpack("<H", p[13:15])[0]
        print(f"armingFlags = {arming:#06x}")
        for bit, name in STATUS_BITS.items():
            if arming & (1 << bit):
                print(f"  [状态] {name}")
        reasons = [name for bit, name in DISABLE_BITS.items() if arming & (1 << bit)]
        print("  阻止解锁: " + (", ".join(reasons) if reasons else "无（可解锁）"))

    rc_payload = ask(s, 105)                         # MSP_RC
    chans = ()
    if rc_payload:
        chans = struct.unpack(f"<{len(rc_payload) // 2}H", rc_payload)
        # 顺序按 INAV 的 rcData：roll/pitch/yaw/throttle 后接 aux1..，不是 CH1.. 的编号顺序
        print(f"RC 通道(roll,pitch,yaw,thr,aux1..): {list(chans[:9])}")

    ids = ask(s, 119)                                # MSP_BOXIDS（永久 id）
    names = ask(s, 116)                              # MSP_BOXNAMES
    ranges = ask(s, 34)                              # MSP_MODE_RANGES
    if ids and names and ranges:
        name_list = names.decode("ascii", "replace").split(";")
        arm_chan = arm_range = None
        for i in range(len(ranges) // 4):
            pid, aux, start, end = ranges[i * 4:i * 4 + 4]
            if start == 0 and end == 0:              # INAV 未使用的条件
                continue
            name = name_list[ids.index(pid)] if pid in ids and ids.index(pid) < len(name_list) else "?"
            if name == "ARM":
                arm_chan, arm_range = aux + 5, (start, end)
                print(f"ARM 盒: CH{arm_chan} 范围[{start},{end}]"
                      f"（PWM {900 + start * 25}~{900 + end * 25}）")
        if arm_chan and chans:
            rc = chans[arm_chan - 1]
            active = arm_range[0] <= (rc - 900) // 25 <= arm_range[1]
            print(f"当前 CH{arm_chan}={rc} -> ARM 盒{'激活' if active else '未激活'}")
            if arming is not None and arming & (1 << 14) and active:
                print("提示：ARM_SWITCH 自锁 —— 在 QGC 点一次『上锁』再点『解锁』，"
                      "让通道回到低位再跳变即可解除")
    s.close()


if __name__ == "__main__":
    main()
