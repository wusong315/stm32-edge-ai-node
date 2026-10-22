#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""iotda_probe.py — 华为云 IoTDA · MQTT 明文 1883 连通性探针（纯标准库 / 零依赖）

为什么要有这个工具（2026-10-20，T10 上云开工轮）
--------------------------------------------------------------------------
ESP8266 D1 mini 的物流到 10.23 才到，但**云侧能不能连通、鉴权公式对不对**这件事
完全不需要硬件：PC 就是一台设备。把这个提前跑通，硬件一到就只剩"接线 + 发 AT"。

它同时是 **STM32 侧 C 实现的对照规格**：`--hex` 会把 CONNECT / PUBLISH 报文
逐字节打出来，C 端拼出来的字节必须和它一模一样（同 `D4` 逐位一致的精神）。

另一个作用：**测出"密码到底怎么算"**。官方中英文文档对 HMAC 的 key/msg 顺序
表述不一致（英文："timestamp as the key"；中文："以时间戳为密钥,对 secret 加密"，
但同句又写成 hmacsha256("secret","时间戳")）。本脚本 --auto 会两种都试，
用 CONNACK 的返回码把**真值测出来**，而不是猜。

官方依据（support.huaweicloud.com/api-iothub/iot_06_v5_3009.html）
--------------------------------------------------------------------------
  ClientId = "{device_id}_0_{sig_type}_{ts}"     sig_type: 0=不校验时间戳 / 1=校验
  Username = device_id
  Password = HMAC-SHA256(...) 的小写十六进制       （key/msg 顺序见 --auto）
  ts       = UTC 时间 "YYYYMMDDHH"

  🟢 **`sig_type=0` 时平台不校验时间戳**（2026-10-22 实测，推翻英文文档那句
     "the platform forcibly verifies the timestamp by default, regardless of
     whether the value is set to 0 or 1"）：
       sig=0 + ts ∈ {2020 年, 2030 年, 2099 年}  -> 全部 rc=0x00 连上
       sig=1 + 同一个错年份                      -> rc=0x04 被拒
     ⇒ `ClientId` 里的 `sig_type` 位**就是开关本身**。设备端**不需要 SNTP / RTC**，
       `ts` 用常量即可。⚠️ 但仍建议留可覆盖的常量，平台行为可能随版本变。
     （复现：`python tools/iotda_link_test.py ... --only ts`）

返回码（CONNACK byte 4）
--------------------------------------------------------------------------
  0x00 连接成功      0x01 协议版本错      0x02 clientId 格式错/心跳不合法
  0x03 服务不可用    0x04 用户名或密码错  0x05 未授权

用法
--------------------------------------------------------------------------
  # 0) 离线自测（不联网，验证报文编码）
  python iotda_probe.py --selftest

  # 1) 只看三样鉴权参数（不联网），顺便看报文十六进制
  python iotda_probe.py --device-id 5f6179df85_xxx --secret 12345678 --hex

  # 2) 真连（--auto 两种 HMAC 顺序都试，用返回码定真值）
  python iotda_probe.py --host <xxx>.st1.iotda-device.cn-north-4.myhuaweicloud.com \
                        --device-id 5f6179df85_xxx --secret 12345678 --auto

  # 3) 连上后顺手发一条 properties/report（验证 topic 与 JSON 格式）
  ... --publish --service-id vibration --similarity 42

注意：本机 pc-venv 未装 paho-mqtt（装包要先摘代理），所以这里直接用 socket 拼
MQTT 3.1.1 报文 —— 这反而是好事：拼报文正是 STM32 端要干的事。
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import socket
import sys
import time
from datetime import datetime, timezone

# ---------------------------------------------------------------- 基础编码

def _u16(n: int) -> bytes:
    return bytes(((n >> 8) & 0xFF, n & 0xFF))


def _mqtt_str(s: str) -> bytes:
    b = s.encode("utf-8")
    if len(b) > 0xFFFF:
        raise ValueError("字符串过长")
    return _u16(len(b)) + b


def _varint(n: int) -> bytes:
    """MQTT Remaining Length 变长编码（每字节 7 位，最高位是续位）。"""
    out = bytearray()
    while True:
        d = n % 128
        n //= 128
        if n > 0:
            out.append(d | 0x80)
        else:
            out.append(d)
            return bytes(out)


def _hexdump(b: bytes, width: int = 16) -> str:
    lines = []
    for i in range(0, len(b), width):
        chunk = b[i:i + width]
        hexs = " ".join(f"{x:02X}" for x in chunk)
        ascii_ = "".join(chr(x) if 32 <= x < 127 else "." for x in chunk)
        lines.append(f"  {i:04X}  {hexs:<{width * 3}}  {ascii_}")
    return "\n".join(lines)


# ---------------------------------------------------------------- 鉴权参数

def utc_stamp(when: datetime | None = None) -> str:
    """UTC 时间戳，格式 YYYYMMDDHH。"""
    dt = when or datetime.now(timezone.utc)
    return dt.strftime("%Y%m%d%H")


def build_auth(device_id: str, secret: str, ts: str, sig_type: int,
               password_mode: str) -> dict:
    """返回 {'client_id','username','password'}。

    password_mode:
      'ts_key'     —— HMAC(key=timestamp, msg=secret)   ← 英文文档口径（默认）
      'secret_key' —— HMAC(key=secret,    msg=timestamp) ← 中文文档另一种读法
    """
    if sig_type not in (0, 1):
        raise ValueError("sig_type 只能是 0 或 1")
    if not ts.isdigit() or len(ts) != 10:
        raise ValueError(f"时间戳必须是 10 位数字 YYYYMMDDHH，收到 {ts!r}")

    if password_mode == "ts_key":
        pw = hmac.new(ts.encode(), secret.encode(), hashlib.sha256).hexdigest()
    elif password_mode == "secret_key":
        pw = hmac.new(secret.encode(), ts.encode(), hashlib.sha256).hexdigest()
    else:
        raise ValueError(password_mode)

    return {
        "client_id": f"{device_id}_0_{sig_type}_{ts}",
        "username": device_id,
        "password": pw,
    }


# ---------------------------------------------------------------- MQTT 报文

def mqtt_connect(client_id: str, username: str, password: str,
                 keepalive: int = 120, clean: bool = True) -> bytes:
    """MQTT 3.1.1 CONNECT（用户名 + 密码）。"""
    flags = 0
    if clean:
        flags |= 0x02
    flags |= 0x80          # username
    flags |= 0x40          # password

    payload = _mqtt_str(client_id) + _mqtt_str(username) + _mqtt_str(password)
    var_hdr = _mqtt_str("MQTT") + bytes((0x04,)) + bytes((flags,)) + _u16(keepalive)
    body = var_hdr + payload
    return b"\x10" + _varint(len(body)) + body


def mqtt_publish(topic: str, payload: bytes, qos: int = 0) -> bytes:
    """PUBLISH（QoS 0）。"""
    body = _mqtt_str(topic) + payload
    return bytes((0x30 | (qos << 1),)) + _varint(len(body)) + body


def mqtt_pingreq() -> bytes:
    return b"\xC0\x00"


def mqtt_disconnect() -> bytes:
    return b"\xE0\x00"


RET_CODES = {
    0x00: "连接成功",
    0x01: "请求拒绝：协议版本错误",
    0x02: "请求拒绝：clientId 格式错 / 心跳不合法",
    0x03: "请求拒绝：服务不可用",
    0x04: "请求拒绝：用户名或密码错误",
    0x05: "请求拒绝：没有授权",
}


# ---------------------------------------------------------------- 网络

def _recv_connack(sock: socket.socket, timeout: float = 8.0) -> tuple[int, int]:
    """读 CONNACK，返回 (session_present, return_code)。"""
    sock.settimeout(timeout)
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            chunk = sock.recv(64)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        if len(buf) >= 4:
            break
    if len(buf) < 4:
        raise RuntimeError(f"没收到完整 CONNACK（收到 {len(buf)} 字节: {buf.hex()}）")
    if buf[0] != 0x20:
        raise RuntimeError(f"不是 CONNACK，首字节 0x{buf[0]:02X}（hex: {buf.hex()}）")
    return buf[2] & 0x01, buf[3]


def probe(host: str, port: int, device_id: str, secret: str, ts: str,
          sig_type: int, password_mode: str, do_publish: bool,
          service_id: str, similarity: int, verbose: bool) -> int:
    auth = build_auth(device_id, secret, ts, sig_type, password_mode)
    pkt = mqtt_connect(auth["client_id"], auth["username"], auth["password"])

    print(f"[鉴权] ts={ts}  sig_type={sig_type}  password_mode={password_mode}")
    print(f"  ClientId = {auth['client_id']}")
    print(f"  Username = {auth['username']}")
    print(f"  Password = {auth['password']}")
    if verbose:
        print(f"[CONNECT] {len(pkt)} 字节")
        print(_hexdump(pkt))

    try:
        sock = socket.create_connection((host, port), timeout=8.0)
    except OSError as e:
        print(f"🔴 TCP 连接失败 {host}:{port} —— {e}")
        print("   排查：域名拼写 / 是否开了 1883 / 本机代理是否拦截（可 env -u http_proxy ...）")
        return 2

    with sock:
        sock.sendall(pkt)
        try:
            sp, rc = _recv_connack(sock)
        except RuntimeError as e:
            print(f"🔴 {e}")
            return 3

        desc = RET_CODES.get(rc, "未知返回码")
        icon = "✅" if rc == 0 else "🔴"
        print(f"{icon} CONNACK: session_present={sp}  rc=0x{rc:02X} ({desc})")

        if rc != 0:
            return 1

        if do_publish:
            topic = f"$oc/devices/{device_id}/sys/properties/report"
            # 🔴 只报物模型里**确实存在**的两个属性（2026-10-22，D33）：
            #    `ts` 已从物模型删除，数据时间交给平台内建的 eventTime。
            #    多发一个未定义的属性不会报警，但平台直接忽略 ——
            #    那样"时间对了"就成了一句没有依据的话。
            #    这串 payload 与固件 main.c 的 App_CloudReport() **逐字一致**，
            #    对拍脚本 _tmp/hosttest/test_mqtt.c 用的也是它。
            body = {
                "services": [{
                    "service_id": service_id,
                    "properties": {
                        "similarity": similarity,
                        "verdict": "anomaly" if similarity < 90 else "normal",
                    },
                }]
            }
            payload = json.dumps(body, separators=(",", ":")).encode()
            pub = mqtt_publish(topic, payload)
            print(f"[PUBLISH] topic={topic}")
            print(f"          payload={payload.decode()}")
            if verbose:
                print(_hexdump(pub))
            sock.sendall(pub)
            sock.sendall(mqtt_pingreq())
            time.sleep(0.5)
            try:
                sock.sendall(mqtt_disconnect())
            except OSError:
                pass
            print("✅ 已发送 properties/report（QoS0，平台不回 ACK；去控制台看「设备影子/最新上报」）")
    return 0


# ---------------------------------------------------------------- 自测

def selftest() -> int:
    ok = 0
    fail = 0

    def chk(name, cond, extra=""):
        nonlocal ok, fail
        if cond:
            ok += 1
        else:
            fail += 1
            print(f"  🔴 {name} {extra}")

    # 1) 变长编码：MQTT 规范给出的边界值
    for n, exp in [(0, b"\x00"), (127, b"\x7f"), (128, b"\x80\x01"),
                   (16383, b"\xff\x7f"), (16384, b"\x80\x80\x01"),
                   (2097151, b"\xff\xff\x7f")]:
        chk(f"varint({n})", _varint(n) == exp, f"得到 {_varint(n).hex()} 期望 {exp.hex()}")

    # 2) 固定向量：拿一个已知 CONNECT 逐字节比
    #    clientId="A" user="U" pass="P" clean=1 keepalive=60
    #    变长头 10 B（2+4 "MQTT" +1 level +1 flags +2 keepalive）+ 负载 9 B（3×3）= 19 = 0x13
    pkt = mqtt_connect("A", "U", "P", keepalive=60)
    expect = bytes.fromhex(
        "101300044D51545404"   # 0x10, len=0x13(19), "MQTT", level 4
        "C2"                   # flags: user+pass+clean
        "003C"                 # keepalive 60
        "000141"               # clientId "A"
        "000155"               # username "U"
        "000150"               # password "P"
    )
    chk("CONNECT 固定向量", pkt == expect,
        f"\n     得到 {pkt.hex()}\n     期望 {expect.hex()}")

    # 3) 时间戳格式
    chk("utc_stamp 长度", len(utc_stamp()) == 10)
    chk("utc_stamp 纯数字", utc_stamp().isdigit())

    # 4) 两种 password_mode 必须不相同（否则说明参数没生效）
    a = build_auth("D", "S", "2026102020", 1, "ts_key")["password"]
    b = build_auth("D", "S", "2026102020", 1, "secret_key")["password"]
    chk("两种 HMAC 顺序不同", a != b)
    chk("password 是 64 位十六进制", len(a) == 64 and len(b) == 64)

    # 5) ClientId 拼装
    chk("ClientId 格式",
        build_auth("dev1", "S", "2026102020", 0, "ts_key")["client_id"]
        == "dev1_0_0_2026102020")

    # 6) sig_type 非法要报错
    try:
        build_auth("D", "S", "2026102020", 2, "ts_key")
        chk("sig_type 校验", False)
    except ValueError:
        chk("sig_type 校验", True)

    print(f"\n{ok} 通过 / {fail} 失败")
    return 0 if fail == 0 else 1


# ---------------------------------------------------------------- main

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="华为云 IoTDA MQTT(1883) 连通性探针 —— 纯标准库，零依赖")
    ap.add_argument("--host", help="设备接入域名，例如 xxx.st1.iotda-device.cn-north-4.myhuaweicloud.com")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--device-id", help="注册设备后平台返回的 device_id")
    ap.add_argument("--secret", help="注册设备时平台返回的 secret")
    ap.add_argument("--ts", help="UTC 时间戳 YYYYMMDDHH（默认取当前 UTC）")
    ap.add_argument("--sig-type", type=int, default=0, choices=[0, 1])
    ap.add_argument("--password-mode", default="ts_key",
                    choices=["ts_key", "secret_key"])
    ap.add_argument("--auto", action="store_true",
                    help="两种 HMAC 顺序都试，用 CONNACK 返回码定真值")
    ap.add_argument("--publish", action="store_true", help="连上后发一条 properties/report")
    ap.add_argument("--service-id", default="vibration")
    ap.add_argument("--similarity", type=int, default=42)
    ap.add_argument("--hex", action="store_true", help="打印报文逐字节十六进制（给 C 端对照）")
    ap.add_argument("--selftest", action="store_true", help="离线自测，不联网")
    a = ap.parse_args(argv)

    if a.selftest:
        return selftest()

    if not a.device_id or not a.secret:
        ap.error("需要 --device-id 与 --secret（--selftest 除外）")

    ts = a.ts or utc_stamp()

    if a.auto:
        if not a.host:
            ap.error("--auto 需要 --host")
        for mode in ("ts_key", "secret_key"):
            print(f"\n===== 尝试 password_mode={mode} =====")
            rc = probe(a.host, a.port, a.device_id, a.secret, ts,
                       a.sig_type, mode, a.publish, a.service_id,
                       a.similarity, a.hex)
            if rc == 0:
                print(f"\n✅ 真值确定：password_mode = {mode}（ts={ts}, sig_type={a.sig_type}）")
                return 0
            if rc != 1:
                return rc        # 不是鉴权问题，换 mode 也没用
        print("\n🔴 两种 HMAC 顺序都返回 0x04 ⇒ 不是顺序问题。查：device_id 是否抄全、"
              "secret 是否被重置过、ts 是否为当前 UTC 小时")
        return 1

    if not a.host:
        # 只出参数，不联网
        auth = build_auth(a.device_id, a.secret, ts, a.sig_type, a.password_mode)
        print(f"[鉴权] ts={ts}  sig_type={a.sig_type}  password_mode={a.password_mode}")
        print(f"  ClientId = {auth['client_id']}")
        print(f"  Username = {auth['username']}")
        print(f"  Password = {auth['password']}")
        if a.hex:
            pkt = mqtt_connect(auth["client_id"], auth["username"], auth["password"])
            print(f"\n[CONNECT] {len(pkt)} 字节 —— C 端必须拼出完全相同的字节")
            print(_hexdump(pkt))
        print("\n（没给 --host，只生成参数。要真连请加 --host）")
        return 0

    return probe(a.host, a.port, a.device_id, a.secret, ts, a.sig_type,
                 a.password_mode, a.publish, a.service_id, a.similarity, a.hex)


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    raise SystemExit(main())
