#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""iotda_link_test.py — IoTDA 连接层语义实测：keepalive 边界 / 心跳 / QoS1 / 同 clientId 双连接

为什么单独一个工具（2026-10-21）
--------------------------------------------------------------------------
`iotda_probe.py` 只回答「能不能连上」。但 10.23 联调会撞上的是另一批问题，官方文档
要么没写、要么写得含糊：

  ① keepalive 能填多少？填小了会怎样？
  ② 不发 PINGREQ 多久被踢？踢之前有什么征兆？
  ③ QoS1 平台回不回 PUBACK？（QoS0 没有回执 ⇒ "发出去了"≠"平台收到了"）
  ④ 同一个 device_id 同时开 PC 探针和板子，会互相踢吗？

这四件事全都**不需要 ESP8266**，PC 就能测完。测出来的结论写进 `RUNBOOK` R14.8。

2026-10-21 实测结论（本项目实例 cn-north-4）
--------------------------------------------------------------------------
  keepalive 合法区间  : **30 .. 1200**（0/1/5/10/20 与 1201/3600 全部 rc=0x02）
  不发心跳被踢        : keepalive=30 ⇒ **44.6 s**（≈1.5×）
  PINGREQ             : 回 `D0 00`（PINGRESP）
  QoS1 PUBLISH        : 回 `40 02 00 01`（PUBACK，packet id = 1）
  同 clientId 双连接  : 🔴 **两条都 rc=0x00、都能收到 PUBACK —— 平台不踢！**
                       ⇒ 影子值是"最后一条"的，而你在 PC 上看不出板子还在发

用法
--------------------------------------------------------------------------
  python iotda_link_test.py --host <域名> --device-id <id> --secret <s>
  python iotda_link_test.py ... --only keepalive     # 只测边界（快，~20 s）
  python iotda_link_test.py ... --only dup           # 只测双连接（快）

⚠️ 会占用 device_id；测之前先把板子断电 / 断开，否则测出来的"双连接"是三条。
"""
import argparse
import socket
import sys
import time

import iotda_probe as P


def _open(host, port, device_id, secret, keepalive):
    ts = P.utc_stamp()
    a = P.build_auth(device_id, secret, ts, 0, "ts_key")
    s = socket.create_connection((host, port), timeout=8)
    s.sendall(P.mqtt_connect(a["client_id"], a["username"], a["password"],
                             keepalive=keepalive))
    sp, rc = P._recv_connack(s)
    s.settimeout(3.0)
    return s, rc


def test_keepalive(host, port, dev, sec):
    """扫 keepalive 边界：找最小与最大合法值。"""
    print("\n===== ① keepalive 边界 =====")
    ok = []
    for ka in (0, 1, 5, 10, 20, 29, 30, 60, 120, 600, 1200, 1201, 3600):
        try:
            s, rc = _open(host, port, dev, sec, ka)
            s.sendall(P.mqtt_disconnect())
            s.close()
        except OSError as e:
            print(f"  keepalive={ka:5d} -> 连接异常 {e}")
            continue
        tag = "OK" if rc == 0 else f"REJECT(rc=0x{rc:02X})"
        print(f"  keepalive={ka:5d} -> {tag}")
        if rc == 0:
            ok.append(ka)
    if ok:
        print(f"  ⇒ 合法区间落在 [{min(ok)}, {max(ok)}]；rc=0x02 表示平台认为心跳值不合法")
    return bool(ok)


def test_heartbeat(host, port, dev, sec, keepalive=30):
    """① 空等看何时被踢  ② 发 PINGREQ 看能否保活。"""
    print(f"\n===== ② 心跳（keepalive={keepalive}）=====")
    print(f"  [2a] 不发 PINGREQ，最多等 {int(keepalive*2)} s ……")
    s, rc = _open(host, port, dev, sec, keepalive)
    if rc:
        print(f"  ❌ 连不上 rc=0x{rc:02X}")
        return False
    s.settimeout(0.5)
    t0, kicked_at = time.time(), None
    while time.time() - t0 < keepalive * 2:
        try:
            d = s.recv(64)
        except socket.timeout:
            continue
        except OSError:
            kicked_at = time.time() - t0
            break
        if not d:
            kicked_at = time.time() - t0
            break
    if kicked_at:
        print(f"  ✅ {kicked_at:.1f} s 被平台断开（≈1.5×keepalive）")
    else:
        print(f"  ⚠️ {keepalive*2} s 内没被踢")
    s.close()

    print("  [2b] 每 5 s 发 PINGREQ ……")
    s, rc = _open(host, port, dev, sec, keepalive)
    good, t0 = 0, time.time()
    for _ in range(4):
        time.sleep(5)
        s.sendall(P.mqtt_pingreq())
        try:
            d = s.recv(64)
        except socket.timeout:
            print("   ⚠️ 无 PINGRESP")
            continue
        if not d:
            print("   ❌ 连接已断")
            break
        good += 1
    print(f"  ⇒ {good}/4 收到 PINGRESP，{time.time()-t0:.1f} s 未掉线")
    s.sendall(P.mqtt_disconnect())
    s.close()
    return kicked_at is not None and good >= 4


def _pub_qos1(s, dev, sim):
    topic = f"$oc/devices/{dev}/sys/properties/report".encode()
    payload = ('{"services":[{"service_id":"vibration",'
               '"properties":{"similarity":%d}}]}' % sim).encode()
    body = P._u16(len(topic)) + topic + P._u16(1) + payload
    s.sendall(bytes([0x32]) + P._varint(len(body)) + body)
    try:
        d = s.recv(64)
    except socket.timeout:
        return None
    if not d:
        return False
    return (d[0] & 0xF0) == 0x40  # PUBACK


def test_dup(host, port, dev, sec):
    """同 clientId 两条连接：平台踢不踢？谁还能上报？用 QoS1 的 PUBACK 判活。"""
    print("\n===== ③ 同 clientId 双连接 =====")
    a, rca = _open(host, port, dev, sec, 120)
    print(f"  A 连接 rc=0x{rca:02X}")
    b, rcb = _open(host, port, dev, sec, 120)
    print(f"  B 连接 rc=0x{rcb:02X}（同 clientId）")
    time.sleep(2)
    ra = _pub_qos1(a, dev, 88)
    rb = _pub_qos1(b, dev, 77)
    print(f"  A QoS1 -> {'PUBACK ✅' if ra else ('被踢' if ra is False else '超时')}")
    print(f"  B QoS1 -> {'PUBACK ✅' if rb else ('被踢' if rb is False else '超时')}")
    if ra and rb:
        print("  🔴 两条连接同时有效 ⇒ 平台**不踢重复 clientId**。")
        print("     ⇒ 影子值是最后一条的；而 PC 上看不出板子还在发。联调时务必只留一条。")
    a.close()
    b.close()
    return ra is True and rb is True


def test_ts(host, port, dev, sec):
    """sig_type 与 ts 的组合矩阵 —— 判平台到底校不校时间。

    2026-10-22 实测（cn-north-4）：sig=0 对任何 ts 都放行；sig=1 才严格校验。
    ⇒ ClientId 里的 sig_type 位就是开关本身；设备端不需要 SNTP / RTC。
    """
    print("\n===== ④ sig_type × ts 矩阵 =====")
    cases = [
        (0, P.utc_stamp(), "当小时"),
        (0, "2020010100", "六年前"),
        (0, "2030010100", "四年后"),
        (0, "2099123123", "下世纪"),
        (1, P.utc_stamp(), "当小时"),
        (1, "2020010100", "六年前"),
        (1, "2030010100", "四年后"),
    ]
    res = {}
    for sig, ts, label in cases:
        try:
            s, rc = _open_ts(host, port, dev, sec, ts, sig)
            s.sendall(P.mqtt_disconnect())
            s.close()
        except OSError as e:
            print(f"  sig={sig} ts={ts:14s} {label:8s} 连接异常 {e}")
            continue
        res[(sig, ts)] = rc
        print(f"  sig={sig} ts={ts:14s} {label:8s} rc=0x{rc:02X} "
              f"{'OK' if rc == 0 else 'REJECT'}")
    # 对照：ts 正确但密码错 ⇒ 应 0x04（证明 HMAC 本身在生效）
    s, rcbad = _open_ts(host, port, dev, sec, P.utc_stamp(), 0, bad_pw=True)
    print(f"  对照：ts 正确 + 密码错 -> rc=0x{rcbad:02X} "
          f"{'OK（符合预期：HMAC 在生效）' if rcbad == 0x04 else '⚠️ 居然连上！'}")
    s.close()

    sig0_ok = all(v == 0 for (sig, _), v in res.items() if sig == 0)
    sig1_bad_ok = all(v != 0 for (sig, ts), v in res.items()
                      if sig == 1 and ts != P.utc_stamp())
    if sig0_ok and sig1_bad_ok:
        print("  🟢 结论：sig_type=0 不校验时间戳；sig_type=1 严格校验。")
        print("     ⇒ 设备端不需要 SNTP / RTC，AUTH_TS 用常量即可。")
    return sig0_ok and sig1_bad_ok


def _open_ts(host, port, dev, sec, ts, sig, bad_pw=False):
    """用指定的 ts / sig_type 建连（密码仍按 ts_key 算；bad_pw 时故意写错）。"""
    pw = ("00" * 32) if bad_pw else __import__("hmac").new(
        ts.encode(), sec.encode(), __import__("hashlib").sha256).hexdigest()
    cid = f"{dev}_0_{sig}_{ts}"
    s = socket.create_connection((host, port), timeout=8)
    s.sendall(P.mqtt_connect(cid, dev, pw, keepalive=120))
    sp, rc = P._recv_connack(s)
    s.settimeout(3.0)
    return s, rc


def main(argv=None):
    ap = argparse.ArgumentParser(description="IoTDA 连接层语义实测")
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--device-id", required=True)
    ap.add_argument("--secret", required=True)
    ap.add_argument("--only", choices=["keepalive", "heartbeat", "dup", "ts"])
    a = ap.parse_args(argv)

    res = {}
    if a.only in (None, "keepalive"):
        res["keepalive"] = test_keepalive(a.host, a.port, a.device_id, a.secret)
    if a.only in (None, "heartbeat"):
        res["heartbeat"] = test_heartbeat(a.host, a.port, a.device_id, a.secret)
    if a.only in (None, "dup"):
        res["dup"] = test_dup(a.host, a.port, a.device_id, a.secret)
    if a.only in (None, "ts"):
        res["ts"] = test_ts(a.host, a.port, a.device_id, a.secret)

    print("\n===== 汇总 =====")
    for k, v in res.items():
        print(f"  {k:10s}: {'通过' if v else '未通过／需人工判读'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
