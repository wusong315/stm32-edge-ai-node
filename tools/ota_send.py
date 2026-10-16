#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""S6 串口升级上位机：把一份 .bin 分包灌进 Bootloader 的 **Slot B**。

    python tools/ota_send.py --port COM4 --bin SlotB/app_slotB.bin --ver 2

🔴 帧格式的**唯一权威**在设备侧：`bootloader/Core/Inc/bl_ota.h`。
   本文件必须与它逐字段一致。两边各有一份固定的**黄金向量**自检：

       本文件      `--selftest`                      → 3 条向量
       设备侧      `BL_OtaSelfTest()`（开机横幅里）    → crc16 向量

   本文件的向量是**先在 Python 里算好、再硬编码**的，不是运行时自证 ——
   否则"算错"和"写错"会互相抵消，那样自检就没意义了。

为什么是"停等"（发一帧 → 等一个应答 → 再发下一帧）：
   STM32F4 在擦/写 Flash 的这段时间里**没法取指**（同 bank 不能边写边读），
   一次 128 KB 扇区擦除典型 1 s、最坏 4 s。而 UART 的接收寄存器只有
   **1 个字节** —— 主机若还在灌，就是**静默丢字节**。停等把"设备忙"变成
   "主机在等"，不用任何流控就绕开了这个坑。代价只有每帧一次往返（≈0.4 ms）。

退出码：0 = 升级成功且判据全过；2 = 参数错 / 升级失败 / 判据不全。
"""

import argparse
import struct
import sys
import time
import zlib

# ---- 与 bl_ota.h 逐字段对齐的常量 -----------------------------------------
SOF0, SOF1 = 0xA5, 0x5A
HDR_LEN = 5
CRC_LEN = 2
CHUNK = 1024
ACK, NAK, FATAL = "K", "N", "E"
TOKEN_CHARS = (ACK, NAK, FATAL)

# ST-Link 命令行：`--stlink-reset` 用它自动复位（省得手按 RESET）。
# 🔴 `-rst` 必须**带上 `-run`** —— 只给 `-rst` 的话核会停在 halt 状态，
#    串口一个字都不会有（这是实测踩过的坑）。
STLINK_CLI = r"D:\MeSTM32CubePR\bin\STM32_Programmer_CLI.exe"

# ---- 黄金向量（先在 PC 上算好再抄进来：crc16("123456789")=0xBB3D）---------
GOLDEN_CRC16_VECTOR = (b"123456789", 0xBB3D)
GOLDEN_FRAMES = [
    # (seq, payload.hex(), 整帧期望 hex)
    (0, "00112233", "a55a000400001122334b20"),
    (7, "00000220c1240608", "a55a07080000000220c12406083056"),
]


# =====================================================================
# 帧层（与 bl_ota.c 的 BlCrc16 / BlRecvFrame 对应）
# =====================================================================
def crc16_arc(data, crc=0):
    """CRC-16/ARC：poly 0xA001 反射、初值 0、无末尾异或。与设备侧同族。"""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF


def build_frame(seq, payload):
    """A5 5A | seq | len(LE) | payload | crc16(LE, 覆盖 hdr+payload)"""
    hdr = struct.pack("<BBBH", SOF0, SOF1, seq & 0xFF, len(payload))
    body = hdr + payload
    return body + struct.pack("<H", crc16_arc(body))


def build_start_line(length, crc32, ver=None):
    if ver is None:
        return "s%u,%08x\n" % (length, crc32 & 0xFFFFFFFF)
    return "s%u,%08x,%u\n" % (length, crc32 & 0xFFFFFFFF, ver)


def selftest():
    """固定向量自检。全部通过返回 0。"""
    bad = 0

    data, want = GOLDEN_CRC16_VECTOR
    got = crc16_arc(data)
    ok = (got == want)
    print("crc16(\"123456789\") = 0x%04X (expect 0x%04X) %s"
          % (got, want, "PASS" if ok else "FAIL"))
    bad += 0 if ok else 1

    for seq, plhex, wanthex in GOLDEN_FRAMES:
        got = build_frame(seq, bytes.fromhex(plhex)).hex()
        ok = (got == wanthex)
        print("frame(seq=%d, %d B) = %s (expect %s) %s"
              % (seq, len(plhex) // 2, got, wanthex, "PASS" if ok else "FAIL"))
        bad += 0 if ok else 1

    line = build_start_line(42788, 0x612D7DCE, 2)
    ok = (line == "s42788,612d7dce,2\n")
    print("start line = %r %s" % (line, "PASS" if ok else "FAIL"))
    bad += 0 if ok else 1

    print("---- selftest: %s ----" % ("ALL PASS" if bad == 0 else "%d FAILED" % bad))
    return 0 if bad == 0 else 1


# =====================================================================
# 串口侧
# =====================================================================
class Dev(object):
    """读设备输出：把**完整行**打成日志，同时把行首的 K/N/E 当**应答令牌**返回。

    🔴 为什么必须"按行"分辨：设备日志里就有大写 K（`[OTA] erased OK`）。
       若见 K 就当应答，握手阶段会被自己的日志骗过去。
       设备不会在一行中途插令牌 —— 令牌总是紧跟在一个 `\\r\\n` 之后。
    """

    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()
        self.lines = []
        self._mark = 0

    # -- 内部：读 1 字节，必要时打行 / 返回令牌 ---------------------------
    def _pump(self):
        d = self.ser.read(1)
        if not d:
            return None
        b = d[0]
        if b in (0x0A, 0x0D):
            if self.buf:
                line = self.buf.decode("utf-8", "replace")
                self.buf.clear()
                self.lines.append(line)
                print("   | " + line, flush=True)
            return None
        if (not self.buf) and (chr(b) in TOKEN_CHARS):
            return chr(b)
        self.buf.append(b)
        return None

    def mark(self):
        self._mark = len(self.lines)

    def new_lines(self):
        return self.lines[self._mark:]

    def wait_token(self, timeout):
        """等 K/N/E；途中任何文本都照打。超时返回 None。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            tok = self._pump()
            if tok:
                return tok
        return None

    def wait_for(self, needle, timeout):
        """等**新出现**的某一行里含 needle。返回该行或 None。"""
        self.mark()
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._pump()
            for ln in self.new_lines():
                if needle in ln:
                    return ln
        return None

    def drain(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self._pump()


def pick_port():
    """挑一个像 CH340 的串口；没有就报错让用户显式指定。"""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        desc = ("%s %s" % (p.description, p.hwid)).upper()
        if "CH340" in desc or "CH341" in desc or "USB-SERIAL" in desc:
            return p.device
    if len(ports) == 1:
        return ports[0].device
    cand = ", ".join(p.device for p in ports) if ports else "（一个都没有）"
    raise SystemExit("自动找串口失败（候选：%s），请用 --port 指定" % cand)


def main():
    ap = argparse.ArgumentParser(
        description="S6 串口升级：把 .bin 灌进 Bootloader 的 Slot B",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", help="要发送的镜像 .bin（SlotB 链接版）")
    ap.add_argument("--port", default="auto", help="串口号，如 COM4；默认自动找 CH340")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ver", type=int, default=None, help="写进元数据的 app_version")
    ap.add_argument("--retries", type=int, default=5, help="单帧最多重发几次（默认 5）")
    ap.add_argument("--wait", type=float, default=40.0, help="等 'u' 窗口的总秒数（默认 40）")
    ap.add_argument("--tail", type=float, default=8.0, help="升级后继续抓串口的秒数（默认 8）")
    ap.add_argument("--corrupt", type=int, default=None,
                    help="故意把第 N 帧（0 起）的 payload 首字节改掉 —— 给 S7 用")
    ap.add_argument("--stlink-reset", action="store_true",
                    help="用 ST-Link 自动复位一次，省得手按板上的 RESET")
    ap.add_argument("--stlink-cli", default=STLINK_CLI,
                    help="STM32_Programmer_CLI.exe 的路径")
    ap.add_argument("--dry-run", action="store_true", help="只算不发自检，不碰串口")
    ap.add_argument("--selftest", action="store_true", help="跑固定向量自检后退出")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if not args.bin:
        ap.error("必须给 --bin（或用 --selftest）")

    blob = open(args.bin, "rb").read()
    n = len(blob)
    crc = zlib.crc32(blob) & 0xFFFFFFFF
    nframes = (n + CHUNK - 1) // CHUNK

    print("镜像  : %s" % args.bin)
    print("长度  : %d B" % n)
    print("CRC32 : 0x%08X  (= zlib.crc32，与设备侧 BL_Crc32 同族)" % crc)
    print("帧数  : %d 帧 x <=%d B，帧开销 7 B -> 传输 %d B (%.2f%%)"
          % (nframes, CHUNK, n + nframes * 7, 100.0 * (n + nframes * 7) / n))
    print("估算  : 115200-8N1 约 %.1f s（不含擦除）" % ((n + nframes * 7) * 10.0 / 115200))
    if args.corrupt is not None:
        print("⚠️  --corrupt %d：第 %d 帧每次都被改坏，设备应当**一直 NAK 并最终放弃**"
              % (args.corrupt, args.corrupt))
    if args.dry_run:
        print("---- dry-run：首帧 = %s" % build_frame(0, blob[:CHUNK]).hex()[:64])
        print("---- dry-run：起始行 = %r" % build_start_line(n, crc, args.ver))
        return 0

    import serial  # 只有真要发的时候才 import

    port = args.port if args.port != "auto" else pick_port()
    print("串口  : %s @ %d 8N1" % (port, args.baud))
    ser = serial.Serial(port, args.baud, timeout=0.2)
    dev = Dev(ser)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # ---- 1) 在上电窗口里喊 'u' ----------------------------------------
    def stlink_reset_thread():
        time.sleep(0.3)
        print("   >> ST-Link: -c port=SWD -rst -run")
        try:
            import subprocess
            p = subprocess.run([args.stlink_cli, "-c", "port=SWD", "-rst", "-run"],
                               capture_output=True, text=True,
                               encoding="utf-8", errors="replace")
            tail = [l.strip() for l in p.stdout.splitlines() if l.strip()][-2:]
            print("   >> ST-Link rc=%d %s" % (p.returncode, tail))
        except Exception as e:                       # noqa: BLE001
            print("   >> ST-Link 复位失败：%r（请手动按 RESET）" % e)

    if args.stlink_reset:
        import threading
        print("\n>> 每 200 ms 喊一次 'u'，并让 ST-Link 自动复位一次（--stlink-reset）")
        threading.Thread(target=stlink_reset_thread, daemon=True).start()
    else:
        print("\n>> 每 200 ms 喊一次 'u'；请在 %g 秒内按一下板上的 RESET（或重新上电）"
              % args.wait)
    print("   （只有上电后 2 秒内喊到的 'u' 才算数；喊不到就重来，所以必须一直喊）")

    # 🔴 必须是**循环喊**：单发一次 'u' 再傻等是错的 ——
    #    复位发生在我们那次 'u' 之后的话，那一个字符早就被 App 吃掉了，
    #    窗口里什么都没收到，只能干等到超时。
    t0 = time.time()
    got = None
    while time.time() - t0 < args.wait:
        ser.write(b"u\n")
        got = dev.wait_for("[OTA] send:", 0.25)
        if got:
            break
    if not got:
        print("\n🔴 没能进入升级模式。排查：① 板上跑的是 Bootloader 吗（看横幅）"
              "② CH340 的 TXD/RXD 是不是接反 ③ 有没有在上电 2 秒内按 RESET")
        return 2

    # ---- 2) 起始行 ----------------------------------------------------
    line = build_start_line(n, crc, args.ver)
    print("\n>> 起始行: %r" % line)
    ser.write(line.encode("ascii"))

    got = dev.wait_for("[OTA] erased OK", 40.0)
    if not got:
        print("\n🔴 擦除/握手没有成功（上面有设备的原话）")
        return 2

    # ---- 3) 停等发帧 --------------------------------------------------
    print("\n>> 开始灌数据（%d 帧，停等：每帧等一个 'K'）" % nframes)
    t0 = time.time()
    sent = 0
    for seq in range(nframes):
        payload = bytearray(blob[seq * CHUNK:(seq + 1) * CHUNK])
        if args.corrupt is not None and seq == args.corrupt:
            payload[0] ^= 0xFF

        frame = build_frame(seq, bytes(payload))
        tried = 0
        while True:
            ser.write(frame)
            tok = dev.wait_token(12.0)
            if tok == ACK:
                break
            if tok == NAK:
                tried += 1
                if tried > args.retries:
                    print("🔴 第 %d 帧连发 %d 次都被拒（设备也放弃了）" % (seq, tried))
                    dev.drain(4.0)
                    return 2
                print("   ... 第 %d 帧被 NAK，重发（第 %d 次）" % (seq, tried + 1))
                continue
            print("🔴 第 %d 帧等不到应答（token=%r）" % (seq, tok))
            dev.drain(3.0)
            return 2
        sent += len(payload)
        if (seq + 1) % 8 == 0 or seq == nframes - 1:
            el = time.time() - t0
            print("   已发 %6d / %6d B  (%.1f s, %.1f KB/s)"
                  % (sent, n, el, sent / 1024.0 / max(el, 1e-3)), flush=True)

    print("\n>> 全部发完，等设备自校验 + 复位 + 安装 ……")
    dev.drain(args.tail)

    # ---- 4) 判据（机器判，不靠肉眼看）--------------------------------
    lines = "\n".join(dev.lines)
    checks = [
        ("读完回 CRC32 与声明值一致", "[OTA] readback crc32" in lines and "MATCH" in lines),
        ("已置 update_pending=1",     "[OTA] staged: update_pending=1" in lines),
        ("复位后走到提交步骤",         "update_pending=1 -> verify slot B" in lines),
        ("active_slot 切到 B",         "install: active_slot A -> B" in lines
                                      or "install: active_slot B -> B" in lines),
        ("真的跳到 Slot B",            "JUMP -> slot B" in lines),
        ("App 横幅出现（新镜像在跑）", "=== stm32-edge-ai-node boot ===" in lines),
    ]
    print("\n================ S6 判据 ================")
    allok = True
    for name, ok in checks:
        print("  [%s] %s" % ("PASS" if ok else "FAIL", name))
        allok = allok and ok
    print("================ %s ================" % ("全部通过" if allok else "有未通过项"))
    if allok:
        print("⚠️ 板上现在是「Bootloader + Slot A + Slot B」且 active_slot=B。")
        print("   要回到从 A 启动：走 RUNBOOK R13.2 用 ST-Link 烧回，或等 S8 的回滚。")
    return 0 if allok else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n(中断)")
        sys.exit(2)
