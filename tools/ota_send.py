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

    def wait_any(self, needles, timeout):
        """等**任一个** needle 出现，返回命中的那一行（不知道是哪个就看内容）。"""
        self.mark()
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._pump()
            for ln in self.new_lines():
                for nd in needles:
                    if nd in ln:
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


def evaluate(lines, corrupt, expect_rollback=False):
    """机器判据（不靠肉眼看）。三套：`--corrupt` 判坏包拒收 / `--expect-rollback` 判回滚 / 否则判升级成功。

    ⚠️ **失败路径上也要调用它** —— S7 想看的恰恰是"该失败的地方失败了"，
       不打印就等于白跑一趟（2026-10-16 第一版就踩了这个：坏帧路线提前 `return 2`，
       判据块根本没打印）。返回 True = 全部通过。
    """
    if corrupt is not None:
        title = "S7/S8 判据（坏包拒收 / 重传 / 中止 / 不置标志 / 不跳坏槽）"
        checks = [
            ("设备对坏帧回了 NAK（帧层 CRC16 真的在工作）",
             "crc16 FAIL" in lines and "-> NAK" in lines),
            ("设备自己的重传预算用尽后中止（gave up -> abort）",
             "gave up after" in lines and "-> abort" in lines),
            ("坏帧始终没有被接受（没有 [OTA] staged:）", "[OTA] staged:" not in lines),
            ("没有走提交/安装步骤（没有 install: active_slot）",
             "install: active_slot" not in lines),
            ("没有跳进那个刚被判坏的槽（没有 JUMP -> slot B）",
             "JUMP -> slot B" not in lines),
        ]
        # 🔴 上面五条 `S7` 就有了。下面这两条是 `S8` 新增的**关键区别**：
        #    坏在第 0 帧 ⇒ Slot B 被擦成空白 ⇒ 头 8 字节就判 BLANK（旧路径够用）；
        #    坏在中间帧 ⇒ Slot B 半截但**向量表完好** ⇒ 只有 S8 的**镜像级
        #    CRC32** 能抓住它。两者是**不同的现象**，判据也必须分开写 ——
        #    否则"坏在第 20 帧"和"什么都没测到"在输出上是同一个样子。
        if corrupt == 0:
            checks.append((
                "Slot B 被擦成空白 -> 明确判不合法、不跳（S7 路径）",
                "became INVALID during the update window" in lines))
        else:
            checks.append((
                "半截镜像被镜像级 CRC32 抓住（S8 新增判据）",
                "image crc32 MISMATCH" in lines))
            checks.append((
                "最终回退跳到 Slot A、板子没静默（S8 新增判据）",
                "JUMP -> slot A" in lines))
    elif expect_rollback:
        # 🔴 砖镜像**没有 App 横幅**，所以不能套用"升级成功"那一套（第 8 条必然 FAIL）——
        #    那不是缺陷，是**判据套错了**。回滚路线要看的是"计数涨到顶、指针被改回 A"。
        title = "S8 判据（砖镜像试运行 -> 试运行计数 -> 自动回滚到 Slot A）"
        checks = [
            ("读完回 CRC32 与声明值一致", "[OTA] readback crc32" in lines and "MATCH" in lines),
            ("元数据里记下了 Slot B 的镜像描述子（S8）", "img B desc" in lines),
            ("提交后进入试运行（install + trial=1）",
             "trial=1" in lines and "install: active_slot" in lines),
            ("试运行计数真的在涨（trial boot 1/3）", "trial boot 1/3" in lines),
            ("计数到顶触发回滚（trial failed N/N -> ROLLBACK slot B -> A）",
             "trial failed" in lines and "ROLLBACK slot B -> A" in lines),
            ("回滚只改指针：下一行从 A 启动（ROLLBACK recorded -> booting slot A）",
             "ROLLBACK recorded; booting slot A instead" in lines),
            ("回滚后真的跳到 Slot A（板子没静默）", "JUMP -> slot A" in lines),
        ]
    else:
        title = "S6/S8 判据（升级成功 + 描述子落盘 + 试运行计数）"
        checks = [
            ("读完回 CRC32 与声明值一致", "[OTA] readback crc32" in lines and "MATCH" in lines),
            ("已置 update_pending=1",     "[OTA] staged: update_pending=1" in lines),
            ("元数据里记下了 Slot B 的镜像描述子（S8）",
             "img B desc" in lines),
            ("复位后走到提交步骤",         "update_pending=1 -> verify slot B" in lines),
            ("提交前用**镜像级 CRC32** 复验了 Slot B（S8）",
             "image crc32 OK" in lines),
            ("active_slot 切到 B 且标成试运行", "trial=1" in lines
                                          and ("install: active_slot A -> B" in lines
                                               or "install: active_slot B -> B" in lines)),
            ("真的跳到 Slot B",            "JUMP -> slot B" in lines),
            ("App 横幅出现（新镜像在跑）", "=== stm32-edge-ai-node boot ===" in lines),
        ]
    print("\n================ %s ================" % title)
    allok = True
    for name, ok in checks:
        print("  [%s] %s" % ("PASS" if ok else "FAIL", name))
        allok = allok and ok
    print("================ %s ================" % ("全部通过" if allok else "有未通过项"))
    return allok


def do_confirm(args):
    """`S8`：在**上电那 2 s 窗口**里发 `k`，把试运行计数清零（= 确认这个槽）。

    🔴 **为什么必须"抢窗口"**：`k` 只在 **Bootloader** 的命令循环里被处理；而槽一旦合法，
       Bootloader 打几行日志、开 2 s 窗口，然后就跳进 App 了 —— **App 里没有这条命令**。
       ⇒ 每次确认都得"复位 + 窗口里补发"，不能指望板子在跑 App 时发得进去。

    🔴 **不发会怎样**：`trial=1` 的槽**每次复位**都让 `boot_attempts` 加一，到 `max`（默认 3）
       就自动回滚到另一个槽。所以正常升级完之后**必须发这一下**，否则日后正常断电重启
       累计三次，新固件就被"冤枉"回滚了（`DECISIONS.md` D30 ①）。
    """
    import serial
    port = args.port if args.port != "auto" else pick_port()
    print("串口  : %s @ %d 8N1" % (port, args.baud))
    ser = serial.Serial(port, args.baud, timeout=0.01)
    dev = Dev(ser)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print(">> 复位一次，然后在窗口里每 150 ms 补发一次 'k'（设备窗口只有 2 s）")
    try:
        import subprocess
        p = subprocess.run([args.stlink_cli, "-c", "port=SWD", "-rst", "-run"],
                           capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        print("   >> ST-Link rc=%d" % p.returncode)
    except Exception as e:                           # noqa: BLE001
        print("   >> ST-Link 复位失败：%r（请现在手动按一下板上的 RESET）" % e)

    t0 = time.time()
    nxt = 0.0
    while time.time() - t0 < 12.0:
        dev._pump()
        for ln in dev.new_lines():
            if "confirming slot" in ln:
                print("\n[OK] 已确认 -> %s" % ln.strip())
                return 0
            if "already confirmed" in ln:
                print("\n[OK] 本来就是确认态 -> %s" % ln.strip())
                return 0
            if "confirm write FAILED" in ln:
                print("\n[!!] 确认写失败 -> %s" % ln.strip())
                return 2
        if time.time() >= nxt:
            # 🔴 **只发 `k` 一个字节、不要前置 `\n`**：设备的窗口轮询是"每 X ms 读 1 个字节"，
            #    而 UART 的 RX 寄存器只有 1 个字节 ⇒ `\n` 和 `k` 紧挨着发时，`\n` 占住寄存器、
            #    `k` 直接被溢出丢掉 —— 于是"每次都在发、设备一个都没收到"。2026-10-17 实测踩到。
            ser.write(b"k")
            nxt = time.time() + 0.15
    print("\n[!!] 12 s 内没等到确认结果 —— 窗口只有 2 s，多半没撞上；重跑一次即可")
    return 2


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
                    help="故意把第 N 帧（0 起）的 payload 首字节改掉、"
                         "且**不改 CRC** ⇒ 帧自相矛盾，设备应在帧层拒绝。给 S7 用")
    ap.add_argument("--stlink-reset", action="store_true",
                    help="用 ST-Link 自动复位一次，省得手按板上的 RESET")
    ap.add_argument("--stlink-cli", default=STLINK_CLI,
                    help="STM32_Programmer_CLI.exe 的路径")
    ap.add_argument("--dry-run", action="store_true", help="只算不发自检，不碰串口")
    ap.add_argument("--selftest", action="store_true", help="跑固定向量自检后退出")
    ap.add_argument("--expect-rollback", action="store_true",
                    help="S8：换成「砖镜像试运行 -> 自动回滚」那套判据"
                         "（砖镜像没有 App 横幅，套升级判据必然假失败）")
    ap.add_argument("--confirm", action="store_true",
                    help="S8：复位一次、在 2 s 窗口里发 'k' 确认当前槽（清零 trial/计数）。"
                         "正常升级完**必须**跑一次，否则复位 3 次会自动回滚")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.confirm:
        return do_confirm(args)

    if not args.bin:
        ap.error("必须给 --bin（或用 --selftest / --confirm）")

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
        print("⚠️  --corrupt %d：第 %d 帧的 payload 首字节被改坏、**CRC 保持旧值** ⇒ "
              "帧自相矛盾，设备应当在帧层 NAK 并最终放弃" % (args.corrupt, args.corrupt))
    if args.dry_run:
        print("---- dry-run：首帧 = %s" % build_frame(0, blob[:CHUNK]).hex()[:64])
        print("---- dry-run：起始行 = %r" % build_start_line(n, crc, args.ver))
        return 0

    import serial  # 只有真要发的时候才 import

    port = args.port if args.port != "auto" else pick_port()
    print("串口  : %s @ %d 8N1" % (port, args.baud))
    # 🔴 `timeout` **必须小**（0.01，不是 0.2）。`Dev._pump()` 每次只读 1 字节，
    #    端口 timeout 就是它"没数据时最多阻塞多久" —— 给 0.2 s 时，安静期每次
    #    pump 都要空转 0.2 s，于是 `wait_for(0.25)` 一次只搬得动 2 个字节，
    #    整条横幅要几十次调用才读完 ⇒ **主机发现 `[OTA] send:` 时，设备早就
    #    起始行超时 abort 了**。2026-10-17 实测就是这个（表现成"设备已经跳回
    #    App 了我们还以为它在等握手"）。0.01 s 让轮询跟得上 115200 的字节流。
    ser = serial.Serial(port, args.baud, timeout=0.01)
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
        # 🔴 **只发 `u` 一个字节、不要带 `\n`**（2026-10-17 修）：
        #    窗口里的按键处理是**逐字符**的（`HAL_UART_Receive(...,1,0)`），根本不需要回车；
        #    而带上 `\n` 之后，设备进 OTA 前会把这半个空行留到 `BlReadLine` 里，
        #    变成一条 `not a start line: "u"`。更要命的是**那条 printf 要打 3~4 ms**，
        #    期间 UART 接收寄存器（只有 1 个字节）把我们随后灌进去的握手行吃掉大半，
        #    设备最终只读到 `"s"` —— 现象是**6 次补发全部失败**、看起来像"握手协议写错了"。
        #    去掉 `\n` 之后通道是干净的，第一次就该成功。
        ser.write(b"u")
        got = dev.wait_for("[OTA] send:", 0.25)
        if got:
            break
    if not got:
        print("\n🔴 没能进入升级模式。排查：① 板上跑的是 Bootloader 吗（看横幅）"
              "② CH340 的 TXD/RXD 是不是接反 ③ 有没有在上电 2 秒内按 RESET")
        return 2

    # ---- 2) 起始行 ----------------------------------------------------
    # 🔴 **连踩三轮才找到的完整教训（2026-10-16 / 10-17）—— 这一节别再"猜"了**：
    #
    #    设备侧的事实（读 `bl_ota.c` 就知道）：`BlReadLine()` 用
    #    `HAL_UART_Receive(&huart1, &b, 1, 3000)` **逐个字节**读，遇 `\r`/`\n` 结束。
    #    而 UART 的**接收寄存器只有 1 个字节**，且 F4 的溢出语义是"**新字节被丢弃**"。
    #
    #    ① 喊 `u` 的循环每 200~250 ms 发一次。一旦某一次进了 OTA，**后一次 `u` 仍在路上**，
    #       它会滞留在设备缓冲区里，等着和后面某个 `\n` 拼成一行 —— 日志里那条
    #       `not a start line: "u"` 就是它。**这是确定性的，不是偶发。**
    #    ② 设备打印这条日志要 **3~4 ms**，期间它**不在** `BlReadLine` 里 ⇒ 我们灌进去的
    #       握手行会被 1 字节寄存器吃剩 **1 个字符**（`"s"`）⇒ 再等 3 s 没下文。
    #       现象极像"握手协议写错了"，其实是**时序**问题。
    #    ③ 所以主机侧要做三件事，缺一不可：
    #       a. 进 OTA 后**先发一个孤立的 `\n`** 把滞留的 `u` 结束掉（清场），
    #       b. **等它把 `not a start line` 打完**（120 ms 够），
    #       c. 正式发起始行时**不再加前置 `\n`**（`build_start_line()` 自带行尾；
    #          2026-10-17 加过一个尾 `\n`，结果第一帧变成 `bad SOF (0A A5)`，整段错位）。
    #    ④ 判据仍然是"**看设备说没说这行不对**"，说不对就等 60 ms 再补发一次。
    ser.write(b"\n")                    # a) 清场：结束滞留的半行
    time.sleep(0.12)                    # b) 等它把提示打完、回到 BlReadLine
    dev.drain(0.02)

    line = build_start_line(n, crc, args.ver)
    print("\n>> 起始行: %r" % line)
    got = None
    for attempt in range(8):
        ser.write(line.encode("ascii"))  # c) 不带前缀，自己带行尾
        hit = dev.wait_any(["[OTA] erased OK", "not a start line", "no start line"], 4.0)
        if hit is not None and "erased OK" in hit:
            got = hit
            break
        if hit is not None and "no start line" in hit:
            print("   ... 设备等不到起始行、已经 abort —— 这条链路得整套重来")
            break
        print("   ... （第 %d 次没成）等 60 ms 再补发" % (attempt + 1))
        time.sleep(0.06)
    if not got:
        print("\n🔴 擦除/握手没有成功（上面有设备的原话）")
        return 2

    # ---- 3) 停等发帧 --------------------------------------------------
    print("\n>> 开始灌数据（%d 帧，停等：每帧等一个 'K'）" % nframes)
    t0 = time.time()
    sent = 0
    for seq in range(nframes):
        payload = blob[seq * CHUNK:(seq + 1) * CHUNK]
        frame = build_frame(seq, payload)

        # 🔴 `--corrupt` 必须**在算完 CRC 之后再动字节**（2026-10-16 踩过）：
        #    若先改 payload 再 `build_frame()`，CRC16 会**照着改坏后的数据重算**，
        #    帧仍然自洽 ⇒ 设备逐帧校验**正确地通过**，测出来的根本不是"帧层拒收"。
        #    这里改成"改字节但不改 CRC" —— 这才是真实链路上的传输误码模型。
        if args.corrupt is not None and seq == args.corrupt:
            f = bytearray(frame)
            f[HDR_LEN] ^= 0xFF
            frame = bytes(f)

        tried = 0
        while True:
            ser.write(frame)
            tok = dev.wait_token(12.0)
            if tok == ACK:
                break
            if tok == NAK:
                tried += 1
                if tried > args.retries:
                    print("🔴 第 %d 帧连发 %d 次都被拒（对端也放弃了）" % (seq, tried))
                    dev.drain(4.0)
                    # 🔴 退出码要跟着**判据**走，不是跟着"有没有升级成功"走：
                    #    S7 模式下"该失败的地方失败了"就是成功，必须返回 0。
                    return 0 if evaluate("\n".join(dev.lines), args.corrupt, args.expect_rollback) else 2
                print("   ... 第 %d 帧被 NAK，重发（第 %d 次）" % (seq, tried + 1))
                continue
            print("🔴 第 %d 帧等不到应答（token=%r）" % (seq, tok))
            dev.drain(3.0)
            return 0 if evaluate("\n".join(dev.lines), args.corrupt, args.expect_rollback) else 2
        sent += len(payload)
        if (seq + 1) % 8 == 0 or seq == nframes - 1:
            el = time.time() - t0
            print("   已发 %6d / %6d B  (%.1f s, %.1f KB/s)"
                  % (sent, n, el, sent / 1024.0 / max(el, 1e-3)), flush=True)

    print("\n>> 全部发完，等设备自校验 + 复位 + 安装 ……")
    dev.drain(args.tail)

    # ---- 4) 判据（机器判，不靠肉眼看）--------------------------------
    allok = evaluate("\n".join(dev.lines), args.corrupt, args.expect_rollback)
    if allok and args.expect_rollback:
        print("⚠️ 已回滚：active_slot 现在是 **A**，Slot B 里仍躺着那个跑不起来的镜像（没被擦，这是对的）。")
        print("   收尾必须再跑一次正常升级把 B 换回好镜像，然后 `--confirm` —— 否则以后切到 B 仍会回滚。")
    elif allok and args.corrupt is None:
        print("⚠️ 板上现在是「Bootloader + Slot A + Slot B」且 active_slot=B。")
        print("   现在处于试运行（trial）：跑稳了跑一次 `--confirm` 确认，否则复位 3 次会自动回滚。")
    if allok and args.corrupt is not None:
        print("⚠️ 目标槽 Slot B 已被擦除/写坏但**没有安装**；active_slot 仍是原值，")
        print("   下次上电应回退到另一槽（若目标槽本身已不可启动）。")
    return 0 if allok else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n(中断)")
        sys.exit(2)
