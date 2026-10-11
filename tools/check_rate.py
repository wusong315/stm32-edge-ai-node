#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_rate.py — 实测固件采样节拍（不用示波器、不用逻辑分析仪）

R6.4 第 ④ 步要求「1 kHz **必须实测，不得只用理论值**」。本工具就干这一件事：
复位后逐行读串口，给每行打上 PC 侧墙钟时间戳，再用最小二乘拟合

    t(固件拍数) = k · wall(墙钟秒)   →   k 就是实测节拍 (Hz)

为什么不用「末值 − 首值」：只要有一行是坏数据，整个结果就废了；最小二乘用上全部
样本，抗单点噪声。

跑法::

    python check_rate.py --port COM4
    python check_rate.py --port COM4 --expected 1000 --duration 20 --tol 1

退出码：``0`` 在容差内 / ``1`` 超差 / ``2`` 没采到足够数据。

⚠️ **固件必须处于 `APP_OUTPUT_MODE=0`** —— 那是唯一会在每行打印 ``t=`` 的档位。

🔴 **两个坑（都是真机上踩出来的，别绕）**

1. **复位后头几秒必须丢弃。** USB 转串口芯片（CH340 等）的**器件侧 FIFO** 里压着
   复位前的最后几行，会在你复位后与新的数据**挤进同一个 read() chunk**、被打上
   同一个时间戳 → 拟合出负斜率或严重偏小的斜率。
   实测教训：同一块板子，不丢头几秒量出 ``-1.6 Hz`` 和 ``727 Hz``，
   丢 3 秒后立刻是 ``1000.01 Hz``。``ser.reset_input_buffer()`` **清不掉器件 FIFO**。
2. **逐行按 ``\\n`` 切分、每行独立打时间戳。** 不要用「缓冲区末尾保留 N 字节、
   下轮再扫一遍」的写法 —— 那会把同一行算两遍，并且让**旧值排在新值前面**。

> 速查判据：**解析出的 t 增量里一旦出现负数，先怀疑"新旧数据混流"，别怀疑固件。**
"""

from __future__ import annotations

import argparse
import re
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - 环境问题，不属于逻辑分支
    serial = None
    list_ports = None

# mode 0 的实际行格式（权威 = 固件 Core/Src/main.c 的 printf）：
#   t=     100 | ax= 11208 ay=  2640 az= 12360 | T= -5216 | gx= ... | lost=21 ovf=0
LINE_RE = re.compile(r"^t=\s*(\d+)\s*\|.*?lost=(\d+)\s+ovf=(\d+)")

DEFAULT_SKIP = 3.0      # 复位后丢弃的秒数（见模块注释的坑 1）
DEFAULT_TOL = 2.0       # 允许偏差 (%)


def fit_slope(samples):
    """最小二乘拟合 samples=[(wall_s, counter), ...]，返回斜率 counter/wall。

    >>> abs(fit_slope([(0.0, 0), (1.0, 1000), (2.0, 2000)]) - 1000.0) < 1e-9
    True
    """
    n = len(samples)
    if n < 2:
        raise ValueError("样本不足：至少需要 2 个点")
    ma = sum(a for a, _ in samples) / n
    mt = sum(t for _, t in samples) / n
    sxx = sum((a - ma) ** 2 for a, _ in samples)
    if sxx == 0.0:
        raise ValueError("墙钟时间戳全相同：无法拟合（读串口太快？）")
    sxy = sum((a - ma) * (t - mt) for a, t in samples)
    return sxy / sxx


def backwards_count(values):
    """相邻值出现「后一个 <= 前一个」的次数。

    >0 说明读数不单调 —— 要么中途又复位了，要么"新旧数据混流"（见坑 1）。"""
    return sum(1 for i in range(len(values) - 1) if values[i + 1] <= values[i])


def parse_line(line: bytes):
    """解析一行 mode 0 文本 → (t, lost, ovf)；不是数据行则返回 None。"""
    m = LINE_RE.match(line.decode("ascii", "replace").strip())
    if not m:
        return None
    return int(m.group(1)), int(m.group(2)), int(m.group(3))


def collect(port: str, baud: int, duration: float, skip: float,
            reset_cli: str | None, quiet: bool = False):
    """打开串口读 duration 秒，返回 [(wall_s, t, lost, ovf), ...]（已丢掉前 skip 秒）。

    reset_cli 给定时，先调用它复位 MCU（这样 t 会从 0 重新数，便于肉眼核对）。
    """
    if serial is None:
        raise RuntimeError("没装 pyserial：用 D:\\STM32Projects\\pc-venv\\Scripts\\python.exe 跑")
    ser = serial.Serial(port, baud, timeout=0.05)
    ser.reset_input_buffer()          # 注意：清不掉器件 FIFO，真正的保险是 skip

    t0 = time.monotonic()
    if reset_cli:
        import subprocess
        r = subprocess.run([reset_cli, "-c", "port=SWD", "-rst"],
                           capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        if not quiet:
            print("[i] 已请求复位（rc=%d，%.2fs）" % (r.returncode, time.monotonic() - t0))

    rec = []
    pending = b""
    try:
        while time.monotonic() - t0 < duration:
            chunk = ser.read(4096)
            if not chunk:
                continue
            pending += chunk
            while b"\n" in pending:                    # ← 按行切分，逐行打时间戳
                line, pending = pending.split(b"\n", 1)
                now = time.monotonic()
                if now - t0 < skip:                    # ← 丢掉头几秒（见坑 1）
                    continue
                parsed = parse_line(line)
                if parsed:
                    rec.append((now - t0, parsed[0], parsed[1], parsed[2]))
    finally:
        ser.close()
    return rec


def main() -> int:
    ap = argparse.ArgumentParser(
        description="实测固件采样节拍（mode 0 的 t 计数器 vs PC 墙钟）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list-ports", action="store_true", help="列出串口后退出")
    ap.add_argument("--port", help="串口设备（Windows 如 COM4）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--expected", type=float, default=1000.0, help="期望节拍 Hz，默认 1000")
    ap.add_argument("--tol", type=float, default=DEFAULT_TOL, help="允许偏差 %%，默认 2")
    ap.add_argument("--duration", type=float, default=15.0, help="总采集秒数，默认 15")
    ap.add_argument("--skip", type=float, default=DEFAULT_SKIP, help="开头丢弃秒数，默认 3")
    ap.add_argument("--reset-cli", help="STM32_Programmer_CLI.exe 全路径；给了就先复位 MCU")
    ap.add_argument("--quiet", action="store_true", help="少打印过程信息")
    args = ap.parse_args()

    if args.list_ports:
        if list_ports is None:
            print("[!] 没装 pyserial"); return 2
        for p in list_ports.comports():
            print("%s | %s" % (p.device, p.description))
        return 0
    if not args.port:
        ap.error("需要 --port（或 --list-ports）")
    if args.duration <= args.skip + 2:
        ap.error("--duration 要比 --skip 至少大 2 秒，否则有效窗口太短")

    print("=" * 68)
    print("固件采样节拍实测　port=%s @%d　期望 %.1f Hz ±%.1f%%"
          % (args.port, args.baud, args.expected, args.tol))
    print("=" * 68)

    try:
        rec = collect(args.port, args.baud, args.duration, args.skip,
                      args.reset_cli, args.quiet)
    except Exception as e:                                    # noqa: BLE001
        print("[ERROR] %s" % e)
        return 2

    if len(rec) < 20:
        print("[ERROR] 只收到 %d 行有效数据，判不了。" % len(rec))
        print("        检查：① 固件是否为 APP_OUTPUT_MODE=0（唯一会打 t= 的档）")
        print("              ② TX/RX 是否交叉  ③ 波特率 115200")
        return 2

    samples = [(a, t) for a, t, _, _ in rec]
    k = fit_slope(samples)
    dev = (k / args.expected - 1.0) * 100.0

    t_values = [t for _, t, _, _ in rec]
    lost_values = [l for _, _, l, _ in rec]
    ovf_last = rec[-1][3]
    back = backwards_count(t_values)

    print("\n样本        : %d 行" % len(rec))
    print("有效窗口    : %.2f s（已丢弃开头 %.1f s）" % (rec[-1][0] - rec[0][0], args.skip))
    print("t 范围      : %d → %d（跨度 %d 拍）" % (t_values[0], t_values[-1],
                                                   t_values[-1] - t_values[0]))
    print("t 增量取值  : %s" % sorted({t_values[i + 1] - t_values[i]
                                       for i in range(len(t_values) - 1)
                                       if t_values[i + 1] > t_values[i]})[:8])
    print("t 倒退次数  : %d  (>0 = 中途又复位过，或新旧数据混流)" % back)
    print("lost        : 首 %d → 末 %d  (%s)"
          % (lost_values[0], lost_values[-1],
             "恒定 ✅" if len(set(lost_values)) == 1 else "在涨 ❌"))
    print("ovf         : %d" % ovf_last)

    print("\n>>> 实测节拍 = %.2f Hz   偏差 %+.2f %%" % (k, dev))

    ok = abs(dev) <= args.tol and back == 0
    if ok:
        print(">>> 结论：✅ 符合 %.1f Hz（容差 ±%.1f%%）" % (args.expected, args.tol))
        if lost_values[-1] != 0:
            print("    注：lost=%d 恒定不变 = 只在启动期记过一次，稳态没丢帧"
                  % lost_values[-1])
    else:
        if back:
            print(">>> 结论：❌ 计数器不单调 —— 先怀疑「新旧数据混流」或中途复位")
        else:
            print(">>> 结论：❌ 超差 —— 查 TIM2 的 PSC/ARR 与 APB1 定时器时钟")
            print("    本项目正确配置：APB1 分频≠1 → 定时器时钟 = 2×PCLK1 = 84 MHz；")
            print("    PSC=83 / ARR=999 → 84e6/(84×1000) = 1000 Hz")
    print()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
