# -*- coding: utf-8 -*-
"""采集健康度合并基准：帧间隔停顿 + 固件计数器增量，**一次原子测量**。

为什么要"原子"：以前用 frame_timing 与 parse_frames 各跑一次，报出来的
缺口（4.21 s vs 1.74 s）其实是**两次不同采集**，混着说就是假数据。
本工具把「采集前读计数器 → 采 N 秒 → 采集后读计数器」绑成同一次测量。

用法:
    python tools/bench_stall.py --port COM5 --duration 60
    python tools/bench_stall.py --duration 60 --elf SlotB/stm32-edge-ai-node.elf
    python tools/bench_stall.py --print-addrs      # 只打印需要的符号地址
    python tools/bench_stall.py --duration 60 --cli "D:/MeSTM32CubePR/bin/STM32_Programmer_CLI.exe"

判据（采数据集的门禁）:
    ① 停顿 = 0 次（帧间隔 > 3× 中位数算一次）
    ② 完整度 ≥ 99.5 %
    ③ g_offline_events 增量为 0
做不到就先上硬件对策（电机两端 100 nF、I2C 走线远离电机线、共地复检）。

🔴 两个符号地址的来历与风险:
    本工具靠 ST-Link 热插拔读 **RAM 里的固件计数器**，所以需要符号地址。
    地址由 `arm-none-eabi-nm <elf>` 给出，**跟随 bss 布局变化**：
    改了全局变量（尤其是新增/删除）之后必须重新取一次，否则读到的是别的变量，
    而且**读数还是"像模像样"的 0** —— 这正是最难发现的一类假证据。
    所以脚本自带自检：读两次计数器，`g_tick` 必须前进；不前进就明确报"地址已失效"。

    # 重新取地址（改了固件之后）
    nm -n SlotB/stm32-edge-ai-node.elf | grep -E " g_(tick|sample_lost)$"
"""
import argparse
import re
import statistics
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial：用工程 venv 的 python 跑（D:/STM32Projects/pc-venv/Scripts/python.exe）")

DEFAULT_CLI = r"D:\MeSTM32CubePR\bin\STM32_Programmer_CLI.exe"

FRAME = 198          # AA 55 | seq u16 | n u8 | 32×3×int16 | checksum = 198 B

# ---- 符号表（**改固件后必须核对**，见文件头说明）---------------------------
ADDR_TICK        = 0x20000358   # g_tick, g_tick_seen, g_sample_lost, g_sensor_ok
ADDR_COUNTERS    = 0x20000950   # 10 个量化计数器
ADDR_PWM_DUTY    = 0x20000948   # g_pwm_duty（低字节 = 当前占空比 %）

NAMES = ["g_tick", "g_tick_seen", "g_sample_lost", "g_sensor_ok",
         "g_i2c_fail_total", "g_bus_recover", "g_bus_stuck",
         "g_recon_attempts", "g_offline_events",
         "g_gap_max_ms", "g_gap_max_fails", "g_gap_max_recover",
         "g_gap_cur_fails", "g_gap_cur_recover"]


def read_words(cli, addr, size):
    """读一段 RAM，返回 32 位字列表。`-r32` 的第二参数是**字节数**，不是字数。"""
    out = subprocess.run([cli, "-c", "port=SWD", "mode=hotplug",
                          "-r32", hex(addr), hex(size)],
                         capture_output=True, text=True,
                         encoding="utf-8", errors="replace").stdout
    words = []
    for line in out.splitlines():
        m = re.match(r"^0x[0-9A-Fa-f]+\s*:\s*(.+)$", line.strip())
        if m:
            words.extend(int(x, 16) for x in m.group(1).split())
    return words


def read_counters(cli):
    if not cli:
        return {}
    words = read_words(cli, ADDR_TICK, 0x10) + read_words(cli, ADDR_COUNTERS, 0x28)
    return dict(zip(NAMES, words))


def collect(port, duration):
    ser = serial.Serial(port, 115200, timeout=0.05)
    ser.reset_input_buffer()
    buf = bytearray()
    arrivals = []
    t0 = time.time()
    while time.time() - t0 < duration:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        while True:
            i = buf.find(b"\xAA\x55")
            if i < 0:
                if len(buf) > 4096:
                    del buf[:-2]
                break
            if len(buf) - i < FRAME:
                if i:
                    del buf[:i]
                break
            arrivals.append(time.time() - t0)
            del buf[:i + FRAME]
    ser.close()
    return arrivals


def main():
    ap = argparse.ArgumentParser(description="采集健康度合并基准（帧停顿 + 固件计数器）")
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--cli", default=DEFAULT_CLI,
                    help="STM32_Programmer_CLI.exe 全路径；传空串则不读计数器")
    ap.add_argument("--print-addrs", action="store_true", help="打印需要的符号地址后退出")
    a = ap.parse_args()

    if a.print_addrs:
        print("g_tick 段   : 0x%08X (0x10 B)" % ADDR_TICK)
        print("计数器段    : 0x%08X (0x28 B)" % ADDR_COUNTERS)
        print("g_pwm_duty  : 0x%08X (低字节)" % ADDR_PWM_DUTY)
        print("核对方法: nm -n SlotB/stm32-edge-ai-node.elf | grep -E ' g_(tick|sample_lost)$'")
        return 0

    cli = a.cli if a.cli and a.cli.lower() != "none" else None
    before = read_counters(cli)

    # ---- 自检：地址必须真的指向 g_tick（两次读必须前进）----
    if before:
        time.sleep(0.3)
        again = read_counters(cli)
        if again.get("g_tick", 0) <= before.get("g_tick", 0):
            sys.exit("🔴 自检失败：g_tick 没有前进 ⇒ 符号地址已失效。\n"
                     "   改了固件（尤其增删全局变量）之后要重新 nm 取地址，见文件头。")

    arr = collect(a.port, a.duration)
    after = read_counters(cli)

    if len(arr) < 10:
        sys.exit("帧太少（%d），检查：串口对不对 / 是不是 mode 2 / 板子有没有在跑" % len(arr))

    gaps = [b - x for x, b in zip(arr, arr[1:])]
    med = statistics.median(gaps)
    big = [(i, g) for i, g in enumerate(gaps) if g > 3 * med]
    lost = sum(g - med for _, g in big)
    pts = len(arr) * 32
    full = pts / (1000.0 * a.duration) * 100.0

    print("=" * 70)
    print("采集窗口 %.0f s | 帧 %d | 采样点 %d | 实得 %.1f 点/s（完整度 %.1f %%）"
          % (a.duration, len(arr), pts, pts / a.duration, full))
    print("帧间隔：中位数 %.1f ms | 最小 %.1f | 最大 %.1f ms"
          % (med * 1000, min(gaps) * 1000, max(gaps) * 1000))
    print("> 3×中位数的停顿：%d 次，累计缺失 %.2f s" % (len(big), lost))
    for i, g in big[:12]:
        print("    #%-5d t=%6.2fs  间隔 %7.1f ms  折算丢 %5.0f 点"
              % (i, arr[i], g * 1000, (g - med) * 1000))

    ok = True
    if before and after:
        print("-" * 70)
        print("%-20s%12s%12s%12s" % ("计数器", "前", "后", "增量"))
        for k in NAMES:
            x, y = before.get(k, -1), after.get(k, -1)
            print("%-20s%12d%12d%12d" % (k, x, y, y - x))
        ok = (len(big) == 0) and (full >= 99.5) and (after.get("g_offline_events", 1) - before.get("g_offline_events", 1) == 0)
    else:
        ok = (len(big) == 0) and (full >= 99.5)

    print("=" * 70)
    if ok:
        print("✅ 门禁通过：停顿 0 次、完整度 ≥99.5 %、掉线事件 0")
    else:
        print("❌ 门禁未过：%d 次停顿 / 完整度 %.1f %% / 看上面计数器定位" % (len(big), full))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
