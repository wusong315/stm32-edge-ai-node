#!/usr/bin/env python3
"""pwm_sweep.py —— 第 ⑦ 步：扫 PWM 占空比，把转速找到"工作点"上。

为什么需要这个工具
==================
R17 的两条硬约束把工作点钉在一个很窄的区间里：

  · **器件带宽**：MPU6050 加速度计带宽上限只有 **260 Hz**（`DLPF_CFG=0`）；
  · **Nyquist 恒为 500 Hz**：1 kHz 是加速度计的固定输出率，`SMPLRT_DIV`
    只分频陀螺，**往上没有路**。

而叶频 = 3 × 转频（3 叶），所以要让 1×/2×/3× **全部**落进 260 Hz，
转频必须 ≤ 86 Hz —— 工程上取 **80 Hz 附近（≈4800 rpm、叶频 240 Hz）**。

于是问题变成"哪个占空比 = 80 Hz"。本工具把这个循环自动化：
改占空比 → 等转速稳 → 采一段二进制帧 → 算 1× 与叶频 → 下一档，
最后直接给出推荐档位和一张对照表。没有它就得手工重复
"发命令 / 录 / parse_frames / check_blade_peak" 四五轮。

固件侧前置（缺一不可）
======================
1. 固件必须是 **mode 2**（1 kHz 二进制帧）。本工具只解二进制帧，
   mode 0/1 发的是文本 → 一个帧都收不到（工具会检测并提示）。
   烧录：
     STM32_Programmer_CLI.exe -c port=SWD -w <mode2.elf> -v -rst
   怎么确认烧进去的确实是 mode 2：反汇编看 `App_EmitFrame` 里有没有
   `movs r2, #170 @ 0xaa` / `#85 @ 0x55`（R6.3 记录的做法）。
2. 固件必须已带**第 ⑦ 步的 PWM 通道** —— 上电横幅里应有一行
   `PWM: PA6/TIM3_CH1 @20000 Hz, duty=0% (USART1 cmd: dNN + Enter)`。
3. 接线（见 RUNBOOK R6.4 第 ⑦ 步）：
   共地（MOS `GND` ↔ STM32 `GND` ↔ 电池盒负极）、`SIG` ↔ `PA6`、
   电机两端并**续流二极管**（阴极接电机 `+`）、
   电机供电**走电池盒 3 V**（🔴 不是 5V 排针）。

安全须知（真的会烫手 / 真的会丢数据）
=====================================
· **平滑升降**：每档之间按 `--ramp-step` 一档一档走，不突跳。
  `IRF520N` 的 `Rds(on)` 标定在 `Vgs=10 V`，3.3 V 栅压下是**部分导通**，
  突跳占空比 = 突跳电流，没必要冒这个险。
· **摸 MOS 温度**：每档之间摸一下。温热正常，**烫手就降占空比或换 `IRLZ44N`**。
· **看「校验错」**：每档都会报 `校验错 N`。>0 说明 PWM 开关噪声耦合进了 I2C
  （R6.4 记录过"电机开关那一瞬掉线"）→ 先把电机线绞合、与传感器排线分开再继续。
· 工具**每一档都会报壁钟与样本数的账**，不静默吞数据。

用法
====
  # 列端口（插了 ST-Link 虚拟串口时尤其有用）
  python pwm_sweep.py --list-ports

  # 默认扫 30/40/50/60/70 %
  python pwm_sweep.py --port COM4

  # 想要更细：20% 到 90%，步进 10%（也支持 20:90:10 写法）
  python pwm_sweep.py --port COM4 --duties 20:90:10 --dwell 5

  # 单点复测 + 把这一段存成 CSV（可以直接再喂 check_blade_peak.py）
  python pwm_sweep.py --port COM4 --duties 55 --save-dir ./sw55

退出码
======
  0 = 至少有一档满足"1× 过判据 + 叶频在器件带宽内且过判据"
  1 = 有档位但都不"两处都过"（表里能看到差在哪）
  2 = 环境/前置问题（端口打不开、收不到帧、依赖缺失）
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import check_blade_peak as cbp
import vibframe as vf

# ---------------------------------------------------------------------------
# 默认参数（改动前先读模块 docstring 的"为什么"）
# ---------------------------------------------------------------------------
DEFAULT_TARGET_ROT_HZ = 80.0        # 目标 1× 转频 → 叶频 240 Hz（带宽内 + Nyquist 内）
DEFAULT_DUTIES = "30,40,50,60,70"   # 一轮 5 档，约 40 秒
DEFAULT_DWELL = 4.0                 # 每档采样时长（秒）—— 4 s @1kHz = 4000 点，够 Welch 分段
DEFAULT_SETTLE = 3.0                # 改完占空比等转速稳的时间（秒）
DEFAULT_RAMP_STEP = 10.0            # 升降档步长（%）；0 = 直接跳
DEFAULT_RAMP_DELAY = 0.12           # 每档之间等（秒）
# 转频带取得比 check_blade_peak 的默认（100–220 Hz）**更宽**是**故意的**：
# 降速后 1× 会掉到 100 Hz 以下，窄带会直接"没峰"误报。
# 带开宽是安全的 —— 误入的非 1× 峰会被叶频侧的谐波自洽校验逮住（R6.4.4）。
DEFAULT_ROT_BAND = (30.0, 125.0)
# 第一遍的宽叶频带只用来兜底（1× 还没测出来时）。测到 1× 之后，
# 立刻用 blade_band_for(1×, K) 反推窄带重算 —— 那才是真正该用的带。
DEFAULT_BLADE_BAND_WIDE = (110.0, 400.0)
DEVICE_BW_HZ = cbp.DEVICE_BW_HZ     # 260 Hz

VERDICT_OK = "OK"
VERDICT_NO_ROT = "!rot"
VERDICT_ROT_WEAK = "!1x"
VERDICT_BLADE_BW = "!bw"
VERDICT_BLADE_WEAK = "!blade"


# ---------------------------------------------------------------------------
# 纯函数（可单测，不碰串口）
# ---------------------------------------------------------------------------
def parse_duties(text) -> list:
    """解析 `--duties`：支持两种写法。

      · 逗号列表：`30,40,50`
      · 区间步进：`20:90:10` → 20,30,...,90

    越界或空值一律**抛错**而不是悄悄截断 —— 占空比写错了会让电机跑在
    不预期的转速上，宁可让命令失败。
    """
    out = []
    parts = [p.strip() for p in str(text).split(",") if p.strip()]
    if not parts:
        raise ValueError("--duties 为空")
    for part in parts:
        if ":" in part:
            fields = part.split(":")
            if len(fields) != 3:
                raise ValueError("区间写法应是 lo:hi:step，收到 %r" % part)
            lo, hi, step = (float(x) for x in fields)
            if step <= 0.0:
                raise ValueError("区间步进必须 > 0，收到 %r" % part)
            if hi < lo:
                raise ValueError("区间上界小于下界：%r" % part)
            v = lo
            while v <= hi + 1e-9:
                out.append(v)
                v += step
        else:
            out.append(float(part))
    for v in out:
        if not (0.0 <= v <= 100.0):
            raise ValueError("占空比必须在 0..100 之间，收到 %r" % v)
    return out


def pick_rot(r: dict):
    """从 `analyse()` 结果里挑"最可信的 1×"那根轴。

    🔴 **必须先过滤 `rot_ok`**：全零/死掉的轴频谱恒为 0 → 本底 0 → `SNR = inf`，
    排序会把**最安静**的轴排到第一。这是 `check_blade_peak.py` 里已经踩过的同一个坑。
    没有任何轴过判据时才退一步返回"转频带里最响的峰"，但它的 `rot_ok` 是 False，
    **绝不参与推荐**，只用来给线索。

    🔴 兜底那一档还额外要求 **`rot_amp > 0`**：纯死掉的轴（全零）频谱恒 0，
    `peak_in_band` 会返回**带下沿那个频点、幅值 0**，看着像"测到了 30 Hz"。
    不滤掉它，"传感器没在动"就会被报成一个具体转速（`!1x` 而不是 `!rot`），
    把排查引向错误方向。
    """
    ok = [x for x in r["axes"] if x["rot_hz"] and x["rot_ok"]]
    if ok:
        return max(ok, key=lambda x: x["rot_snr"])
    seen = [x for x in r["axes"]
            if x["rot_hz"] and (x["rot_amp"] if x["rot_amp"] is not None else 0.0) > 0.0]
    if not seen:
        return None
    return max(seen, key=lambda x: x["rot_snr"] if x["rot_snr"] is not None else -1e9)


def classify(row: dict) -> str:
    """给一行定判定标记（纯函数，便于单测）。

    判定的**顺序**就是取舍的顺序：
      1. 没有 1× → `!rot`（电机没转 / 传感器没固定 / 端口不对）
      2. 有 1× 但没过判据 → `!1x`
      3. 叶频超出器件带宽 → `!bw`（幅值不可信，**即便频谱上看得见**）
      4. 叶频过判据 → `OK`
      5. 其余 → `!blade`
    """
    if not row.get("rot_hz"):
        return VERDICT_NO_ROT
    if not row.get("rot_ok"):
        return VERDICT_ROT_WEAK
    if row.get("blade_out_of_bw"):
        return VERDICT_BLADE_BW
    if row.get("blade_ok"):
        return VERDICT_OK
    return VERDICT_BLADE_WEAK


def recommend(rows: list, target_rot: float):
    """挑推荐档 → `(row, 是否真正可用)`。

    两级取舍：
      · 优先在"**两处都过**"（1× 过判据 + 叶频在带宽内且过判据）的档里，
        选 1× 最接近 `target_rot` 的；
      · 一个都没有时，退一步在"1× 可见"的档里选最接近的，并明确标出这是
        **降级建议**（叶频那一侧还没解决，拿它采数据集会把已知局限带进去）。

    返回 `(None, False)` 表示连 1× 都没测到 —— 这时**不该给任何档位建议**。
    """
    usable = [r for r in rows if classify(r) == VERDICT_OK]
    if usable:
        return min(usable, key=lambda r: abs(r["rot_hz"] - target_rot)), True
    seen = [r for r in rows if r.get("rot_hz") and r.get("rot_ok")]
    if seen:
        return min(seen, key=lambda r: abs(r["rot_hz"] - target_rot)), False
    return None, False


def rot_spread_pct(rows: list):
    """各档 1× 的相对离散度（%）。用来判"占空比到底有没有生效"。

    🔴 这是个**能证伪**的判据：3.3 V 栅压下占空比从 30% 走到 70%，
    转速不可能不动。若离散度 < 3%，几乎一定是"命令没送到"或"PA6 没接到 MOS"
    —— 而不是"电机很稳"。没有这一条，很容易把"没生效"读成"扫完了"。
    """
    vals = [r["rot_hz"] for r in rows if r.get("rot_hz") and r.get("rot_ok")]
    if len(vals) < 2:
        return None
    lo, hi = min(vals), max(vals)
    mid = 0.5 * (lo + hi)
    if mid <= 0.0:
        return None
    return 100.0 * (hi - lo) / mid


# ---------------------------------------------------------------------------
# 串口侧
# ---------------------------------------------------------------------------
def open_port(port: str, baud: int):
    try:
        import serial
    except ImportError:
        print("[X] 缺 pyserial。装它（注意本机要先摘掉代理）：", file=sys.stderr)
        print("    env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY \\",
              file=sys.stderr)
        print("      <pc-venv>/Scripts/pip install pyserial", file=sys.stderr)
        raise SystemExit(2)
    ser = serial.Serial(port, baud, timeout=0.2)
    ser.reset_input_buffer()
    return ser


def send_duty(ser, pct) -> None:
    """发给固件 `dNN` + 换行（协议见 main.c 里 HAL_UART_RxCpltCallback 的注释）。

    🔴 **mode 2 下固件不回显**（回显会搅乱 0xAA 0x55 二进制流）→
    别指望读回确认，要靠电机声音 / 本工具测出的 1× 变化来判断。
    """
    ser.write(("d%d\n" % int(round(pct))).encode("ascii"))
    ser.flush()


def ramp_to(ser, current, target, step, delay):
    """平滑升降到 target，返回新的当前值（单位 %）。"""
    if step <= 0.0:
        send_duty(ser, target)
        time.sleep(delay)
        return float(target)
    v = float(current)
    target = float(target)
    while abs(v - target) > 1e-9:
        v = min(target, v + step) if target > v else max(target, v - step)
        send_duty(ser, v)
        time.sleep(delay)
    return v


def flush_input(ser) -> int:
    """清空串口输入缓冲，返回丢掉的字节数（读不到就返回 -1）。

    必须在每次 `capture()` 之前调用。原因：`time.sleep(settle)` 期间固件一直
    在发帧而这里没人读，驱动缓冲会积压 ≈ `settle × rate` 个采样点。不清掉的话：
      · `capture()` 的 `wall` 只反映"把积压读出来"的耗时（实测 4.608 s 的数据
        只花 1.5 s 壁钟 ⇒ 报告里那栏 `wall_s` 是假的）；
      · 采到的段还会混进 settle 期的样本，"每档互相独立"这个前提就松了。
    """
    try:
        left = int(getattr(ser, "in_waiting", 0) or 0)
        if hasattr(ser, "reset_input_buffer"):
            ser.reset_input_buffer()
        elif left:
            ser.read(left)
        return left
    except Exception:
        return -1


def capture(ser, parser, want_samples: int, timeout: float):
    """收够 `want_samples` 个采样点（或超时）→ `(frames, wall_seconds)`。

    ⚠️ 调用前先 `flush_input(ser)`，否则 wall 会偏小、且样本混入 settle 期数据。
    """
    frames = []
    got = 0
    t0 = time.time()
    while got < want_samples:
        chunk = ser.read(4096)
        if chunk:
            for fr in parser.feed(chunk):
                if not fr.bad:
                    frames.append(fr)
                    got += fr.samples
        elif (time.time() - t0) > timeout:
            break
    return frames, time.time() - t0


# ---------------------------------------------------------------------------
# 每档评估
# ---------------------------------------------------------------------------
def eval_capture(samples, rate, *, rot_band, blade_band_wide, multiple, guard, bw):
    """对一段采样做**两遍**分析，返回一行结果。

    为什么要两遍：第一遍 1× 还没测出来，叶频带只能给个宽的兜底范围；
    测到 1× 之后立刻用 `blade_band_for(1×, K)` 反推**窄带**重算 ——
    窄带的下沿自动高于 2×，从根上排除"2× 冒充叶频"（R6.4.4 的那个假通过）。
    数据量很小（几秒 @1kHz），多算一遍 FFT 完全不心疼。
    """
    r1 = cbp.analyse(samples, float(rate), rot_band=tuple(rot_band),
                     blade_band=tuple(blade_band_wide),
                     blade_multiple=multiple, harmonic_guard=guard)
    rot = pick_rot(r1)

    row = {
        "duty": None,
        "n": int(r1["n"]),
        "rot_axis": rot["axis"] if rot else None,
        "rot_hz": rot["rot_hz"] if rot else None,
        "rot_snr": rot["rot_snr"] if rot else None,
        "rot_ok": bool(rot["rot_ok"]) if rot else False,
        "rpm": (rot["rot_hz"] * 60.0) if (rot and rot["rot_hz"]) else None,
        "blade_band": None,
        "blade_hz": None,
        "blade_snr": None,
        "blade_ratio": None,
        "blade_ok": False,
        "blade_out_of_bw": False,
        "blade_expected_hz": (multiple * rot["rot_hz"]) if (rot and rot["rot_hz"]) else None,
        "blade_over_bw_by": None,
    }
    if not (rot and rot["rot_hz"]):
        return row

    row["blade_band"] = cbp.blade_band_for(rot["rot_hz"], multiple)
    row["blade_out_of_bw"] = bool(row["blade_expected_hz"] > bw)
    if row["blade_out_of_bw"]:
        row["blade_over_bw_by"] = float(row["blade_expected_hz"] - bw)

    r2 = cbp.analyse(samples, float(rate), rot_band=tuple(rot_band),
                     blade_band=tuple(row["blade_band"]),
                     blade_multiple=multiple, harmonic_guard=guard)
    ok = [x for x in r2["axes"] if x["blade_ok"]]
    cand = ok if ok else [x for x in r2["axes"] if x["blade_hz"]]
    if cand:
        b = max(cand, key=lambda x: x["blade_snr"] if x["blade_snr"] is not None else -1e9)
        row["blade_hz"] = b["blade_hz"]
        row["blade_snr"] = b["blade_snr"]
        row["blade_ratio"] = b["ratio"]
        row["blade_ok"] = bool(b["blade_ok"])
    return row


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------
def _f(v, fmt="%.1f", dash="--"):
    return dash if v is None else (fmt % v)


def print_header(multiple, bw, target_rot):
    print()
    print("=" * 84)
    print(" 第 ⑦ 步 PWM 扫频   |   叶频倍数 K = %.0f   |   器件带宽 %.0f Hz   |   目标 1× = %.0f Hz"
          % (multiple, bw, target_rot))
    print(" -> 目标叶频 = %.0f Hz（必须 < %.0f Hz 才可信）" % (multiple * target_rot, bw))
    print("=" * 84)
    print(" duty%   1x(Hz)     rpm    1x SNR   blade(Hz)  K*1x(Hz)  ratio  blade SNR  verdict")
    print("-" * 84)


def print_row(row):
    tag = classify(row)
    print(" %5.1f  %8s  %7s  %8s  %9s  %9s  %6s  %10s  %s"
          % (row["duty"],
             _f(row["rot_hz"]),
             _f(row["rpm"], "%.0f"),
             _f(row["rot_snr"], "%.1f"),
             _f(row["blade_hz"]),
             _f(row["blade_expected_hz"]),
             _f(row["blade_ratio"], "%.2f"),
             _f(row["blade_snr"], "%.1f"),
             tag))


def print_legend():
    print("-" * 84)
    print(" 判定图例: OK = 1× 与叶频两处都过 | !bw = 叶频超出 %.0f Hz 带宽（幅值不可信）"
          % DEVICE_BW_HZ)
    print("           !rot = 转频带里没峰      | !1x = 1× 没过 SNR/幅值判据")
    print("           !blade = 叶频没过判据（或与 K×1× 不自洽，被谐波校验否决）")


def print_verdict(rows, target_rot, multiple, bw, spread):
    row, truly = recommend(rows, target_rot)
    print()
    print("=" * 84)
    if spread is not None:
        if spread < 3.0:
            print(" 🔴 各档 1× 只差 %.1f%%（< 3%%）—— **占空比很可能根本没生效**。" % spread)
            print("    3.3 V 栅压下占空比从 %d%% 走到 %d%% 转速不可能不动。按顺序查："
                  % (rows[0]["duty"], rows[-1]["duty"]))
            print("      ① 固件是不是带 PWM 的版本？（上电横幅该有 'PWM: PA6/TIM3_CH1' 那行）")
            print("      ② PA6 有没有接到 MOS 模块的 SIG？")
            print("      ③ 共地了吗（MOS GND ↔ STM32 GND ↔ 电池盒负极）？")
            print("      ④ 电机供电是电池盒 3 V 吗（不是 5V 排针）？")
        else:
            print(" ✅ 各档 1× 离散度 %.1f%% —— 占空比确实在控速。" % spread)
        print()

    if row is None:
        print(" ❌ **没有任何档位的 1× 过了判据** → 不给档位建议。")
        print("    表里那些 1× 数字只是转频带里的噪声峰（幅值没过 %.3f g 门限）。"
              % cbp.DEFAULT_MIN_AMP_G)
        print("    先确认：电机真的在转？MPU6050 与电机刚性固定在一起？")
        print("    口选对了吗（--list-ports，插着 CH340 时还可能有 ST-Link 虚拟串口）？")
        print("=" * 84)
        return 1

    if truly:
        print(" ✅ 推荐档位：**duty = %.0f%%** → 1× = %.1f Hz（%.0f rpm），叶频 %.1f Hz"
              % (row["duty"], row["rot_hz"], row["rpm"], row["blade_expected_hz"]))
        print("    （1× SNR %.1f dB，叶频 SNR %s dB）"
              % (row["rot_snr"], _f(row["blade_snr"])))
        print("    下一条命令（采数据集前的正式门禁复跑，注意带要显式给）：")
    else:
        print(" 🟠 **降级建议**：duty = %.0f%% → 1× = %.1f Hz（%.0f rpm，最接近目标 %.0f Hz）"
              % (row["duty"], row["rot_hz"], row["rpm"], target_rot))
        print("    但它 **没有「两处都过」** —— 判定标记 %s。" % classify(row))
        if classify(row) == VERDICT_BLADE_BW:
            print("    叶频 %.0f Hz 超出器件 %.0f Hz 带宽 %.0f Hz → **频率位置可信、幅值不可信**。"
                  % (row["blade_expected_hz"], bw, row["blade_over_bw_by"] or 0.0))
            print("    要么继续降占空比（目标 1× ≤ %.0f Hz），要么接受这个已知局限。"
                  % (bw / multiple))
        elif classify(row) == VERDICT_BLADE_WEAK:
            print("    1× 有了但叶频带里没有过判据的峰 → 先看这一档的 blade 列差多少，")
            print("    再考虑放宽 SNR 门限（但要在报告里写清放宽了）。")
        else:
            print("    1× 本身就没过判据 → 这一档转速下信号不够干净，换一档。")
        print("    仍然可以拿它采数据集，但**必须把上面的局限写进文档**。")

    print()
    print(" 下一步（把带显式传全，别用默认带 —— 默认带照 3 V 档定的，降速后会掉出去）：")
    print("   python parse_frames.py --port COM4 --duration 20 --csv sw_duty%d.csv" % int(row["duty"]))
    print("   python check_blade_peak.py --csv sw_duty%d.csv \\" % int(row["duty"]))
    print("       --rot-band %.0f %.0f --blade-band %.0f %.0f"
          % (DEFAULT_ROT_BAND[0], DEFAULT_ROT_BAND[1],
             row["blade_band"][0] if row["blade_band"] else DEFAULT_BLADE_BAND_WIDE[0],
             row["blade_band"][1] if row["blade_band"] else DEFAULT_BLADE_BAND_WIDE[1]))
    print("   🔴 每档记得摸一下 MOS 温度；烫手就降占空比或换 IRLZ44N。")
    print("=" * 84)
    return 0 if truly else 1


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="pwm_sweep.py",
        description="第 ⑦ 步：扫 PWM 占空比找转速工作点（转频 ≈80 Hz、叶频 ≈240 Hz）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例:
  python pwm_sweep.py --list-ports
  python pwm_sweep.py --port COM4
  python pwm_sweep.py --port COM4 --duties 20:90:10 --dwell 5
  python pwm_sweep.py --port COM4 --duties 55 --save-dir ./sw55
""")
    p.add_argument("--list-ports", action="store_true", help="列出可用串口后退出")
    p.add_argument("--port", help="串口，例如 COM4")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duties", default=DEFAULT_DUTIES,
                   help="占空比列表：'30,40,50' 或区间 '20:90:10'（默认 %s）" % DEFAULT_DUTIES)
    p.add_argument("--dwell", type=float, default=DEFAULT_DWELL,
                   help="每档采样时长（秒，默认 %.1f）" % DEFAULT_DWELL)
    p.add_argument("--settle", type=float, default=DEFAULT_SETTLE,
                   help="改完占空比等转速稳的时间（秒，默认 %.1f）" % DEFAULT_SETTLE)
    p.add_argument("--ramp-step", type=float, default=DEFAULT_RAMP_STEP,
                   # 🔴 两个字面量 `%` 都必须写成 `%%`：argparse 会把 help 字符串再
                   # 做一次 % 格式化（`_expand_help`），单个 `%（中文全角逗号等）`
                   # 会直接抛 `ValueError: unsupported format character`，
                   # 症状是**连 `--help` 都打不开**。这里用 .format() 做自己的替换，
                   # 就是为了别和 argparse 那一趟混在一起。
                   help="升降档步长（%% 计，默认 {}；0 = 直接跳）".format(DEFAULT_RAMP_STEP))
    p.add_argument("--ramp-delay", type=float, default=DEFAULT_RAMP_DELAY,
                   help="升降档之间等（秒，默认 %.2f）" % DEFAULT_RAMP_DELAY)
    p.add_argument("--rate", type=float, default=1000.0, help="采样率（默认 1000）")
    p.add_argument("--target-rot", type=float, default=DEFAULT_TARGET_ROT_HZ,
                   help="目标 1× 转频 Hz（默认 %.0f）" % DEFAULT_TARGET_ROT_HZ)
    p.add_argument("--blade-multiple", type=float, default=cbp.DEFAULT_BLADE_MULTIPLE,
                   help="叶频 = 该倍数 × 转频（默认 %.0f，本项目 3 叶）"
                        % cbp.DEFAULT_BLADE_MULTIPLE)
    p.add_argument("--rot-band", type=float, nargs=2, default=list(DEFAULT_ROT_BAND),
                   metavar=("LO", "HI"), help="转频搜索带（默认 %.0f %.0f）" % DEFAULT_ROT_BAND)
    p.add_argument("--blade-band-wide", type=float, nargs=2,
                   default=list(DEFAULT_BLADE_BAND_WIDE), metavar=("LO", "HI"),
                   help="1× 未知时的兜底叶频带（默认 %s）" % (DEFAULT_BLADE_BAND_WIDE,))
    p.add_argument("--bw", type=float, default=DEVICE_BW_HZ,
                   help="器件带宽上限 Hz（默认 %.0f）" % DEVICE_BW_HZ)
    p.add_argument("--no-harmonic-guard", dest="harmonic_guard", action="store_false",
                   help="关掉谐波自洽否决（只在排查时用，别拿它去「通过」判据）")
    p.add_argument("--json", help="把整张表写进这个 JSON 文件")
    p.add_argument("--save-dir", help="把每档的采样段存成 CSV（t_s,ax_g,ay_g,az_g）")
    return p


def main(argv=None) -> int:
    args = build_argparser().parse_args(argv)

    if args.list_ports:
        import vibsource as vs
        ports = vs.list_serial_ports()
        if not ports:
            try:
                import serial                                       # noqa: F401
                print("[!] 没找到串口 —— pyserial 装了，但系统现在没枚举到任何 COM 口。")
                print("    插上 CH340 / ST-Link 后再试（设备管理器里能看到才作数）。")
            except ImportError:
                print("[X] 没装 pyserial，装它（本机要先摘掉代理）：")
                print("    env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY \\")
                print("      D:/STM32Projects/pc-venv/Scripts/pip install pyserial")
            return 2
        print("可用串口：")
        for it in ports:
            print("  " + it)
        return 0

    if not args.port:
        print("[X] 缺 --port（先用 --list-ports 看有哪些）", file=sys.stderr)
        return 2

    try:
        duties = parse_duties(args.duties)
    except ValueError as exc:
        print("[X] --duties 有问题：%s" % exc, file=sys.stderr)
        return 2

    want = int(round(args.dwell * args.rate))
    timeout = args.dwell * 3.0 + 5.0

    print_header(args.blade_multiple, args.bw, args.target_rot)
    print(" 计划：%d 档（%s），每档 settle %.1f s + 采 %.1f s → 约 %.0f 秒" %
          (len(duties), ", ".join("%g" % d for d in duties),
           args.settle, args.dwell, len(duties) * (args.settle + args.dwell + 1.0)))
    print(" 🔴 电机即将转动 —— 确认桨叶装牢、周围无人无物、MOS 已共地。")

    ser = open_port(args.port, args.baud)
    print("[i] 已打开 %s @ %d 8N1" % (args.port, args.baud))

    rows = []
    saved = {}
    cur = 0.0
    try:
        for duty in duties:
            cur = ramp_to(ser, cur, duty, args.ramp_step, args.ramp_delay)
            time.sleep(args.settle)

            # 🔴 settle 期间固件一直在发帧而这里没读 → 驱动缓冲积压 ≈ settle×rate 个点。
            #    先清掉，wall 才反映真实采样时长（实测不清：4.6 s 的数据只报 1.5 s 壁钟）。
            flush_input(ser)

            parser = vf.FrameParser()
            frames, wall = capture(ser, parser, want, timeout)
            parser.finish()
            if not frames:
                print()
                print(" [X] 收不到任何二进制帧。按顺序查：")
                print("     ① 固件是 **mode 2** 吗？（mode 0/1 发文本，本工具解不了）")
                print("        mode 2 下**没有 LED 心跳、也没有文字输出**，属正常。")
                print("     ② 口选对了吗？CH340 插着时可能同时有 ST-Link 虚拟串口。")
                print("     ③ 波特率 115200、CH340 跳线 3.3V。")
                return 2

            samples = vf.stack_frames(frames)
            row = eval_capture(samples, args.rate,
                               rot_band=args.rot_band,
                               blade_band_wide=args.blade_band_wide,
                               multiple=args.blade_multiple,
                               guard=args.harmonic_guard, bw=args.bw)
            row["duty"] = float(duty)
            row["wall_s"] = round(wall, 2)
            row["samples"] = int(len(samples))
            row["checksum_errors"] = int(parser.stats.checksum_errors)
            row["seq_gaps"] = int(parser.stats.seq_gaps)
            rows.append(row)
            saved[float(duty)] = samples

            print_row(row)
            print("        └ %d 点 / 壁钟 %.1f s | 校验错 %d | 序号断档 %d | 帧 %d"
                  % (len(samples), wall, parser.stats.checksum_errors,
                     parser.stats.seq_gaps, parser.stats.frames_ok))
            if parser.stats.checksum_errors:
                print("        └ 🔴 有校验错 → PWM 开关噪声耦合进了 I2C。"
                      "先把电机线与传感器排线分开、缩短电机线再继续。")
    finally:
        # 收工时把电机停下来 —— 别让它一直转着等人
        try:
            ramp_to(ser, cur, 0.0, args.ramp_step, args.ramp_delay)
            print("[i] 已把占空比归零并关闭串口")
        except Exception:                                   # noqa: BLE001
            print("[!] 归零占空比失败 —— 手动断电，别让电机空转")
        ser.close()

    print_legend()
    spread = rot_spread_pct(rows)
    rc = print_verdict(rows, args.target_rot, args.blade_multiple, args.bw, spread)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"target_rot_hz": args.target_rot,
                       "blade_multiple": args.blade_multiple,
                       "device_bw_hz": args.bw,
                       "rot_band": list(args.rot_band),
                       "rot_spread_pct": spread,
                       "rows": rows}, fh, ensure_ascii=False, indent=2)
        print("[i] 表已写入 %s" % args.json)

    if args.save_dir:
        import parse_frames as pf
        os.makedirs(args.save_dir, exist_ok=True)
        for duty, samples in saved.items():
            if len(samples) == 0:
                continue
            path = os.path.join(args.save_dir, "duty%03d.csv" % int(round(duty)))
            pf.write_csv(path, samples, float(args.rate), lsb=True)

    return rc


if __name__ == "__main__":
    sys.exit(main())
