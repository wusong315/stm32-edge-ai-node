#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""parse_frames.py — 振动数据采集/解析命令行工具

一条命令完成：**数据从哪来 → 解析 → 看健康度 → 存 CSV → 切分析窗口 → 画图**。

典型用法
--------------------------------------------------------------------------
    # 0) 先看看有哪些串口（Windows 上是 COMx）
    python parse_frames.py --list-ports

    # 1) 上板第一件事：录一段原始字节，以后不用重复接线
    python parse_frames.py --port COM5 --duration 10 --save-raw raw.bin

    # 2) 离线反复分析刚才录的数据（不碰硬件）
    python parse_frames.py --file raw.bin --csv data.csv --plot data.png

    # 3) 切 1024 点分析窗口（喂给 NanoEdge AI Studio）
    python parse_frames.py --file raw.bin --window 1024 --window-out windows/

    # 4) 完全没有硬件时，用假数据把整条链路走通
    python parse_frames.py --fake --csv demo.csv --plot demo.png

    # 5) 硬件刚到手、还在 mode 0 / mode 1 文本档时
    python parse_frames.py --port COM5 --text --duration 5

输出口径
--------------------------------------------------------------------------
* `--csv`        连续序列：`t_s,ax_g,ay_g,az_g`（时间单位秒，加速度单位 g）
* `--window-out` 每个文件一个定长窗口，配套 `manifest.txt` 记录来源与顺序
* `--plot`       4 联图 PNG：三轴时域 + az 的频谱（看叶频用）

⚠️ 两个「帧」别搞混
--------------------------------------------------------------------------
* 固件 `SAMPLES_PER_FRAME = 32` 是**串口传输分包单位**（198 字节/包）；
* `--window 1024` 是**送进 AI 的分析窗口**（1024 @1kHz = 1.024 s）。
  本工具的职责就是：把 32 点的小包重组成连续 1 kHz 序列，再切 1024 点窗口。
  **不要为了对齐 1024 去改固件的传输帧长。**
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

import vibframe as vf
import vibsource as vs

# 各输出档的默认采样率（Hz）—— 与固件 main.c 的三档模式口径一致
RATE_BINARY = 1000.0
RATE_TEXT_G = 200.0     # mode 1：文本波形 %.3f g，200 Hz
RATE_TEXT_RAW = 10.0    # mode 0：调试文本原始整数，10 Hz


# ---------------------------------------------------------------------------
# 采集：把不同来源统一成"一段一段的字节"
# ---------------------------------------------------------------------------
def _tee_save_raw(chunks, path: str):
    """边给下游边落盘原始字节。**上板第一件事就该干这个。**"""
    with open(path, "wb") as fh:
        total = 0
        for c in chunks:
            fh.write(c)
            total += len(c)
            yield c
    print("[i] 原始字节已保存到 %s（%d 字节）" % (path, total))


def collect_binary(args) -> tuple[list[np.ndarray], vf.ParserStats, float, bytes]:
    """二进制帧路径：帧流 → (g 值块列表, 统计, 覆盖时长, 头部字节样本)。

    🔴 为什么要带一段"头部字节样本"回去：**一帧都没解出时，固件的诊断信息
       往往就在那几个字节里**。实测踩到过：mode 2 在传感器缺席时不出帧，而是
       反复打印 `[WAIT] MPU6050 not found (id=0x00). Check wiring: ...`；
       工具当时只报"重同步丢弃 870 字节"，于是排查被引向 TX/RX 接线 ——
       而真正的原因，固件早就写在字节里了。

    🔴 串口采集时按 Ctrl-C 是**正常操作**（"采够了"），所以这里捕获它并
       优雅收尾 —— 否则刚采的几万个点会连同 CSV/图一起丢掉。
    """
    parser = vf.FrameParser()
    g_chunks: list[np.ndarray] = []
    n_frames = 0
    t_begin = time.time()
    head = bytearray()
    HEAD_KEEP = 512

    source = _make_source(args, binary=True)
    if args.save_raw:
        source = _tee_save_raw(source, args.save_raw)

    interrupted = False
    try:
        for chunk in source:
            if len(head) < HEAD_KEEP:
                head += chunk[:HEAD_KEEP - len(head)]
            for f in parser.feed(chunk):
                g_chunks.append(f.accel_g().astype(np.float32))
                n_frames += 1
            if args.quiet is False and n_frames and n_frames % 300 == 0:
                print("    ... 已解出 %d 帧 / %d 点"
                      % (n_frames, parser.stats.samples_ok), flush=True)
            if args.max_samples and parser.stats.samples_ok >= args.max_samples:
                break
    except KeyboardInterrupt:
        interrupted = True
    parser.finish()
    if interrupted:
        print("\n[i] 收到 Ctrl-C，停止采集（已收到的数据会照常写出）")
    return g_chunks, parser.stats, time.time() - t_begin, bytes(head)


def _print_text_hint(head: bytes) -> None:
    """收到字节但一帧都没解出时，把可读文本原样打出来 —— 固件的诊断就在里面。

    注意这里**必须能判否**：真正的噪声（波特率不对、线悬空）不能被打成"固件输出"，
    否则又是一条会把人带偏的假线索。

    🔴 而且判据只能看**原始字节**，不能看解码后的字符串：
       `decode(errors="replace")` 会把每个非法字节变成 U+FFFD，而 **U+FFFD 在 Python 里
       `str.isprintable()` 是 True** —— 于是 `bytes(range(256))` 这种纯噪声也能刷出
       88% 的"可打印占比"，被误判成固件输出。这个假阳性是被自测当场抓出来的。
    """
    if not head:
        return
    ok = sum(1 for b in head if 32 <= b < 127 or b in (9, 10, 13))
    if ok < len(head) * 0.9:
        return                      # 不像文本 → 更可能是噪声/波特率不对，别乱说
    text = head.decode("utf-8", errors="replace")
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    if not lines:
        return
    print("  🔎 收到的字节看起来是**固件的文本输出**，不是帧流 —— 先读它：")
    for ln in lines[:6]:
        print("     | " + ln[:88])
    if len(lines) > 6:
        print("     | …（共 %d 行）" % len(lines))
    print("     ⇒ 常见成因：传感器缺席/初始化失败 → 固件停在等待循环里不出帧；")
    print("        或固件在 mode 0/1，而工具按二进制帧解（输出模式不匹配）。")


def collect_text(args) -> tuple[list[np.ndarray], vs.TextParser, float]:
    """文本路径（mode 0 / mode 1）：文本流 → (g 值块列表, 解析器, 覆盖时长)。"""
    parser = vs.TextParser()
    g_chunks: list[np.ndarray] = []
    t_begin = time.time()
    n_bytes = 0

    source = _make_source(args, binary=False)
    if args.save_raw:
        source = _tee_save_raw(source, args.save_raw)

    interrupted = False
    try:
        for chunk in source:
            n_bytes += len(chunk)
            text = chunk.decode("utf-8", errors="replace")
            rows = parser.feed(text)
            if rows:
                g_chunks.append(np.asarray(rows, dtype=np.float32))
            if args.max_samples and len(parser.rows) >= args.max_samples:
                break
    except KeyboardInterrupt:
        interrupted = True
    parser.finish()
    parser.bytes_in = n_bytes
    if parser.rows:
        # rows 已含全部历史，直接以它为准（避免重复拼接）
        g_chunks = [np.asarray(parser.rows, dtype=np.float32)]
    if interrupted:
        print("\n[i] 收到 Ctrl-C，停止采集（已收到的数据会照常写出）")
    return g_chunks, parser, time.time() - t_begin


def reconcile_text_rate(rate_flag: float, text_raw_flag: bool,
                        parser_mode: str | None) -> tuple[float, str | None]:
    """把"CLI 开关推出来的采样率"和"解析器实际识别到的 mode"对齐。

    🔴 为什么必须做这一步：`--text` 不给 `--text-raw` 时，速率会被当成
       mode 1 的 200 Hz，而解析器那边**照样能认出** mode 0 的原始整数行。
       于是报告里出现自相矛盾的一行（标签写 mode 0、数字写 200），
       CSV 的 `t_s` 列也随之整体错 **20 倍** —— 而 `check_blade_peak.py`
       的 `load_csv()` 正是**从 `t_s` 反推采样率**的，错会一路传下去。

    返回 (最终速率, 警告文案或 None)。`parser_mode` 为 None（没认出任何格式）时原样返回。
    """
    if parser_mode is None:
        return rate_flag, None
    detected = RATE_TEXT_RAW if parser_mode == "raw" else RATE_TEXT_G
    if detected == rate_flag:
        return rate_flag, None
    return detected, ("--text-raw %s，但实际解析到 mode %s → 采样率改用 %.0f Hz"
                      "（原先按 %.0f Hz 计）"
                      % ("已给" if text_raw_flag else "未给",
                         parser_mode, detected, rate_flag))


def _make_source(args, *, binary: bool):
    """按 CLI 参数构造字节来源。三种来源互斥，优先级 port > file > fake。"""
    if args.port:
        return vs.serial_source(args.port, args.baud, duration=args.duration,
                                verbose=not args.quiet)
    if args.file:
        return vs.file_source(args.file)
    # 默认假数据：完全不需要硬件就能把整条链路走通
    if not args.quiet:
        print("[i] 未指定 --port/--file，使用假数据（--fake）")
    if binary:
        return vs.fake_source(chunks=16, seconds=args.duration or 3.0,
                              rate=RATE_BINARY)
    txt = vs.make_fake_text(seconds=args.duration or 3.0,
                            rate=RATE_TEXT_RAW if args.text_raw else RATE_TEXT_G,
                            mode="raw" if args.text_raw else "g",
                            banner="\r\n=== fake text source (no hardware) ===\r\n")
    return iter([txt.encode("utf-8")])


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------
def write_csv(path: str, samples: np.ndarray, rate: float, *, lsb: bool) -> None:
    """写连续序列 CSV。首行是表头（NanoEdge / pandas 都能直接吃）。"""
    n = len(samples)
    t = np.arange(n) / rate
    header = "t_s,ax_g,ay_g,az_g"
    cols = [t, samples[:, 0], samples[:, 1], samples[:, 2]]
    if lsb:
        # 原始整数也写出来 —— 调试换算错没错时最有用
        raw = np.rint(samples.astype(np.float64) * vf.ACCEL_LSB_PER_G).astype(np.int32)
        header += ",ax_lsb,ay_lsb,az_lsb"
        cols += [raw[:, 0], raw[:, 1], raw[:, 2]]
    arr = np.column_stack(cols)
    fmt = ["%.6f"] + ["%.6f"] * 3 + (["%d"] * 3 if lsb else [])
    np.savetxt(path, arr, delimiter=",", header=header, comments="", fmt=fmt)
    print("[i] CSV 已写入 %s（%d 点，%.2f 秒）" % (path, n, n / rate))


def write_windows(out_dir: str, samples: np.ndarray, window: int, hop: int,
                  rate: float, source_desc: str, *, lsb: bool) -> int:
    """切定长分析窗口，一个窗口一个 CSV，并写 manifest 记录来源与顺序。"""
    os.makedirs(out_dir, exist_ok=True)
    wins = list(vf.iter_windows(samples, window, hop))
    if not wins:
        print("[!] 数据只有 %d 点，不够切一个 %d 点窗口 —— 什么都没写"
              % (len(samples), window))
        return 0
    lines = ["# stm32-edge-ai-node 分析窗口清单",
             "# 源: %s" % source_desc,
             "# 窗口长度: %d 点（@%.0f Hz = %.3f 秒）" % (window, rate, window / rate),
             "# 步长: %d 点（%s）" % (hop, "不重叠" if hop == window else "重叠"),
             "# 列: %s" % ("t_s,ax_g,ay_g,az_g" + (",ax_lsb,ay_lsb,az_lsb" if lsb else "")),
             "# 共 %d 个窗口" % len(wins), ""]
    for i, w in enumerate(wins):
        name = "win_%04d.csv" % i
        write_csv(os.path.join(out_dir, name), w, rate, lsb=lsb)
        lines.append("%s  start=%d  points=%d" % (name, i * hop, len(w)))
    with open(os.path.join(out_dir, "manifest.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print("[i] 已写出 %d 个 %d 点窗口到 %s/（含 manifest.txt）"
          % (len(wins), window, out_dir.rstrip("/\\")))
    return len(wins)


def write_plot(path: str, samples: np.ndarray, rate: float, *,
               window: int | None, hop: int | None, show: bool) -> None:
    """4 联图：三轴时域 + az 频谱。**用英文标签，避免字体缺字变成方块。**"""
    if not show:
        import matplotlib
        matplotlib.use("Agg")          # 无显示器也能出图（存 PNG）
    import matplotlib.pyplot as plt

    n = len(samples)
    t = np.arange(n) / rate
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), constrained_layout=True)
    for k, (ax, name) in enumerate(zip(axes[:3], vf.AXES)):
        ax.plot(t, samples[:, k], lw=0.7, color=("#d62728", "#2ca02c", "#1f77b4")[k])
        ax.set_ylabel("%s [g]" % name)
        ax.grid(alpha=0.3)
        if window:
            step = hop if hop else window
            for x in np.arange(0, n, step):
                ax.axvline(x / rate, color="k", ls=":", lw=0.4, alpha=0.4)
    axes[0].set_title("stm32-edge-ai-node · acceleration @ %.0f Hz (%d samples)"
                      % (rate, n))
    axes[2].set_xlabel("time [s]")

    # 频谱：看叶频（三叶风扇的叶片通过频率）最直观
    ax = axes[3]
    if n >= 64:
        sig = samples[:, 2].astype(np.float64)
        sig = sig - sig.mean()
        freqs = np.fft.rfftfreq(n, d=1.0 / rate)
        spec = np.abs(np.fft.rfft(sig * np.hanning(n))) / n * 2
        ax.semilogy(freqs, np.maximum(spec, 1e-9), lw=0.8)
        peak = float(freqs[int(np.argmax(spec))])
        ax.axvline(peak, color="r", ls="--", lw=0.8)
        ax.annotate("peak %.1f Hz" % peak, xy=(peak, float(np.max(spec))),
                    xytext=(6, -4), textcoords="offset points", color="r", fontsize=9)
        ax.set_xlim(0, min(rate / 2, max(500.0, peak * 2)))
        ax.set_title("az spectrum (peak = blade-pass freq)")
    else:
        ax.text(0.5, 0.5, "not enough samples for FFT", ha="center", va="center")
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("amplitude [g]")
    ax.grid(alpha=0.3, which="both")

    if show:
        plt.show()
    if path:
        fig.savefig(path, dpi=130)
        print("[i] 图已保存到 %s" % path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
def print_report_text(parser: vs.TextParser, rate: float, seconds: float) -> None:
    s = parser.samples()
    print("\n" + "-" * 68)
    print("文本模式健康度")
    print("  " + parser.as_text())
    print("  采样率口径: %.0f Hz（mode %s）"
          % (rate, "0 → 原始整数" if parser.mode == "raw" else "1 → g 值"))
    if len(s):
        print("  均值 [g]: ax=%.3f ay=%.3f az=%.3f" % (s[:, 0].mean(), s[:, 1].mean(),
                                                      s[:, 2].mean()))
    if parser.mode == "raw":
        # ⚠️ 这条必须说清楚：mode 0 只有 10 Hz，而振动主频上百 Hz，
        #    混叠会把正弦的均值拉走 —— 用户会以为传感器校准错了。
        print("  ⚠️ mode 0 是 10 Hz 抽样，振动信号会混叠，均值不代表真实零偏。")
        print("     想验「静置 ≈ +1 g」请停机采样，或改用 mode 1（200 Hz）。")
    elif len(s):
        print("     停机静置时这里应 ≈ 0.000 0.000 +1.000")
    print("  收到原始字节 %d | 实际接收耗时 %.2f 秒" % (parser.bytes_in, seconds))
    if parser.bytes_in == 0:
        # 🔴 必须和"解析口径不对"区分开：这两者排查方向完全相反。
        #    实测教训（2026-10-12）：0 字节的真因就是 **CH340 的 TX/RX 接反**；
        #    而当时报告只写"文本行 0"，看不出"一个字节都没到"，
        #    于是被误判成解析口径问题，白绕了一圈。
        print("  🔴 一个字节都没收到 —— 这**不是**解析问题，是链路问题。按这个顺序查：")
        print("     ① CH340 的 TX/RX 是否**交叉**（TXD→PA10、RXD→PA9，最常犯）")
        print("     ② GND 是否共地  ③ 端口号是否选对（--list-ports）")
        print("     ④ 板子是否在跑（mode 0 下应有 10 Hz 打印 + LED 5 Hz 闪）")
    elif len(s) == 0:
        print("  🔴 收到 %d 字节但一行都没解出 —— 这才是**解析口径**问题："
              % parser.bytes_in)
        print("     固件 mode 与开关是否一致（mode 0 请用 --text --text-raw）。")
    print("-" * 68)


def print_report_binary(stats: vf.ParserStats, samples: np.ndarray,
                        rate: float, seconds: float, head: bytes = b"") -> None:
    print("\n" + "-" * 68)
    print("二进制帧健康度")
    print("  " + stats.as_text())
    if len(samples):
        print("  时长 %.3f 秒 @ %.0f Hz（应 ≈ 帧数 × %d / %.0f）"
              % (len(samples) / rate, rate, vf.DEFAULT_SAMPLES_PER_FRAME, rate))
        print("  均值 [g]: ax=%.3f ay=%.3f az=%.3f   （静置时应 ≈ 0, 0, +1）"
              % (samples[:, 0].mean(), samples[:, 1].mean(), samples[:, 2].mean()))
    print("  实际接收耗时 %.2f 秒" % seconds)

    if len(samples) and seconds >= 0.1:
        # 🔴 `t_s` 是**按样本索引**算的（`arange(n)/rate`），不是壁钟。
        #    固件一停（掉线重试 / 总线恢复 / 打诊断），这段壁钟就被压缩掉了 ——
        #    而校验和与序号**全是绿的**（停掉的是时间，不是帧计数）。
        #    这个差值是从外部唯一能察觉"接缝"的量，所以必须显式摊出来。
        #    ⚠️ `--file` 离线重解析没有墙钟（seconds≈0），此时跳过 —— 否则会打出负数。
        gap = seconds - len(samples) / rate
        print("  壁钟缺口 %+.2f 秒 = 接收耗时 − 样本时长（在线正常值约 +0.05 秒）" % gap)
        if gap > 0.30:
            print("     ↑ 偏大 → 采集期间固件停过（掉线重试 / 总线恢复），时间轴有缺失")
        elif gap < -0.10:
            print("     ↑ 偏负 → 收到的字节比实时还多：适配器 FIFO 里的旧数据混进来了")

    if stats.big_drops:
        # 大段丢弃 = 固件在那一刻停了。只报**样本索引 > 0** 的：
        # 索引 0 一般是采集开始前的启动横幅残留，不影响切窗。
        frame_bytes = float(vf.frame_total(vf.DEFAULT_SAMPLES_PER_FRAME))
        mid = [(i, n) for (i, n) in stats.big_drops if i > 0]
        if mid:
            print("  🔴 时间轴断点 %d 处 —— t_s 在这些位置不连续，**别当成连续序列切窗**:"
                  % len(mid))
            for i, n in mid[:5]:
                print("       样本 %-8d 附近丢弃 %5d 字节 ≈ %.1f 帧 → 该处壁钟有缺失"
                      % (i, n, n / frame_bytes))
            if len(mid) > 5:
                print("       …另有 %d 处" % (len(mid) - 5))

    if not len(samples) and head:
        # 🔴 一帧都没有但"收到过字节" —— 先别让用户去查接线：
        #    固件自己的诊断（传感器缺席、启动横幅、复位原因）通常就在这些字节里。
        _print_text_hint(head)
    bad = []
    if not len(samples):
        # 一个点都没有时打 ✅ 会误导人（"没问题"和"没收到"是两回事）
        bad.append("一个点都没解出来 —— 先看接线与波特率，再看下面 4 条排查")
    if stats.checksum_errors:
        bad.append("校验错 %d 帧 → 串口有噪声/波特率不对/接线接触不良"
                   % stats.checksum_errors)
    if stats.seq_gaps:
        bad.append("序号断档 %d 次（丢帧约 %d）→ PC 端读取太慢，先降采样率或提高波特率"
                   % (stats.seq_gaps, stats.frames_lost))
    if stats.resync_bytes > 200:
        bad.append("重同步丢弃 %d 字节 → 前面混着文字属正常；若持续增长则是误同步"
                   % stats.resync_bytes)
    if any(i > 0 for i, _ in stats.big_drops):
        # 校验错 0 / 序号断档 0 并不等于"数据可以当连续序列用" ——
        # 这两项查的是帧有没有丢，查不出"时间有没有停"。见上面的壁钟缺口。
        bad.append("采集期间固件停过（见上面「时间轴断点」）→ 这批数据有接缝，"
                   "切窗前必须跳过断点位置；要干净数据请重采")
    if len(samples) and seconds >= 0.1 and (seconds - len(samples) / rate) > 0.30:
        bad.append("壁钟缺口偏大 → 采集期间固件停过，t_s 已被压缩（接缝位置见上）；"
                   "要干净数据请重采")
    if len(samples) and abs(float(samples[:, 2].mean()) - 1.0) > 0.25:
        bad.append("az 均值 %.3f 偏离 1 g 太多 → 芯片没贴平，或换算/LSB 档位写错"
                   % float(samples[:, 2].mean()))
    if bad:
        print("  ⚠️ 需要看的点:")
        for b in bad:
            print("     - " + b)
    else:
        print("  ✅ 未发现异常（校验、序号、物理量口径都正常）")
    print("-" * 68)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="parse_frames.py",
        description="STM32 边缘 AI 节点 · 振动数据采集与解析工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例:
  python parse_frames.py --list-ports
  python parse_frames.py --port COM5 --duration 10 --save-raw raw.bin
  python parse_frames.py --file raw.bin --csv data.csv --plot data.png
  python parse_frames.py --file raw.bin --window 1024 --window-out windows/
  python parse_frames.py --fake --csv demo.csv --plot demo.png
  python parse_frames.py --port COM5 --text --duration 5
""")

    src = p.add_argument_group("数据来源（三选一，都不给则用 --fake）")
    src.add_argument("--port", metavar="COMx",
                     help="串口设备名，Windows 形如 COM5（先跑 --list-ports）")
    src.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    src.add_argument("--file", metavar="PATH", help="离线解析已录制的文件")
    src.add_argument("--fake", action="store_true", help="用假数据（无需硬件）")
    src.add_argument("--duration", type=float, metavar="SEC",
                     help="采集时长（秒）。串口到点自动停；--fake 决定造多长。缺省=一直采到 Ctrl-C/串口静默")
    src.add_argument("--max-samples", type=int, metavar="N", help="采够 N 点就停（便于脚本化）")
    src.add_argument("--save-raw", metavar="PATH", help="把收到的原始字节原样存盘（强烈建议）")
    src.add_argument("--list-ports", action="store_true", help="列出可用串口后退出")

    md = p.add_argument_group("数据格式")
    md.add_argument("--text", action="store_true",
                    help="按文本解析（mode 0/1），而不是二进制帧")
    md.add_argument("--text-raw", action="store_true",
                    help="文本档是 mode 0（原始整数、10 Hz），配合 --text 使用")
    md.add_argument("--rate", type=float, metavar="HZ",
                    help="时间轴采样率，默认按模式自动推断（1kHz/200Hz/10Hz）")

    out = p.add_argument_group("输出")
    out.add_argument("--csv", metavar="PATH", help="写连续序列 CSV")
    out.add_argument("--lsb", action="store_true", help="CSV 同时写出原始整数列")
    out.add_argument("--window", type=int, metavar="N",
                     help="切定长分析窗口，如 1024（配合 --window-out）")
    out.add_argument("--hop", type=int, metavar="N", help="窗口步长，缺省=不重叠")
    out.add_argument("--window-out", metavar="DIR", help="窗口 CSV 输出目录")
    out.add_argument("--plot", metavar="PATH", nargs="?", const="vibration.png",
                     help="出图（默认存 vibration.png）")
    out.add_argument("--show", action="store_true", help="弹窗显示图（而非只存文件）")
    out.add_argument("--quiet", action="store_true", help="少打印过程信息")
    return p


def main(argv=None) -> int:
    args = build_argparser().parse_args(argv)

    if args.list_ports:
        ports = vs.list_serial_ports()
        if ports:
            print("可用串口：")
            for x in ports:
                print("  " + x)
        else:
            print("没找到串口（或未安装 pyserial）。插上板子后重试。")
        return 0

    if args.text and args.window:
        print("[!] --text 与 --window 同时给出：文本档速率低（10/200 Hz），"
              "切窗口通常没有意义，但仍会按你要求执行。", file=sys.stderr)

    use_text = bool(args.text or args.text_raw)
    t0 = time.time()

    print("=" * 68)
    print("stm32-edge-ai-node · 振动数据工具")
    print("=" * 68)

    if use_text:
        rate = args.rate or (RATE_TEXT_RAW if args.text_raw else RATE_TEXT_G)
        chunks, parser, wall = collect_text(args)
        if args.rate is None:
            # 以解析器**实际识别到的** mode 为准；显式给了 --rate 才不覆盖
            rate, warn = reconcile_text_rate(rate, args.text_raw, parser.mode)
            if warn:
                print("[!] " + warn, file=sys.stderr)
        samples = (np.concatenate(chunks, axis=0) if chunks
                   else np.zeros((0, 3), dtype=np.float32))
        print_report_text(parser, rate, wall)
    else:
        rate = args.rate or RATE_BINARY
        chunks, stats, wall, head = collect_binary(args)
        samples = (np.concatenate(chunks, axis=0) if chunks
                   else np.zeros((0, 3), dtype=np.float32))
        print_report_binary(stats, samples, rate, wall, head=head)

    if len(samples) == 0:
        print("\n[!] 一个点都没解出来。排查顺序：")
        print("    1) 接线：TX/RX 是否交叉、GND 是否共地")
        print("    2) 端口选对没：--list-ports")
        print("    3) 波特率是否 115200")
        print("    4) 固件 APP_OUTPUT_MODE 与解析方式是否一致：")
        print("       二进制帧 → 不加 --text；mode 1 → --text；mode 0 → --text --text-raw")
        return 2

    desc = ("串口 %s" % args.port if args.port
            else ("文件 %s" % args.file if args.file else "假数据（无硬件）"))
    desc += " · %s · %.0f Hz" % ("文本" if use_text else "二进制帧", rate)

    if args.csv:
        write_csv(args.csv, samples, rate, lsb=args.lsb)
    if args.window or args.window_out:
        if not args.window:
            print("[!] 给了 --window-out 但没给 --window，默认按 1024 切")
            args.window = 1024
        write_windows(args.window_out or "windows", samples, args.window,
                      args.hop or args.window, rate, desc, lsb=args.lsb)
    if args.plot or args.show:
        write_plot(args.plot or "vibration.png", samples, rate, window=args.window,
                   hop=args.hop, show=args.show)

    print("\n共解出 %d 个采样点（%.3f 秒 @ %.0f Hz），总耗时 %.2f 秒"
          % (len(samples), len(samples) / rate, rate, time.time() - t0))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # 串口采集时按 Ctrl-C 是正常操作（"采够了"），不要打栈
        print("\n[i] 已中断（Ctrl-C）。若已指定 --csv / --plot，"
              "请在采集完成后重跑一次以生成这些输出。")
        sys.exit(130)
