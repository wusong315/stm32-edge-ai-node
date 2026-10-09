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


def collect_binary(args) -> tuple[list[np.ndarray], vf.ParserStats, float]:
    """二进制帧路径：帧流 → (g 值块列表, 统计, 覆盖时长)。

    🔴 串口采集时按 Ctrl-C 是**正常操作**（"采够了"），所以这里捕获它并
       优雅收尾 —— 否则刚采的几万个点会连同 CSV/图一起丢掉。
    """
    parser = vf.FrameParser()
    g_chunks: list[np.ndarray] = []
    n_frames = 0
    t_begin = time.time()

    source = _make_source(args, binary=True)
    if args.save_raw:
        source = _tee_save_raw(source, args.save_raw)

    interrupted = False
    try:
        for chunk in source:
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
    return g_chunks, parser.stats, time.time() - t_begin


def collect_text(args) -> tuple[list[np.ndarray], vs.TextParser, float]:
    """文本路径（mode 0 / mode 1）：文本流 → (g 值块列表, 解析器, 覆盖时长)。"""
    parser = vs.TextParser()
    g_chunks: list[np.ndarray] = []
    t_begin = time.time()

    source = _make_source(args, binary=False)
    if args.save_raw:
        source = _tee_save_raw(source, args.save_raw)

    interrupted = False
    try:
        for chunk in source:
            text = chunk.decode("utf-8", errors="replace")
            rows = parser.feed(text)
            if rows:
                g_chunks.append(np.asarray(rows, dtype=np.float32))
            if args.max_samples and len(parser.rows) >= args.max_samples:
                break
    except KeyboardInterrupt:
        interrupted = True
    parser.finish()
    if parser.rows:
        # rows 已含全部历史，直接以它为准（避免重复拼接）
        g_chunks = [np.asarray(parser.rows, dtype=np.float32)]
    if interrupted:
        print("\n[i] 收到 Ctrl-C，停止采集（已收到的数据会照常写出）")
    return g_chunks, parser, time.time() - t_begin


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
    print("  实际接收耗时 %.2f 秒" % seconds)
    print("-" * 68)


def print_report_binary(stats: vf.ParserStats, samples: np.ndarray,
                        rate: float, seconds: float) -> None:
    print("\n" + "-" * 68)
    print("二进制帧健康度")
    print("  " + stats.as_text())
    if len(samples):
        print("  时长 %.3f 秒 @ %.0f Hz（应 ≈ 帧数 × %d / %.0f）"
              % (len(samples) / rate, rate, vf.DEFAULT_SAMPLES_PER_FRAME, rate))
        print("  均值 [g]: ax=%.3f ay=%.3f az=%.3f   （静置时应 ≈ 0, 0, +1）"
              % (samples[:, 0].mean(), samples[:, 1].mean(), samples[:, 2].mean()))
    print("  实际接收耗时 %.2f 秒" % seconds)
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
        samples = (np.concatenate(chunks, axis=0) if chunks
                   else np.zeros((0, 3), dtype=np.float32))
        print_report_text(parser, rate, wall)
    else:
        rate = args.rate or RATE_BINARY
        chunks, stats, wall = collect_binary(args)
        samples = (np.concatenate(chunks, axis=0) if chunks
                   else np.zeros((0, 3), dtype=np.float32))
        print_report_binary(stats, samples, rate, wall)

    if len(samples) == 0:
        print("\n[!] 一个点都没解出来。排查顺序：")
        print("    1) 接线：TX/RX 是否交叉、GND 是否共地")
        print("    2) 端口选对没：--list-ports")
        print("    3) 波特率是否 115200")
        print("    4) 固件 APP_OUTPUT_MODE 与解析方式是否一致"
              "（二进制要用默认，文本要加 --text）")
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
