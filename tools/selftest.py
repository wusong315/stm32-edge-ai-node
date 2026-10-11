#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""selftest.py — 不需要硬件的数据链路自测

跑法::

    python selftest.py           # 全绿则退出码 0

覆盖的是**真实用到的那条路径**：假数据 → 二进制帧 → 流式解析 → 连续序列 → 窗口切片。
串口只是把同样的字节喂进来而已，所以这些测试通过之后，上板只会剩"接线对不对"
这一个变量。

测试设计原则：**每条都必须能真的失败**。只做"不抛异常就算过"的断言没有意义。
"""

from __future__ import annotations

import sys

import numpy as np

import vibframe as vf
import vibsource as vs
import check_rate as cr

_fail = 0
_pass = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global _fail, _pass
    if cond:
        _pass += 1
        print("  PASS  %s" % name)
    else:
        _fail += 1
        print("  FAIL  %s   %s" % (name, detail))


def check_eq(name: str, got, want) -> None:
    check(name, got == want, "got=%r want=%r" % (got, want))


# ---------------------------------------------------------------------------
def t_golden_format():
    """帧格式的黄金样本：任何一个偏移/字节序写错，这条都会挂。"""
    print("\n[1] 帧格式黄金样本（逐字节，锁定偏移与字节序）")
    frame = vf.encode_frame(0, [1], [2], [3])
    # 手算：AA 55 | 00 00 | 01 | ax=0100 ay=0200 az=0300 | 校验 06
    want = bytes([0xAA, 0x55, 0x00, 0x00, 0x01,
                  0x01, 0x00, 0x02, 0x00, 0x03, 0x00,
                  0x06])
    check_eq("1 点帧的字节序列", frame.hex(" "), want.hex(" "))
    check_eq("整帧长度 == 12", len(frame), 12)

    # 与固件 FRAME_TOTAL 对齐
    check_eq("默认总长 == 198（固件 FRAME_TOTAL）", vf.DEFAULT_FRAME_TOTAL, 198)
    check_eq("frame_total(32) == 198", vf.frame_total(32), 198)
    check_eq("固件 SAMPLES_PER_FRAME == 解析器默认值",
             vf.DEFAULT_SAMPLES_PER_FRAME, 32)


def t_roundtrip():
    """编码→解析 必须逐位还原，且负数/极值不能错。"""
    print("\n[2] 编解码往返（含负数与 int16 边界）")
    rng = np.random.default_rng(7)
    ax = rng.integers(-32768, 32767, 32).astype("<i2")
    ay = rng.integers(-32768, 32767, 32).astype("<i2")
    az = rng.integers(-32768, 32767, 32).astype("<i2")
    blob = vf.encode_frame(1234, ax, ay, az)

    p = vf.FrameParser()
    frames = p.feed(blob)
    p.finish()
    check_eq("解出 1 帧", len(frames), 1)
    if not frames:
        return
    f = frames[0]
    check_eq("序号", f.seq, 1234)
    check_eq("点数", f.samples, 32)
    check("ax 完全一致", np.array_equal(f.ax, ax), "%s" % f.ax[:4])
    check("ay 完全一致", np.array_equal(f.ay, ay))
    check("az 完全一致", np.array_equal(f.az, az))
    check_eq("无校验错", p.stats.checksum_errors, 0)

    # 边界值单测
    b = vf.encode_frame(0, [-32768], [32767], [0])
    f2 = vf.FrameParser().feed(b)[0]
    check("int16 极值 -32768 还原正确", int(f2.ax[0]) == -32768)
    check("int16 极值  32767 还原正确", int(f2.ay[0]) == 32767)


def t_arbitrary_chunking():
    """帧被切成任意片段（极端情况：1 字节）也必须能还原。"""
    print("\n[3] 流式切分：逐字节喂入")
    blob = vs.make_fake_frames(seconds=0.2, rate=1000, banner="")
    whole = vf.FrameParser()
    n_whole = len(whole.feed(blob))

    one = vf.FrameParser()
    got = 0
    for i in range(len(blob)):
        got += len(one.feed(blob[i:i + 1]))
    check_eq("单块喂入的帧数 vs 逐字节喂入", n_whole, got)
    check("帧数 > 0", n_whole > 0, "n=%d" % n_whole)
    check_eq("逐字节喂入无校验错", one.stats.checksum_errors, 0)
    check_eq("逐字节喂入无重同步丢弃", one.stats.resync_bytes, 0)

    # 随机切分
    rng = np.random.default_rng(3)
    rnd = vf.FrameParser()
    pos = 0
    cnt = 0
    while pos < len(blob):
        k = int(rng.integers(1, 97))
        cnt += len(rnd.feed(blob[pos:pos + k]))
        pos += k
    check_eq("随机切分的帧数与单块一致", cnt, n_whole)


def t_noise_and_banner():
    """真实串口流前面一定有人可读文字 → 解析器要能丢掉它并同步上。"""
    print("\n[4] 前置噪声/开机横幅的自恢复")
    banner = ("\r\n\r\n=== stm32-edge-ai-node boot ===\r\n"
              "SYSCLK = 168000000 Hz, I2C1 = 400 kHz, TIM2 = 1 kHz\r\n"
              "mode 2: binary frames @1 kHz, 198 bytes/frame\r\n"
              "[OK] MPU6050 online (WHO_AM_I=0x68), 1kHz sampling starts\r\n")
    blob = vs.make_fake_frames(seconds=0.1, rate=1000, banner=banner)
    p = vf.FrameParser()
    frames = p.feed(blob)
    p.finish()
    check("解出帧", len(frames) > 0, "n=%d" % len(frames))
    check("丢弃了横幅字节", p.stats.resync_bytes >= len(banner) - 4,
          "resync=%d banner=%d" % (p.stats.resync_bytes, len(banner)))
    check_eq("无校验错", p.stats.checksum_errors, 0)
    check_eq("序号从 0 开始", frames[0].seq, 0)


def t_checksum_gate():
    """校验和必须真的挡住坏帧 —— 包括 payload 里偶然出现的 AA 55。"""
    print("\n[5] 校验和门禁 + 假同步头")
    good = vs.make_fake_frames(seconds=0.1, rate=1000)
    f0 = good[:vf.DEFAULT_FRAME_TOTAL]

    # 5a) 翻转一个 payload 字节 → 该帧必须被拒，且不能吃掉后面的帧
    bad = bytearray(f0)
    bad[10] ^= 0xFF
    p = vf.FrameParser()
    frames = p.feed(bytes(bad) + good[vf.DEFAULT_FRAME_TOTAL:])
    p.finish()
    check_eq("坏帧被拒（校验错计数 =1）", p.stats.checksum_errors, 1)
    check("坏帧之后的帧仍被解出", len(frames) >= 1, "n=%d" % len(frames))
    check("解出的第一帧是好的那一帧", frames and frames[0].seq == 1,
          "seq=%s" % (frames[0].seq if frames else None))

    # 5b) payload 里故意放 0x55AA（小端字节 = AA 55）→ 不能被误当成同步头
    ax = np.zeros(32, dtype="<i2"); ay = np.zeros(32, dtype="<i2"); az = np.zeros(32, dtype="<i2")
    ax[0] = 0x55AA                      # 字节序列 AA 55，正好构成假同步头
    tricky = vf.encode_frame(9, ax, ay, az)
    check("构造成功：payload 内含 AA 55", tricky[5:7] == b"\xAA\x55")
    p2 = vf.FrameParser()
    got = p2.feed(tricky)
    check_eq("含假同步头的帧只解出 1 帧（没被重复/错位）", len(got), 1)
    check("数据正确", got and int(got[0].ax[0]) == 0x55AA)

    # 5c) 直接喂一段"假头 + 垃圾 198 字节"，必须不产出任何帧
    junk = b"\xAA\x55" + bytes(range(1, 197))
    p3 = vf.FrameParser()
    got3 = p3.feed(junk)
    p3.finish()
    check_eq("纯垃圾流解出 0 帧", len(got3), 0)
    check("垃圾流计入校验错或重同步", (p3.stats.checksum_errors + p3.stats.resync_bytes) > 0)


def t_seq_gap():
    """序号断档要能被发现并计数 —— 这是"有没有丢点"的唯一客观证据。"""
    print("\n[6] 序号断档检测")
    # 0.4 s @1kHz = 400 点 → 13 帧，才够取到第 7、8 帧
    a = vs.make_fake_frames(seconds=0.4, rate=1000, seq0=0)
    ft = vf.DEFAULT_FRAME_TOTAL
    check("假数据帧数足够", len(a) // ft >= 9, "总长=%d 帧数=%d" % (len(a), len(a) // ft))
    kept = a[0 * ft:3 * ft]              # 帧 0,1,2
    jumped = a[7 * ft:9 * ft]            # 帧 7,8（丢了 3,4,5,6）
    p = vf.FrameParser()
    frames = p.feed(kept + jumped)
    p.finish()
    check_eq("解出 5 帧", len(frames), 5)
    check_eq("序号断档 1 次", p.stats.seq_gaps, 1)
    check_eq("推算丢帧 4 帧", p.stats.frames_lost, 4)
    seqs = [f.seq for f in frames]
    check_eq("序号序列", seqs, [0, 1, 2, 7, 8])


def t_truncated_tail():
    """流在帧中间被切断（拔串口/断电）不能崩，且要如实报残留。"""
    print("\n[7] 截断的尾部")
    blob = vs.make_fake_frames(seconds=0.5, rate=1000)     # 15 帧
    p = vf.FrameParser()
    frames = p.feed(blob[:-40])
    p.finish()
    check_eq("解出 14 个完整帧", len(frames), 14)
    check_eq("残留字节如实统计", p.stats.truncated, vf.DEFAULT_FRAME_TOTAL - 40)
    check_eq("残缺帧没有被当成好帧", p.stats.checksum_errors, 0)


def t_stats_and_stack():
    """统计口径与连续序列拼接。"""
    print("\n[8] 统计与序列拼接")
    blob = vs.make_fake_frames(seconds=1.0, rate=1000, seq0=0,
                              anomaly_start=0.5, anomaly_seconds=0.3)
    p = vf.FrameParser()
    frames = p.feed(blob)
    p.finish()
    check_eq("1000 点 / 32 点每帧 → 31 帧", len(frames), 31)
    check_eq("采样点数 = 31*32", p.stats.samples_ok, 31 * 32)
    check_eq("无校验错", p.stats.checksum_errors, 0)
    check_eq("无断档", p.stats.seq_gaps, 0)

    s = vf.stack_frames(frames)
    check_eq("拼接后形状", s.shape, (31 * 32, 3))
    check("静置 az 接近 1 g", abs(float(np.mean(s[:, 2])) - 1.0) < 0.05,
          "mean_az=%.4f" % float(np.mean(s[:, 2])))
    check("ax 接近 0 g", abs(float(np.mean(s[:, 0]))) < 0.05,
          "mean_ax=%.4f" % float(np.mean(s[:, 0])))

    # 异常段能量应明显高于正常段
    n = len(s)
    seg_norm = s[:n // 2, 2]
    seg_anom = s[n // 2:, 2]
    check("异常段标准差 > 正常段（假数据确实带异常）",
          float(np.std(seg_anom)) > float(np.std(seg_norm)) * 2,
          "norm=%.4f anom=%.4f" % (np.std(seg_norm), np.std(seg_anom)))


def t_windows():
    """窗口切片（喂给 NanoEdge AI Studio 的分析窗口）。"""
    print("\n[9] 分析窗口切片")
    x = np.arange(10 * 3, dtype=np.float32).reshape(10, 3)
    w = list(vf.iter_windows(x, 4))
    check_eq("10 点 / 窗 4 / 步 4 → 2 个窗口", len(w), 2)
    check_eq("第 1 个窗口首值", float(w[0][0, 0]), 0.0)
    check_eq("第 2 个窗口首值", float(w[1][0, 0]), 12.0)
    w2 = list(vf.iter_windows(x, 4, hop=2))
    check_eq("步长 2 → 4 个窗口", len(w2), 4)

    s = vf.stack_frames(vf.FrameParser().feed(
        vs.make_fake_frames(seconds=2.5, rate=1000)))
    ws = list(vf.iter_windows(s, 1024))
    check("2500 点 → 2 个 1024 窗口", len(ws) == 2, "n=%d" % len(ws))
    check_eq("窗口形状", ws[0].shape, (1024, 3))


def t_text_parsers():
    """mode 0 / mode 1 两条文本路径 —— 上板最先用到的就是它们。"""
    print("\n[10] 文本模式解析（mode 0 / mode 1）")
    txt1 = vs.make_fake_text(seconds=1.0, rate=200, mode="g",
                             banner="\r\nmode 1: text waveform @200 Hz\r\n")
    p1 = vs.TextParser()
    rows = p1.feed(txt1)
    p1.finish()
    check_eq("mode 1 识别为 g 值", p1.mode, "g")
    check_eq("行数 = 200", p1.lines, 200)
    check_eq("无解析失败行", p1.bad_lines, 0)
    check("横幅被当注释跳过", p1.comment_lines >= 1)
    s1 = p1.samples()
    check("az 均值 ≈ 1 g", abs(float(np.mean(s1[:, 2])) - 1.0) < 0.05,
          "%.4f" % float(np.mean(s1[:, 2])))

    txt0 = vs.make_fake_text(seconds=1.0, rate=10, mode="raw",
                             banner="\r\nmode 0: debug text @10 Hz\r\n")
    p0 = vs.TextParser()
    p0.feed(txt0)
    p0.finish()
    check_eq("mode 0 识别为原始整数", p0.mode, "raw")
    check_eq("mode 0 行数 = 10", len(p0.samples()), 10)

    # ⚠️ 这里**不能**用 "az 均值 ≈ 1 g" 来断言：mode 0 只有 10 Hz，
    #    而叶频 180 Hz 采到 10 Hz 会发生混叠，均值天然偏离 1（实测 0.9454）。
    #    正确的判据是**恒等性**：原始整数 / 16384 再乘回去必须逐点还原。
    _t0, ax0, ay0, az0 = vs.synth_vibration(seconds=1.0, rate=10)
    laz = vs.to_lsb(az0).astype(np.int64)
    back = np.rint(p0.samples()[:, 2].astype(np.float64) * vf.ACCEL_LSB_PER_G)
    check("mode 0 原始整数逐点还原（恒等性，非统计）",
          np.array_equal(back.astype(np.int64), laz),
          "首 3 点 back=%s laz=%s" % (back[:3].astype(int), laz[:3]))
    check("mode 0 与 mode 1 单位口径不同（同一点 raw 与 g 之比 = 16384）",
          abs(float(p0.samples()[0, 2] * vf.ACCEL_LSB_PER_G) - float(laz[0])) < 0.5,
          "raw_g=%.6f laz0=%d" % (float(p0.samples()[0, 2]), int(laz[0])))

    # 文本被任意切分
    p2 = vs.TextParser()
    total = 0
    for i in range(0, len(txt1), 7):
        total += len(p2.feed(txt1[i:i + 7]))
    p2.finish()
    check_eq("文本按 7 字符切分后行数不变", total, 200)


def t_fake_physical_sanity():
    """假数据的物理合理性：静置时 az≈+1g、翻转后 az≈-1g。"""
    print("\n[11] 假数据的物理口径")
    _t, ax, ay, az = vs.synth_vibration(seconds=0.5, rate=1000)
    check("静置 az ∈ [0.9, 1.1] g", 0.9 < float(np.mean(az)) < 1.1)
    lsb = vs.to_lsb(az)
    check("转 LSB 后均值 ≈ 16384±1000",
          abs(float(np.mean(lsb)) - 16384) < 1000, "%.0f" % float(np.mean(lsb)))
    check("LSB 全部落在 int16 范围内",
          bool(np.all(lsb >= -32768) and np.all(lsb <= 32767)))


def t_cli_smoke():
    """CLI 参数接线的冒烟测试 —— 纯逻辑测不出"参数名写错/没接上"这类 bug。

    不起串口、不弹窗（不传 --plot），只验证：假数据 → CSV / 窗口目录 真的落盘。
    """
    print("\n[12] CLI 冒烟（假数据 → CSV / 窗口，全部写进临时目录）")
    import contextlib
    import io
    import os
    import tempfile

    import parse_frames as pf

    def _run(argv):
        """跑 CLI 但吞掉它自己的输出 —— check() 的 PASS 行要留在报告里。"""
        with contextlib.redirect_stdout(io.StringIO()):
            return pf.main(argv)

    with tempfile.TemporaryDirectory() as d:
        csv_path = os.path.join(d, "out.csv")
        rc = _run(["--fake", "--duration", "1.0", "--csv", csv_path, "--quiet"])
        check_eq("退出码 0", rc, 0)
        check("CSV 已生成", os.path.isfile(csv_path))
        with open(csv_path, "r", encoding="utf-8") as fh:
            head = fh.readline().strip()
            rows = sum(1 for _ in fh)
        check_eq("CSV 表头", head, "t_s,ax_g,ay_g,az_g")
        # 1000 个合成点只能凑出 31 个完整 32 点帧 → 992 点（不是 1000！）
        check_eq("CSV 行数 = 31*32 = 992", rows, 992)

        # 切窗口 + manifest
        wdir = os.path.join(d, "win")
        rc2 = _run(["--fake", "--duration", "2.5", "--window", "1024",
                    "--window-out", wdir, "--quiet"])
        check_eq("切窗口退出码 0", rc2, 0)
        check("manifest.txt 已生成", os.path.isfile(os.path.join(wdir, "manifest.txt")))
        wins = [f for f in os.listdir(wdir) if f.startswith("win_")]
        check_eq("写出 2 个窗口文件", len(wins), 2)

        # 无数据必须给非 0 退出码（脚本里能据此判断）
        junk = os.path.join(d, "junk.bin")
        with open(junk, "wb") as fh:
            fh.write(b"not a frame at all")
        rc3 = _run(["--file", junk, "--quiet"])
        check_eq("无数据时退出码 2", rc3, 2)


def t_rate_check():
    """采样节拍实测：既要算得准，也要能识破真机上踩过的那个坑。"""
    print("\n[13] 采样节拍实测（check_rate.py 的纯逻辑）")

    # --- 黄金样本：固件 mode 0 的真实行格式（权威 = main.c 里的 printf）---
    real = (b"t=     100 | ax= 11208 ay=  2640 az= 12360 | T= -5216"
            b" | gx=    13 gy=   208 gz=  -272 | lost=21 ovf=0")
    check_eq("解析固件真实数据行", cr.parse_line(real), (100, 21, 0))
    check_eq("开机横幅不是数据行",
             cr.parse_line(b"=== stm32-edge-ai-node boot ==="), None)
    check_eq("WAIT 提示不是数据行",
             cr.parse_line(b"[WAIT] MPU6050 not found (id=0x00). "
                           b"Check wiring: VCC=3V3 GND SDA=PB7 SCL=PB6 AD0=GND"), None)
    check_eq("空行不是数据行", cr.parse_line(b""), None)

    # --- 拟合：完美 1000 Hz 必须被精确还原 ---
    # 10 Hz 打印 × 每行 100 拍 ⇒ 1000 Hz
    perfect = [(i / 10.0, i * 100) for i in range(120)]
    k = cr.fit_slope(perfect)
    check("完美数据拟合 == 1000 Hz", abs(k - 1000.0) < 1e-6, "k=%.6f" % k)

    # --- 拟合 vs "末值−首值"：末点漂移时前者更稳 ---
    # 理由不是"抗离群点"（最小二乘并不免疫），而是它用上了全部 N 个样本，
    # 单点误差被摊薄；而末值−首值法只有那两个点，其中一个坏掉就全废。
    drifted = list(perfect)
    drifted[-1] = (drifted[-1][0], drifted[-1][1] - 300)     # 末点少 300 拍
    k_d = cr.fit_slope(drifted)
    naive_d = (drifted[-1][1] - drifted[0][1]) / (drifted[-1][0] - drifted[0][0])
    check("末点漂移时，拟合的误差 < 末值−首值法",
          abs(k_d - 1000.0) < abs(naive_d - 1000.0),
          "fit=%.2f naive=%.2f" % (k_d, naive_d))

    # --- 单调性检查器 ---
    check_eq("单调数列的倒退次数 == 0", cr.backwards_count([1, 2, 3, 4]), 0)
    check_eq("重复值也算一次倒退", cr.backwards_count([1, 2, 2, 3]), 1)

    # --- 🔴 复现真机症状：USB 转串口 FIFO 吐出的"旧 run 尾部"混进新数据 ---
    # 真机上就是这样：一行复位前的 t=35100 混进了新的 1000 Hz 序列，
    # 结果量出 -1.6 Hz（第一次）和 727 Hz（第二次），让人以为固件有问题。
    stale = [(0.000, 35100)] + [(i / 10.0, i * 100) for i in range(1, 100)]
    naive_bad = (stale[-1][1] - stale[0][1]) / (stale[-1][0] - stale[0][0])
    k_bad = cr.fit_slope(stale)
    check("（对照）末值−首值法在这份数据上给出负节拍（复现真机）",
          naive_bad < 0.0, "naive=%.1f" % naive_bad)
    check("拟合虽好得多，但单靠拟合仍偏 >15% —— 所以必须靠守卫拦住",
          abs(k_bad - 1000.0) > 150.0, "k=%.1f" % k_bad)
    check("倒退次数 > 0 能把混流逮住",
          cr.backwards_count([t for _, t in stale]) > 0,
          "backwards=%d" % cr.backwards_count([t for _, t in stale]))
    # 丢弃那行旧数据（等价于工具里的 --skip）后必须精确恢复
    k_fixed = cr.fit_slope([s for s in stale if s[1] < 10000])
    check("丢弃旧数据后精确恢复 1000 Hz", abs(k_fixed - 1000.0) < 1e-6,
          "k=%.6f" % k_fixed)

    # --- 参数守卫：样本不足 / 墙钟无变化 必须抛错，而不是返回一个假数 ---
    for name, bad in (("样本 < 2 要抛 ValueError", [(0.0, 0)]),
                      ("墙钟全相同要抛 ValueError", [(1.0, 5), (1.0, 9)])):
        try:
            cr.fit_slope(bad)
            check(name, False, "居然没抛错")
        except ValueError:
            check(name, True)


def main() -> int:
    print("=" * 68)
    print("stm32-edge-ai-node · PC 端数据链路自测（不需要硬件）")
    print("=" * 68)
    t_golden_format()
    t_roundtrip()
    t_arbitrary_chunking()
    t_noise_and_banner()
    t_checksum_gate()
    t_seq_gap()
    t_truncated_tail()
    t_stats_and_stack()
    t_windows()
    t_text_parsers()
    t_fake_physical_sanity()
    t_cli_smoke()
    t_rate_check()

    print("\n" + "=" * 68)
    print("结果：%d 通过 / %d 失败" % (_pass, _fail))
    print("=" * 68)
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
