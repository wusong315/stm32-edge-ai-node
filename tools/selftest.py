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

import os
import shutil
import sys
import tempfile

import numpy as np

import vibframe as vf
import vibsource as vs
import check_rate as cr
import check_blade_peak as cb
import pwm_sweep as ps
import check_pwm_cmd as cpc
import calib_dev as cd

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


def t_text_rate_agreement():
    """🔴 采样率必须由**解析器实际识别到的** mode 决定，而不是 CLI 开关。

    回归用例来自真实踩到的一个坑：`--text` 但不给 `--text-raw`，打在 mode 0 的
    真实固件输出上 —— 解析器照样认出 40 行，但报告会打出「mode 0 → 原始整数」
    的标签 + 「200 Hz」的数字（**自相矛盾**），CSV 的 `t_s` 列整体错 **20 倍**。
    而 `check_blade_peak.load_csv()` 正是**从 `t_s` 反推采样率**的 —— 错会一路传下去。
    """
    print("\n[10b] 文本档采样率口径（回归：报告自相矛盾 / 时间轴错 20 倍）")
    import parse_frames as pf

    # ① 开关说 mode 1（200 Hz），实测是 mode 0 → 必须改成 10 Hz 并给警告
    rate, warn = pf.reconcile_text_rate(pf.RATE_TEXT_G, False, "raw")
    check_eq("开关给 200 Hz、实测 mode 0 → 速率改成 10 Hz",
             rate, pf.RATE_TEXT_RAW)
    check("不一致时必须给出警告文案", bool(warn), repr(warn))

    # ② 一致时不打扰用户
    rate2, warn2 = pf.reconcile_text_rate(pf.RATE_TEXT_RAW, True, "raw")
    check_eq("一致时速率不变", rate2, pf.RATE_TEXT_RAW)
    check("一致时不给警告", warn2 is None, repr(warn2))

    # ③ 反方向：开关说 mode 0、实测是 mode 1 → 改成 200 Hz
    rate3, warn3 = pf.reconcile_text_rate(pf.RATE_TEXT_RAW, True, "g")
    check_eq("开关给 10 Hz、实测 mode 1 → 速率改成 200 Hz",
             rate3, pf.RATE_TEXT_G)
    check("反方向也要警告", bool(warn3), repr(warn3))

    # ④ 一行都没认出来（parser.mode 仍是 None）→ 保持原样，别瞎改
    rate4, warn4 = pf.reconcile_text_rate(pf.RATE_TEXT_G, False, None)
    check_eq("未识别出 mode 时不改速率", rate4, pf.RATE_TEXT_G)
    check("未识别出 mode 时不给警告", warn4 is None, repr(warn4))

    # ⑤ 端到端：mode 0 文本走 CLI，**故意不给 --text-raw**，时间轴必须覆盖真实时长
    import contextlib
    import io

    with tempfile.TemporaryDirectory() as td:
        raw = os.path.join(td, "m0.txt")
        with open(raw, "w", encoding="utf-8") as fh:
            fh.write(vs.make_fake_text(seconds=4.0, rate=10, mode="raw",
                                       banner="\r\nmode 0: debug text @10 Hz\r\n"))
        csv_path = os.path.join(td, "m0.csv")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            rc = pf.main(["--file", raw, "--text", "--csv", csv_path])
        check_eq("CLI 退出码 0", rc, 0)
        check("CLI 打出了『采样率改用 10 Hz』的警告", "10 Hz" in buf.getvalue(),
              buf.getvalue()[-160:].replace("\n", " | "))
        arr = np.genfromtxt(csv_path, delimiter=",", names=True)
        n = int(np.size(arr["t_s"]))
        span = float(arr["t_s"][-1]) if n else 0.0
        check("CSV 点数 = 40", n == 40, "n=%d" % n)
        check("CSV 时间轴覆盖 ≈3.9 秒（错档时只会是 0.195 秒）",
              abs(span - 3.9) < 0.25, "span=%.3f" % span)


def t_binary_zero_frame_hint():
    """二进制档"0 帧但收到字节"时，必须把**固件的文本诊断**打出来。

    回归用例来自真实踩到的一个坑：mode 2 在 MPU6050 缺席时不出帧，只反复打印
    `[WAIT] MPU6050 not found (id=0x00). Check wiring: ...`。工具当时只报
    "重同步丢弃 870 字节"，排查被引向 TX/RX 接线 —— 而原因固件早就写在字节里了。
    """
    print("\n[10c] 二进制 0 帧时的文本诊断（回归：固件的话就在字节里）")
    import contextlib
    import io

    import parse_frames as pf

    banner = (b"[WAIT] MPU6050 not found (id=0x00). Check wiring: VCC=3V3 GND "
              b"SDA=PB7 SCL=PB6 AD0=GND\r\n") * 4
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        pf._print_text_hint(banner)
    out = buf.getvalue()
    check("固件的 [WAIT] 行被原样打出来", "MPU6050 not found" in out, out[:120])
    check("提示里点出'传感器缺席'这类成因", "传感器缺席" in out, out[:200])

    # 🔴 这条是"判据必须能判否"：真噪声不能被误报成"固件输出"
    buf2 = io.StringIO()
    with contextlib.redirect_stdout(buf2):
        pf._print_text_hint(bytes(range(256)) * 2)
    check("非文本字节不误报为固件输出", buf2.getvalue() == "",
          buf2.getvalue()[:80])

    buf3 = io.StringIO()
    with contextlib.redirect_stdout(buf3):
        pf._print_text_hint(b"")
    check("空字节样本不打印任何东西", buf3.getvalue() == "",
          buf3.getvalue()[:80])


def t_big_drop_positions():
    """🔴 固件停机 = 时间轴上的**隐形接缝**。

    这一条是上板实测逼出来的：电机启动的电磁扰动把 I2C 打掉，固件停机去重试，
    恢复后继续吐帧。于是**校验错 0 / 序号断档 0 —— 全绿**，
    但 `t_s` 是按样本索引算的，那段壁钟被压缩掉了。
    "帧有没有丢"和"时间有没有停"是两件事，前者查不出后者，所以必须单独判。"""
    print("\n[12] 时间轴断点：大段丢弃必须记下位置")
    import contextlib
    import io
    import parse_frames as pf

    a = vs.make_fake_frames(seconds=0.096, rate=1000)            # 3 帧，seq 0..2
    b = vs.make_fake_frames(seconds=0.096, rate=1000, seq0=3)    # 3 帧，seq 3..5
    stall = (b"[WAIT] MPU6050 not found (id=0x00). Check wiring: "
             b"VCC=3V3 GND SDA=PB7 SCL=PB6 AD0=GND\r\n") * 8
    check("构造的停机文本 > 阈值", len(stall) >= vf.BIG_DROP_BYTES,
          "len=%d" % len(stall))

    p = vf.FrameParser()
    frames = p.feed(a + stall + b)
    p.finish()

    check_eq("两侧各 3 帧都解出", len(frames), 6)
    check_eq("校验错 0 —— 帧本身没坏", p.stats.checksum_errors, 0)
    check_eq("序号断档 0 —— 停的是时间，不是帧计数", p.stats.seq_gaps, 0)
    check_eq("记到 1 处大段丢弃", len(p.stats.big_drops), 1)
    if len(p.stats.big_drops) == 1:
        pos, n = p.stats.big_drops[0]
        check_eq("断点位置 = 前 3 帧的样本数", pos, 96)
        check_eq("丢弃字节数 = 停机文本长度", n, len(stall))

    # 判据必须能判否：只是切坏几个字节（链路常态）绝不能报成"断点"。
    p2 = vf.FrameParser()
    p2.feed(a + b"XYZ" + b)
    p2.finish()
    check_eq("小段丢弃不报断点", len(p2.stats.big_drops), 0)
    check("小段丢弃仍计入 resync_bytes", p2.stats.resync_bytes >= 3,
          "resync=%d" % p2.stats.resync_bytes)

    # 报告层：数据层记了位置，报告层不打出来等于白记（"有特征没断言 = 没覆盖"）。
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        pf.print_report_binary(p.stats, np.zeros((96, 3)), 1000.0, 10.0)
    out = buf.getvalue()
    check("报告打出「时间轴断点」", "时间轴断点" in out, out[:240])
    check("报告打出「壁钟缺口」", "壁钟缺口" in out, out[:240])
    check("报告把断点位置写成样本索引", "样本 96" in out, out[:400])

    buf2 = io.StringIO()
    with contextlib.redirect_stdout(buf2):
        pf.print_report_binary(p2.stats, np.zeros((96, 3)), 1000.0, 0.15)
    check("无断点时报告不打断点行", "时间轴断点" not in buf2.getvalue(),
          buf2.getvalue()[:240])

    # `--file` 离线重解析没有墙钟（seconds≈0）→ 不能打出"壁钟缺口"这种假数
    # （第一版漏了这个守卫，实测打出 `壁钟缺口 -9.79 秒` —— 一眼假的数字）
    buf4 = io.StringIO()
    with contextlib.redirect_stdout(buf4):
        pf.print_report_binary(p.stats, np.zeros((96, 3)), 1000.0, 0.002)
    check("离线重解析不打印壁钟缺口（没有墙钟可测）",
          "壁钟缺口" not in buf4.getvalue(), buf4.getvalue()[:240])

    # 在线且缺口偏大 → 必须进"需要看的点"，不能只说 ✅ 未发现异常
    buf5 = io.StringIO()
    with contextlib.redirect_stdout(buf5):
        pf.print_report_binary(p.stats, np.zeros((96, 3)), 1000.0, 0.6)
    check("壁钟缺口偏大时给出警告", "壁钟缺口偏大" in buf5.getvalue(),
          buf5.getvalue()[:400])


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


def t_blade_peak():
    """R17 频谱门禁：既要抓得到已知频率，也要能识破噪声与混叠。

    🔴 这一节里最重要的不是"能不能抓到正弦"，而是**判据本身可不可信**：
    门限若落在纯噪声可达的范围内，工具就是在掷硬币。所以下面有两条断言专门
    去量"纯噪声的 SNR 上界"，还有两条去量"模型有没有低估这个上界"。
    """
    print("\n[14] 频谱门禁（check_blade_peak.py 的纯逻辑）")

    rate = 1000.0
    df = rate / cb.DEFAULT_NPERSEG          # ≈0.977 Hz
    rng = np.random.default_rng(20261011)

    def bin_sine(f_hz, amp, n, axes=(1,)):
        """造一段 bin 对齐的正弦（幅度定标检查必须用对齐频率，否则峰会被窗稀释）。"""
        t = np.arange(n) / rate
        s = np.zeros((n, 3))
        for k in axes:
            s[:, k] = amp * np.sin(2.0 * np.pi * f_hz * t)
        return s

    # --- A) 幅度定标：给 0.05 g 的正弦，谱峰就得是 0.05 g ±10% ---
    n = 10240                                  # 1024 的整数倍，避免半段
    f_bin = 369 * df                           # = 360.35 Hz，正好落在频点上
    r = cb.analyse(bin_sine(f_bin, 0.05, n), rate)
    row = r["axes"][1]
    check("幅度定标：0.05 g 正弦 → 谱峰 ≈ 0.05 g",
          abs(row["blade_amp"] - 0.05) < 0.005, "got=%.5f" % row["blade_amp"])

    # --- B) 频率定位：误差必须小于一个频率分辨单元 ---
    check("频率定位：%.2f Hz 被还原且误差 < Δf" % f_bin,
          abs(row["blade_hz"] - f_bin) < df,
          "got=%.2f want=%.2f" % (row["blade_hz"], f_bin))
    r180 = cb.analyse(bin_sine(180.0, 0.05, n), rate)
    check("转频带也能定位（180 Hz）",
          abs(r180["axes"][1]["rot_hz"] - 180.0) <= df,
          "got=%.2f" % r180["axes"][1]["rot_hz"])

    # --- C) 选轴：只有 ay 有振动时，best_axis 必须是 ay 且只有它过 ---
    # 注意必须连 1× 一起给：新语义下"叶频带那个峰要与 K×1× 自洽"才算通过，
    # 只放一根叶频正弦会被谐波校验正确地判为"无法校验/不自洽"。
    f_rot_ay = f_bin / 3.0                     # = 123·df ≈120.12 Hz，落在转频带
    s = rng.normal(0.0, 0.005, (n, 3))
    s[:, 1] += (0.05 * np.sin(2.0 * np.pi * f_bin * np.arange(n) / rate)
                + 0.20 * np.sin(2.0 * np.pi * f_rot_ay * np.arange(n) / rate))
    r = cb.analyse(s, rate)
    check_eq("best_axis 选中有振动的那一轴", r["best_axis"], "ay")
    check("只有 ay 过判据（ax/az 是噪声）",
          r["axes"][1]["blade_ok"] and not r["axes"][0]["blade_ok"]
          and not r["axes"][2]["blade_ok"])
    check("ay 的叶频/转频比值 ≈3（谐波自洽）",
          abs(r["axes"][1]["ratio"] - 3.0) < 0.06, "ratio=%.3f" % r["axes"][1]["ratio"])

    # --- D) 🔴 纯噪声的 SNR 上界：判据可信度的地基 ---
    # 1 段（1 秒）和 18 段（10 秒）各试 12 次，取最大值。
    def noise_max(n_pts, segs_hint, trials=12):
        vals = []
        for _ in range(trials):
            s = np.zeros((n_pts, 3))
            s[:, 2] = rng.normal(0.0, 0.01, n_pts)
            vals.append(cb.analyse(s, rate)["axes"][2]["blade_snr"])
        return max(vals), cb.segment_count(n_pts, cb.DEFAULT_NPERSEG,
                                           cb.DEFAULT_OVERLAP)

    m18, k18 = noise_max(10000, 18)
    m1, k1 = noise_max(1000, 1)
    check_eq("10 秒数据的段数 == 18", k18, 18)
    check_eq("1 秒数据的段数 == 1", k1, 1)
    check("纯噪声（18 段）刷不出 8 dB —— 12 dB 门限有 ≥4 dB 余量", m18 < 8.0,
          "max=%.2f dB" % m18)
    check("纯噪声（1 段）却能刷过 8 dB —— 所以段数少时判据不可信", m1 > 8.0,
          "max=%.2f dB" % m1)

    # --- E) 模型必须"不低估"实测上界，否则那两个警告数字就是假的 ---
    check("null_bound(18 段) >= 实测 max（模型保守）",
          cb.null_bound_db(18) >= m18,
          "model=%.2f 实测=%.2f" % (cb.null_bound_db(18), m18))
    check("null_bound(1 段) >= 实测 max（模型保守）",
          cb.null_bound_db(1) >= m1,
          "model=%.2f 实测=%.2f" % (cb.null_bound_db(1), m1))
    check("段数越多上界越低（单调不增）",
          all(cb.null_bound_db(i) >= cb.null_bound_db(i + 4) for i in range(2, 20)))

    # --- F) 建议时长：按它采，上界必须真的落到门限下方 6 dB ---
    need_s = cb.suggest_seconds(12.0)
    need_n = int(need_s * rate)
    need_k = cb.segment_count(need_n, cb.DEFAULT_NPERSEG, cb.DEFAULT_OVERLAP)
    check("suggest_seconds(12) 是整秒", float(need_s).is_integer(), "%.3f" % need_s)
    check("按建议时长采，噪声上界 <= 门限 − 6 dB",
          cb.null_bound_db(need_k) <= 12.0 - 6.0 + 0.05,
          "segs=%d bound=%.2f" % (need_k, cb.null_bound_db(need_k)))
    check("建议时长落在 R6.4 用的 10 秒之内", need_s <= 10.0, "%.0f s" % need_s)

    # --- G) 混叠：真实 612 Hz（> Nyquist 500）会被报成 388 Hz 的镜像 ---
    rr = cb.analyse(bin_sine(612.0, 0.05, n), rate)
    alias_row = rr["axes"][1]
    check("612 Hz（> Nyquist）被折成 ≈388 Hz",
          abs(alias_row["blade_hz"] - 388.0) < 3.0,
          "got=%.1f" % alias_row["blade_hz"])
    check("该峰被标记为'在可能混叠的区间内'（结论里会点出歧义）",
          alias_row["blade_alias"])
    check_eq("fold_range：带顶 660 > 500 时应给出 340–500",
             cb.fold_range((300.0, 660.0), 1000.0), (340.0, 500.0))
    check_eq("fold_range：带顶 450 < 500 时无歧义，返回 None",
             cb.fold_range((300.0, 450.0), 1000.0), None)

    # --- H) 判据看的是"相对本底"，不是绝对幅值 ---
    s = np.zeros((n, 3))
    s[:, 1] = rng.normal(0.0, 0.5, n)          # 幅值比上一组大 100 倍，但仍是噪声
    check("0.5 g 的纯噪声也过不了（判据是 SNR，不是幅值）",
          not cb.analyse(s, rate)["axes"][1]["blade_ok"])

    # --- I) 去趋势：线性漂移不该被当成振动能量 ---
    t = np.arange(n) / rate
    drift = 0.5 * t / t[-1]                    # 0 → 0.5 g 的线性漂移
    d_before = float(np.std(drift))
    d_after = float(np.std(cb.detrend(drift)))
    check("detrend 把线性漂移压掉 >100 倍", d_after < d_before / 100.0,
          "before=%.4f after=%.6f" % (d_before, d_after))

    # --- K) 🔴 谐波自洽：带内最大峰若是 2× 谐波，不许冒充叶频 ---
    # 场景取自真实工况：1× 最强、2× 与 3× 同量级 ⇒ 只要叶频带下沿低于 2×，
    # "带内最大幅值"就会挑中 2×，判据假通过。这是必须能判否的一条。
    f1 = 154 * df          # 1× 转频 ≈150.4 Hz（落在默认转频带 100–220）
    f2 = 308 * df          # 2× ≈300.8 Hz（正好落进默认叶频带 300–660）
    f3 = 462 * df          # 3× ≈451.2 Hz（真叶频）
    tk = np.arange(n) / rate

    harm2 = np.zeros((n, 3))
    harm2[:, 2] = (0.20 * np.sin(2.0 * np.pi * f1 * tk)
                   + 0.03 * np.sin(2.0 * np.pi * f2 * tk))
    r2 = cb.analyse(harm2, rate)
    row2 = r2["axes"][2]
    check_eq("2× 落进叶频带 → 被识破为 2 阶谐波", row2["harmonic"], 2)
    check("这种情况下判**不通过**（2× 不许冒充叶频）",
          (not row2["blade_ok"]) and row2["blade_harmonic"],
          "ok=%s harm=%s ratio=%.2f"
          % (row2["blade_ok"], row2["blade_harmonic"], row2["ratio"] or 0.0))
    check_eq("整体结论也是不通过", r2["blade_ok"], False)
    check_eq("闸门前的阈值判定本来是「通过」（说明确实是谐波校验救回来的）",
             row2["blade_thresh_ok"], True)
    check("给出「由 1× 反推」的建议叶频带 ≈3×150 Hz",
          r2["blade_suggest_band"] is not None
          and abs(r2["blade_suggest_band"][0] - 3 * f1) < 3 * f1 * 0.13,
          "got=%r" % (r2["blade_suggest_band"],))

    # 对照：真叶频（3×）在带里 → 必须通过，且比值 ≈3
    harm3 = np.zeros((n, 3))
    harm3[:, 2] = (0.20 * np.sin(2.0 * np.pi * f1 * tk)
                   + 0.03 * np.sin(2.0 * np.pi * f3 * tk))
    r3 = cb.analyse(harm3, rate)
    row3 = r3["axes"][2]
    check_eq("真叶频（3×）→ 谐波阶次 3", row3["harmonic"], 3)
    check("真叶频（3×）→ 通过，且没被谐波校验误伤",
          row3["blade_ok"] and not row3["blade_harmonic"],
          "ratio=%.2f" % (row3["ratio"] or 0.0))

    # 4 叶桨：叶频 = 4×，把 3× 当叶频同样该被识破
    harm4 = np.zeros((n, 3))
    harm4[:, 2] = (0.20 * np.sin(2.0 * np.pi * f1 * tk)
                   + 0.03 * np.sin(2.0 * np.pi * f3 * tk))
    r4 = cb.analyse(harm4, rate, blade_multiple=4.0)
    check("blade-multiple=4 时，3× 的峰被识破为不匹配阶次",
          r4["axes"][2]["blade_harmonic"] and not r4["axes"][2]["blade_ok"],
          "harm=%s" % (r4["axes"][2]["harmonic"],))

    # 🔴 带内那个峰既不是 K× 也不是低次谐波（比如环境峰）→ 同样必须否决。
    # 这一条是真实踩过的坑：--rot-band 框歪了也会走到这里，所以"不自洽"必须一票否决，
    # 否则 2× 会在转频带量错时照旧冒充叶频通过。
    f_env = 410 * df       # 410.1 Hz 落在默认叶频带，与 1×（150.4）毫无整数倍关系
    mis = np.zeros((n, 3))
    mis[:, 2] = (0.20 * np.sin(2.0 * np.pi * f1 * tk)
                 + 0.05 * np.sin(2.0 * np.pi * f_env * tk))
    r5 = cb.analyse(mis, rate)
    row5 = r5["axes"][2]
    check_eq("410 Hz 环境峰：最近谐波阶次 → None", row5["harmonic"], None)
    check("环境峰落进叶频带 → 被标为「不自洽」并否决",
          row5["blade_inconsistent"] and not row5["blade_ok"]
          and row5["blade_thresh_ok"],
          "ratio=%.2f" % (row5["ratio"] or 0.0))
    check_eq("整体结论：不通过", r5["blade_ok"], False)
    check_eq("r5 记录了「有比值否决」", r5["any_ratio_veto"], True)

    # 逃生开关：确认 1× 在器件上看不到时，可关掉否决权退回纯阈值判定
    r5x = cb.analyse(mis, rate, harmonic_guard=False)
    check("--no-harmonic-guard 时退回阈值判定（该峰确实过 SNR/幅值门限）",
          r5x["axes"][2]["blade_ok"] and not r5x["any_ratio_veto"],
          "ok=%s" % r5x["axes"][2]["blade_ok"])

    check_eq("nearest_harmonic(2.02) == 2", cb.nearest_harmonic(2.02), 2)
    check_eq("nearest_harmonic(3.09) == 3", cb.nearest_harmonic(3.09), 3)
    check_eq("nearest_harmonic(2.40) 落不进任何阶次 → None",
             cb.nearest_harmonic(2.40), None)
    check_eq("nearest_harmonic(None) → None", cb.nearest_harmonic(None), None)
    bb = cb.blade_band_for(150.0, 3.0)
    check("blade_band_for(150) 覆盖 450 Hz",
          bb is not None and bb[0] < 450.0 < bb[1], "got=%r" % (bb,))

    # --- J) 参数守卫 + CSV 读写 ---
    for name, fn in (("点数 < 64 要抛 ValueError",
                      lambda: cb.analyse(np.zeros((40, 3)), rate)),
                     ("形状不是 (N,3) 要抛 ValueError",
                      lambda: cb.analyse(np.zeros((100, 2)), rate)),
                     ("样本 < 8 点算频谱要抛 ValueError",
                      lambda: cb.amplitude_spectrum(np.zeros(4), rate))):
        try:
            fn()
            check(name, False, "居然没抛错")
        except ValueError:
            check(name, True)

    tmp = tempfile.mkdtemp(prefix="cbp_")
    try:
        good = os.path.join(tmp, "good.csv")
        tt = np.arange(3000) / rate
        np.savetxt(good, np.column_stack([tt, np.sin(tt), np.cos(tt), tt * 0]),
                   delimiter=",", header="t_s,ax_g,ay_g,az_g", comments="",
                   fmt="%.6f")
        got_rate, got_s, src = cb.load_csv(good)
        check("从 t_s 列反推采样率 == 1000 Hz", abs(got_rate - 1000.0) < 1.0,
              "got=%.3f (%s)" % (got_rate, src))
        check_eq("样本形状 == (3000,3)",
                 (len(got_s), got_s.shape[1]), (3000, 3))

        bad = os.path.join(tmp, "bad.csv")
        with open(bad, "w", encoding="utf-8") as fh:
            fh.write("t_s,ax_g\n0.0,1.0\n0.001,1.1\n")
        try:
            cb.load_csv(bad)
            check("缺列必须抛 ValueError（而不是静默算错）", False, "居然没抛错")
        except ValueError:
            check("缺列必须抛 ValueError（而不是静默报错）", True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def t_pwm_sweep():
    """第 ⑦ 步 PWM 扫频（pwm_sweep.py 的纯逻辑：不做串口，只喂合成数据）。

    🔴 这一节要证明的不是「能不能算出频率」，而是**判定顺序**：
    先问 1× 可不可信 → 再问叶频在不在器件带宽内 → 最后才问叶频过不过判据。
    顺序错了就会出现最危险的那种结论：叶频明明超出 260 Hz 带宽，
    却因为频谱上看得见而被报成 OK（幅值已经不可信了）。
    """
    print("\n[15] PWM 扫频（pwm_sweep.py 的纯逻辑）")

    rate = 1000.0
    df = rate / cb.DEFAULT_NPERSEG          # ≈0.977 Hz
    rng = np.random.default_rng(20261012)

    def sig(f_rot, comps, n=6000):
        """造一段"电机振动"：f_rot 是 1×，comps = {谐波倍数: 幅度 g}，只加在 ay 上。"""
        t = np.arange(n) / rate
        s = rng.normal(0.0, 0.0015, (n, 3))
        for k, amp in comps.items():
            s[:, 1] += amp * np.sin(2.0 * np.pi * (k * f_rot) * t)
        return s

    def run(f_rot, comps):
        row = ps.eval_capture(sig(f_rot, comps), rate,
                              rot_band=ps.DEFAULT_ROT_BAND,
                              blade_band_wide=ps.DEFAULT_BLADE_BAND_WIDE,
                              multiple=3.0, guard=True, bw=260.0)
        return row

    # --- A) --duties 的写法与越界都必须管住 ---------------------------------
    check_eq("duties 逗号列表", ps.parse_duties("30,40,50"), [30.0, 40.0, 50.0])
    check_eq("duties 区间步进 20:60:20", ps.parse_duties("20:60:20"), [20.0, 40.0, 60.0])
    check_eq("duties 单点 55", ps.parse_duties("55"), [55.0])
    for bad, why in (("110", "越界 >100"), ("", "空"), ("60:20:10", "上界 < 下界"),
                     ("10:90:0", "步进为 0"), ("20:30:40:50", "区间字段数不对")):
        try:
            ps.parse_duties(bad)
            check("duties 拒绝 %r（%s）" % (bad, why), False, "居然没抛错")
        except ValueError:
            check("duties 拒绝 %r（%s）" % (bad, why), True)

    # --- B) 两处都过：1× = 80.1 Hz、叶频 = 240.2 Hz（< 260 Hz 带宽）--------
    f80 = 82 * df                            # bin 对齐 → 测出来就是它
    row = run(f80, {1: 0.20, 3: 0.05})
    row["duty"] = 60.0
    check("1× 定位（bin 对齐，误差 < 一个频率分辨单元）",
          abs(row["rot_hz"] - f80) < df, "got=%.2f want=%.2f" % (row["rot_hz"], f80))
    check("1× 过判据（SNR 与幅值两项）", row["rot_ok"],
          "snr=%r" % row["rot_snr"])
    check("叶频 240 Hz 未超出 260 Hz 带宽", row["blade_out_of_bw"] is False)
    check_eq("判定 = OK", ps.classify(row), ps.VERDICT_OK)
    check_eq("转速换算 = 1× × 60", round(row["rpm"]), round(f80 * 60))
    # 🔑 两遍分析的关键性质：窄带下沿必须高于 2×，否则 2× 会顶替叶频（R6.4.4）
    check("叶频窄带下沿 > 2×1×（从根上排除 2× 冒充）",
          row["blade_band"][0] > 2.0 * row["rot_hz"],
          "band=(%.1f,%.1f) 2x=%.1f" % (row["blade_band"][0], row["blade_band"][1],
                                        2.0 * row["rot_hz"]))

    # --- C) 叶频超带宽：必须是 !bw，**不能**因为"看得见"就报 OK ------------
    f100 = 102 * df                          # 1× = 99.6 Hz → 叶频 298.9 Hz > 260
    row = run(f100, {1: 0.20, 3: 0.05})
    row["duty"] = 90.0
    check("叶频超带宽被标出来", row["blade_out_of_bw"] is True)
    check("超出量 == 叶频 − 260 Hz（用的是算出来的 K×1×，不是测出来的峰）",
          abs(row["blade_over_bw_by"] - (3.0 * row["rot_hz"] - 260.0)) < 1e-6,
          "got=%.2f" % (row["blade_over_bw_by"] or -1.0))
    check_eq("判定 = !bw（**不是** OK）", ps.classify(row), ps.VERDICT_BLADE_BW)

    # --- D) 只有 1×、没有叶频 → !blade（说明这一档还不够干净）---------------
    row = run(f80, {1: 0.20})
    row["duty"] = 60.0
    check("只有 1× 时 1× 仍然过判据", row["rot_ok"])
    check_eq("判定 = !blade", ps.classify(row), ps.VERDICT_BLADE_WEAK)

    # --- E) 死掉的轴不能靠 SNR=inf 抢到"最佳轴"（老坑，R6.4.4 踩过）-------
    s = sig(f80, {1: 0.20, 3: 0.05})
    s[:, 0] = 0.0                            # ax 彻底死掉（全零）
    pick = ps.pick_rot(cb.analyse(s, rate, rot_band=ps.DEFAULT_ROT_BAND))
    check_eq("pick_rot 跳过全零轴，选中真正的振动轴", pick["axis"], "ay")
    check("全零输入 → pick_rot 返回 None（不编出一个「假转速」）",
          ps.pick_rot(cb.analyse(np.zeros((4000, 3)), rate,
                                 rot_band=ps.DEFAULT_ROT_BAND)) is None)
    check_eq("rot_hz 为空时判定 = !rot",
             ps.classify({"rot_hz": None, "rot_ok": False}), ps.VERDICT_NO_ROT)

    # --- F) 推荐逻辑：优先"两处都过"，再挑 1× 最接近目标的 ----------------
    def row_of(duty, rot, ok=True, bw_hit=False):
        """造一行"长得像 eval_capture 输出"的记录 —— 少了键会让渲染层炸掉，
        所以字段要跟真实输出对齐（这也正是本次要测的东西）。"""
        return {"duty": duty, "rot_hz": rot, "rot_ok": True, "rot_axis": "ay",
                "rot_snr": 30.0, "rpm": 60.0 * rot,
                "blade_hz": 3.0 * rot, "blade_snr": 18.0, "blade_ratio": 3.0,
                "blade_band": (2.6 * rot, 3.4 * rot),
                "blade_ok": ok, "blade_out_of_bw": bw_hit,
                "blade_expected_hz": 3.0 * rot, "blade_over_bw_by": None}

    best, truly = ps.recommend([row_of(40.0, 55.0, ok=False),
                                row_of(60.0, 78.0),
                                row_of(70.0, 95.0)], 80.0)
    check_eq("推荐 = 可用档里 1× 最接近 80 Hz 的那一档", best["duty"], 60.0)
    check("可用档要给 truly=True", truly is True)

    best2, truly2 = ps.recommend([row_of(40.0, 50.0, ok=False),
                                  row_of(60.0, 82.0, ok=False)], 80.0)
    check_eq("一个可用档都没有 → 退一步选最接近的", best2["duty"], 60.0)
    check("降级建议必须标明「不是真可用」，别把已知局限藏起来", truly2 is False)

    check("连 1× 都没有 → 不给任何档位建议",
          ps.recommend([{"duty": 30.0, "rot_hz": None, "rot_ok": False,
                         "blade_ok": False, "blade_out_of_bw": False}], 80.0)
          == (None, False))

    # --- G) "占空比到底有没有生效"的判据 ----------------------------------
    check("离散度：40/60/80 Hz → ≈66.7%",
          abs(ps.rot_spread_pct([row_of(1, 40.0), row_of(2, 60.0),
                                 row_of(3, 80.0)]) - 66.6667) < 0.01)
    check("离散度 < 3% 会被报成「占空比没生效」（3.3 V 栅压下不可能这么稳）",
          ps.rot_spread_pct([row_of(1, 79.0), row_of(2, 81.0)]) < 3.0)
    check("只有一档时离散度无意义 → None",
          ps.rot_spread_pct([row_of(1, 80.0)]) is None
          or ps.rot_spread_pct([row_of(1, 80.0)]) is None)

    # --- H) 输出层至少要能跑，不抛异常（表格与结论）-----------------------
    # 🔴 顺带守住一个只在运行期才会炸的坑：argparse 会把 help 字符串**再做一次
    #    % 格式化**，所以 help 里的字面量 % 必须写成 %%，否则连 --help 都打不开
    #    （实测就是这个 `ValueError: unsupported format character`，测试跑不到它）。
    try:
        txt = ps.build_argparser().format_help()
        check("--help 能渲染（help 里的字面量 % 必须写成 %%）",
              "--ramp-step" in txt and len(txt) > 400, "len=%d" % len(txt))
    except Exception as exc:                                  # noqa: BLE001
        check("--help 能渲染（help 里的字面量 % 必须写成 %%）", False, repr(exc))

    rows = [row_of(40.0, 55.0, ok=False), row_of(60.0, 78.0)]
    try:
        ps.print_header(3.0, 260.0, 80.0)
        for rr in rows:
            ps.print_row(rr)
        ps.print_legend()
        rc = ps.print_verdict(rows, 80.0, 3.0, 260.0, 34.6)
        check("渲染整张表 + 结论不抛异常", True)
        check_eq("有可用档时退出码 = 0", rc, 0)
    except Exception as exc:                                  # noqa: BLE001
        check("渲染整张表 + 结论不抛异常", False, repr(exc))


def t_pwm_cmd():
    """第 ⑦ 步命令验收工具（check_pwm_cmd.py 的解析层：不碰串口、不碰硬件）。

    这一节守两件容易出错的事：
    ① **子串 vs 独立数字** —— 判 `duty = 4%` 不能被 `duty = 40%` 的子串骗过（纪律 7）；
    ② **两种回显的形态不同** —— `dNN` 只有一行短回显，`?` 才带括号说明，
       解析器必须能分清，否则 `?` 的验收会永远 FAIL。
    """
    print("\n[16] PWM 命令验收（check_pwm_cmd.py 的解析层）")

    banner = ("=== stm32-edge-ai-node boot ===\r\n"
              "SYSCLK = 168000000 Hz, I2C1 = 400 kHz, TIM2 = 1 kHz\r\n"
              "PWM: PA6/TIM3_CH1 @20000 Hz, duty=0% (USART1 cmd: dNN + Enter)\r\n")
    check_eq("横幅 -> (20000, 0)", cpc.parse_banner(banner), (20000, 0))
    check_eq("横幅 duty 非零也能读",
             cpc.parse_banner("PWM: PA6/TIM3_CH1 @20000 Hz, duty=37%"), (20000, 37))
    check_eq("改了 PSC 后频率跟着变",
             cpc.parse_banner("PWM: PA6/TIM3_CH1 @1000 Hz, duty=0%"), (1000, 0))
    check_eq("没有横幅行 -> (None, None)",
             cpc.parse_banner("t=   100 | ax= 1 ay= 2 az= 3 | lost=0 ovf=0"), (None, None))
    check_eq("空文本 -> (None, None)", cpc.parse_banner(""), (None, None))
    check_eq("None -> (None, None)", cpc.parse_banner(None), (None, None))

    # 真实回显：前面还夹着 10 Hz 数据行
    echo = ("t=    100 | ax=  700 ay= -150 az= 16400 | lost=0 ovf=0\r\n"
            "[PWM] duty = 40%\r\n"
            "t=    200 | ax=  710 ay= -160 az= 16390 | lost=0 ovf=0\r\n")
    check_eq("在数据行之间也能抓到回显", cpc.parse_echo(echo, 40), 40)
    check_eq("🔴 duty=4 不能被 duty=40 的子串骗过", cpc.parse_echo(echo, 4), None)
    check_eq("要求 0 而文本是 40 -> None", cpc.parse_echo(echo, 0), None)
    check_eq("空文本 -> None", cpc.parse_echo("", 40), None)
    check_eq("只有数据行 -> None", cpc.parse_echo("t= 100 | ax= 1\r\n", 40), None)

    info = ("t=    300 | ax=  720 ay= -180 az= 16400 | lost=0 ovf=0\r\n"
            "[PWM] duty = 40%  (PA6/TIM3_CH1 @20 kHz; cmd: dNN, NN = 0..100)\r\n")
    duty_info, note = cpc.parse_info(info)
    check_eq("信息行 duty", duty_info, 40)
    check("信息行带上引脚与频率说明",
          isinstance(note, str) and "PA6/TIM3_CH1" in note and "20 kHz" in note, repr(note))
    check_eq("短回显不含括号 -> (None, None)", cpc.parse_info(echo), (None, None))
    check_eq("信息行不认空文本", cpc.parse_info(""), (None, None))

    # ---- LED 目视验收的步骤表（--led-test 用的纯函数） ----
    plan1 = cpc.led_plan(1, 3.0)
    check_eq("LED 一轮 = 3 步", len(plan1), 3)
    check_eq("🔴 LED duty 序列 = 100/40/0（有对照才排除误判）",
             [d for d, _, _ in plan1], [100, 40, 0])
    check_eq("LED 每步说明各不相同", len({n for _, n, _ in plan1}), 3)
    check_eq("LED hold 透传", {h for _, _, h in plan1}, {3.0})
    check_eq("🔴 cycles=0 被夹到 1 轮（不会一步都不发）", len(cpc.led_plan(0, 2.0)), 3)
    check_eq("cycles=2 -> 6 步，序列重复两遍",
             [d for d, _, _ in cpc.led_plan(2, 1.5)], [100, 40, 0, 100, 40, 0])
    check_eq("hold 透传到每一轮", {h for _, _, h in cpc.led_plan(3, 2.5)}, {2.5})

    # --help 必须能渲染（守 argparse 会把 help 再做一次 % 格式化的坑）
    try:
        import io
        import contextlib
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                cpc.main(["--help"])
        except SystemExit:
            pass
        check("--help 能渲染", "PWM" in buf.getvalue(), buf.getvalue()[:60])
        check("--help 含 --led-test", "--led-test" in buf.getvalue(), buf.getvalue()[:60])
        check("--help 含 --cycles", "--cycles" in buf.getvalue(), buf.getvalue()[:60])
        check("--help 含 --hold", "--hold" in buf.getvalue(), buf.getvalue()[:60])
        check("--help 含 --set", "--set" in buf.getvalue(), buf.getvalue()[:60])
    except Exception as exc:                                  # noqa: BLE001
        check("--help 能渲染", False, repr(exc))

    check_eq("--duty 越界被拒（退出码 2）", cpc.main(["--port", "COMx", "--duty", "110"]), 2)
    check_eq("🔴 --set 越界被拒（退出码 2）", cpc.main(["--port", "COMx", "--set", "110"]), 2)
    check_eq("--set 下界外也被拒", cpc.main(["--port", "COMx", "--led-test", "--set", "-3"]), 2)


# ---------------------------------------------------------------------------
def t_calib():
    """第 ⑧ 件工具 calib_dev.py 的**纯函数层**（不碰串口、不碰硬件）。

    只测能"真的失败"的东西：姿态表的轴/符号映射、六姿态解算能否**精确还原**
    注入的 k/b、越界/缺姿态时是否拒绝、判据函数对**构造出来的坏姿态**是否报错。
    采集层（要串口）与真机验证不在自测范围内 —— 那属于 T3 的现场判据。
    """
    print("\n[17] 六姿态静态标定（calib_dev.py 的纯逻辑：轴映射 / 判据 / 解算）")

    # ---- 姿态表：轴序号 + 期望符号 ----
    check_eq('axis_sign("+z")', cd.axis_sign("+z"), (2, 1))
    check_eq('axis_sign("-y")', cd.axis_sign("-y"), (1, -1))
    check_eq('axis_sign("+x")', cd.axis_sign("+x"), (0, 1))
    check_eq("axis_of_pose('-z')", cd.axis_of_pose("-z"), 2)
    check("POSE_HINT 覆盖全部 6 个姿态", set(cd.POSE_HINT) == set(cd.POSE_NAMES),
          sorted(cd.POSE_HINT))
    try:
        cd.axis_sign("+w")
        check("未知姿态抛 ValueError", False, "没抛异常")
    except ValueError:
        check("未知姿态抛 ValueError", True)

    # ---- mean_sigma ----
    const = np.tile([1.0, 2.0, 3.0], (50, 1))
    m, sg = cd.mean_sigma(const)
    check("mean_sigma：常量序列 σ 全 0、均值正确",
          bool(np.allclose(sg, 0.0)) and bool(np.allclose(m, [1.0, 2.0, 3.0])),
          "m=%s sg=%s" % (m, sg))
    m0, s0 = cd.mean_sigma(np.zeros((0, 3)))
    check("mean_sigma：空输入返回全 0（由调用方判 n < min_samples）",
          bool(np.allclose(m0, 0.0)) and bool(np.allclose(s0, 0.0)))
    _, sig = cd.mean_sigma(np.array([[1.0, 0, 0], [0.9, 0, 0], [1.1, 0, 0]]))
    check("mean_sigma：σ 真能算出非零（±0.1 抖动 → 0.0816）",
          abs(float(sig[0]) - 0.08165) < 0.002, "σx=%.5f" % sig[0])

    # ---- diagnose_pose：好姿态必须"零问题"，否则判据形同虚设 ----
    Z = np.zeros(3)

    def has(problems, kw):
        return any(kw in p for p in problems)

    check_eq("理想 +z 无问题", cd.diagnose_pose("+z", np.array([0.0, 0.0, 1.0]), Z), [])
    check_eq("理想 -x 无问题", cd.diagnose_pose("-x", np.array([-1.0, 0.0, 0.0]), Z), [])

    p = cd.diagnose_pose("+z", np.array([0.0, 0.0, -1.0]), Z)
    check("diagnose：**符号反了**被抓（`-z` 当成 `+z`）", has(p, "符号反了"), p)

    p = cd.diagnose_pose("+z", np.array([1.0, 0.0, 0.0]), Z)
    check("diagnose：主分量轴不对被抓（说 +z 其实躺着）", has(p, "主分量落在"), p)

    p = cd.diagnose_pose("+z", np.array([0.2, 0.0, 0.98]), Z)
    check("diagnose：侧轴残留被抓（倾斜 12° → 0.2 g 跑到 X）", has(p, "残留"), p)

    p = cd.diagnose_pose("+z", np.array([0.0, 0.0, 0.9]), Z)
    check("diagnose：主分量幅值偏离 1 g 被抓，且不误报轴错",
          has(p, "偏离 1 g") and not has(p, "主分量落在"), p)

    p = cd.diagnose_pose("+z", np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.05]))
    check("diagnose：轴内 σ 超限（采集时手还在动）被抓", has(p, "板子在动"), p)

    # ---- solve：理想六姿态 → k=1 / b=0 ----
    ideal = {}
    for name, j, s in cd.POSES:
        v = [0.0, 0.0, 0.0]
        v[j] = float(s)
        ideal[name] = np.asarray(v)

    sol = cd.solve(ideal)
    check("solve：理想数据还原 k=1", bool(np.allclose(sol["k"], 1.0, atol=1e-12)), sol["k"])
    check("solve：理想数据还原 b=0", bool(np.allclose(sol["b_g"], 0.0, atol=1e-12)), sol["b_g"])

    # ---- solve：注入已知 k/b，必须**精确还原** ----
    # 物理模型 R = k · a_true + b，**零偏作用于每个轴**（真值为 0 时读数就是 b）。
    K, B = 1.025, 0.012                     # 增益 +2.5%；零偏 +0.012 g ≈ 197 LSB
    raw = {}
    for name, j, s in cd.POSES:
        v = np.full(3, B)
        v[j] = K * s + B
        raw[name] = v

    sol2 = cd.solve(raw)
    check("solve：精确还原注入增益 k=%.3f" % K,
          bool(np.allclose(sol2["k"], K, atol=1e-12)), sol2["k"])
    check("solve：精确还原注入零偏 b=%.3f g" % B,
          bool(np.allclose(sol2["b_g"], B, atol=1e-12)), sol2["b_g"])
    check("solve：b_lsb = b_g × 16384（对得上固件口径）",
          abs(sol2["b_lsb"][0] - B * vf.ACCEL_LSB_PER_G) < 1e-9, sol2["b_lsb"])
    check("solve：raw_plus 记的是该轴朝上时的读数",
          abs(sol2["raw_plus"][0] - (K + B) * vf.ACCEL_LSB_PER_G) < 1e-9, sol2["raw_plus"])

    # ---- apply_calib：(R-b)/k 应把注入数据修回 |a| = 1 ----
    stack_raw = np.vstack([raw[p] for p in cd.POSE_NAMES])
    fixed = cd.apply_calib(stack_raw, sol2)
    check("apply_calib：把注入的 6 姿态幅值修回 1 g",
          bool(np.allclose(np.sqrt((fixed ** 2).sum(axis=1)), 1.0, atol=1e-12)),
          np.sqrt((fixed ** 2).sum(axis=1)))

    # ---- dev_pct：判据函数的算术 ----
    check_eq("dev_pct：|a|=1 → 0%", cd.dev_pct(np.array([[0.0, 0.0, 1.0]])), (0.0, 0.0))
    d = cd.dev_pct(np.array([[0.0, 0.0, 1.02]]))
    check("dev_pct：|a|=1.02 → +2%",
          abs(d[0] - 2.0) < 1e-9 and abs(d[1] - 2.0) < 1e-9, d)
    dm, dx = cd.dev_pct(np.array([[0.0, 0.0, 1.01], [0.0, 0.0, 0.98]]))
    check("dev_pct：多样本取均值(-0.5%)与最大绝对偏差(2%)",
          abs(dm + 0.5) < 1e-9 and abs(dx - 2.0) < 1e-9, (dm, dx))
    raw_mean, raw_max = cd.dev_pct(stack_raw)
    fix_mean, fix_max = cd.dev_pct(fixed)
    check("dev_pct：标定前均值 +2.514% / 最大 +3.714%，标定后归零（自洽，不是验证）",
          abs(raw_mean - 2.51405) < 1e-4 and abs(raw_max - 3.71394) < 1e-4
          and abs(fix_mean) < 1e-12 and abs(fix_max) < 1e-12,
          (raw_mean, raw_max, fix_mean, fix_max))

    # ---- poses_from_json ----
    check("poses_from_json：6 姿态齐",
          sorted(cd.poses_from_json(
              {"poses": {p: {"mean_g": list(raw[p])} for p in cd.POSE_NAMES}}))
          == sorted(cd.POSE_NAMES))
    check_eq("poses_from_json：跳过没有 mean_g 的记录（不抛异常）",
             cd.poses_from_json({"poses": {"+x": {"n": 3}}}), {})

    # ---- identify_pose：由一次读数反推「现在拿的是哪个面」----
    for name, j, s in cd.POSES:
        v = [0.0, 0.0, 0.0]
        v[j] = float(s)
        got, probs, _ = cd.identify_pose(np.asarray(v))
        check_eq("identify_pose：理想 %s → 认出 %r 且无问题" % (name, name),
                 (got, probs), (name, []))

    got, probs, _ = cd.identify_pose(np.array([0.2, 0.0, 0.98]))
    check("identify_pose：斜着拿仍认出 +z，但报出侧轴残留",
          got == "+z" and has(probs, "残留"), (got, probs))

    got, _, info = cd.identify_pose(np.array([0.4, 0.4, 0.45]))
    check("identify_pose：主分量 < 0.7 g → 返回 None（不猜）",
          got is None and abs(info["mag_g"] - np.sqrt(0.4 ** 2 * 2 + 0.45 ** 2)) < 1e-9,
          (got, info))

    got, _, info = cd.identify_pose(np.array([0.0, -1.0, 0.0]))
    check("identify_pose：-y 的轴与符号都要对",
          (got, info["axis"], info["sign"]) == ("-y", 1, -1), (got, info))

    _, _, info = cd.identify_pose(np.array([0.0, 0.0, 1.0]))
    check("identify_pose：raw_lsb 按 16384 LSB/g 换算",
          abs(info["raw_lsb"][2] - vf.ACCEL_LSB_PER_G) < 1e-9, info["raw_lsb"])

    got, probs, _ = cd.identify_pose(np.array([0.0, 0.0, 1.0]),
                                     np.array([0.0, 0.0, 0.05]))
    check("identify_pose：把轴内 σ 传进去 → 报出「板子在动」",
          got == "+z" and has(probs, "板子在动"), probs)

    # ---- usable_poses ----
    dp = {"poses": {"+x": {"problems": []}, "-x": {"problems": ["符号反了"]},
                    "+y": {"problems": []}}}
    check_eq("usable_poses：只收判据全过的，且按 POSE_NAMES 顺序",
             cd.usable_poses(dp), ["+x", "+y"])
    check_eq("usable_poses：空文件 → 空列表", cd.usable_poses({}), [])

    # ---- pick_port：自动选串口（宁可报错也不猜）----
    p, _ = cd.pick_port([])
    check_eq("pick_port：0 个串口 → None", p, None)
    p, _ = cd.pick_port(["COM4  [USB-SERIAL CH340 | 1A86:7523]"])
    check_eq("pick_port：只有 1 个 → 直接用它", p, "COM4")
    p, _ = cd.pick_port(["COM4  [USB-SERIAL CH340 | 1A86:7523]",
                         "COM7  [STMicroelectronics STLink Virtual COM Port]"])
    check_eq("pick_port：多口中唯一命中 CH340 → 选它", p, "COM4")
    p, why = cd.pick_port(["COM4  [USB-SERIAL CH340 | 1A86:7523]",
                           "COM5  [USB-SERIAL CH341 | 1A86:5523]"])
    check("pick_port：多个都像 USB 串口 → 不猜，返回 None 并列出全部",
          p is None and "COM4" in why and "COM5" in why, why)
    check_eq("device_of：从清单行取设备名", cd.device_of("COM12  [CH340]"), "COM12")

    # ---- short_problem / watch_line：实时刷新那一行 ----
    check_eq("short_problem：符号反", cd.short_problem("**符号反了**：az 轴实测 -1.0 g"), "符号反")
    check_eq("short_problem：侧轴残留", cd.short_problem("侧轴 ax 残留 +0.2000 g 超过 0.080 g"), "侧轴残留")
    check_eq("short_problem：在动", cd.short_problem("**采集期间板子在动**：az 轴标准差 0.05 g"), "在动")

    line = cd.watch_line([[0.0, 0.0, 1.0]] * 10, 3.5)
    check("watch_line：干净 +z 那一行带 OK 与 +z",
          "OK" in line and "+z" in line and "t=   3.5s" in line, line)
    line = cd.watch_line([[0.2, 0.0, 0.98]] * 10, 1.0)
    check("watch_line：摆歪时给出 !! 与短标签（不是长句子）",
          "!!" in line and "侧轴残留" in line, line)
    line = cd.watch_line([[0.4, 0.41, 0.45]] * 10, 2.0)
    check("watch_line：认不出时给 ?? 并说明原因",
          "??" in line and "认不出" in line, line)
    line = cd.watch_line([[0.0, 0.0, 1.0]] * 10, 1.0)
    check("watch_line：合矢量按 16384 口径显示",
          "|a|=1.0000" in line and "az=  +16384" in line, line)

    # ---- solve 的两条拒绝路径 ----
    try:
        cd.solve({k: v for k, v in ideal.items() if k != "+z"})
        check("solve：缺姿态抛 ValueError", False, "没抛异常")
    except ValueError as e:
        check("solve：缺姿态抛 ValueError", "缺少姿态" in str(e), str(e))

    bad = dict(ideal)
    bad["+x"] = np.array([0.2, 0.0, 0.0])
    bad["-x"] = np.array([0.9, 0.0, 0.0])   # 两次都朝"正" → k 变负
    try:
        cd.solve(bad)
        check("solve：增益越界抛 ValueError（姿态摆反的信号）", False, "没抛异常")
    except ValueError as e:
        check("solve：增益越界抛 ValueError（姿态摆反的信号）", "增益" in str(e), str(e))

    # ---- render_c_header ----
    hdr = cd.render_c_header(sol2, {"solved_at": "TEST"})
    check("render_c_header：含头保护", "#ifndef MPU6050_CALIB_H" in hdr)
    check("render_c_header：K/B 六个宏齐全",
          all(("MPU6050_CALIB_%s_%s" % (a, b)) in hdr
              for a in ("K", "B") for b in ("X", "Y", "Z")))

    # ---- run_solve：离线端到端（含导出）+ 拒算 + JSON 往返 ----
    tmp = tempfile.mkdtemp(prefix="calib_test_")
    try:
        doc = {"tool": "calib_dev.py", "poses": {}}
        for p in cd.POSE_NAMES:
            doc["poses"][p] = {"mean_g": [float(v) for v in raw[p]],
                               "sigma_g": [0.0, 0.0, 0.0], "n": 30,
                               "text_mode": "raw", "problems": []}

        cpath = os.path.join(tmp, "mpu6050_calib.h")
        check_eq("run_solve：6 姿态齐 → 退出码 0", cd.run_solve(doc, emit_c=cpath), 0)
        check("run_solve：写入了 doc['solved']", "solved" in doc)
        ctext = open(cpath, encoding="utf-8").read() if os.path.exists(cpath) else ""
        check("run_solve：导出的 C 头文件落地且含宏",
              "MPU6050_CALIB_K_X" in ctext and "MPU6050_CALIB_B_Z" in ctext)

        doc_bad = {"poses": {p: dict(doc["poses"][p]) for p in cd.POSE_NAMES}}
        doc_bad["poses"]["+z"]["problems"] = ["符号反了"]
        check_eq("run_solve：有姿态未通过 → 退出码 2（拒算）",
                 cd.run_solve(doc_bad, emit_c=None), 2)

        jpath = os.path.join(tmp, "calib.json")
        cd.save_doc(jpath, doc)
        back = cd.load_doc(jpath)
        check("save_doc/load_doc：往返后 solved.k 不变",
              bool(np.allclose(back["solved"]["k"], K, atol=1e-12)), back["solved"]["k"])
        check("save_doc：不把内部 `_path` 写进 JSON",
              "_path" not in open(jpath, encoding="utf-8").read())
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # ---- run_verify 的早退（没 solved 就不该碰串口）----
    import argparse as _ap
    check_eq("run_verify：没有 solved 字段 → 直接退 2（不碰串口）",
             cd.run_verify(_ap.Namespace(json="nope.json"), {}), 2)

    # ---- CLI 冒烟 ----
    check_eq("CLI：--emit-c 缺 --solve → 退出码 2", cd.main(["--emit-c", "x.h"]), 2)
    helps = cd.build_parser().format_help()
    check("--help 含 --identify", "--identify" in helps)
    check("--help 含 --watch", "--watch" in helps)
    check("--help 含 --interval", "--interval" in helps)
    # 串口打不开时必须"快速退 2"而不是卡住或抛栈
    check_eq("CLI：--identify 打不开串口 → 退 2",
             cd.main(["--port", "COMx", "--identify", "--yes", "--countdown", "0",
                      "--quiet"]), 2)
    check_eq("CLI：--watch 打不开串口 → 退 2",
             cd.main(["--port", "COMx", "--watch", "--quiet"]), 2)
    smoke = os.path.join(tempfile.gettempdir(), "calib_smoke_%d.json" % os.getpid())
    if os.path.exists(smoke):
        os.remove(smoke)
    check_eq("CLI：--solve 面对不存在的 json → 退出码 2",
             cd.main(["--solve", "--json", smoke]), 2)


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
    t_text_rate_agreement()
    t_binary_zero_frame_hint()
    t_big_drop_positions()
    t_fake_physical_sanity()
    t_cli_smoke()
    t_rate_check()
    t_blade_peak()
    t_pwm_sweep()
    t_pwm_cmd()
    t_calib()

    print("\n" + "=" * 68)
    print("结果：%d 通过 / %d 失败" % (_pass, _fail))
    print("=" * 68)
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
