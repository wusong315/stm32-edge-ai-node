#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vibsource.py — 数据来源：串口 / 录制文件 / 假数据，以及文本模式解析

为什么要单独一层：**解析逻辑必须能在没有硬件时被完整测试**。
所以"数据从哪来"和"怎么解析"彻底分开 —— 假数据源和串口在接口上是同一种东西
（都吐 bytes），于是自测可以覆盖真实路径的全部代码。

三种来源都实现成一个生成器 `Iterator[bytes]`，随便给多少字节都行。
"""

from __future__ import annotations

import math
import re
import time
from typing import Iterator

import numpy as np

import vibframe as vf

# ---------------------------------------------------------------------------
# 1) 串口
# ---------------------------------------------------------------------------
def list_serial_ports() -> list[str]:
    """列出可用串口。没装 pyserial 时返回空列表（不抛异常）。

    🔴 要带上 description 与 VID:PID —— 插了不止一个适配器时（例如 ST-Link 的
       虚拟串口 + CH340），只报 "COM5" 根本分不出是哪一个；而**选错端口**的
       现象（收不到数据）和**接线故障**一模一样，会把排查引向错误方向。
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    out = []
    for p in list_ports.comports():
        extra = []
        if p.description:
            extra.append(p.description)
        if p.vid is not None and p.pid is not None:
            extra.append("%04X:%04X" % (p.vid, p.pid))
        out.append("%s%s" % (p.device, "  [" + " | ".join(extra) + "]" if extra else ""))
    return out


def serial_source(port: str, baud: int = 115200, *, chunk: int = 4096,
                  duration: float | None = None, idle_timeout: float = 2.0,
                  verbose: bool = True) -> Iterator[bytes]:
    """从串口读字节流。

    duration 为 None 时一直读到 Ctrl-C 或串口静默超过 idle_timeout 秒。

    🔴 **读多少由「在缓冲里有多少」决定，绝不用 `chunk` 死等**（2026-10-13 修正）。
    老实现写成 `ser.read(chunk)`，但 pyserial 在 Windows 上会**一直等到攒够 `chunk`
    字节**才返回 ⇒ 实测 115200 下 mode 0 只有 **1365 B/s**，攒满 `chunk=4096` 要
    **3 秒**！后果是**三个时间旋钮全部失真**：
      · `--seconds 4` 实际是"读满一个 4096 块"，与 4 秒无关；
      · 实时视图 `--interval 1.0` 实际每 **4 秒**才刷一行；
      · `idle_timeout` 最短也只能按 3 秒级粒度生效。
    这也解释了为什么 `--identify --seconds 4` 恰好拿到 `4096 字节 / 40 样本` ——
    那不是"采了 4 秒"，是"读满了一块"，数字对上纯属巧合。
    现在改成按 `in_waiting` 取：有数据就立刻拿走，没数据用 `read(1)` 让
    `timeout=0.2` 兜底（于是循环最长 0.2 s 一轮，三个旋钮都恢复按时钟工作）。
    """
    import serial  # 延迟导入：只解析文件时不需要串口库

    ser = serial.Serial(port, baud, timeout=0.2)
    if verbose:
        print("[i] 已打开 %s @ %d 8N1" % (port, baud))
    t0 = time.time()
    last_data = time.time()
    try:
        while True:
            want = ser.in_waiting
            # 有货就按实际数量取（不超过 chunk）；没货就 blocking 读 1 字节，
            # 靠 timeout 兜底 —— 这样循环节奏跟得上时钟，而不是跟得上缓冲区大小。
            data = ser.read(min(chunk, want) if want else 1)
            now = time.time()
            if data:
                last_data = now
                yield data
            elif duration is None and (now - last_data) > idle_timeout:
                if verbose:
                    print("[i] 串口静默超过 %.0f 秒，停止接收" % idle_timeout)
                break
            if duration is not None and (now - t0) >= duration:
                break
    finally:
        ser.close()
        if verbose:
            print("[i] 串口已关闭，共接收 %.1f 秒" % (time.time() - t0))


# ---------------------------------------------------------------------------
# 2) 录制文件（把串口原始字节存成 .bin，便于离线反复分析）
# ---------------------------------------------------------------------------
def file_source(path: str, *, chunk: int = 8192) -> Iterator[bytes]:
    with open(path, "rb") as fh:
        while True:
            data = fh.read(chunk)
            if not data:
                break
            yield data


def save_raw(path: str, chunks: Iterator[bytes]) -> int:
    """把原始字节流落盘，返回总字节数。**建议上板第一件事就是录一份原始数据**。"""
    total = 0
    with open(path, "wb") as fh:
        for c in chunks:
            fh.write(c)
            total += len(c)
    return total


# ---------------------------------------------------------------------------
# 3) 假数据源 —— 无硬件也能把整条链路（解析→画图→切窗口→写 CSV）跑通
# ---------------------------------------------------------------------------
def synth_vibration(seconds: float = 2.0, rate: float = 1000.0, *,
                    blade_hz: float = 180.0, noise_g: float = 0.004,
                    anomaly_start: float | None = None,
                    anomaly_seconds: float = 0.5,
                    anomaly_gain: float = 4.0, seed: int = 1):
    """造一段"像那么回事"的电机振动信号，返回 (t, ax, ay, az)，单位 g。

    物理依据（够用就好，不追求精确）：
      * 三叶风扇的**叶片通过频率** = 叶频，是振动主频（项目口径 3 扇叶、叶频
        300–540 Hz，见《整体方案》§4.1）。这里默认 180 Hz 只是为了画图好看，
        真实叶频上板后再填。
      * 静置时 az ≈ +1 g（重力），ax / ay ≈ 0 —— 与固件 mode 0 的验收口径一致。
      * "异常"（轻触机壳 / 加不平衡质量）的特征：**基频幅值跳变 + 出现新的
        谐波**。所以这里把幅值放大若干倍并塞进一个 3.5 次谐波。
    """
    n = int(round(seconds * rate))
    t = np.arange(n) / rate
    rng = np.random.default_rng(seed)

    def axis_phase(phase):
        sig = np.sin(2 * math.pi * blade_hz * t + phase)
        sig += 0.35 * np.sin(2 * math.pi * 2 * blade_hz * t + 1.1 * phase)
        sig += 0.15 * np.sin(2 * math.pi * 3 * blade_hz * t + 0.4 * phase)
        return sig

    vx, vy, vz = axis_phase(0.0), axis_phase(2.1), axis_phase(4.2)
    # 基础振动幅度：0.05 g 量级（真实值待实测标定）
    amp = 0.05

    if anomaly_start is not None:
        a0 = int(round(anomaly_start * rate))
        a1 = min(n, a0 + int(round(anomaly_seconds * rate)))
        if a0 < a1:
            extra = anomaly_gain * amp * np.sin(2 * math.pi * 3.5 * blade_hz * t[a0:a1])
            vx[a0:a1] *= anomaly_gain
            vy[a0:a1] *= anomaly_gain
            vz[a0:a1] *= anomaly_gain
            vz[a0:a1] += extra

    ax = rng.normal(0.0, noise_g, n) + amp * vx
    ay = rng.normal(0.0, noise_g, n) + amp * vy
    az = 1.0 + rng.normal(0.0, noise_g, n) + amp * vz   # 重力 1 g 在 az 上
    return t, ax, ay, az


def to_lsb(g: np.ndarray) -> np.ndarray:
    """g → 加速度计原始 LSB（±2g 档，16384 LSB/g），并做 int16 饱和。"""
    lsb = np.rint(np.asarray(g) * vf.ACCEL_LSB_PER_G)
    return np.clip(lsb, -32768, 32767).astype("<i2")


def make_fake_frames(seconds: float = 2.0, rate: float = 1000.0, *,
                     samples_per_frame: int = vf.DEFAULT_SAMPLES_PER_FRAME,
                     seq0: int = 0, banner: str = "", **kw) -> bytes:
    """造一整段**二进制帧字节流**（等价于串口上真正会出现的字节）。

    banner 用于模拟开机横幅：真实的串口流前面一定混着人可读文字，
    解析器必须先把它当噪声丢掉 —— 这正是要测的场景。
    """
    _t, ax, ay, az = synth_vibration(seconds, rate, **kw)
    axl, ayl, azl = to_lsb(ax), to_lsb(ay), to_lsb(az)
    out = bytearray(banner.encode("utf-8"))

    seq = seq0
    for start in range(0, len(axl) - samples_per_frame + 1, samples_per_frame):
        sl = slice(start, start + samples_per_frame)
        out += vf.encode_frame(seq, axl[sl], ayl[sl], azl[sl])
        seq += 1
    return bytes(out)


def fake_source(chunks: int = 8, **kw) -> Iterator[bytes]:
    """把假数据流按 chunks 段切成小块，模拟串口"分多次到达"。"""
    blob = make_fake_frames(**kw)
    step = max(1, len(blob) // max(1, chunks))
    for i in range(0, len(blob), step):
        yield blob[i:i + step]


# ---------------------------------------------------------------------------
# 4) 文本模式解析（mode 0 / mode 1）—— 硬件刚到手时最先用到的那两档
# ---------------------------------------------------------------------------
_RE_CSV3 = re.compile(r"^\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*$")
_RE_RAW = re.compile(r"ax=\s*(-?\d+)\s+ay=\s*(-?\d+)\s+az=\s*(-?\d+)")


class TextParser:
    """解析固件 mode 0（`t=.. | ax=.. ay=.. az=.. | ..`）与 mode 1（`x,y,z`）。

    两档的单位不同，务必分清：
      * mode 1 输出的是 **g 值**（`%.3f`），200 Hz；
      * mode 0 输出的是 **原始整数 LSB**，10 Hz。
    本类统一折算成 g，并用 `self.mode` 记录实际识别到的是哪一档。
    """

    def __init__(self):
        self._tail = ""
        self.mode: str | None = None       # 'g' 或 'raw'
        self.lines = 0
        self.bad_lines = 0
        self.comment_lines = 0
        self.rows: list[tuple[float, float, float]] = []
        # 🔴 收到的**原始字节数**，由采集层填入。报告里必须显示它：
        #    "0 行" 既可能是"解析口径不对"，也可能是"一个字节都没收到"——
        #    这两者的排查方向完全相反，不能靠猜。
        self.bytes_in = 0

    def _parse_line(self, line: str):
        s = line.strip()
        if not s:
            return None
        if s.startswith("#") or s.startswith("===") or s.startswith("[OK]") \
                or s.startswith("[WAIT]") or s.startswith("[ERR]") \
                or s.startswith("SYSCLK") or s.startswith("mode "):
            self.comment_lines += 1
            return None

        m = _RE_CSV3.match(s)
        if m:
            self.mode = self.mode or "g"
            return tuple(float(g) for g in m.groups())

        m = _RE_RAW.search(s)
        if m:
            self.mode = self.mode or "raw"
            raw = [int(x) for x in m.groups()]
            return tuple(v / vf.ACCEL_LSB_PER_G for v in raw)

        self.bad_lines += 1
        return None

    def feed(self, text: str) -> list[tuple[float, float, float]]:
        """喂入一段文本（可被任意切开），返回这次新解出的行。"""
        buf = self._tail + text
        parts = buf.split("\n")
        self._tail = parts.pop()            # 最后一段可能不完整，留到下次
        new = []
        for line in parts:
            row = self._parse_line(line)
            if row is not None:
                self.lines += 1
                self.rows.append(row)
                new.append(row)
        return new

    def finish(self) -> None:
        if self._tail.strip():
            row = self._parse_line(self._tail)
            if row is not None:
                self.lines += 1
                self.rows.append(row)
        self._tail = ""

    def samples(self) -> np.ndarray:
        if not self.rows:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(self.rows, dtype=np.float32)

    def as_text(self) -> str:
        unit = {"g": "g 值（mode 1）", "raw": "原始整数（mode 0）", None: "未知"}[self.mode]
        return ("文本行 %d（%s）| 注释/横幅 %d 行 | 无法解析 %d 行"
                % (self.lines, unit, self.comment_lines, self.bad_lines))


def make_fake_text(seconds: float = 2.0, rate: float = 200.0, mode: str = "g",
                   banner: str = "", **kw) -> str:
    """造一段文本模式的假输出，用于自测 TextParser。"""
    _t, ax, ay, az = synth_vibration(seconds, rate, **kw)
    out = [banner] if banner else []
    for i in range(len(ax)):
        if mode == "g":
            out.append("%.3f,%.3f,%.3f\r" % (ax[i], ay[i], az[i]))
        else:
            out.append("t=%8d | ax=%6d ay=%6d az=%6d\r"
                       % (i, to_lsb(ax)[i], to_lsb(ay)[i], to_lsb(az)[i]))
        if i % 200 == 199:
            out.append("# lost=0 ovf=0\r")
    return "\n".join(out) + "\n"
