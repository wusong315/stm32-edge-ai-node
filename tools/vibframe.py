#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vibframe.py — 振动数据二进制帧的协议层（编码 / 流式解析 / 窗口切片）

🔴 **本文件的常量必须与固件逐字节对齐**，唯一权威是工程里的
   `Core/Src/main.c`（`USER CODE BEGIN PV` 里的 FRAME_* 宏）。
   改固件帧格式时，这里要同步改 —— 三方（固件 / 本模块 / 文档）不一致是最
   难查的一类 bug：数据能收到，但每个数都是错的。

帧格式（小端，共 198 字节）
--------------------------------------------------------------------------
  偏移      长度      内容
  0         2         同步头 0xAA 0x55
  2         2         帧序号 uint16（掉帧一眼看出）
  4         1         点数   uint8（= SAMPLES_PER_FRAME，默认 32）
  5         n*6       数据：每点 ax, ay, az 各 int16
  5+n*6     1         校验和 = 前面所有字节之和 & 0xFF

设计要点
--------------------------------------------------------------------------
* 解析器是**流式**的：串口一次给多少字节都行，帧可以被切成任意片段。
* 解析器是**自恢复**的：前面混进开机横幅文字、中间丢字节、校验错，都能重新
  找到同步头。只要有噪声，就一定要有自恢复，否则一次丢字节就永久错位。
* 校验和 + 点数合法性双重把关，能挡住 payload 里偶然出现的 `AA 55`。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# 协议常量 —— 与固件 `Core/Src/main.c` 一一对应
# ---------------------------------------------------------------------------
FRAME_HDR0 = 0xAA
FRAME_HDR1 = 0x55
FRAME_OFF_SEQ = 2
FRAME_OFF_N = 4
FRAME_OFF_DATA = 5
FRAME_OVERHEAD = FRAME_OFF_DATA + 1          # 头 5 字节 + 校验 1 字节
DEFAULT_SAMPLES_PER_FRAME = 32              # 与固件 SAMPLES_PER_FRAME 一致
DEFAULT_FRAME_TOTAL = FRAME_OFF_DATA + DEFAULT_SAMPLES_PER_FRAME * 6 + 1   # 198
AXES = ("ax", "ay", "az")

# 加速度计灵敏度（±2g 档）：16384 LSB/g —— 与固件 MPU6050_ACCEL_FS_2G_PA 一致
ACCEL_LSB_PER_G = 16384.0

# 一个 uint8 的点数最多 255 → 最长帧 1536 字节。超过它一定是误同步。
MAX_SAMPLES_PER_FRAME = 255


def frame_total(samples: int) -> int:
    """给定点数，算出整帧字节数。"""
    return FRAME_OFF_DATA + samples * 6 + 1


def checksum(data: bytes) -> int:
    """与固件同一套算法：前面所有字节求和后取低 8 位。

    ⚠️ 固件是 `sum = (uint8_t)(sum + byte)`，等价于不加权求和后 & 0xFF。
    """
    return sum(data) & 0xFF


def encode_frame(seq: int, ax, ay, az, *, samples: int | None = None) -> bytes:
    """把三轴数组打包成一帧（假数据发生器与自测都用它）。"""
    ax = np.asarray(ax, dtype="<i2")
    ay = np.asarray(ay, dtype="<i2")
    az = np.asarray(az, dtype="<i2")
    if not (len(ax) == len(ay) == len(az)):
        raise ValueError("三轴长度必须一致")
    n = len(ax)
    if samples is not None and n != samples:
        raise ValueError("点数与 samples 参数不一致")
    if not 1 <= n <= MAX_SAMPLES_PER_FRAME:
        raise ValueError("点数必须在 1..255")

    body = bytearray()
    body.append(FRAME_HDR0)
    body.append(FRAME_HDR1)
    body += struct.pack("<H", seq & 0xFFFF)
    body.append(n)

    # 交错存放：ax0 ay0 az0 ax1 ay1 az1 ...（与固件 App_EmitFrame 的写出顺序一致）
    interleaved = np.empty(n * 3, dtype="<i2")
    interleaved[0::3] = ax
    interleaved[1::3] = ay
    interleaved[2::3] = az
    body += interleaved.tobytes()

    body.append(checksum(bytes(body)))
    return bytes(body)


# ---------------------------------------------------------------------------
# 解析结果
# ---------------------------------------------------------------------------
@dataclass
class Frame:
    seq: int
    samples: int          # 本帧点数
    ax: np.ndarray        # int16，单位 LSB
    ay: np.ndarray
    az: np.ndarray
    bad: bool = False     # 为 True 时表示这是一帧"校验失败的占位"，只用于统计

    def accel_g(self) -> np.ndarray:
        """转成物理量 g，返回 (n, 3) 数组。"""
        return np.stack([self.ax, self.ay, self.az], axis=1) / ACCEL_LSB_PER_G


@dataclass
class ParserStats:
    """健康度计数 —— 上板后**先看这几个数**，比看波形更能说明问题。"""
    frames_ok: int = 0          # 成功解出的帧数
    samples_ok: int = 0         # 成功解出的采样点数
    checksum_errors: int = 0    # 校验和错误的帧数
    resync_bytes: int = 0       # 为了找同步头而丢弃的字节数
    seq_gaps: int = 0           # 序号跳变的次数
    frames_lost: int = 0        # 由序号跳变推算出的丢帧总数
    truncated: int = 0          # 流结束时残留在缓冲里的字节数

    def as_text(self) -> str:
        return ("帧 %d / 采样点 %d | 校验错 %d | 重同步丢弃 %d 字节 | "
                "序号断档 %d 次(丢帧约 %d) | 尾部残留 %d 字节"
                % (self.frames_ok, self.samples_ok, self.checksum_errors,
                   self.resync_bytes, self.seq_gaps, self.frames_lost,
                   self.truncated))


# ---------------------------------------------------------------------------
# 流式解析器
# ---------------------------------------------------------------------------
class FrameParser:
    """把任意切分的字节流还原成帧序列。

    用法::

        p = FrameParser()
        for chunk in source:          # 串口/文件/假数据，给多少都行
            for frame in p.feed(chunk):
                ...
        p.finish()                    # 收尾，统计残留
        print(p.stats.as_text())
    """

    def __init__(self, expect_samples: int | None = None):
        # expect_samples 给定时，会额外校验点数是否等于它（默认不强制，
        # 这样固件改了 SAMPLES_PER_FRAME 之后解析器仍然能用）。
        self.expect_samples = expect_samples
        self.buf = bytearray()
        self.stats = ParserStats()
        self._last_seq: int | None = None
        self._primed = False      # 是否已经成功解出过至少一帧

    # -- 内部：尝试从缓冲头部解出一帧 ------------------------------------
    def _try_one(self) -> Frame | None:
        """解出缓冲头部的一帧。

        🔴 **这里必须是循环，不能"拒绝一次就 return"**：
        遇到坏帧时如果直接返回，同一批数据里**后面的好帧会被一起丢掉**，
        要等下次 feed 才继续 —— 表现为"偶尔丢一大段"，极难查。
        每次拒绝只丢 1 个字节后 continue 继续扫描（而不是丢掉整个缓冲），
        这样即使真帧紧跟在一个坏字节后面也能救回来。
        循环必然终止：每轮要么 return，要么 buf 至少少 1 字节。
        """
        buf = self.buf

        while True:
            # 1) 找同步头。找不到就只保留最后 1 字节 —— 因为 0xAA 0x55 可能正好
            #    被切在这批数据的末尾，丢掉它就永远同步不上了。
            if not self._primed or len(buf) < 2 or buf[0] != FRAME_HDR0 or buf[1] != FRAME_HDR1:
                idx = buf.find(bytes((FRAME_HDR0, FRAME_HDR1)))
                if idx < 0:
                    keep = 1 if (len(buf) and buf[-1] == FRAME_HDR0) else 0
                    drop = len(buf) - keep
                    if drop > 0:
                        self.stats.resync_bytes += drop
                        del buf[:drop]
                    return None
                if idx > 0:
                    self.stats.resync_bytes += idx
                    del buf[:idx]

            # 2) 头 5 字节齐了才能知道点数
            if len(buf) < FRAME_OFF_DATA:
                return None
            n = buf[FRAME_OFF_N]
            if n == 0 or n > MAX_SAMPLES_PER_FRAME:
                # 非法点数 → 这一定是误同步，丢掉 1 字节继续找
                self.stats.resync_bytes += 1
                del buf[:1]
                self._primed = False
                continue
            total = frame_total(n)
            if len(buf) < total:
                return None                      # 还没收全，等下一批数据

            raw = bytes(buf[:total])

            # 3) 校验和
            if checksum(raw[:-1]) != raw[-1]:
                self.stats.checksum_errors += 1
                self.stats.resync_bytes += 1
                del buf[:1]                      # 只丢 1 字节，避免把真帧也丢掉
                self._primed = False
                continue

            # 4) 点数校验（可选）
            if self.expect_samples is not None and n != self.expect_samples:
                self.stats.checksum_errors += 1
                self.stats.resync_bytes += 1
                del buf[:1]
                self._primed = False
                continue

            # 5) 序号连续性（只在已经同步上之后统计）
            seq = struct.unpack_from("<H", raw, FRAME_OFF_SEQ)[0]
            if self._last_seq is not None:
                expected = (self._last_seq + 1) & 0xFFFF
                if seq != expected:
                    self.stats.seq_gaps += 1
                    gap = (seq - expected) & 0xFFFF
                    if 0 < gap < 0x8000:         # 正常的丢帧（不是回绕/重启）
                        self.stats.frames_lost += gap
            self._last_seq = seq

            data = np.frombuffer(raw, dtype="<i2", count=n * 3, offset=FRAME_OFF_DATA)
            frame = Frame(seq=seq, samples=n,
                          ax=data[0::3].copy(), ay=data[1::3].copy(), az=data[2::3].copy())

            del buf[:total]
            self.stats.frames_ok += 1
            self.stats.samples_ok += n
            self._primed = True
            return frame

    # -- 对外接口 --------------------------------------------------------
    def feed(self, chunk: bytes) -> list[Frame]:
        """喂入一段字节，返回这次能解出的所有帧。"""
        if chunk:
            self.buf += chunk
        out = []
        while True:
            f = self._try_one()
            if f is None:
                break
            out.append(f)
        return out

    def finish(self) -> None:
        """数据流结束。残留字节要么是被截断的最后一帧，要么是尾部噪声。"""
        self.stats.truncated = len(self.buf)
        self.buf.clear()


# ---------------------------------------------------------------------------
# 帧序列 → 连续采样序列 / 窗口切片
# ---------------------------------------------------------------------------
def stack_frames(frames) -> np.ndarray:
    """把一批帧拼成连续序列，返回 (N, 3) 的 float32 g 值数组。

    ⚠️ 这里**不做插值、不做去重**：序号断档就是断档，插值会把不存在的振动
    编出来，训练出来的模型在真机上会莫名误报。断档由 stats 报出来。
    """
    if not frames:
        return np.zeros((0, 3), dtype=np.float32)
    return np.concatenate([f.accel_g() for f in frames], axis=0)


def iter_windows(samples: np.ndarray, window: int, hop: int | None = None):
    """把连续序列切成定长窗口（默认不重叠，即 hop == window）。

    NanoEdge AI Studio 的分析窗口用的就是这个概念（计划书里写 1024 点/帧，
    1024 @1kHz = 1.024 s）。**注意别和固件的"传输帧 32 点"搞混**：那是串口
    分包单位，与 AI 分析窗口是两个层次的东西。
    """
    if hop is None:
        hop = window
    if window <= 0 or hop <= 0:
        raise ValueError("window / hop 必须为正")
    for start in range(0, len(samples) - window + 1, hop):
        yield samples[start:start + window]
