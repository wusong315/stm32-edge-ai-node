#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calib_dev.py — MPU6050 六姿态静态标定（把零偏 b 与增益 k 分开）

**为什么要做这件事**：单姿态只能量到「实测值 − 真值」这一个差，而它同时含着
**零偏**（b，常数偏移）与**增益**（k，标度误差）两个未知量 —— 一个方程两个未知数，
解不出来。把板子翻转 180° 再采一次，同一个轴就得到两个方程：

    R₊ = k · (+16384) + b        （该轴朝上）
    R₋ = k · (−16384) + b        （该轴朝下）

⇒  ``b = (R₊ + R₋) / 2``　``k = (R₊ − R₋) / (2 × 16384)``

修正：``a_true = (R_meas − b) / k``。三个轴各来一遍，共 6 个姿态。

跑法::

    # 🔴 先"自己看"：实时刷新三轴 + 推断姿态（翻板子看它怎么变）
    python calib_dev.py --watch

    # 🔴 再问它「我现在拿的是哪个面」—— 不用自己对着丝印猜
    python calib_dev.py --identify

    # 单姿态（一次摆一个，可分批做）
    python calib_dev.py --port COM5 --pose +z

    # 引导式：依次提示 6 个姿态，每个按回车开采
    python calib_dev.py --port COM5 --guided

    # 解算（不碰硬件）
    python calib_dev.py --solve --json calib.json

    # 🔴 独立验证：随便一个姿态，比较标定前后 |a| 的偏差（T3 的判据）
    python calib_dev.py --port COM5 --verify --json calib.json

    # 导出给固件用的常数（C 头文件片段）
    python calib_dev.py --solve --json calib.json --emit-c mpu6050_calib.h

退出码：``0`` 判据通过 / ``1`` 采集到了但判据不过 / ``2`` 环境或数据问题。

⚠️ **固件必须处于 `APP_OUTPUT_MODE=0`（推荐）或 1** —— 那两档才输出文本。mode 0 是
**原始整数（10 Hz）**，精度无损；mode 1 是 ``%.3f`` 的 g 值（200 Hz），**打印分辨率
只有 0.001 g ≈ 16 LSB**，会限制 k 的可测精度。本工具两档都认（靠 `vibsource.TextParser`
自动识别），但**推荐 mode 0**。

🔴 **三个坑（都是这套流程特有的，别绕）**

1. **采集期间板子必须真的静止。** 手扶着不算 —— 手的抖动是 0.1 g 量级，而我们要分的
   零偏通常在 0.005 g 量级，**会把结果彻底淹掉**。本工具对每个姿态算**轴内标准差**，
   超 `--static-tol`（默认 0.02 g）直接判该姿态无效。
   实测教训：第 ③ 步就吃过一次 —— 有一段数据 `|a|` 偏 +6.26%，事后按秒切窗才看出
   **那段是翻转过程中采的**，不是传感器的问题。
2. **姿态要摆正，且本工具会替你验。** 说好的 `+z` 如果实际是 `−z` 或躺着，解出来的
   k/b 是**反向或错的**，但数字看起来一样"合理"。所以每个姿态都会检查
   **主分量落在哪个轴、符号对不对、另两轴残留有多大**，不对就拒收。
   （判据通过 ≠ 结论可用 —— 这条是本项目的头号原则。）
3. **别拿标定数据自证。** 用同一批姿态数据算「标定后 |a| 更接近 1 g」是**自洽**，
   不是验证 —— 拟合本来就会让这 6 个点变好。真正的验证要用**标定之后新采的、任意
   姿态**的数据：重力大小与朝向无关，所以**任意姿态下 |a| 都应该等于 1 g**。
   这正是 `--verify` 干的事，也是 T3 判据（±1.5% → ±0.5%）的唯一合法来源。

> 📌 **固件目前尚未消费 k/b**（`grep CALIB` 在 `Core/` 下无结果）。本工具先产出
> `calib.json` 与 C 头文件片段；等固件加上接口（OTA 参数区字段表已为「6 姿态标定
> k/b」预留 24 B）再接上。在那之前，标定数据用于**离线修正数据集**同样有效。
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import sys
import time

import numpy as np

import vibframe as vf
import vibsource as vs

# ---------------------------------------------------------------------------
# 姿态表：名字 → (轴序号, 期望符号)。顺序 = 引导流程的顺序。
#   轴序号 0/1/2 与 vibframe.AXES = ("ax","ay","az") 一致。
# ---------------------------------------------------------------------------
AXES = vf.AXES
POSES: list[tuple[str, int, int]] = [
    ("+x", 0, +1), ("-x", 0, -1),
    ("+y", 1, +1), ("-y", 1, -1),
    ("+z", 2, +1), ("-z", 2, -1),
]
POSE_NAMES = [p for p, _, _ in POSES]
_POSE_LOOKUP = {p: (i, s) for p, i, s in POSES}

# 每个姿态的摆放提示（写清楚"哪个面朝上"，避免用户翻译"±X"）
POSE_HINT = {
    "+x": "把板上 **X 轴正向** 朝上（丝印/丝印箭头那面朝上），另两轴水平",
    "-x": "把 +X 姿态**翻转 180°**（X 轴负向朝上）",
    "+y": "把板上 **Y 轴正向** 朝上",
    "-y": "把 +Y 姿态**翻转 180°**（Y 轴负向朝上）",
    "+z": "把 MPU6050 **芯片面朝上、平放**（Z 轴正向朝上 = 最常见的那一面）",
    "-z": "把芯片面**朝下**平放（Z 轴负向朝上）",
}

# 判据默认值。理由都写在对应函数里，不在这里堆注释。
DEFAULT_STATIC_TOL = 0.02     # g，单姿态轴内标准差上限
DEFAULT_OFF_TOL = 0.08        # g，非主分量轴的残留上限
DEFAULT_MAG_TOL = 0.06        # g，主分量 |均值| 与 1 g 的容差
DEFAULT_SECONDS = 6.0
DEFAULT_MIN_SAMPLES = 20
K_LO, K_HI = 0.5, 1.5         # 增益合法区间；超出说明姿态摆错或器件坏了


# ---------------------------------------------------------------------------
# 1) 纯函数层（不碰硬件、不碰串口 —— 自测就测这一层）
# ---------------------------------------------------------------------------
def axis_sign(pose: str) -> tuple[int, int]:
    """姿态名 → (轴序号, 期望符号)。

    >>> axis_sign("+z")
    (2, 1)
    >>> axis_sign("-y")
    (1, -1)
    """
    if pose not in _POSE_LOOKUP:
        raise ValueError("未知姿态 %r，合法值：%s" % (pose, ", ".join(POSE_NAMES)))
    return _POSE_LOOKUP[pose]


def axis_of_pose(pose: str) -> int:
    """该姿态的主分量轴序号。"""
    return axis_sign(pose)[0]


def mean_sigma(samples: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """把 (N,3) 的样本（单位 g）压成 (均值, 标准差) 两个三元组。

    N=0 时返回全 0 —— 由调用方判 `n < min_samples`，不在这里抛异常。
    """
    a = np.asarray(samples, dtype=np.float64)
    if a.ndim != 2 or a.shape[0] == 0:
        return np.zeros(3), np.zeros(3)
    return a.mean(axis=0), a.std(axis=0)


def diagnose_pose(pose: str, mean: np.ndarray, sigma: np.ndarray,
                  *, static_tol: float = DEFAULT_STATIC_TOL,
                  off_tol: float = DEFAULT_OFF_TOL,
                  mag_tol: float = DEFAULT_MAG_TOL) -> list[str]:
    """检查一个姿态采得对不对，返回问题列表（空 = 通过）。

    这是**唯一能挡住"数字看起来合理但其实摆错姿态"的地方**，所以每一条都要能判否：

    * 主分量轴不对 → 用户把姿态说错了（或板子装反了）
    * 符号不对 → 极常见的失误：把 `-z` 当成 `+z`（芯片面朝下）
    * 主分量幅值偏离 1 g 超过 `mag_tol` → 摆得不平／器件问题
    * 另两轴残留超过 `off_tol` → 摆歪了（倾斜会给主分量"偷走"一部分重力）
    * 轴内标准差超过 `static_tol` → **采集时手还在动**（见模块注释坑 1）

    倾斜与残留的关系：倾斜 θ 时，主分量 ≈ cos θ、侧轴 ≈ sin θ。θ = 5° 就有
    0.087 g 的侧轴残留 —— 所以 `off_tol = 0.08` 约等于「允许倾斜 4.6°」。
    """
    idx, sgn = axis_sign(pose)
    problems: list[str] = []

    got_axis = int(np.argmax(np.abs(mean)))
    if got_axis != idx:
        problems.append(
            "主分量落在 **%s** 轴（%+.4f g），但姿态 %s 期望在 **%s** 轴"
            % (AXES[got_axis], mean[got_axis], pose, AXES[idx]))

    if mean[idx] == 0.0:
        problems.append("%s 轴读数为 0 —— 数据可能全是空行" % AXES[idx])
    elif np.sign(mean[idx]) != sgn:
        problems.append(
            "**符号反了**：%s 轴实测 %+.4f g，但姿态 %s 期望 %+d g"
            "（最常见的失误是把 `%s` 与 `%s` 弄反）"
            % (AXES[idx], mean[idx], pose, sgn,
               "+" + AXES[idx][1:], "-" + AXES[idx][1:]))

    if abs(abs(mean[idx]) - 1.0) > mag_tol:
        problems.append(
            "主分量幅值 |%+.4f| g 偏离 1 g 超过 %.3f g —— 摆放不平或器件异常"
            % (mean[idx], mag_tol))

    for j in range(3):
        if j != idx and abs(mean[j]) > off_tol:
            problems.append(
                "侧轴 %s 残留 %+.4f g 超过 %.3f g —— 板子摆歪了（倾斜越大，"
                "主分量被偷走的重力越多，k/b 会算不准）" % (AXES[j], mean[j], off_tol))

    if float(np.max(sigma)) > static_tol:
        worst = int(np.argmax(sigma))
        problems.append(
            "**采集期间板子在动**：%s 轴标准差 %.4f g > %.3f g —— 手扶着不算静止，"
            "要放到桌面上松手后等稳定" % (AXES[worst], sigma[worst], static_tol))

    return problems


def identify_pose(mean: np.ndarray, sigma: np.ndarray | None = None,
                  *, static_tol: float = DEFAULT_STATIC_TOL,
                  off_tol: float = DEFAULT_OFF_TOL,
                  mag_tol: float = DEFAULT_MAG_TOL):
    """由**一次静止读数**反推「现在拿的是哪个面」，返回 ``(pose, problems, info)``。

    存在的理由：六姿态的名字（`+x`/`-z`…）是**相对芯片自己的坐标系**说的，而手上
    的板子只有一行丝印 —— 让用户"照着文字猜方向"是最容易把姿态说错的地方。所以
    先采一段、让工具报出它认出的姿态，人只需确认"是不是这样"。

    * ``pose`` = 推断的姿态名；读数太斜（主分量 < 0.7 g）时**返回 None**（不猜）
    * ``problems`` = 拿这个推断去跑 `diagnose_pose` 的结果（空 = 干净）
    * ``info`` = ``{"axis","sign","mag_g","raw_lsb"}``，供打印用

    ⚠️ 判据只认两件事：**主分量落在哪个轴** 与 **符号**。这两条恰好是用户最容易
    弄错的两条，也正是本工具能替他判定的两条 —— 其余（幅值/残留/静止）由
    `diagnose_pose` 归入 `problems`。
    """
    m = np.asarray(mean, dtype=np.float64).reshape(3)
    j = int(np.argmax(np.abs(m)))
    sign = 1 if m[j] >= 0 else -1
    pose = ("+" if sign > 0 else "-") + AXES[j][1:]
    info = {
        "axis": j,
        "sign": sign,
        "mag_g": float(np.sqrt(float((m ** 2).sum()))),
        "raw_lsb": [float(v) * vf.ACCEL_LSB_PER_G for v in m],
    }
    sig = np.zeros(3) if sigma is None else np.asarray(sigma, dtype=np.float64)
    problems = diagnose_pose(pose, m, sig, static_tol=static_tol,
                             off_tol=off_tol, mag_tol=mag_tol)
    # 主分量不足 0.7 g ⇒ 大概率是斜着拿/还在动，"认定某个面"就是猜 ⇒ 干脆不猜
    if abs(float(m[j])) < 0.7:
        return None, problems, info
    return pose, problems, info


def usable_poses(doc: dict) -> list[str]:
    """从标定文件里挑出**已经采过且判据全过**的姿态（按 `POSE_NAMES` 顺序）。"""
    out = []
    for p in POSE_NAMES:
        rec = (doc.get("poses") or {}).get(p)
        if rec and not rec.get("problems"):
            out.append(p)
    return out


# 自动选串口时优先匹配的芯片/描述关键字（不区分大小写）。
PORT_PREFER = ("CH340", "CH341", "CH343", "CP210", "USB-SERIAL", "USB SERIAL",
               "SILICON LABS", "FT232", "PROLIFIC")


def device_of(line: str) -> str:
    """从 `list_serial_ports()` 的一行里取出设备名（如 `COM4`）。"""
    parts = line.split()
    return parts[0] if parts else ""


def pick_port(lines: list[str], prefer: tuple[str, ...] = PORT_PREFER):
    """自动挑一个串口，返回 ``(port 或 None, 说明)``。

    为什么要这一层：本机可能同时插着 **ST-Link 的虚拟串口**与 **CH340**，而
    **选错端口**的现象（收不到数据）与**接线故障**一模一样 —— 会把排查引向错误方向。
    所以宁可多一层"按描述优先"的筛选，并且**必须把最终用了哪一个打印出来**。

    规则（**宁可报错，也不猜**）：
      * 0 个 → `(None, …)`；1 个 → 直接用它；
      * 多个、但只有 1 个命中 `prefer` → 用它；
      * 其余（多个命中 / 一个都不命中）→ `(None, 列出全部)`，让用户显式 `--port`。
    """
    if not lines:
        return None, ("没有找到任何串口 —— 板子插好了吗？CH340 驱动装了吗？"
                      "（可用 `--list-ports` 看）")
    if len(lines) == 1:
        return device_of(lines[0]), "只有一个串口，直接用它"
    hits = [ln for ln in lines if any(k.lower() in ln.lower() for k in prefer)]
    if len(hits) == 1:
        return device_of(hits[0]), "按描述唯一匹配到一个 USB 转串口芯片"
    return None, ("有 %d 个串口，无法自动确定 —— 请显式指定 `--port`：\n    %s"
                  % (len(lines), "\n    ".join(lines)))


def short_problem(p: str) -> str:
    """把 `diagnose_pose` 的长句子压成一个短标签（给实时刷新那行用）。"""
    for key, tag in (("符号反了", "符号反"), ("主分量落在", "轴不对"),
                     ("偏离 1 g", "幅值偏"), ("残留", "侧轴残留"),
                     ("板子在动", "在动"), ("读数为 0", "轴为 0")):
        if key in p:
            return tag
    return "?"


def watch_line(rows, t_s: float, *, static_tol: float = DEFAULT_STATIC_TOL,
               off_tol: float = DEFAULT_OFF_TOL,
               mag_tol: float = DEFAULT_MAG_TOL) -> str:
    """把一小段样本压成**一行**实时显示（含合矢量、轴内 σ 与推断出的姿态）。

    纯函数：`--watch` 的刷新行全靠它，所以自测能覆盖。
    """
    arr = np.asarray(rows, dtype=np.float64)
    mean, sigma = mean_sigma(arr)
    pose, problems, info = identify_pose(mean, sigma, static_tol=static_tol,
                                         off_tol=off_tol, mag_tol=mag_tol)
    flag = "OK" if (pose and not problems) else ("??" if pose is None else "!!")
    reason = ""
    if pose is None and arr.shape[0]:
        reason = "认不出（没落在任何一个面上）"
    elif problems:
        reason = "；".join(short_problem(p) for p in problems)
    smax = float(np.max(sigma)) if arr.shape[0] else 0.0
    return ("  t=%6.1fs  n=%3d   ax=%+8.0f  ay=%+8.0f  az=%+8.0f"
            "   |a|=%.4f  σ=%.4f   %-3s %s%s"
            % (t_s, int(arr.shape[0]), info["raw_lsb"][0], info["raw_lsb"][1],
               info["raw_lsb"][2], info["mag_g"], smax,
               pose or "--", flag, ("   " + reason) if reason else ""))


def solve(pose_means: dict[str, np.ndarray]) -> dict:
    """由 6 个姿态的均值解出每轴 k/b；缺姿态或增益离谱时抛 ValueError。

    输入 `pose_means[name] = (ax, ay, az)`（单位 g）。返回::

        {"k": [kx,ky,kz], "b_g": [...], "b_lsb": [...], "raw_plus": [...], "raw_minus": [...]}

    ``b_lsb`` 是为了对得上固件/寄存器口径（±2g → 16384 LSB/g）。
    """
    missing = [p for p in POSE_NAMES if p not in pose_means]
    if missing:
        raise ValueError("缺少姿态：%s（6 个都要有）" % ", ".join(missing))

    k, b_g, b_lsb, raw_plus, raw_minus = [], [], [], [], []
    for j in range(3):
        name_p = "+" + AXES[j][1:]
        name_m = "-" + AXES[j][1:]
        g_p = float(pose_means[name_p][j])       # 该轴朝上时，本轴的读数
        g_m = float(pose_means[name_m][j])       # 该轴朝下时，本轴的读数

        kj = (g_p - g_m) / 2.0
        bj = (g_p + g_m) / 2.0

        if not (K_LO <= kj <= K_HI):
            raise ValueError(
                "轴 %s 解出增益 k = %.4f，超出合法区间 [%.2f, %.2f] —— "
                "通常是姿态摆错（比如 +/ 弄反会让 k 变负）或该姿态数据无效"
                % (AXES[j], kj, K_LO, K_HI))

        k.append(kj)
        b_g.append(bj)
        b_lsb.append(bj * vf.ACCEL_LSB_PER_G)
        raw_plus.append(g_p * vf.ACCEL_LSB_PER_G)
        raw_minus.append(g_m * vf.ACCEL_LSB_PER_G)

    return {"k": k, "b_g": b_g, "b_lsb": b_lsb,
            "raw_plus": raw_plus, "raw_minus": raw_minus}


def apply_calib(samples: np.ndarray, solved: dict) -> np.ndarray:
    """把 (N,3) 的原始 g 值按 ``(g − b) / k`` 修正。"""
    a = np.asarray(samples, dtype=np.float64)
    k = np.asarray(solved["k"], dtype=np.float64)
    b = np.asarray(solved["b_g"], dtype=np.float64)
    return (a - b) / k


def mag(a: np.ndarray) -> np.ndarray:
    """逐样本的 |a|（单位 g）。"""
    x = np.asarray(a, dtype=np.float64)
    if x.shape[0] == 0:
        return np.zeros(0)
    return np.sqrt((x ** 2).sum(axis=1))


def dev_pct(a: np.ndarray) -> tuple[float, float]:
    """|a| 相对 1 g 的偏差，返回 (均值偏差 %, 最大绝对偏差 %)。

    重力大小与朝向无关 —— 所以**任意姿态静止**时 |a| 都应等于 1 g。这是标定效果
    唯一合法的判据来源（别用标定数据自证，见模块注释坑 3）。
    """
    m = mag(a)
    if m.size == 0:
        return float("nan"), float("nan")
    d = (m - 1.0) * 100.0
    return float(d.mean()), float(np.abs(d).max())


def fmt_pose_row(pose: str, mean: np.ndarray, sigma: np.ndarray, n: int) -> str:
    """把一个姿态压成一行报告（含 |a| 与偏差，便于肉眼扫）。"""
    m = float(np.sqrt((np.asarray(mean, dtype=np.float64) ** 2).sum()))
    return ("  %-3s %5d   %+9.2f %+9.2f %+9.2f   %7.4f (%5.0f LSB)   %.4f  %+6.2f%%"
            % (pose, n, mean[0] * vf.ACCEL_LSB_PER_G, mean[1] * vf.ACCEL_LSB_PER_G,
               mean[2] * vf.ACCEL_LSB_PER_G, float(np.max(sigma)),
               float(np.max(sigma)) * vf.ACCEL_LSB_PER_G, m, (m - 1.0) * 100.0))


def poses_from_json(doc: dict) -> dict[str, np.ndarray]:
    """从 calib.json 取出 `{姿态: (ax,ay,az)}`（单位 g）。"""
    out = {}
    for name, rec in (doc.get("poses") or {}).items():
        mg = rec.get("mean_g")
        if mg is None or len(mg) != 3:
            continue
        out[name] = np.asarray(mg, dtype=np.float64)
    return out


# ---------------------------------------------------------------------------
# 2) 采集层
# ---------------------------------------------------------------------------
def capture(port: str, baud: int, seconds: float, *, quiet: bool = False):
    """采 seconds 秒文本数据，返回 (均值, 标准差, 样本数, 文本模式, 原始字节数)。"""
    tp = vs.TextParser()
    nbytes = 0
    for chunk in vs.serial_source(port, baud, duration=seconds, verbose=not quiet):
        nbytes += len(chunk)
        tp.feed(chunk.decode("ascii", "replace"))
    tp.finish()
    s = tp.samples()
    mean, sigma = mean_sigma(s)
    return mean, sigma, int(s.shape[0]), tp.mode, nbytes


def explain_serial_error(e: BaseException, port: str | None = None) -> str:
    """把串口异常翻译成「照着做就行」的中文提示（返回多行文本）。

    为什么值得单列一个函数：``PermissionError(13, '拒绝访问。')`` 是本工具**最高频**
    的失败（一个串口同一时刻只能被一端打开），而它的原始字样完全看不出该干什么 ——
    照抄原文 = 让人自己去搜。实测 2026-10-13 一个晚上撞了两次（都是双击开的
    ``watch.cmd`` 没关）。

    🔴 判据顺序有讲究：pyserial 把底层异常包成 ``SerialException``，消息形如
    ``could not open port 'COM5': PermissionError(13, '拒绝访问。', None, 5)``
    —— 它**同时**含「打不开」与「拒绝访问」两种特征，所以**占用必须排在不存在之前**，
    否则会被后一条抢走。
    """
    msg = str(e)
    low = msg.lower()
    where = port or "串口"

    if (isinstance(e, PermissionError) or "permissionerror" in low
            or "拒绝访问" in msg or "access is denied" in low):
        return (
            "[ERROR] 打不开 %s：**已被别的程序占用**"
            "（一个串口同一时刻只能被一端打开）。\n"
            "        常见占用者（按概率）：\n"
            "          ① 另一个双击开的 `watch.cmd` 黑窗口 —— 关掉它\n"
            "             （窗口标题形如 `MPU6050 - live view ...`）\n"
            "          ② MobaXterm / 串口助手里已打开的串口会话\n"
            "          ③ 上一个没退干净的采集进程（Ctrl-C 后残留）\n"
            "        处理：关掉上面任一个后重跑；或 `--port COMx` 换一个口。\n"
            "        ⚠️ `--list-ports` **只枚举、不打开**，口被占着它照样列得出来 ——\n"
            "           列得出来 ≠ 能用。" % where)

    if (isinstance(e, FileNotFoundError) or "filenotfound" in low
            or "could not open port" in low or "no such" in low):
        return (
            "[ERROR] 打不开 %s：**口不存在或已被拔掉**。\n"
            "        处理：① 确认 CH340 的 USB 插好（板上 D1 亮）\n"
            "              ② 跑 `--list-ports` 看它现在叫什么 —— 换 USB 口会换号"
            "（COM6/COM7…）。" % where)

    return "[ERROR] 打不开 %s：%s" % (where, msg)


def pause_before_capture(note: str, *, auto: bool, countdown: float,
                         quiet: bool) -> None:
    """打印提示，然后（回车 或 倒数）放行。`wait_for_user` 的通用版。"""
    print("\n" + "-" * 68)
    for line in note.splitlines():
        print(line)
    if auto:
        for i in range(int(max(1, countdown)), 0, -1):
            print("  %d…" % i, end="", flush=True)
            time.sleep(1.0)
        print(" 开始")
    else:
        try:
            input("  → 准备好后按回车开始采集：")
        except EOFError:
            print("  [i] 非交互环境（stdin 已结束），直接开始")
    if not quiet:
        print("  采集中…")


def wait_for_user(pose: str, *, auto: bool, countdown: float, quiet: bool) -> None:
    """提示用户摆好某个姿态，然后（回车 或 倒数）放行。"""
    pause_before_capture(
        "姿态 **%s** ：%s\n摆好后**不要用手扶**（松手让它静止），再开始采集。"
        % (pose, POSE_HINT.get(pose, "")),
        auto=auto, countdown=countdown, quiet=quiet)


def capture_pose(args, pose: str, doc: dict) -> tuple[bool, str]:
    """采集一个姿态并写入 doc['poses']；返回 (是否通过, 说明)。"""
    wait_for_user(pose, auto=args.yes, countdown=args.countdown, quiet=args.quiet)
    try:
        mean, sigma, n, tmode, nbytes = capture(args.port, args.baud, args.seconds,
                                                quiet=args.quiet)
    except Exception as e:                                     # noqa: BLE001
        print()
        print(explain_serial_error(e, args.port))
        return False, "%s 采集失败（打不开 %s）" % (pose, args.port)

    if n < args.min_samples:
        return False, ("只解出 %d 个样本（收到 %d 字节，文本模式 %r），少于 %d —— "
                       "检查：① 固件是否 APP_OUTPUT_MODE=0/1（只有这两档输出文本）"
                       "② TX/RX 是否交叉 ③ 波特率 115200"
                       % (n, nbytes, tmode, args.min_samples))

    problems = diagnose_pose(pose, mean, sigma,
                            static_tol=args.static_tol, off_tol=args.off_tol,
                            mag_tol=args.mag_tol)

    print("  样本 %d 个（%d 字节，%s）" % (n, nbytes,
                                        {"raw": "mode 0 原始整数", "g": "mode 1 g 值",
                                         None: "未识别"}.get(tmode, str(tmode))))
    print("  均值 LSB  ax=%+.2f  ay=%+.2f  az=%+.2f"
          % tuple(v * vf.ACCEL_LSB_PER_G for v in mean))
    print("  轴内 σ g  %.4f  %.4f  %.4f" % tuple(sigma))
    print(fmt_pose_row(pose, mean, sigma, n))

    if tmode == "g":
        print("  ⚠️ 检测到 mode 1（`%.3f` g 打印，分辨率仅 0.001 g ≈ 16 LSB）"
              "—— 精度不如 mode 0 的原始整数，条件允许请切回 mode 0")

    if problems:
        print("\n  ❌ 该姿态**未通过**，已保存但**不可用于解算**：")
        for p in problems:
            print("     - %s" % p)
        doc.setdefault("poses", {})[pose] = _pose_record(mean, sigma, n, tmode,
                                                         problems)
        return False, "%s 未通过（%d 条问题）" % (pose, len(problems))

    print("\n  ✅ 该姿态通过")
    doc.setdefault("poses", {})[pose] = _pose_record(mean, sigma, n, tmode, [])
    return True, "%s 通过（n=%d）" % (pose, n)


def _pose_record(mean, sigma, n, tmode, problems) -> dict:
    return {
        "mean_g": [float(v) for v in mean],
        "sigma_g": [float(v) for v in sigma],
        "n": int(n),
        "text_mode": tmode,
        "problems": list(problems),
    }


# ---------------------------------------------------------------------------
# 3) 报告与产出
# ---------------------------------------------------------------------------
def print_pose_table(doc: dict) -> None:
    print("\n" + "=" * 68)
    print("六姿态汇总（LSB = 原始整数，±2g → 16384 LSB/g）")
    print("=" * 68)
    print("  姿态   样本      ax 均值     ay 均值     az 均值     最大轴内 σ      |a|     偏差")
    for p in POSE_NAMES:
        rec = (doc.get("poses") or {}).get(p)
        if not rec:
            print("  %-3s    --   （缺）" % p)
            continue
        row = fmt_pose_row(p, np.asarray(rec["mean_g"]), np.asarray(rec["sigma_g"]),
                           rec["n"])
        bad = "  ❌" if rec.get("problems") else ""
        print(row + bad)

    ok = usable = [p for p in POSE_NAMES
                   if (doc.get("poses") or {}).get(p)
                   and not (doc.get("poses") or {})[p].get("problems")]
    print("\n  可用于解算的姿态：%d / 6 %s" % (len(usable),
                                             ("（" + " ".join(usable) + "）") if usable else ""))
    if len(usable) < 6:
        miss = [p for p in POSE_NAMES if p not in usable]
        print("  ⚠️ 还缺：%s" % " ".join(miss))


def run_solve(doc: dict, *, emit_c: str | None) -> int:
    """解算 + 自洽性报告 + 可选导出。返回退出码。"""
    means = poses_from_json(doc)
    bad = {p: r.get("problems") for p, r in (doc.get("poses") or {}).items()
           if r.get("problems")}
    if bad:
        print("\n[ERROR] 有姿态未通过检查，不予解算：")
        for p, ps in sorted(bad.items()):
            print("  %s: %s" % (p, "；".join(ps)))
        print("        先重采这些姿态（加 --pose <名字>），再 --solve。")
        return 2
    try:
        solved = solve(means)
    except ValueError as e:
        print("\n[ERROR] %s" % e)
        return 2

    k, b_g, b_lsb = solved["k"], solved["b_g"], solved["b_lsb"]

    print("\n" + "=" * 68)
    print("解算结果　a_true = (R_meas − b) / k")
    print("=" * 68)
    print("  轴     R₊(LSB)     R₋(LSB)        b(LSB)      b(g)        k")
    for j in range(3):
        print("  %-3s  %+10.1f  %+10.1f  %+12.1f  %+9.5f   %.6f"
              % (AXES[j], solved["raw_plus"][j], solved["raw_minus"][j],
                 b_lsb[j], b_g[j], k[j]))

    k_err = [(kj - 1.0) * 100.0 for kj in k]
    print("\n  增益偏差：%s" % "   ".join("%s %+.3f%%" % (AXES[j], k_err[j])
                                         for j in range(3)))
    print("  零偏(LSB)：%s" % "   ".join("%s %+.1f" % (AXES[j], b_lsb[j])
                                         for j in range(3)))

    # 自洽性检查（**不是**验证，必须说清楚）
    stack = np.vstack([means[p] for p in POSE_NAMES])
    before_mean, before_max = dev_pct(stack)
    after = apply_calib(stack, solved)
    after_mean, after_max = dev_pct(after)

    print("\n  ── 自洽性检查（**同一批标定数据**，只能证明解算没算错）──")
    print("  标定前 |a| 偏差：均值 %+.3f%%  最大 %+.3f%%" % (before_mean, before_max))
    print("  标定后 |a| 偏差：均值 %+.3f%%  最大 %+.3f%%" % (after_mean, after_max))
    print("  🔴 这**不是**验证：拟合本来就会让这 6 个点变好。真正的验证请跑")
    print("     `--verify`（任意姿态新采一段，看 |a| 是否更接近 1 g）。")

    doc["solved"] = {
        "k": [float(v) for v in k],
        "b_g": [float(v) for v in b_g],
        "b_lsb": [float(v) for v in b_lsb],
        "selfcheck_dev_pct": {"before_mean": before_mean, "before_max": before_max,
                              "after_mean": after_mean, "after_max": after_max},
    }
    doc["solved_at"] = _now_iso()
    print("\n  ✅ 已写入 `%s` 的 `solved` 字段" % (doc.get("_path", "calib.json")))

    rc = 0
    if emit_c:
        try:
            with open(emit_c, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(render_c_header(solved, doc))
            print("  ✅ 已导出 C 头文件片段 → %s" % emit_c)
        except OSError as e:
            print("  [ERROR] 写 %s 失败：%s" % (emit_c, e))
            rc = 2
    return rc


def render_c_header(solved: dict, doc: dict) -> str:
    """生成给固件用的常数头文件片段。

    ⚠️ 固件**尚未消费**这些常数（`Core/` 下没有标定接口）；先产出备用，
    等固件加上接口（OTA 参数区已为「6 姿态 k/b」预留 24 B）再接。
    """
    k, b_lsb, b_g = solved["k"], solved["b_lsb"], solved["b_g"]
    lines = [
        "/* 自动生成 —— 由 tools/calib_dev.py 产出，请勿手改。",
        " * 生成时间：%s" % doc.get("solved_at", _now_iso()),
        " * 模型：a_true = (R_meas - b) / k    （R 单位 LSB，±2g → 16384 LSB/g）",
        " *",
        " * 🔴 固件当前**尚未**消费这些常数（Core/ 下无标定接口）；",
        " *    等接口就绪（OTA 参数区已预留 24 B）再接。此前可用于离线修正数据集。",
        " *",
        " * 使用示例：",
        " *     float ax = (raw_ax - MPU6050_CALIB_B_X) / MPU6050_CALIB_K_X;",
        " */",
        "#ifndef MPU6050_CALIB_H",
        "#define MPU6050_CALIB_H",
        "",
        "#define MPU6050_CALIB_VALID        (1U)",
        "",
    ]
    for j, name in enumerate(("X", "Y", "Z")):
        lines.append("#define MPU6050_CALIB_K_%s         (%.6ff)" % (name, k[j]))
    lines.append("")
    for j, name in enumerate(("X", "Y", "Z")):
        lines.append("#define MPU6050_CALIB_B_%s         (%.2ff)   /* %.6f g */"
                     % (name, b_lsb[j], b_g[j]))
    lines.append("")
    lines.append("#endif /* MPU6050_CALIB_H */")
    return "\n".join(lines) + "\n"


def run_identify(args, doc: dict) -> int:
    """采一段，告诉用户「现在拿的是哪个面」，并指出下一步该摆哪个。

    这是六姿态流程的**入口**：姿态名是相对芯片自己坐标系说的，而板子上只有一行
    丝印 —— 让用户照着文字猜方向，正是最容易出错的地方。所以先让工具报出它认出
    的姿态，人只需点头。
    """
    print("=" * 68)
    print("姿态识别：随便摆一个面，让工具告诉你它是哪个（不用对着丝印猜）")
    print("=" * 68)
    print("把板子**按任意一个面摆正、松手静止**（不要用手扶）。任意一面都行 ——")
    print("识别不需要你先知道它是哪个面。")

    pause_before_capture("准备开始识别。", auto=args.yes, countdown=args.countdown,
                         quiet=args.quiet)
    try:
        mean, sigma, n, tmode, nbytes = capture(args.port, args.baud, args.seconds,
                                                quiet=args.quiet)
    except Exception as e:                                     # noqa: BLE001
        print(explain_serial_error(e, args.port))
        return 2
    if n < args.min_samples:
        print("[ERROR] 只解出 %d 个样本（%d 字节）—— 见 --pose 的排查清单。" % (n, nbytes))
        return 2

    pose, problems, info = identify_pose(
        mean, sigma, static_tol=args.static_tol, off_tol=args.off_tol,
        mag_tol=args.mag_tol)
    done = usable_poses(doc)

    print("\n  样本 %d 个（%d 字节，文本模式 %r）" % (n, nbytes, tmode))
    print("  均值 LSB  ax=%+.0f  ay=%+.0f  az=%+.0f   |a| = %.4f g   最大轴内 σ = %.4f g"
          % (info["raw_lsb"][0], info["raw_lsb"][1], info["raw_lsb"][2],
             info["mag_g"], float(np.max(sigma))))

    if pose is None:
        print("\n  ❓ **认不出来**：三个轴都没到 0.7 g —— 板子没落在任何一个面上。")
        print("     可能：斜着拿 / 采集时还在动 / 传感器根本没出数。摆平、松手，再来一次。")
        for p in problems:
            print("       - %s" % p)
        return 1

    print("\n  你现在拿着的姿态是：**%s**" % pose)

    if problems:
        print("  ⚠️ 但这次读数有 %d 项不符 —— 先别采，按提示摆正：" % len(problems))
        for p in problems:
            print("       - %s" % p)
        print("\n  → 摆正后再跑一次 `--identify`；也可以直接 `--guided`"
              "（它逐个提示并要求你确认，摆错会拒收）。")
        return 1

    print("  ✅ 四项判据（主分量轴 / 符号 / 幅值 / 残留 / 静止）全过 —— 可以直接采它：")
    print("       python calib_dev.py --port %s --pose %s" % (args.port, pose))
    if pose in done:
        print("\n  📌 这个姿态**之前已经采过**（且已通过），再采会覆盖旧记录。")

    print("\n  六姿态进度：%d / 6%s" % (len(done),
                                       ("   已过：%s" % " ".join(done)) if done else ""))
    rest = [p for p in POSE_NAMES if p not in done and p != pose]
    if rest:
        print("  还差：%s" % " ".join(rest))
        print("  → 建议下一个摆 **%s**：%s" % (rest[0], POSE_HINT[rest[0]]))
    elif pose in done:
        print("  🎉 六个姿态齐了 → python calib_dev.py --solve --json %s" % args.json)
    else:
        print("  → 采下这个姿态后，六个就齐了。")
    return 0


def run_watch(args) -> int:
    """实时看三轴读数与推断姿态（**只为"看清方向"，不是采集**）。

    设计取舍：**只开一次串口**。`vibsource.serial_source` 每调用一次就开/关一次
    串口，而反复开关串口既慢又可能让器件复位 —— 所以这里用一个长连接
    （`duration=None`），把 `TextParser.feed()` 的返回值按时间窗切段统计。
    这样既不丢行（parser 的 `_tail` 一直保留），内存也不随时间增长。
    """
    print("=" * 68)
    print("实时看三轴　port=%s @%d　每 %.1f s 刷新一行"
          % (args.port, args.baud, args.interval))
    print("=" * 68)
    print("让它一直跑，**翻动板子**看最后一列「推断」怎么变。Ctrl-C 停止。")
    print("mode 0（原始整数，10 Hz）与 mode 1（g 值，200 Hz）都认。")
    print("🔴 这一档只是让你**先看清方向**；采标定数据仍要走 `--guided`（要求松手静止）。")
    print("-" * 68)
    print("   %s   %s      %s        %s        %s   %s  %s   %s"
          % ("时间", "样本", "ax(LSB)", "ay(LSB)", "az(LSB)", "合矢量", "轴内σ", "推断"))

    tp = vs.TextParser()
    t0 = time.time()
    win_start = t0
    buf: list = []
    total = 0
    rows_shown = 0
    try:
        for chunk in vs.serial_source(args.port, args.baud, duration=None,
                                      idle_timeout=args.idle_timeout,
                                      verbose=not args.quiet):
            tp.bytes_in += len(chunk)
            buf.extend(tp.feed(chunk.decode("ascii", "replace")))
            now = time.time()
            if now - win_start >= args.interval:
                if buf:
                    total += len(buf)
                    print(watch_line(buf, now - t0, static_tol=args.static_tol,
                                     off_tol=args.off_tol, mag_tol=args.mag_tol))
                    rows_shown += 1
                    buf = []
                else:
                    print("  t=%6.1fs   —— 这段时间内没有解出任何数据行"
                          "（累计收到 %d 字节，文本模式 %r）" % (now - t0,
                                                              tp.bytes_in, tp.mode))
                win_start = now
    except KeyboardInterrupt:
        print("\n[Ctrl-C] 已停止。")
    except Exception as e:                                     # noqa: BLE001
        print("\n" + explain_serial_error(e, args.port))
        print("        另外核对：板子在跑 `APP_OUTPUT_MODE=0/1` 吗？（只有这两档输出文本）")
        return 2

    print("-" * 68)
    print("共显示 %d 行、解出 %d 个样本；累计收到 %d 字节（%s）"
          % (rows_shown, total, tp.bytes_in, tp.as_text()))
    if total == 0:
        print("🔴 一个样本都没解出来 —— 先怀疑**观察链路**（端口 / 模式 / 波特率），"
              "再怀疑传感器（本项目踩坑 3）。")
        return 1
    return 0


def run_verify(args, doc: dict) -> int:
    """独立验证：任意姿态新采一段，比较标定前后的 |a| 偏差。"""
    solved = doc.get("solved")
    if not solved:
        print("[ERROR] `%s` 里没有 `solved` 字段 —— 先跑 `--solve`。" % args.json)
        return 2

    print("\n" + "=" * 68)
    print("独立验证：任意姿态静止时 |a| 必须 = 1 g（重力大小与朝向无关）")
    print("=" * 68)
    print("把板子**随便摆一个姿态**（别刻意对齐），松手静止后开始。")
    wait_for_user("verify", auto=args.yes, countdown=args.countdown, quiet=args.quiet)
    # wait_for_user 会打印 POSE_HINT[pose]，这里 pose 不是六姿态之一，单独处理
    try:
        mean, sigma, n, tmode, nbytes = capture(args.port, args.baud, args.seconds,
                                                quiet=args.quiet)
    except Exception as e:                                     # noqa: BLE001
        print("\n" + explain_serial_error(e, args.port))
        return 2

    if n < args.min_samples:
        print("[ERROR] 只解出 %d 个样本（%d 字节）—— 见 --pose 的排查清单。" % (n, nbytes))
        return 2

    if float(np.max(sigma)) > args.static_tol:
        print("[ERROR] 采集期间板子在动：最大轴内 σ = %.4f g > %.3f g —— "
              "松手静止后重来。" % (float(np.max(sigma)), args.static_tol))
        return 1

    # 用均值点做判据（比逐样本更稳，且静止时逐样本与均值等价）
    raw_pt = np.asarray(mean, dtype=np.float64).reshape(1, 3)
    cal_pt = apply_calib(raw_pt, solved)

    before_mean, before_max = dev_pct(raw_pt)
    after_mean, after_max = dev_pct(cal_pt)

    print("\n  样本 %d 个（%d 字节，文本模式 %r）" % (n, nbytes, tmode))
    print("  未标定均值  ax=%+.2f ay=%+.2f az=%+.2f LSB" % tuple(mean * vf.ACCEL_LSB_PER_G))
    print("  已标定均值  ax=%+.2f ay=%+.2f az=%+.2f LSB" % tuple(cal_pt[0] * vf.ACCEL_LSB_PER_G))
    print("\n  |a| 偏差（相对 1 g）")
    print("    未标定：%+.3f %%" % before_mean)
    print("    已标定：%+.3f %%" % after_mean)

    ok = abs(after_mean) <= abs(before_mean) and abs(after_mean) <= args.tol
    print()
    if abs(after_mean) <= args.tol:
        if abs(after_mean) <= abs(before_mean):
            print("  ✅ **通过**：标定后 |a| 偏差 %.3f%% ≤ 容差 %.2f%%，且比标定前更小"
                  % (after_mean, args.tol))
            rc = 0
        else:
            print("  ⚠️ 虽在容差内（%.3f%% ≤ %.2f%%），但**比标定前更差**（%.3f%%）"
                  "—— 这次验证的朝向恰好没暴露误差，多试几个姿态。" % (after_mean, args.tol, before_mean))
            rc = 1
    else:
        print("  ❌ **未通过**：标定后 |a| 偏差 %.3f%% > 容差 %.2f%%" % (after_mean, args.tol))
        print("     排查：① 零偏是**温度相关**的 —— 标定与验证之间如果器件凉了/热了，")
        print("     零偏会漂；② 标定时的 6 个姿态本身摆得不准；③ 采集时板子在动。")
        rc = 1

    doc.setdefault("verifications", []).append({
        "at": _now_iso(),
        "n": int(n),
        "raw_g": [float(v) for v in mean],
        "cal_g": [float(v) for v in cal_pt[0]],
        "dev_pct_before": before_mean,
        "dev_pct_after": after_mean,
        "pass": bool(rc == 0),
    })
    return rc


def _now_iso() -> str:
    return _dt.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")


def load_doc(path: str) -> dict:
    if not os.path.exists(path):
        return {"tool": "calib_dev.py", "version": 1, "created": _now_iso(), "poses": {}}
    with open(path, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    doc.setdefault("poses", {})
    return doc


def save_doc(path: str, doc: dict) -> None:
    doc.pop("_path", None)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=2, sort_keys=False)
        fh.write("\n")


# ---------------------------------------------------------------------------
# 4) CLI
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="MPU6050 六姿态静态标定（分离零偏 b 与增益 k）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="退出码：0 通过 / 1 判据不过 / 2 环境或数据问题")
    ap.add_argument("--list-ports", action="store_true", help="列出串口后退出")
    ap.add_argument("--port", help="串口设备（不给则自动挑一个；多个候选时会让你显式指定）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--pose", choices=POSE_NAMES,
                    help="采单个姿态（可分批做，结果累积在 --json 里）")
    ap.add_argument("--guided", action="store_true", help="依次提示 6 个姿态")
    ap.add_argument("--identify", action="store_true",
                    help="🔴 采一段并告诉你「现在拿的是哪个面」（不用对着丝印猜）")
    ap.add_argument("--solve", action="store_true", help="解算（不碰硬件，读 --json）")
    ap.add_argument("--verify", action="store_true",
                    help="独立验证：任意姿态采集，比较标定前后 |a| 偏差")
    ap.add_argument("--watch", action="store_true",
                    help="实时看三轴读数与推断姿态（只为看清方向，不是采集）")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="--watch 每行覆盖的秒数，默认 1.0（10 Hz 下约 10 个样本）")
    ap.add_argument("--idle-timeout", type=float, default=5.0,
                    help="--watch 静默多少秒后自动退出，默认 5")
    ap.add_argument("--emit-c", metavar="FILE", help="导出 C 头文件片段（配合 --solve）")
    ap.add_argument("--json", default="calib.json", help="标定数据文件，默认 calib.json")
    ap.add_argument("--seconds", type=float, default=DEFAULT_SECONDS,
                    help="每个姿态采集秒数，默认 %.1f" % DEFAULT_SECONDS)
    ap.add_argument("--min-samples", type=int, default=DEFAULT_MIN_SAMPLES,
                    help="最少样本数，默认 %d" % DEFAULT_MIN_SAMPLES)
    ap.add_argument("--static-tol", type=float, default=DEFAULT_STATIC_TOL,
                    help="轴内标准差上限 g，默认 %.3f" % DEFAULT_STATIC_TOL)
    ap.add_argument("--off-tol", type=float, default=DEFAULT_OFF_TOL,
                    help="侧轴残留上限 g，默认 %.3f（约合允许倾斜 4.6°）" % DEFAULT_OFF_TOL)
    ap.add_argument("--mag-tol", type=float, default=DEFAULT_MAG_TOL,
                    help="主分量偏离 1 g 的容差，默认 %.3f" % DEFAULT_MAG_TOL)
    ap.add_argument("--tol", type=float, default=0.5,
                    help="--verify 的 |a| 偏差容差 %%，默认 0.5（= T3 判据）")
    ap.add_argument("--yes", action="store_true", help="不等回车，倒数后自动开始")
    ap.add_argument("--countdown", type=float, default=3.0, help="--yes 的倒数秒数，默认 3")
    ap.add_argument("--quiet", action="store_true", help="少打印过程信息")
    return ap


def resolve_port(args) -> tuple[str | None, int]:
    """确定用哪个串口：给了 `--port` 就用它，否则按描述自动挑一个。

    自动挑**一定会打印用了哪一个** —— "选错端口"与"接线故障"现象相同，
    不把选择结果摆到明面上，排查就会走错方向。
    """
    if args.port:
        return args.port, 0
    lines = vs.list_serial_ports()
    port, why = pick_port(lines)
    if port is None:
        print("[ERROR] %s" % why)
        return None, 2
    matched = [ln for ln in lines if device_of(ln) == port]
    print("[i] 自动选择串口：**%s**（%s）%s"
          % (port, why, ("\n    命中：" + matched[0]) if matched else ""))
    print("    （要指定别的，加 `--port COMx`；想先看清单，跑 `--list-ports`）")
    return port, 0


def _unbuffer_stdout() -> None:
    """让 stdout 在**非终端**（管道 / 重定向 / 被别的程序抓）下也逐行输出。

    为什么需要：Python 只在 stdout **是终端**时才行缓冲；是管道时改成 4~8 KB 块缓冲。
    `--watch` 这类"无限刷新的实时视图"每行才几十字节，于是**一屏都攒不满** ⇒
    屏幕上**一行都不出现**，看起来像卡死。实测（2026-10-13）：后台跑 `--watch`
    13 s、零输出，而同一时刻串口数据明明在流 —— 正是"0 字节先怀疑观察链路"那条
    纪律的镜像版本：**别把"自己的输出被缓冲了"误判成"设备没数据"**。

    终端下本来就是行缓冲，所以对"双击 `watch.cmd`"这个主场景无影响（幂等、无副作用）。

    ⚠️ 只给 `line_buffering=True` **不够** —— 它仅在写入 `\\n` 时才 flush，而本工具
    里还有 `print("  3…", end="")` 这种**不带换行**的倒数进度（`pause_before_capture`）。
    所以同时开 `write_through=True`（2026-10-13 自测探针实测：只给 line_buffering 时
    \"写了 5 字节但不带换行\"仍要等到进程退出才可见）。
    """
    try:
        sys.stdout.reconfigure(line_buffering=True, write_through=True)   # Py 3.7+
    except Exception:                                   # noqa: BLE001
        pass


def _normalize_argv(argv: list[str]) -> list[str]:
    """把 `["--pose", "-z"]` 改写成 `["--pose=-z"]`。

    🔴 **必须做**：argparse 遇到以 `-` 开头的值会把它当成**选项** ⇒ `--pose -z` 直接
    报 ``error: argument --pose: expected one argument``；而六个面里有**三个是负号
    开头的**（`-x` / `-y` / `-z`）⇒ 它们**根本传不进去**。2026-10-15 真机第一次采
    `-z` 时撞到（`--pose=-z` 本来能用，但没人想得到要这么写）。

    刻意只改这一种形态、不做通用解析 —— 通用化会把"用户其实写错了"也悄悄吞掉。
    """
    out: list[str] = []
    i = 0
    while i < len(argv):
        if argv[i] == "--pose" and i + 1 < len(argv) and argv[i + 1] in POSE_NAMES:
            out.append("--pose=" + argv[i + 1])
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def main(argv: list[str] | None = None) -> int:
    _unbuffer_stdout()
    raw = list(sys.argv[1:]) if argv is None else list(argv)
    args = build_parser().parse_args(_normalize_argv(raw))

    if args.list_ports:
        for line in vs.list_serial_ports():
            print(line)
        return 0

    if args.solve:
        doc = load_doc(args.json)
        rc = run_solve(doc, emit_c=args.emit_c)
        save_doc(args.json, doc)
        return rc
    if args.emit_c and not args.solve:
        print("[ERROR] --emit-c 需要配合 --solve")
        return 2

    # ↓ 以下全部要用串口 —— 统一在这里解析一次
    port, rc = resolve_port(args)
    if port is None:
        return rc
    args.port = port

    if args.watch:
        return run_watch(args)

    if args.identify:
        doc = load_doc(args.json)      # 只读：用来看"六姿态还差几个"
        return run_identify(args, doc)

    if args.verify:
        doc = load_doc(args.json)
        rc = _run_verify_hardware(args, doc)
        doc["_path"] = args.json
        save_doc(args.json, doc)
        return rc

    doc = load_doc(args.json)
    doc["_path"] = args.json
    for key, val in (("port", args.port), ("baud", args.baud),
                     ("seconds", args.seconds), ("updated", _now_iso())):
        doc[key] = val

    print("=" * 68)
    print("MPU6050 六姿态静态标定　port=%s @%d　每姿态 %.1f s" % (args.port, args.baud,
                                                              args.seconds))
    print("=" * 68)
    print("模  型：a_true = (R_meas − b) / k")
    print("要  求：固件处于 APP_OUTPUT_MODE=0（原始整数，10 Hz，推荐）或 1（g 值 200 Hz）")
    print("摆  法：每个姿态都要**平放且松手静止**；倾斜会让结果失真。")

    todo = POSE_NAMES if args.guided else [args.pose]
    results = []
    for pose in todo:
        ok, msg = capture_pose(args, pose, doc)
        results.append((pose, ok, msg))
        save_doc(args.json, doc)          # 每个姿态落盘一次，中途断了不白采

    print("\n" + "=" * 68)
    print("本轮小结")
    print("=" * 68)
    for pose, ok, msg in results:
        print("  %s  %s" % ("✅" if ok else "❌", msg))
    print_pose_table(doc)

    if all(p in poses_from_json(doc) for p in POSE_NAMES) and not any(
            (doc["poses"].get(p) or {}).get("problems") for p in POSE_NAMES):
        print("\n6 个姿态齐了 → 接着跑：python calib_dev.py --solve --json %s" % args.json)
    else:
        print("\n还差姿态 → 继续跑 `--pose <名字>`（或 `--guided`）。")

    return 0 if all(ok for _, ok, _ in results) else 1


def _run_verify_hardware(args, doc: dict) -> int:
    """把 --verify 的采集部分与纯逻辑分开，便于以后做自测。"""
    return run_verify(args, doc)


if __name__ == "__main__":
    sys.exit(main())
