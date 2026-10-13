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


def wait_for_user(pose: str, *, auto: bool, countdown: float, quiet: bool) -> None:
    """提示用户摆好姿态，然后（回车 或 倒数）放行。"""
    print("\n" + "-" * 68)
    print("姿态 **%s** ：%s" % (pose, POSE_HINT[pose]))
    print("摆好后**不要用手扶**（松手让它静止），再开始采集。")
    if auto:
        for i in range(int(max(1, countdown)), 0, -1):
            print("  %d…" % i, end="", flush=True)
            time.sleep(1.0)
        print(" 开始")
    else:
        try:
            input("  → 摆好后按回车开始采集：")
        except EOFError:
            print("  [i] 非交互环境（stdin 已结束），直接开始")
    if not quiet:
        print("  采集中…")


def capture_pose(args, pose: str, doc: dict) -> tuple[bool, str]:
    """采集一个姿态并写入 doc['poses']；返回 (是否通过, 说明)。"""
    wait_for_user(pose, auto=args.yes, countdown=args.countdown, quiet=args.quiet)
    try:
        mean, sigma, n, tmode, nbytes = capture(args.port, args.baud, args.seconds,
                                                quiet=args.quiet)
    except Exception as e:                                     # noqa: BLE001
        return False, "采集失败：%s" % e

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
        print("[ERROR] 采集失败：%s" % e)
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
    ap.add_argument("--port", help="串口设备（Windows 如 COM5）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--pose", choices=POSE_NAMES,
                    help="采单个姿态（可分批做，结果累积在 --json 里）")
    ap.add_argument("--guided", action="store_true", help="依次提示 6 个姿态")
    ap.add_argument("--solve", action="store_true", help="解算（不碰硬件，读 --json）")
    ap.add_argument("--verify", action="store_true",
                    help="独立验证：任意姿态采集，比较标定前后 |a| 偏差")
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


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

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

    if args.verify:
        doc = load_doc(args.json)
        rc = _run_verify_hardware(args, doc)
        doc["_path"] = args.json
        save_doc(args.json, doc)
        return rc

    if not args.port:
        print("[ERROR] 需要 --port（或 --list-ports / --solve）")
        print("        例：python calib_dev.py --port COM5 --guided")
        return 2

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
