#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_blade_peak.py — R17 频谱门禁：叶频/转频峰到底压过本底没有

`RUNBOOK.md` R6.4 第 ⑥ 步要求：采一段振动数据，**在频谱里确认叶频峰真的可见且
明显高于本底**，不过关就别先采几十段数据集。本工具把"肉眼判断"换成数值判据。

判据（逐轴独立计算）
--------------------------------------------------------------------------
    本底 = median(幅度谱, f ≥ 5 Hz, 并挖掉峰两侧 ±notch Hz)
    SNR  = 20·log10(峰幅值 / 本底)          [dB]
    通过 = SNR ≥ --min-snr  且  峰幅值 ≥ --min-amp

**任一轴通过即整条记录通过**（电机振动方向取决于安装姿态，不必三轴都有）。

跑法::

    # R6.4 第 ⑥ 步：先用 parse_frames.py 采出 CSV，再来判定
    python parse_frames.py --port COM4 --duration 10 --csv d.csv --plot d.png
    python check_blade_peak.py --csv d.csv --plot peak.png

退出码：``0`` 通过 / ``1`` 不通过 / ``2`` 判不了（数据不足、带宽配置矛盾、读文件失败）。

🔴 三条必须知道的原理（不然会误判）
--------------------------------------------------------------------------
1. **本底要用"分段平均谱"（Welch）**。单段 FFT 的每个谱线是个随机量，
   **纯白噪声下 max/median 也能到 ≈9 dB** —— 那样 12 dB 的判据形同虚设。
   分段平均（本项目 10 s 数据 ≈18 段）把每个谱线的相对起伏压到 ≈1/(2√K)，
   纯噪声的 max/median 掉到 ≈3 dB，判据才有区分度。**段数 <4 时工具会警告。**
2. **采样率从 CSV 的 `t_s` 列反推**（`rate = 1/median(diff(t_s))`），不硬编码 1000。
   R6.4 第 ④ 步实测过 1000.01 Hz；拿实测值算频率才不会把峰的频率标错。
3. **两个上限先记住**：`MPU6050` 加速度带宽上限 **260 Hz**（R17，`DLPF_CFG=0`），
   而 Nyquist = 采样率/2 = **500 Hz**。峰 > 260 Hz 幅值已被带宽滚降压过
   （真实幅值更大）；峰 > 450 Hz 要怀疑是混叠镜像；**整条带都在 Nyquist 之外时
   看到的频率是假的**（不是"看不到"，是"位置是假的"）。
4. 🔴 **搜索带取的是"带的**最大**幅值"，所以别的谐波会冒充叶频**。本项目叶频 =
   `--blade-multiple`（默认 3）× 转频，而 `1×` 通常最强、`2×` 常与 `3×` 同量级 ⇒
   只要叶频带下沿低于 `2 × 转频`，**`2×` 就会顶替 `3×` 被报成"叶频"并让门禁假通过**。
   为此本工具会做**谐波自洽校验**：用实测 `1×` 反推期望叶频 `K × 转频`，再看带内
   那个峰落在第几阶谐波上；若它其实是别的阶次（如 2×），**直接判不通过**并给出
   该用的叶频带。⇒ **叶频带下沿应高于 `2 × 转频`**（3 V 档 410 Hz 之所以没事，
   是因为默认带下沿 300 Hz 恰好把 273 Hz 的 2× 挡在外面 —— 降速后这个巧合就没了）。
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

RATE_FALLBACK = 1000.0          # CSV 里没有可用的 t_s 列时的兜底采样率
DEFAULT_NPERSEG = 1024          # ≈1.024 s @1kHz → Δf ≈ 0.98 Hz
DEFAULT_OVERLAP = 0.5
DEFAULT_BLADE_BAND = (300.0, 660.0)   # 3 叶 × (6000–13000 RPM) / 60
DEFAULT_ROT_BAND = (100.0, 220.0)     # 6000–13000 RPM 的 1× 转频
DEFAULT_MIN_SNR_DB = 12.0
DEFAULT_MIN_AMP_G = 0.002       # 2 mg：MPU6050 在 260 Hz 带内的噪声本底量级
FLOOR_MIN_HZ = 5.0              # 本底统计与找峰都跳过 DC 附近
NOTCH_HZ = 50.0                 # 算本底时挖掉峰两侧这么宽，免得峰把自己抬进本底
DEVICE_BW_HZ = 260.0            # MPU6050 加速度带宽上限（R17）
DEFAULT_BLADE_MULTIPLE = 3.0    # 叶频 = 该倍数 × 转频（本项目 3 叶）
HARMONIC_TOL = 0.06             # 谐波阶次的相对容差（±6%）
HARMONIC_ORDERS = (1, 2, 3, 4, 5)   # 参与比对的低次谐波阶次
AXIS_NAMES = ("ax", "ay", "az")


# ---------------------------------------------------------------------------
# 频谱
# ---------------------------------------------------------------------------
def segment_count(n: int, nperseg: int, overlap: float) -> int:
    """数据能切成几段（不分段时算 1）。"""
    nperseg = int(min(nperseg, n))
    step = max(1, int(nperseg * (1.0 - overlap)))
    if n < nperseg:
        return 1
    return (n - nperseg) // step + 1


def amplitude_spectrum(sig, rate: float, nperseg: int = DEFAULT_NPERSEG,
                       overlap: float = DEFAULT_OVERLAP):
    """Hann 窗 Welch **幅度**谱（分段功率平均后再开方）。返回 (freqs, amp)。

    幅度单位与输入相同（本项目是 g）。分段数为 1 时退化成单个加窗 FFT。
    """
    sig = np.asarray(sig, dtype=np.float64)
    n = len(sig)
    if n < 8:
        raise ValueError("样本太少（%d 点），算不出频谱" % n)
    nperseg = int(min(nperseg, n))
    if nperseg < 8:
        raise ValueError("nperseg 至少 8，收到 %d" % nperseg)

    freqs = np.fft.rfftfreq(nperseg, d=1.0 / rate)
    win = np.hanning(nperseg)
    cg = win.sum()                       # 窗增益 → 幅度归一化系数
    step = max(1, int(nperseg * (1.0 - overlap)))

    acc = None
    k = 0
    for start in range(0, n - nperseg + 1, step):
        seg = sig[start:start + nperseg]
        seg = seg - seg.mean()           # 去均值：DC 不参与
        sp = np.abs(np.fft.rfft(seg * win)) * (2.0 / cg)
        acc = sp * sp if acc is None else acc + sp * sp
        k += 1

    if k == 0:                           # 数据比一段还短：居中补零凑一段
        seg = np.zeros(nperseg)
        off = (nperseg - n) // 2
        seg[off:off + n] = sig - sig.mean()
        return freqs, np.abs(np.fft.rfft(seg * win)) * (2.0 / cg)

    return freqs, np.sqrt(acc / k)


def detrend(sig):
    """去线性趋势。

    热漂移与 1/f 会把低频段抬高，抬高的是**本底**而不是峰 → 使 SNR 被低估。
    本项目传感器挂排线上，缓慢摆动是常态，所以这一步不是可选项。
    """
    sig = np.asarray(sig, dtype=np.float64)
    n = len(sig)
    x = np.arange(n, dtype=np.float64)
    mx = x.mean()
    my = sig.mean()
    sxx = float(((x - mx) ** 2).sum())
    if sxx == 0.0:
        return sig - my
    slope = float(((x - mx) * (sig - my)).sum()) / sxx
    return sig - (my + slope * (x - mx))


# ---------------------------------------------------------------------------
# 峰与本底
# ---------------------------------------------------------------------------
def peak_in_band(freqs, amp, band, min_hz: float = FLOOR_MIN_HZ):
    """band=(lo,hi) 内的最大峰 → (peak_hz, peak_amp)；带内没频点则 None。"""
    lo = max(float(band[0]), min_hz)
    hi = float(band[1])
    m = (freqs >= lo) & (freqs <= hi)
    if not m.any():
        return None
    f = freqs[m]
    a = amp[m]
    i = int(np.argmax(a))
    return float(f[i]), float(a[i])


def noise_floor(freqs, amp, peak_hz: float, notch_hz: float = NOTCH_HZ,
                min_hz: float = FLOOR_MIN_HZ):
    """本底 = 全带中位数（挖掉峰 ±notch）。返回 (floor, 用到的频点数)。"""
    keep = (freqs >= min_hz) & (np.abs(freqs - peak_hz) > notch_hz)
    if int(keep.sum()) < 8:               # 挖太狠就放宽，别拿个位数当"中位数"
        keep = freqs >= min_hz
    return float(np.median(amp[keep])), int(keep.sum())


def snr_db(peak_amp: float, floor: float) -> float:
    """峰对本底的 dB 比。"""
    if floor <= 0.0:
        return float("inf")
    return 20.0 * np.log10(max(peak_amp, 1e-30) / floor)


def null_bound_db(segs: int) -> float:
    """**纯噪声**能刷出的 SNR 上界（dB）—— 用来判断门限本身可不可信。

    🔴 这是本工具最容易被忽略的一环：门限若落在噪声可达范围内，判据就是假的。
    实测（`selftest.py` 的 `t_blade_peak()` 复现，每档 30 次试验取最大值）::

         1.0 s /  1 段 → 纯噪声 SNR 最高 11.6 dB   ← 12 dB 门限几乎被噪声够到
         3.0 s /  4 段 → 纯噪声 SNR 最高  6.2 dB
        10.0 s / 18 段 → 纯噪声 SNR 最高  3.2 dB

    机理：K 段平均后每个谱线的相对起伏 ≈ 2.8/√K，全带取最大值 ≈ 中位数 ×
    (1 + 2.8/√K)；K=1 时谱线服从 Rayleigh，实测上界 11.6 dB，取 13.0 留余量。
    🔴 **系数 2.8 是按上表实测标定的**（不是理论推导），比实测值高 ≈1.0–1.4 dB，
    所以它是"不会低估"的保守界。改这里的常数前请先跑 `selftest.py`。
    """
    if segs <= 1:
        return 13.0
    return max(3.0, 20.0 * math.log10(1.0 + 2.8 / math.sqrt(float(segs))))


def suggest_seconds(min_snr: float, nperseg: int = DEFAULT_NPERSEG,
                    overlap: float = DEFAULT_OVERLAP,
                    rate: float = RATE_FALLBACK) -> float:
    """要让"纯噪声上界"比门限低 6 dB，至少要采多少秒（向上取整到整秒）。

    由 `null_bound_db` 反解 K，再换算成点数/秒数。给用户一个**可执行的数字**，
    而不是一句"多采点"。
    """
    target = max(1.0, float(min_snr) - 6.0)
    r = 10.0 ** (target / 20.0) - 1.0
    if r <= 0.0:
        k = 1
    else:
        k = int(math.ceil((2.8 / r) ** 2))
    k = max(1, k)
    step = max(1, int(nperseg * (1.0 - overlap)))
    n = nperseg + (k - 1) * step
    return math.ceil(n / float(rate))


def fold_range(band, rate: float):
    """带顶超过 Nyquist 时，给出**可能混叠进来**的报告频率区间。

    真实频率 f > Nyquist 会折到 ``|f - k·fs|``；本项目只关心折一次的情形，
    即真实值在 ``(Nyquist, fs - band_lo]`` 的那些会落进
    ``[fs - band_hi, Nyquist]``。落在这个区间的峰，**位置是假的**。
    返回 (lo, hi) 或 None。
    """
    nyq = rate / 2.0
    hi = float(band[1])
    if hi <= nyq:
        return None
    lo = max(0.0, rate - hi)
    if lo >= nyq:
        return None
    return (lo, nyq)


def top_peaks(freqs, amp, n: int = 5, min_hz: float = FLOOR_MIN_HZ):
    """全谱前 n 个局部极大（按幅值降序）→ [(hz, amp), ...]。

    用途：**峰跑到预期带之外**时（转速跟估算不一样）仍能一眼看出真峰在哪。
    """
    idx = np.where(freqs >= min_hz)[0]
    if len(idx) < 3:
        return []
    f = freqs[idx]
    a = amp[idx]
    local = np.zeros(len(a), dtype=bool)
    local[1:-1] = (a[1:-1] > a[:-2]) & (a[1:-1] >= a[2:])
    cand = np.where(local)[0]
    if len(cand) == 0:
        cand = np.argsort(a)[::-1]
    else:
        cand = cand[np.argsort(a[cand])[::-1]]
    return [(float(f[i]), float(a[i])) for i in cand[:n]]


def nearest_harmonic(ratio: float, tol: float = HARMONIC_TOL,
                     orders=HARMONIC_ORDERS):
    """比值最接近哪个谐波阶次？落不进任何一阶就返回 None。

    `tol` 是**相对**容差（0.06 = ±6%），因为 273 Hz 与 410 Hz 的绝对偏差
    差着一个数量级，只有相对判据才同时管得住低阶与高阶。
    """
    if ratio is None or ratio <= 0.0:
        return None
    best = None
    for n in orders:
        if abs(ratio - n) <= n * float(tol):
            if best is None or abs(ratio - n) < abs(ratio - best):
                best = n
    return best


def blade_band_for(rot_hz: float, multiple: float = DEFAULT_BLADE_MULTIPLE,
                   tol: float = 0.12):
    """由实测 1× 反推该用的叶频搜索带（±12%），用于"带开错了"时给可执行建议。"""
    if rot_hz is None or rot_hz <= 0.0:
        return None
    fc = float(multiple) * float(rot_hz)
    return (fc * (1.0 - float(tol)), fc * (1.0 + float(tol)))


def band_notes(band, rate: float):
    """搜索带本身的配置性警告（Nyquist 折叠 / 器件带宽）。"""
    lo, hi = float(band[0]), float(band[1])
    nyq = rate / 2.0
    out = []
    if lo >= nyq:
        out.append("整条带 [%.0f, %.0f] Hz 都在 Nyquist（%.0f Hz）之外 —— "
                   "这一带看到的频率是**混叠镜像**，位置是假的" % (lo, hi, nyq))
    elif hi > nyq:
        fr = fold_range(band, rate)
        msg = ("带顶 %.0f Hz 超过 Nyquist（%.0f Hz），超出部分会折回来" % (hi, nyq))
        if fr:
            msg += ("：本配置下落在 %.0f–%.0f Hz 的峰**分不出**真实频率是 f 还是"
                    "「采样率 − f」（即 %.0f − f），结论里会点出这个歧义"
                    % (fr[0], fr[1], rate))
        out.append(msg)
    if hi > DEVICE_BW_HZ:
        out.append("带顶 %.0f Hz 超过器件加速度带宽 %.0f Hz（R17）—— "
                   "这一带的幅值已被带宽滚降压过，峰可能被压低甚至埋掉"
                   % (hi, DEVICE_BW_HZ))
    return out


def peak_notes(peak_hz: float, rate: float):
    """某个具体峰的位置性警告。"""
    nyq = rate / 2.0
    out = []
    if peak_hz > 0.9 * nyq:
        out.append("峰 %.1f Hz 贴近 Nyquist（%.0f Hz）→ 可能是混叠镜像"
                   % (peak_hz, nyq))
    if peak_hz > DEVICE_BW_HZ:
        out.append("峰 %.1f Hz 超出器件带宽 %.0f Hz → 幅值已被压过，真实幅值更大"
                   % (peak_hz, DEVICE_BW_HZ))
    return out


# ---------------------------------------------------------------------------
# 逐轴分析
# ---------------------------------------------------------------------------
def analyse(samples, rate: float, *, blade_band=DEFAULT_BLADE_BAND,
            rot_band=DEFAULT_ROT_BAND, nperseg: int = DEFAULT_NPERSEG,
            overlap: float = DEFAULT_OVERLAP, notch_hz: float = NOTCH_HZ,
            min_hz: float = FLOOR_MIN_HZ, min_snr: float = DEFAULT_MIN_SNR_DB,
            min_amp: float = DEFAULT_MIN_AMP_G, floor_floor: float | None = None,
            blade_multiple: float = DEFAULT_BLADE_MULTIPLE,
            harmonic_guard: bool = True):
    """三轴各算一次 → 结果字典。

    `floor_floor`（可选的**全局本底下限**）用于跨轴比较：真实系统里各轴本底
    应当同量级，若某轴本底异常低（纯数值噪声），会把那一轴的 SNR 抬得虚高。
    给了它就把各轴本底统一抬到该值，避免"最安静的轴"骗过判据。

    🔴 `blade_multiple`（默认 3，本项目 3 叶）用于**谐波自洽校验**：叶频带取的
    是带内最大幅值，`2×` 常常比 `3×` 还强 ⇒ 不校验就会把 `2×` 当叶频报"通过"。
    """
    samples = np.asarray(samples, dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError("samples 应为 (N,3)，收到 %r" % (samples.shape,))
    n = len(samples)
    if n < 64:
        raise ValueError("样本太少（%d 点），判不了（至少 64 点）" % n)

    segs = segment_count(n, nperseg, overlap)
    fold = fold_range(blade_band, rate)
    want_order = int(round(float(blade_multiple)))
    freqs = None
    spectra = []
    axes = []

    for k in range(3):
        sig = detrend(samples[:, k])
        f, a = amplitude_spectrum(sig, rate, nperseg, overlap)
        if freqs is None:
            freqs = f
        spectra.append(a)

        row = {"axis": AXIS_NAMES[k], "index": k,
               "ratio": None, "harmonic": None, "blade_harmonic": False,
               "blade_inconsistent": False, "blade_expected_hz": None,
               "blade_thresh_ok": False}
        for tag, band in (("blade", blade_band), ("rot", rot_band)):
            pk = peak_in_band(f, a, band, min_hz=min_hz)
            if pk is None:
                row[tag + "_hz"] = None
                row[tag + "_amp"] = None
                row[tag + "_floor"] = None
                row[tag + "_snr"] = None
                row[tag + "_ok"] = False
                continue
            p_hz, p_amp = pk
            fl, _ = noise_floor(f, a, p_hz, notch_hz=notch_hz, min_hz=min_hz)
            if floor_floor is not None:
                fl = max(fl, float(floor_floor))
            row[tag + "_hz"] = p_hz
            row[tag + "_amp"] = p_amp
            row[tag + "_floor"] = fl
            row[tag + "_snr"] = snr_db(p_amp, fl)
            row[tag + "_ok"] = bool(row[tag + "_snr"] >= min_snr
                                    and p_amp >= min_amp)

        # 🔴 谐波自洽：带内那个"叶频峰"到底是第几阶谐波？
        # 只要转频带里有峰（哪怕它其实是噪声频点），比值对不上 K 就**否决**：
        # 宁可判"不通过"让人去查搜索带，也不能拿别的谐波冒充叶频放行。
        row["blade_thresh_ok"] = row["blade_ok"]
        if row["blade_hz"] and row["rot_hz"]:
            row["blade_expected_hz"] = float(blade_multiple) * row["rot_hz"]
            row["ratio"] = row["blade_hz"] / row["rot_hz"]
            row["harmonic"] = nearest_harmonic(row["ratio"])
            if harmonic_guard and row["harmonic"] != want_order:
                row["blade_ok"] = False
                if row["harmonic"] is not None:
                    row["blade_harmonic"] = True
                else:
                    row["blade_inconsistent"] = True
        row["notes"] = peak_notes(row["blade_hz"] or 0.0, rate)
        row["blade_alias"] = bool(
            fold and row["blade_hz"] is not None
            and fold[0] <= row["blade_hz"] <= fold[1])
        axes.append(row)

    # 最强轴 = 叶频带 SNR 最高的那轴（电机振动方向取决于安装姿态）
    ranked = sorted(axes, key=lambda r: (r["blade_snr"] is not None,
                                         r["blade_snr"] or -1e9), reverse=True)
    blade_ok = any(r["blade_ok"] for r in axes)
    rot_ok = any(r["rot_ok"] for r in axes)
    # 用"过了转频判据的最强轴"的 1× 反推建议叶频带（谐波校验失败时给可执行建议）
    # 用"确实测到了 1× 且过了判据"的轴的 1× 反推建议叶频带。
    # 🔴 必须带 rot_ok 这一条：全零/死掉的轴频谱恒为 0 → 本底 0 → SNR = inf，
    #    会把"最安静"的轴排到第一，反推出一个凭空的建议带（实测踩过）。
    rot_rows = sorted([r for r in axes if r["rot_hz"] and r["rot_ok"]],
                      key=lambda r: r["rot_snr"] or -1e9, reverse=True)
    suggest = blade_band_for(rot_rows[0]["rot_hz"], blade_multiple) if rot_rows else None

    return {
        "rate": rate, "n": n, "segs": segs, "freqs": freqs, "spectra": spectra,
        "axes": axes, "best_axis": ranked[0]["axis"] if ranked else None,
        "blade_ok": blade_ok, "rot_ok": rot_ok,
        "blade_band": tuple(blade_band), "rot_band": tuple(rot_band),
        "min_snr": min_snr, "min_amp": min_amp,
        "nperseg": int(nperseg), "overlap": float(overlap),
        "null_db": null_bound_db(segs), "fold": fold,
        "blade_multiple": float(blade_multiple),
        "harmonic_guard": bool(harmonic_guard),
        "blade_suggest_band": suggest,
        "any_ratio_veto": any(r["blade_harmonic"] or r["blade_inconsistent"]
                              for r in axes),
        "top": top_peaks(freqs, spectra[ranked[0]["index"]] if ranked else spectra[0],
                         n=5, min_hz=min_hz),
        "notes": band_notes(blade_band, rate) + band_notes(rot_band, rate),
    }


# ---------------------------------------------------------------------------
# 读 CSV
# ---------------------------------------------------------------------------
def load_csv(path: str, rate: float | None = None):
    """读 `parse_frames.py --csv` 写的连续序列 → (rate, samples(N,3), rate_src)。

    采样率优先**从 `t_s` 列反推**（`1/median(diff)`），因为那是实测值；
    `--rate` 只在该列缺失/不可用时兜底，且两者差 >2% 时要在报告里点出来。
    """
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    names = list(d.dtype.names or ())
    if not names:
        raise ValueError("CSV 没有表头（应形如 t_s,ax_g,ay_g,az_g）")
    need = ["ax_g", "ay_g", "az_g"]
    missing = [x for x in need if x not in names]
    if missing:
        raise ValueError("CSV 缺列 %s（现有列：%s）" % (missing, names))

    cols = [np.atleast_1d(d[x]).astype(np.float64) for x in need]
    samples = np.column_stack(cols)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError("CSV 列形状异常：%r" % (samples.shape,))

    rate_src = None
    r = None
    if "t_s" in names and len(samples) >= 3:
        t = np.atleast_1d(d["t_s"]).astype(np.float64)
        dt = np.diff(t)
        dt = dt[dt > 0]
        if len(dt) >= 2:
            r = 1.0 / float(np.median(dt))
            rate_src = "由 t_s 列反推"
    if r is None:
        r = float(rate) if rate else RATE_FALLBACK
        rate_src = "--rate 参数" if rate else "兜底默认 %g Hz（CSV 无可用的 t_s 列）" % RATE_FALLBACK
    return r, samples, rate_src


# ---------------------------------------------------------------------------
# 出图
# ---------------------------------------------------------------------------
def write_plot(path: str, r: dict, *, show: bool = False) -> None:
    """三轴幅度谱（对数纵轴）+ 两个搜索带阴影 + 标出峰。**英文标签，避免缺字变方块。**"""
    if not show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    freqs = r["freqs"]
    nyq = r["rate"] / 2.0
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True,
                             constrained_layout=True)
    for k, ax in enumerate(axes):
        row = r["axes"][k]
        ax.semilogy(freqs, np.maximum(r["spectra"][k], 1e-9), lw=0.7,
                    color="#1f77b4")
        ax.axvspan(r["rot_band"][0], r["rot_band"][1], color="#2ca02c",
                   alpha=0.10, lw=0)
        ax.axvspan(r["blade_band"][0], r["blade_band"][1], color="#d62728",
                   alpha=0.10, lw=0)
        if row["blade_hz"]:
            ax.axvline(row["blade_hz"], color="#d62728", ls="--", lw=0.9)
            ax.annotate("%s blade %.1f Hz (%.1f dB)"
                        % (row["axis"], row["blade_hz"], row["blade_snr"] or 0.0),
                        xy=(row["blade_hz"], row["blade_amp"] or 1e-6),
                        xytext=(6, -2), textcoords="offset points",
                        color="#d62728", fontsize=8)
        ax.axvline(DEVICE_BW_HZ, color="k", ls=":", lw=0.6, alpha=0.6)
        ax.grid(alpha=0.3, which="both")
        ax.set_ylabel("%s [g]" % row["axis"])
    axes[0].set_title("blade-pass / rotation band check @ %.1f Hz  "
                      "(nperseg=%d, %d segments; dotted = %.0f Hz device BW)"
                      % (r["rate"], r["nperseg"], r["segs"], DEVICE_BW_HZ))
    axes[-1].set_xlabel("frequency [Hz]")
    axes[-1].set_xlim(0, min(nyq, max(r["blade_band"][1] * 1.15, 700.0)))

    if show:
        plt.show()
    if path:
        fig.savefig(path, dpi=130)
        print("[i] 图已保存到 %s" % path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
def print_report(r: dict, src: str, rate_src: str) -> None:
    print("\n" + "-" * 68)
    print("数据      : %s" % src)
    print("点数/时长 : %d 点（%.3f 秒）" % (r["n"], r["n"] / r["rate"]))
    print("采样率    : %.2f Hz（%s）" % (r["rate"], rate_src))
    print("频率分辨  : %.3f Hz（nperseg=%d，重叠 %g，共 %d 段）"
          % (r["rate"] / r["nperseg"], r["nperseg"], r["overlap"], r["segs"]))
    print("搜索带    : 叶频 %.0f–%.0f Hz ｜ 转频 %.0f–%.0f Hz"
          % (r["blade_band"] + r["rot_band"]))
    print("判据      : SNR ≥ %.1f dB  且  峰幅值 ≥ %.4f g"
          % (r["min_snr"], r["min_amp"]))

    # 🔴 门限本身可不可信：纯噪声能刷到多高？先算这个，再谈结论。
    margin = r["min_snr"] - r["null_db"]
    print("噪声上界  : 纯噪声时 SNR 最高约 %.1f dB（%d 段）→ 门限余量 %.1f dB"
          % (r["null_db"], r["segs"], margin))
    if margin < 3.0:
        need = suggest_seconds(r["min_snr"], r["nperseg"], r["overlap"], r["rate"])
        print("  🔴 余量不足 3 dB —— 门限落在噪声可达范围内，这个判定不可信！")
        print("     把数据采到 ≥%.1f 秒（@%.0f Hz，nperseg=%d），或用 --min-snr 提高门限。"
              % (need, r["rate"], r["nperseg"]))
    for nt in r["notes"]:
        print("⚠️ %s" % nt)

    print("\n轴   叶频带峰      幅值[g]    本底[g]    SNR[dB]  判定")
    for row in r["axes"]:
        if row["blade_hz"] is None:
            print("  %-4s (带内无频点)" % row["axis"])
            continue
        print("  %-4s %7.1f Hz   %.5f   %.5f   %6.1f   %s"
              % (row["axis"], row["blade_hz"], row["blade_amp"],
                 row["blade_floor"], row["blade_snr"],
                 "OK" if row["blade_ok"] else "--"))

    print("\n全谱前 5 个峰（不受搜索带限制 —— 看峰到底跑哪去了）")
    if not r["top"]:
        print("  （没找到局部极大）")
    for i, (hz, amp) in enumerate(r["top"], 1):
        if r["blade_band"][0] <= hz <= r["blade_band"][1]:
            tag = "叶频带"
        elif r["rot_band"][0] <= hz <= r["rot_band"][1]:
            tag = "转频带"
        else:
            tag = "带外"
        extra = "  ← 上/下调 1/3 即叶频" if tag == "转频带" else ""
        print("  %d) %7.1f Hz  %.5f g   [%s]%s" % (i, hz, amp, tag, extra))

    # 🔴 谐波自洽：带内那个峰到底是不是"K × 转频"？不是的话上面的判定不算数。
    print("\n谐波自洽（叶频应 = %.2f × 转频）" % r["blade_multiple"])
    for row in r["axes"]:
        if not row["rot_hz"]:
            print("  %-4s 转频带无峰 → 校验不了" % row["axis"])
        elif not row["blade_hz"]:
            print("  %-4s 1× = %6.1f Hz → 期望叶频 %6.1f Hz，但叶频带内没有峰"
                  % (row["axis"], row["rot_hz"], row["blade_expected_hz"] or 0.0))
        else:
            want = int(round(float(r["blade_multiple"])))
            if row["harmonic"] == want:
                mark, what = "✅", "与 %.0f× 一致" % r["blade_multiple"]
            elif row["blade_harmonic"]:
                mark, what = "❌", "其实是 %d× 转频谐波，**不是叶频**" % row["harmonic"]
            else:
                mark, what = "❌", ("与 %.0f× 不符、也不是低次谐波 → 多半是环境峰"
                                    % r["blade_multiple"])
            print("  %-4s 1× = %6.1f Hz → 期望叶频 %6.1f Hz ｜ 带内峰 %6.1f Hz，"
                  "比值 %.2f %s %s"
                  % (row["axis"], row["rot_hz"], row["blade_expected_hz"] or 0.0,
                     row["blade_hz"], row["ratio"] or 0.0, mark, what))
            if mark == "❌":
                print("        → 这个峰 **不算通过**（与 1× 不自洽，别拿它当叶频）")
    if r["blade_suggest_band"]:
        print("  建议叶频带（由 1× 反推，±12%%）： %.0f %.0f Hz"
              % r["blade_suggest_band"])

    if r["blade_ok"]:
        best = [x for x in r["axes"] if x["blade_ok"]]
        top = max(best, key=lambda x: x["blade_snr"])
        print("\n最强轴    : %s（叶频峰 %.1f Hz，SNR %.1f dB，幅值 %.5f g）"
              % (top["axis"], top["blade_hz"], top["blade_snr"], top["blade_amp"]))
    print("-" * 68)


def verdict(r: dict) -> int:
    """打印结论并给出退出码。"""
    if r["blade_ok"]:
        best = [x for x in r["axes"] if x["blade_ok"]]
        top = max(best, key=lambda x: x["blade_snr"])
        print(">>> 结论：✅ 通过 —— %s 轴叶频峰 %.1f Hz，SNR %.1f dB，明显高于本底"
              % (top["axis"], top["blade_hz"], top["blade_snr"]))
        if top["blade_alias"]:
            print("    ⚠️ 但这个峰落在可能混叠的区间：真实频率也可能是 %.0f − %.1f "
                  "= %.1f Hz。" % (r["rate"], top["blade_hz"],
                                   r["rate"] - top["blade_hz"]))
            print("       判据本身过了（能量确实在），但**频率位置别写进结论**。")
            print("       要拿到真实叶频：PWM 限速把叶频压到 Nyquist 以下再测一次。")
        return 0
    print(">>> 结论：❌ 不通过 —— 叶频带没有明显高于本底的峰")
    if r.get("any_ratio_veto"):
        bad = [x for x in r["axes"] if x["blade_harmonic"] or x["blade_inconsistent"]]
        b = max(bad, key=lambda x: x["blade_snr"] or -1e9)
        if b["blade_harmonic"]:
            print("    🔴 更要紧的是：叶频带里那个最强的峰 %.1f Hz 是 **%d× 转频谐波**，"
                  % (b["blade_hz"], b["harmonic"]))
        else:
            print("    🔴 更要紧的是：叶频带里那个最强的峰 %.1f Hz **与 1× 不自洽**"
                  "（比值 %.2f，既不是 %.0f×、也不是低次谐波），"
                  % (b["blade_hz"], b["ratio"] or 0.0, r["blade_multiple"]))
            print("       它多半是别的谐波或者环境峰 —— 别把它当成叶频。")
        print("       （1× = %.1f Hz，是在 --rot-band %.0f–%.0f 里量到的）"
              % (b["rot_hz"], r["rot_band"][0], r["rot_band"][1]))
        top = r["top"][0] if r["top"] else None
        if top and not (r["rot_band"][0] <= top[0] <= r["rot_band"][1]):
            print("       ⚠️ 全谱最强峰是 %.1f Hz（%.5f g），**不在转频带内** —— "
                  "它很可能才是真正的 1×。" % top)
            print("          → 把 --rot-band 改成覆盖它再跑一次。")
        if r["blade_suggest_band"]:
            lo, hi = r["blade_suggest_band"]
            print("       → 按实测 1× 反推，叶频应在 %.0f Hz 附近 → "
                  "改成 --blade-band %.0f %.0f 重跑。"
                  % (r["blade_multiple"] * (b["rot_hz"] or 0.0), lo, hi))
        print("       ⚠️ 叶频带下沿必须高于 2× 转频（%.0f Hz），否则 2× 总会顶替叶频。"
              % (2.0 * (b["rot_hz"] or 0.0)))
    elif r["rot_ok"]:
        best = [x for x in r["axes"] if x["rot_ok"]]
        top = max(best, key=lambda x: x["rot_snr"])
        print("    但 %s 轴的 1× 转频可见（%.1f Hz，SNR %.1f dB）"
              % (top["axis"], top["rot_hz"], top["rot_snr"]))
        print("    → 按 R17 的取舍顺序：① 用 1× 转频当特征（它更稳）；")
        print("      ② 或 PWM 限速把转频压到 80 Hz 内（叶频 ≤240 Hz，"
              "同时落进 260 Hz 带宽与 500 Hz Nyquist，最干净）。")
    else:
        print("    转频带也没峰 → 先确认电机真的在转、MPU6050 与电机刚性固定在一起，")
        print("    再确认没有把 --rate 写错（本工具会从 t_s 列反推，一般不用管）。")
    print("    取舍顺序见 RISKS.md R17；不可见时也可改用对绝对幅值不敏感的特征")
    print("    （频谱峭度 / 谱峰比）或记为已知局限。")
    return 1


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="check_blade_peak.py",
        description="R17 频谱门禁：确认叶频/转频峰真的明显高于本底",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例:
  python parse_frames.py --port COM4 --duration 10 --csv d.csv
  python check_blade_peak.py --csv d.csv --plot peak.png
  python check_blade_peak.py --csv d.csv --min-snr 8 --blade-band 280 520
""")
    p.add_argument("--csv", required=True, help="parse_frames.py --csv 写出的连续序列")
    p.add_argument("--rate", type=float,
                   help="采样率兜底值（默认从 CSV 的 t_s 列反推，推不出才用它）")
    p.add_argument("--nperseg", type=int, default=DEFAULT_NPERSEG,
                   help="分段长度，默认 1024（Δf≈1 Hz）")
    p.add_argument("--min-snr", type=float, default=DEFAULT_MIN_SNR_DB,
                   help="通过门限 dB，默认 12")
    p.add_argument("--min-amp", type=float, default=DEFAULT_MIN_AMP_G,
                   help="峰幅值绝对下限 g，默认 0.002")
    p.add_argument("--blade-band", type=float, nargs=2,
                   default=None, metavar=("LO", "HI"),
                   help="叶频搜索带 Hz。缺省：给了 `--rot-band` 就按 "
                        "%g×[rot]×[0.85,1.15] **自动推导**，否则用计划值 %g %g"
                        "（🔴 自动推导时下沿仍可能压到 2× 转频，注意看提示）"
                        % (DEFAULT_BLADE_MULTIPLE, DEFAULT_BLADE_BAND[0], DEFAULT_BLADE_BAND[1]))
    p.add_argument("--rot-band", type=float, nargs=2,
                   default=None, metavar=("LO", "HI"),
                   help=("转频搜索带 Hz，默认 %g %g。🔴 本机实测工作点会在 **40–87 Hz** "
                         "之间漂（电池/负载不同），默认带是按旧工作点定的 ⇒ 跑完**必须看"
                         "『全谱前 5 个峰』那一段**：若最强峰在带外，它很可能才是真正的 1×，"
                         "按提示改这一带重跑（实例：默认带会把 218.8 Hz 的 3× 误当 1×，"
                         "结论从『通过』翻成『不通过』）" % DEFAULT_ROT_BAND))
    p.add_argument("--blade-multiple", type=float,
                   default=DEFAULT_BLADE_MULTIPLE, metavar="K",
                   help="叶频 = K × 转频，默认 %g（3 叶桨）；用于谐波自洽校验"
                        % DEFAULT_BLADE_MULTIPLE)
    p.add_argument("--no-harmonic-guard", dest="harmonic_guard",
                   action="store_false",
                   help="关掉谐波自洽否决权（只在「1× 确实在器件上看不到、"
                        "但叶频确实存在」时用；默认开着）")
    p.add_argument("--plot", metavar="PATH", nargs="?", const="blade_peak.png",
                   help="画三轴频谱图（默认存 blade_peak.png）")
    p.add_argument("--show", action="store_true", help="弹窗显示图")
    return p


def main(argv=None) -> int:
    args = build_argparser().parse_args(argv)

    # 🔴 2026-10-17 修正：`--blade-band` 老默认是**计划值** 300–660 Hz
    #    （3 叶 × 6000–13000 RPM）。真机实测叶频只有 **155–195 Hz**，于是门禁去
    #    300–660 Hz 里找峰，抓到 325 Hz 的环境峰，最后给出一句
    #    「❌ 叶频带没有明显高于本底的峰」—— 而 160 Hz 处的叶频峰明明白白就在那儿。
    #    这不是算法错，是**默认带和现实脱节**，报错文案却像在否定数据本身。
    #    现在：只要给了 `--rot-band`，就按 K×[rot]×(1±15%) 自动推开搜索带；
    #    没给才退回计划值（并明说一句，别让人以为这是实测值）。
    rot_explicit = args.rot_band is not None
    if args.rot_band is None:
        args.rot_band = list(DEFAULT_ROT_BAND)
    if args.blade_band is None:
        if rot_explicit:
            k = args.blade_multiple
            lo, hi = args.rot_band
            args.blade_band = [k * lo * 0.85, k * hi * 1.15]
            print("[i] 叶频带未给 ⇒ 由 --rot-band 自动推导：%g × [%g, %g] × [0.85, 1.15] "
                  "= [%.0f, %.0f] Hz" % (k, lo, hi, args.blade_band[0], args.blade_band[1]))
        else:
            args.blade_band = list(DEFAULT_BLADE_BAND)
            print("[!] 叶频带用**计划值** %g–%g Hz（不是实测值）。"
                  % (DEFAULT_BLADE_BAND[0], DEFAULT_BLADE_BAND[1]))
            print("    真机叶频不在这个范围时会给出误导性的「❌ 没有峰」"
                  "—— 请改用 --rot-band 让它自动推导，或直接 --blade-band 指定。")

    print("=" * 68)
    print("check_blade_peak.py · R17 频谱门禁")
    print("=" * 68)

    try:
        rate, samples, rate_src = load_csv(args.csv, args.rate)
    except Exception as e:                                    # noqa: BLE001
        print("[ERROR] 读不了 %s：%s" % (args.csv, e))
        return 2

    if args.rate and rate_src == "由 t_s 列反推":
        if abs(rate / args.rate - 1.0) > 0.02:
            print("⚠️ --rate=%g 与 t_s 反推的 %.2f Hz 差 %.1f%%，以实测值为准"
                  % (args.rate, rate, abs(rate / args.rate - 1.0) * 100.0))

    try:
        r = analyse(samples, rate, blade_band=tuple(args.blade_band),
                    rot_band=tuple(args.rot_band), nperseg=args.nperseg,
                    min_snr=args.min_snr, min_amp=args.min_amp,
                    blade_multiple=args.blade_multiple,
                    harmonic_guard=args.harmonic_guard)
    except Exception as e:                                    # noqa: BLE001
        print("[ERROR] 算不了：%s" % e)
        return 2

    print_report(r, args.csv, rate_src)

    if args.plot or args.show:
        write_plot(args.plot or "blade_peak.png", r, show=args.show)

    return verdict(r)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[i] 已中断（Ctrl-C）")
        sys.exit(130)
