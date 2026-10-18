# -*- coding: utf-8 -*-
"""把 parse_frames 产出的连续序列 CSV，转成 NanoEdge AI Studio 能吃的"每行一个信号"CSV。

为什么需要它：`parse_frames.py --csv` 写出的是**连续时间序列**（一行一个采样点，
列 = t, ax, ay, az），而 Studio 的异常检测导入要求**一行一个信号（窗口）**，
列 = 该窗口里所有采样点摊平。两个东西维度不同，不能直接喂。

用法（典型）:
    # 1) 看一眼会切出多少窗、每窗什么形态
    python tools/to_neai_csv.py --in seg01.csv seg02.csv --dry-run
    # 2) 真正转换（默认 128 点 / 不重叠 / 按轴拼接 / 单位 g）
    python tools/to_neai_csv.py --in ds1018/seg*.csv --out ds1018/neai/train.csv
    # 3) 转完自检（逐元素比对源数据重新切窗的结果）
    python tools/to_neai_csv.py --in ds1018/seg*.csv --out ... --verify

🔴 三个必须在 Studio 里核对的约定（不同版本/不同工程类型可能不一样）：
    ① **窗口长度**：默认 128（`DECISIONS` D19 的起步值）。Studio 里的
       `DATA_INPUT_USER` 必须与这里的 `--window` **完全一致**，否则上板输入错位。
    ② **多轴排列**：`--layout axis-major`（默认，X 全 128 点 → Y → Z）还是
       `--layout point-major`（x0,y0,z0, x1,y1,z1, …）。Studio 的导入界面/生成的头文件
       注释会说明它要哪一种；固件侧 `APP_NEAI_AXIS_MAJOR` 必须与**同一选择**对齐（R10.5 第 3 步）。
    ③ **单位**：`--unit g`（默认，浮点）还是 `--unit lsb`（原始 int16 计数）。
       本项目传感器 ±2 g / 16384 LSB/g。**两种都能训**，但训练与推理必须同一口径。
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

FS_DEFAULT = 1000.0
LSB_PER_G = 16384.0


def load_csv(path):
    """读 parse_frames 的连续序列 CSV：t, ax, ay, az（单位 g）。"""
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    if d.ndim != 2 or d.shape[1] < 4:
        raise SystemExit("🔴 %s 不是 parse_frames 的连续序列 CSV（需要 t,ax,ay,az 四列）" % path)
    t, x = d[:, 0], d[:, 1:4]
    dt = np.diff(t)
    fs = float(1.0 / np.median(dt)) if len(dt) else FS_DEFAULT
    return t, x, fs


def cut(t, x, win, hop):
    """切窗；返回 [n_win, win, 3] 与每窗起始时间（便于回溯到源文件）。"""
    n = len(x)
    starts = list(range(0, n - win + 1, hop))
    if not starts:
        return np.zeros((0, win, 3)), []
    seg = np.stack([x[s:s + win] for s in starts])
    return seg, [float(t[s]) for s in starts]


def flatten(seg, layout):
    """把 [n, win, 3] 摊平成 Studio 的一行。"""
    if layout == "axis-major":            # X 全窗 → Y 全窗 → Z 全窗
        return np.concatenate([seg[:, :, k] for k in range(3)], axis=1)
    elif layout == "point-major":         # x0,y0,z0, x1,y1,z1, ...
        return seg.reshape(seg.shape[0], -1)
    raise SystemExit("未知 layout: %s" % layout)


def main():
    ap = argparse.ArgumentParser(description="连续序列 CSV → NanoEdge Studio 的『每行一个信号』CSV")
    ap.add_argument("--in", dest="inputs", nargs="+", required=True,
                    help="一个或多个 parse_frames 产出的 CSV（支持通配符）")
    ap.add_argument("--out", default=None, help="输出 CSV（缺省时只做 --dry-run 统计）")
    ap.add_argument("--window", type=int, default=128, help="每窗点数，默认 128")
    ap.add_argument("--hop", type=int, default=None, help="窗步长，默认 = --window（不重叠）")
    ap.add_argument("--layout", choices=["axis-major", "point-major"], default="axis-major")
    ap.add_argument("--unit", choices=["g", "lsb"], default="g")
    ap.add_argument("--digits", type=int, default=6, help="g 单位的小数位，默认 6")
    ap.add_argument("--keep-dc", action="store_true",
                    help="保留每轴的直流分量（默认**去掉**，见下方说明）")
    ap.add_argument("--header", action="store_true", help="写一行列名（Studio 通常要 headerless）")
    ap.add_argument("--dry-run", action="store_true", help="只统计，不写文件")
    ap.add_argument("--verify", action="store_true", help="写完读回，逐元素比对")
    ap.add_argument("--seed-report", default=None, help="把统计结果写成 json")
    a = ap.parse_args()

    win = a.window
    hop = a.hop or win
    files = []
    for pat in a.inputs:
        files.extend(sorted(glob.glob(pat)))
    if not files:
        raise SystemExit("没有匹配到输入文件")

    rows, per_file, fs_all, t_span = [], [], [], 0.0
    for f in files:
        t, x, fs = load_csv(f)
        seg, starts = cut(t, x, win, hop)
        if len(seg) == 0:
            print("[跳过] %s：只有 %d 点，不够一个 %d 点窗" % (os.path.basename(f), len(x), win))
            continue
        rows.append(seg)
        fs_all.append(fs)
        t_span += float(t[-1] - t[0]) if len(t) > 1 else 0.0
        per_file.append({"file": os.path.basename(f), "samples": int(len(x)),
                         "windows": int(len(seg)), "seconds": round(float(t[-1] - t[0]), 3)})
        print("[读入] %-12s %6d 点 / %5.2f s（%.1f Hz）→ %4d 个 %d 点窗"
              % (os.path.basename(f), len(x), t[-1] - t[0], fs, len(seg), win))

    if not rows:
        raise SystemExit("🔴 一个窗都切不出来")

    seg = np.concatenate(rows, axis=0)

    # 🔴 默认去掉每轴每窗的直流分量。
    #    理由（本项目实测）：模块没有贴平，三轴直流是 ax≈0.59 / ay≈0 / az≈0.15 g，
    #    而真正的振动只有 0.003–0.03 g —— **直流比信号大两个数量级，且完全由装夹角度决定**。
    #    不去的后果：模型主要学的是"板子朝哪个方向"，用户把模块转个角度就全判异常；
    #    振动特征反而被淹没。工业振动诊断的标准做法也是先去均值再看谱。
    #    要保留（例如想连"姿态"一起学）用 --keep-dc。
    if not a.keep_dc:
        seg = seg - seg.mean(axis=1, keepdims=True)

    flat = flatten(seg, a.layout)
    if a.unit == "lsb":
        flat = np.rint(flat * LSB_PER_G)

    # 逐窗形态自检：全窗一模一样 = 数据没变化（采样卡死 / 抄了同一段）
    std_per_win = seg.reshape(len(seg), -1).std(axis=1)
    identical = int(np.sum(std_per_win < 1e-12))
    print("-" * 68)
    print("总窗口 %d 个 × %d 列 | 布局 %s | 单位 %s | 步长 %d | 源时长 %.1f s"
          % (len(flat), flat.shape[1], a.layout, a.unit, hop, t_span))
    print("每窗标准差：中位 %.5f / 最小 %.5f g%s"
          % (np.median(std_per_win), std_per_win.min(),
             "" if identical == 0 else "  🔴 有 %d 个窗完全恒定，检查采样链路" % identical))

    report = {
        "inputs": per_file,
        "window": win, "hop": hop, "layout": a.layout, "unit": a.unit,
        "columns": int(flat.shape[1]),
        "windows": int(len(flat)),
        "sample_rate_hz": [round(float(v), 3) for v in fs_all],
        "source_seconds": round(t_span, 3),
        "per_window_std_median_g": float(np.median(std_per_win)),
        "identical_windows": identical,
    }

    if a.dry_run or not a.out:
        if not a.out:
            print("\n（没有 --out，只做统计；加 --out 才会写文件）")
        if a.seed_report:
            with open(a.seed_report, "w", encoding="utf-8") as fp:
                json.dump(report, fp, ensure_ascii=False, indent=2)
        return 0

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fmt = "%." + str(a.digits) + "f" if a.unit == "g" else "%d"
    with open(a.out, "w", encoding="utf-8", newline="") as fp:
        if a.header:
            fp.write(",".join("s%d" % i for i in range(flat.shape[1])) + "\n")
        np.savetxt(fp, flat, delimiter=",", fmt=fmt)
    size = os.path.getsize(a.out)
    print("✅ 已写出 %s（%.1f KB，%d 行 × %d 列）"
          % (a.out, size / 1024.0, flat.shape[0], flat.shape[1]))

    man = a.out + ".manifest.json"
    with open(man, "w", encoding="utf-8") as fp:
        json.dump(report, fp, ensure_ascii=False, indent=2)
    print("   清单 %s（含窗口数/布局/单位/采样率，换库出错时先看它）" % man)

    if a.verify:
        back = np.loadtxt(a.out, delimiter=",", skiprows=1 if a.header else 0, ndmin=2)
        if back.shape != flat.shape:
            raise SystemExit("🔴 读回形状不一致：%s vs %s" % (back.shape, flat.shape))
        err = float(np.max(np.abs(back - flat)))
        tol = 10 ** (-a.digits) if a.unit == "g" else 0.5
        ok = err <= tol * 1.01
        print(("✅ 自检通过：读回 %d×%d，最大偏差 %.3g（容差 %.3g）"
               % (back.shape[0], back.shape[1], err, tol)) if ok
              else ("🔴 自检失败：最大偏差 %.3g > 容差 %.3g" % (err, tol)))
        if not ok:
            return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
