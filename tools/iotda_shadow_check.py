#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""iotda_shadow_check.py — 云端影子 ↔ 串口日志「末值 + 条数」对账（纯标准库）

为什么要有这个工具（2026-10-28）
--------------------------------------------------------------------------
`T10` 上云闭环当时用的判据是 **`puback` 72/72**。但那只能证明一件事：
**平台收到了这些字节**。收下来的内容对不对，它一个字都没说 ——
证据就在 `main.c` 里那段注释（第 136 行）：

    影子停在 `similarity=55`，而板子还在照常发（`puback` 冻结不动）。

也就是说：**payload 里的 `similarity` 就算一直发同一个错值，
`puback` 照样一条不少地回来**。所以"上云闭环"这四个字，缺了这一步就只能算半个。

为什么不做"逐条对账"
--------------------------------------------------------------------------
做过调研，结论是**平台侧不保存全量历史**：
  · 设备影子只保留**每个属性的最新值**（每次上报覆盖）；
  · 历史报文要看「设备消息」，那是另一条链路（控制台查询 / 数据转发到 OBS 或 DB），
    要么手工翻页，要么先配转发规则 —— 都不属于"跑个脚本就能验"。
所以本工具做两件**确定能做对**的事：

  J1  **条数**：串口末行的 `#N`（设备自增上报计数）与同一行的 `(puback=M)` 必须相等；
  J2  **末值**：云端影子的 `similarity` / `verdict` 必须等于串口**最后一条**上报的值。

这两条一旦 PASS，就能说："平台不只是收到了，**收到的和板子认为它发的是同一个东西**"。
比 `puback` 强一层，且不需要改固件、不需要抓包。

影子 JSON 从哪来
--------------------------------------------------------------------------
三个来源都支持，任选：
  1. `--shadow <file>`   控制台「设备详情 → 设备影子」右上角的 JSON，另存为文件；
  2. `stdin`             把上面那段 JSON 直接粘进来：`... --shadow -`；
  3. `--emit-template`   打印一份骨架，照着把值填进去（用于离线演练 / 写自测）。

🔴 平台返回的嵌套层级会随版本变（`shadow[].reported.properties.similarity` 是见过的一种，
    但**不保证**）。所以本脚本**按 key 递归查找**，不写死路径，并把"从哪条路径取到的"
    一并打印出来 —— 路径长得不对，一眼就能看见，而不是静默取到一个同名的无关字段。

用法
--------------------------------------------------------------------------
  # 0) 离线自测（不联网、不需要板子）
  python iotda_shadow_check.py --selftest

  # 1) 只解析日志，看设备侧到底报了什么（不需要影子）
  python iotda_shadow_check.py --log cloud.log

  # 2) 完整对账（板子刚断线时跑，趁影子还是最后一次上报的值）
  python iotda_shadow_check.py --log cloud.log --shadow shadow.json

  # 3) 从串口实时录一段再对账（需要 pyserial；板子要在跑 mode 3 + 上云）
  python iotda_shadow_check.py --serial COM5 --seconds 120 --shadow shadow.json

⚠️ 时序很重要：**先停设备、再看影子**。设备还在发的时候影子一直在变，
   对上的可能只是"碰巧撞上"，而不是"最后那条对上"。

退出码
--------------------------------------------------------------------------
  0 = 全部判据 PASS        1 = 有 FAIL        2 = 输入有问题（没日志 / 日志里没有上报行）
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
import pathlib

# 固件 main.c: App_CloudReport() 里 printf 的那一行，格式必须与之逐字对应：
#   printf("[CLOUD] #%lu sim=%u %s (puback=%lu)\r\n", ...)
CLOUD_LINE = re.compile(
    r"\[CLOUD\]\s*#(?P<seq>\d+)\s+sim=(?P<sim>\d+)\s+"
    r"(?P<verdict>normal|ANOMALY)\s*\(puback=(?P<puback>\d+)\)"
)


# ─────────────────────────────────────────────────────────────
# 日志侧
# ─────────────────────────────────────────────────────────────

def parse_log(text: str) -> list[dict]:
    """把所有 `[CLOUD] #N sim=X ...` 行按出现顺序解析出来。

    不假设"编号必须连续"：设备重连后 `reports` 会继续自增，但移植/复位后可能重置。
    这里只按**出现顺序**给序号，同时保留固件自己的 `seq`，两者不一致时能看出来。
    """
    rows: list[dict] = []
    for i, line in enumerate(text.splitlines(), 1):
        m = CLOUD_LINE.search(line)
        if not m:
            continue
        rows.append(
            {
                "line_no": i,
                "seq": int(m.group("seq")),
                "sim": int(m.group("sim")),
                # 固件打的是 ANOMALY / normal，统一成小写便于与影子比较
                "verdict": "anomaly" if m.group("verdict").upper() == "ANOMALY" else "normal",
                "puback": int(m.group("puback")),
            }
        )
    return rows


# ─────────────────────────────────────────────────────────────
# 影子侧
# ─────────────────────────────────────────────────────────────

def find_keys(obj, key: str, path: str = "$") -> list[tuple[str, object]]:
    """按 key **递归**收集所有命中（返回 [(路径, 值)]）。

    刻意不写死 `shadow[0].reported.properties.similarity` —— 平台版本一换层级就可能变，
    写死会得到"脚本永远 FAIL"而不是"结构变了"。把路径打出来，人一眼能判断取对没有。
    """
    hits: list[tuple[str, object]] = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            sub = f"{path}.{k}"
            if k == key and not isinstance(v, (dict, list)):
                hits.append((sub, v))
            hits.extend(find_keys(v, key, sub))
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            hits.extend(find_keys(v, key, f"{path}[{i}]"))
    return hits


def _norm_verdict(v) -> str | None:
    if not isinstance(v, str):
        return None
    s = v.strip().lower()
    if s in ("normal", "ok", "healthy"):
        return "normal"
    if s in ("anomaly", "abnormal", "alarm", "alert"):
        return "anomaly"
    return None


def parse_shadow(text: str) -> dict:
    """从影子 JSON 里把 similarity / verdict / service_id / event_time 捞出来。

    返回 dict，其中 `notes` 记录"取了哪条路径、有没有歧义"，用于打印而不是猜。
    """
    raw = json.loads(text)
    out: dict = {"similarity": None, "verdict": None, "service_id": None,
                 "event_time": None, "notes": []}

    # similarity：优先取整数值；命中多个就全列出来让人看
    sims = [(p, v) for p, v in find_keys(raw, "similarity") if isinstance(v, (int, float))]
    if len(sims) == 1:
        out["similarity"] = int(sims[0][1])
        out["notes"].append(f"similarity ← {sims[0][0]}")
    elif len(sims) > 1:
        out["notes"].append("⚠️ similarity 命中多个：" + ", ".join(f"{p}={v}" for p, v in sims))
        out["similarity"] = int(sims[-1][1])          # 取最后一个（reported 通常在后）
        out["notes"].append(f"similarity 暂取 {sims[-1][0]}")
    else:
        out["notes"].append("⚠️ 影子里没找到 similarity")

    verds = [(p, _norm_verdict(v)) for p, v in find_keys(raw, "verdict")]
    verds = [(p, v) for p, v in verds if v is not None]
    if len(verds) == 1:
        out["verdict"] = verds[0][1]
        out["notes"].append(f"verdict ← {verds[0][0]}")
    elif len(verds) > 1:
        out["notes"].append("⚠️ verdict 命中多个：" + ", ".join(f"{p}={v}" for p, v in verds))
        out["verdict"] = verds[-1][1]
        out["notes"].append(f"verdict 暂取 {verds[-1][0]}")
    else:
        out["notes"].append("⚠️ 影子里没找到 verdict")

    sids = [(p, v) for p, v in find_keys(raw, "service_id") if isinstance(v, str)]
    if sids:
        out["service_id"] = sids[0][1]
        out["notes"].append(f"service_id ← {sids[0][0]}")

    ets = [(p, v) for p, v in find_keys(raw, "event_time") if isinstance(v, str)]
    if not ets:                                        # 有的版本叫 eventTime
        ets = [(p, v) for p, v in find_keys(raw, "eventTime") if isinstance(v, str)]
    if ets:
        out["event_time"] = ets[0][1]
        out["notes"].append(f"event_time ← {ets[0][0]}")

    return out


# ─────────────────────────────────────────────────────────────
# 对账
# ─────────────────────────────────────────────────────────────

def run_checks(rows: list[dict], shadow: dict, service_id: str) -> list[tuple[str, bool, str]]:
    """返回 [(判据名, 是否通过, 说明)]。全部针对"末条"—— 影子只存最新值。"""
    res: list[tuple[str, bool, str]] = []
    last = rows[-1]

    # J1 条数：末行的 #N 与同行的 puback 至多差 1
    #
    # 🔴 不能写成 `seq == puback` —— 那样**永远 FAIL**，而且不是设备有问题：
    #    固件里 `reports++` 在 printf 之前（main.c:1494），`pubacks++` 在收到
    #    PUBACK 时（main.c:1574）⇒ 第 N 条刚发出去、它的回执还没回来，
    #    所以同一行必然是 `#N ... puback=N-1`。这是**机制使然**，不是丢包。
    #    真正的判据是"最多只有 1 条在途"。
    gap = last["seq"] - last["puback"]
    res.append((
        "J1 上报数 - 回执数 ≤ 1（只允许 1 条在途）",
        0 <= gap <= 1,
        "末行 #%d / puback=%d ⇒ 差 %d" % (last["seq"], last["puback"], gap),
    ))

    # J2 固件序号单调（末行 seq 应等于日志里上报行的条数，除非中途复位过）
    res.append((
        "J2 序号未回绕",
        last["seq"] >= len(rows),
        f"末行 #%d，日志里共 {len(rows)} 条（#N 小于条数 ⇒ 中途复位过）" % last["seq"],
    ))

    # J3 影子 similarity == 末条 sim
    if shadow.get("similarity") is None:
        res.append(("J3 影子 similarity == 末条", False, "影子侧没取到 similarity"))
    else:
        res.append((
            "J3 影子 similarity == 末条",
            shadow["similarity"] == last["sim"],
            "影子 %d vs 末条 %d" % (shadow["similarity"], last["sim"]),
        ))

    # J4 影子 verdict == 末条
    if shadow.get("verdict") is None:
        res.append(("J4 影子 verdict == 末条", False, "影子侧没取到 verdict"))
    else:
        res.append((
            "J4 影子 verdict == 末条",
            shadow["verdict"] == last["verdict"],
            "影子 %s vs 末条 %s" % (shadow["verdict"], last["verdict"]),
        ))

    # J5 服务 ID 对得上（取不到就跳过，不算失败）
    if shadow.get("service_id") is None:
        res.append(("J5 service_id", True, "影子侧未带 service_id，跳过"))
    else:
        res.append((
            "J5 service_id 一致",
            shadow["service_id"] == service_id,
            "影子 %s vs 固件 %s" % (shadow["service_id"], service_id),
        ))

    return res


# ─────────────────────────────────────────────────────────────
# 自测（不需要板子、不联网）
# ─────────────────────────────────────────────────────────────

_SAMPLE_LOG = """\
[BOOT] VECT_TAB 0x08060000
[CLOUD] #1 sim=98 normal (puback=0)
[CLOUD] #2 sim=97 normal (puback=1)
[CLOUD] #3 sim=96 normal (puback=2)
[CLOUD] #4 sim=55 ANOMALY (puback=3)
[CLOUD] #5 sim=55 ANOMALY (puback=4)
[CLOUD] #6 sim=55 ANOMALY (puback=5)
"""

_SAMPLE_SHADOW_OK = """\
{"shadow":[{"service_id":"vibration","desired":{},
 "reported":{"properties":{"similarity":55,"verdict":"anomaly"},
             "event_time":"20261028T060000Z"},"version":7}]}
"""

# 影子停在旧值、puback 却一条不少 —— 这就是 puback 判据抓不到的故障
_SAMPLE_SHADOW_STALE = """\
{"shadow":[{"service_id":"vibration","desired":{},
 "reported":{"properties":{"similarity":55,"verdict":"normal"},
             "event_time":"20261028T055000Z"},"version":6}]}
"""


def selftest() -> int:
    fails = 0

    def ck(name: str, cond: bool, extra: str = "") -> None:
        nonlocal fails
        print(("  PASS  " if cond else "  FAIL  ") + name + (("  " + extra) if extra and not cond else ""))
        if not cond:
            fails += 1

    print("=" * 68)
    print("iotda_shadow_check · 自测")
    print("=" * 68)

    rows = parse_log(_SAMPLE_LOG)
    ck("日志解析出 6 条", len(rows) == 6, f"got {len(rows)}")
    ck("末条 sim=55", rows[-1]["sim"] == 55)
    ck("ANOMALY 归一成 anomaly", rows[-1]["verdict"] == "anomaly")
    ck("normal 归一成 normal", rows[0]["verdict"] == "normal")
    ck("没被 [BOOT] 行干扰", all("BOOT" not in str(r) for r in rows))

    # 影子：正常结构
    sh = parse_shadow(_SAMPLE_SHADOW_OK)
    ck("影子取到 similarity=55", sh["similarity"] == 55, str(sh["similarity"]))
    ck("影子取到 verdict=anomaly", sh["verdict"] == "anomaly", str(sh["verdict"]))
    ck("影子取到 service_id", sh["service_id"] == "vibration")
    ck("影子取到 event_time", sh["event_time"] == "20261028T060000Z")

    good = run_checks(rows, sh, "vibration")
    ck("一致性场景 5 条判据全 PASS", all(ok for _, ok, _ in good),
       str([(n, ok) for n, ok, _ in good]))

    # 影子陈旧/内容错：必须 FAIL 在 J4（verdict 对不上）
    sh2 = parse_shadow(_SAMPLE_SHADOW_STALE)
    bad = run_checks(rows, sh2, "vibration")
    d = {n: ok for n, ok, _ in bad}
    ck("陈旧影子 J4 判 FAIL", d["J4 影子 verdict == 末条"] is False, str(d))
    ck("陈旧影子 J1/J3 仍 PASS（模拟 puback 照常回）",
       d["J1 上报数 - 回执数 ≤ 1（只允许 1 条在途）"] and d["J3 影子 similarity == 末条"], str(d))

    # 嵌套层级变了也要能取到（不写死路径）
    weird = parse_shadow('{"a":{"b":[{"c":{"similarity":7,"verdict":"NORMAL"}}]}}')
    ck("任意层级都能取到（不写死路径）",
       weird["similarity"] == 7 and weird["verdict"] == "normal", str(weird))

    # 影子缺失字段：给 FAIL 而不是崩
    empty = parse_shadow("{}")
    ck("空影子不崩、similarity 为 None",
       empty["similarity"] is None and empty["verdict"] is None)

    # 日志里没有上报行
    ck("无上报行时解析为空", parse_log("[BOOT] nothing here\n") == [])

    # 固件 seq 回绕：J2 必须能识别
    wrap = parse_log("[CLOUD] #1 sim=90 normal (puback=0)\n[CLOUD] #2 sim=90 normal (puback=1)\n")
    d2 = {n: ok for n, ok, _ in run_checks(wrap, sh, "vibration")}
    ck("序号正常时 J2 PASS", d2["J2 序号未回绕"] is True, str(d2))

    # J1 的边界：差 1 合法（在途），差 2 必须 FAIL（丢了一条回执）
    two_lag = parse_log("[CLOUD] #9 sim=90 normal (puback=7)\n")
    d3 = {n: ok for n, ok, _ in run_checks(two_lag, sh, "vibration")}
    ck("差 2 条时 J1 判 FAIL（真丢回执）",
       d3["J1 上报数 - 回执数 ≤ 1（只允许 1 条在途）"] is False, str(d3))
    one_lag = parse_log("[CLOUD] #9 sim=90 normal (puback=8)\n")
    d4 = {n: ok for n, ok, _ in run_checks(one_lag, sh, "vibration")}
    ck("差 1 条时 J1 判 PASS（在途 1 条属正常）",
       d4["J1 上报数 - 回执数 ≤ 1（只允许 1 条在途）"] is True, str(d4))

    print("=" * 68)
    print(("全部通过" if fails == 0 else f"{fails} 项失败"))
    print("=" * 68)
    return 0 if fails == 0 else 1


# ─────────────────────────────────────────────────────────────
# 主流程
# ─────────────────────────────────────────────────────────────

def read_serial(port: str, seconds: float, baud: int = 115200) -> str:
    """从串口录一段（需要 pyserial）。板子在跑 mode 3 + 上云时用。"""
    try:
        import serial  # type: ignore
    except ImportError:
        sys.exit("需要 pyserial 才能用 --serial。装法（先摘代理）：\n"
                 "  pip install pyserial\n"
                 "或者改用 --log <把串口日志存成文件>。")

    print(f"[serial] 打开 {port} @{baud}，录 {seconds:.0f} s（Ctrl-C 可提前结束）", file=sys.stderr)
    buf: list[str] = []
    t0 = time.time()
    with serial.Serial(port, baud, timeout=0.2) as sp:
        try:
            while time.time() - t0 < seconds:
                line = sp.readline()
                if line:
                    buf.append(line.decode("utf-8", "replace"))
        except KeyboardInterrupt:
            pass
    return "".join(buf)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="云端影子 ↔ 串口日志的对账（末值 + 条数）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--log", help="串口日志文件（含 [CLOUD] #N ... 行）")
    ap.add_argument("--serial", help="改为从串口实时录（需要 pyserial）")
    ap.add_argument("--seconds", type=float, default=120.0, help="--serial 录制时长，默认 120 s")
    ap.add_argument("--shadow", help="设备影子 JSON 文件；写 - 表示从 stdin 读")
    ap.add_argument("--service-id", default="vibration", help="物模型服务 ID，默认 vibration")
    ap.add_argument("--emit-template", action="store_true", help="打印一份影子 JSON 骨架后退出")
    ap.add_argument("--selftest", action="store_true", help="离线自测")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.emit_template:
        print(_SAMPLE_SHADOW_OK.replace('"similarity":55', '"similarity":<末条 sim>')
                                .replace('"verdict":"anomaly"', '"verdict":"<normal|anomaly>"'))
        return 0

    # ── 取日志
    if args.serial:
        text = read_serial(args.serial, args.seconds)
    elif args.log:
        p = pathlib.Path(args.log)
        if not p.exists():
            sys.exit(f"找不到日志文件：{args.log}")
        text = p.read_text(encoding="utf-8", errors="replace")
    else:
        sys.exit("需要 --log 或 --serial（或用 --selftest）")

    rows = parse_log(text)
    if not rows:
        print("日志里没有 `[CLOUD] #N sim=...` 行。", file=sys.stderr)
        print("→ 确认板子跑的是 mode 3 + APP_CLOUD_ENABLE=1，且这段日志确实覆盖了上报时刻。",
              file=sys.stderr)
        return 2

    last = rows[-1]
    print(f"设备侧：共 {len(rows)} 条上报，末条 #%d sim=%d %s (puback=%d)"
          % (last["seq"], last["sim"], last["verdict"], last["puback"]))
    # 顺带给一眼分布 —— 只有一条值是"看起来像"而没在变，往往说明推理卡住了
    uniq = sorted({r["sim"] for r in rows})
    print(f"        sim 取值集合：{uniq[:12]}{' ...' if len(uniq) > 12 else ''}")
    if len(uniq) == 1 and len(rows) >= 5:
        print("       ⚠️ 所有上报的 sim 都一样 —— 要么工况真的没变，要么推理侧卡住了，值得查一下")

    if not args.shadow:
        print("\n（没给 --shadow，只做了设备侧解析。加 --shadow shadow.json 才能对账。）")
        return 0

    # ── 取影子
    if args.shadow == "-":
        stext = sys.stdin.read()
    else:
        sp = pathlib.Path(args.shadow)
        if not sp.exists():
            sys.exit(f"找不到影子文件：{args.shadow}")
        stext = sp.read_text(encoding="utf-8", errors="replace")

    try:
        shadow = parse_shadow(stext)
    except json.JSONDecodeError as e:
        sys.exit(f"影子 JSON 解析失败（{e}）。控制台复制时容易漏掉最外层花括号。")

    print("\n云端侧（取自影子）：")
    for n in shadow["notes"]:
        print("  · " + n)

    print("\n---------------- 判据 ----------------")
    results = run_checks(rows, shadow, args.service_id)
    for name, ok, detail in results:
        print(("  PASS  " if ok else "  FAIL  ") + name + "  —— " + detail)

    npass = sum(1 for _, ok, _ in results if ok)
    print("--------------------------------------")
    print(f"{npass}/{len(results)} 通过")
    if npass == len(results):
        print("✅ 平台存下来的值 == 板子认为它发的值。这比 puback 强一层。")
    else:
        print("🔴 有判据没过。最常见两种：影子是**旧值**（设备停得太久/中间没上报），"
              "或 payload 字段名与物模型不一致。")
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
