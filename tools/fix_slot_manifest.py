# -*- coding: utf-8 -*-
"""修某个 Slot 配置的构建清单（缺源文件 + 档位与 .cproject 不一致）。

用法:  python fix_slot_manifest.py SlotA

安全策略（见 skill `stm32-cubeide-cli-build`）：
  · 全程**字节操作**，行尾从被插入的那一行继承，不假设 CRLF/LF；
  · **只插入、不删行**（`subdir.mk` 里 OBJS 条目和 `-$(RM)` clean 规则在同一物理行，
    按行删会静默删掉整条 clean 规则）；
  · 每步断言：锚点唯一 / gcc 规则逐字节不变 / 行数增量精确 / 每个新文件出现次数正确；
  · 改动前留 `.bak_a4` 备份。
"""
import os
import re
import shutil
import sys

ROOT = r"D:\STM32Projects\stm32-edge-ai-node"
SRC_DIR = os.path.join(ROOT, "Core", "Src")


def find_line(data, needle):
    i = data.find(needle)
    assert i >= 0, "找不到 %r" % needle
    assert data.find(needle, i + 1) < 0, "%r 出现不止一次" % needle
    start = data.rfind(b"\n", 0, i) + 1
    j = data.find(b"\n", i)
    assert j >= 0
    return start, j + 1


def eol_of(line):
    return b"\r\n" if line.endswith(b"\r\n") else (b"\n" if line.endswith(b"\n") else None)


def fix_sources(cfg):
    mk_p = os.path.join(ROOT, cfg, "Core", "Src", "subdir.mk")
    ol_p = os.path.join(ROOT, cfg, "objects.list")
    mk = open(mk_p, "rb").read()
    ol = open(ol_p, "rb").read()

    on_disk = sorted(f[:-2] for f in os.listdir(SRC_DIR) if f.endswith(".c"))
    listed = re.findall(rb"\.\./Core/Src/([A-Za-z0-9_]+)\.c", mk)
    missing = [n for n in on_disk if n.encode() not in listed]
    if not missing:
        print("  %s: 源文件已齐（%d 个）" % (cfg, len(listed)))
        return 0

    print("  %s: 缺 %d 个 -> %s" % (cfg, len(missing), ", ".join(missing)))
    for p in (mk_p, ol_p):
        if not os.path.exists(p + ".bak_a4"):
            shutil.copy2(p, p + ".bak_a4")

    mk0 = mk
    for needle, fmt in ((b"../Core/Src/main.c \\", b"../Core/Src/%s.c \\"),
                        (b"./Core/Src/main.o \\", b"./Core/Src/%s.o \\"),
                        (b"./Core/Src/main.d \\", b"./Core/Src/%s.d \\")):
        s, e = find_line(mk, needle)
        eol = eol_of(mk[s:e])
        mk = mk[:e] + b"".join(fmt % n.encode() + eol for n in missing) + mk[e:]

    s_rm, e_rm = find_line(mk, b"-$(RM)")
    eol = eol_of(mk[s_rm:e_rm])
    body = mk[s_rm:e_rm][:-len(eol)]
    add = b"".join(b" ./Core/Src/%s.cyclo ./Core/Src/%s.d ./Core/Src/%s.o ./Core/Src/%s.su"
                   % (n.encode(), n.encode(), n.encode(), n.encode()) for n in missing)
    mk = mk[:s_rm] + body + add + eol + mk[e_rm:]

    s, e = find_line(ol, b'"./Core/Src/neai_app.o"')
    eol = eol_of(ol[s:e])
    ol = ol[:e] + b"".join(b'"./Core/Src/%s.o"' % n.encode() + eol for n in missing) + ol[e:]

    # 自检
    for n in missing:
        nb = n.encode()
        assert mk.count(b"../Core/Src/%s.c" % nb) == 1
        assert mk.count(b"./Core/Src/%s.o" % nb) == 2
        assert mk.count(b"./Core/Src/%s.d" % nb) == 2
        assert ol.count(b'"./Core/Src/%s.o"' % nb) == 1
    assert mk.count(b"\n") == mk0.count(b"\n") + 3 * len(missing), "行数不对"
    assert mk.count(b"clean-Core-2f-Src:") == 1 and mk.count(b".PHONY: clean-Core-2f-Src") == 1

    open(mk_p, "wb").write(mk)
    open(ol_p, "wb").write(ol)
    return len(missing)


def fix_mode(cfg, want):
    """把 subdir.mk 里的 -DAPP_OUTPUT_MODE=<n> 改成 .cproject 声明的值。"""
    d = os.path.join(ROOT, cfg)
    changed = 0
    for dirpath, _, files in os.walk(d):
        for f in files:
            if f != "subdir.mk":
                continue
            p = os.path.join(dirpath, f)
            b = open(p, "rb").read()
            vals = set(int(x) for x in re.findall(rb"APP_OUTPUT_MODE=(\d)", b))
            if not vals or vals == {want}:
                continue
            if not os.path.exists(p + ".bak_a4"):
                shutil.copy2(p, p + ".bak_a4")
            nb = re.sub(rb"APP_OUTPUT_MODE=\d", b"APP_OUTPUT_MODE=%d" % want, b)
            # 自检：只改了这几个字节
            assert len(nb) == len(b)
            assert set(int(x) for x in re.findall(rb"APP_OUTPUT_MODE=(\d)", nb)) == {want}
            open(p, "wb").write(nb)
            changed += 1
    return changed


def main():
    cfg = sys.argv[1] if len(sys.argv) > 1 else "SlotA"
    cproj = open(os.path.join(ROOT, ".cproject"), encoding="utf-8", errors="replace").read()
    want = int(re.search(r"APP_OUTPUT_MODE=(\d)", cproj).group(1))
    print("%s：目标档位 = %d（来自 .cproject）" % (cfg, want))
    n_src = fix_sources(cfg)
    n_mode = fix_mode(cfg, want)
    print("  %s: 补 %d 个源；修正档位的文件 %d 个" % (cfg, n_src, n_mode))


if __name__ == "__main__":
    main()
