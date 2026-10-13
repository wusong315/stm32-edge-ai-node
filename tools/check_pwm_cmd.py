#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_pwm_cmd.py —— 第 ⑦ 步「不需要电机」的 PWM 命令通路验收。

它把 RUNBOOK R6.4 第 ⑦ 步的「验收 ①②」变成一条命令，并把两个**必须做对**的顺序
固化在代码里，手工做极容易做错：

  🔴 顺序一：**先开串口，再复位**。启动横幅只在复位后立刻打印，
     Windows 上端口未打开时到达的数据会被丢弃（PLAYBOOK 踩坑 15）。
  🔴 顺序二：**单线程读写**。同一个 Serial 对象上「后台线程读 + 主线程写」，
     读线程可能抛异常后静默退出 —— 症状是「发完命令一个字节都收不到」，
     看起来像固件挂了，实际是观察手段自己坏了（PLAYBOOK 踩坑 20）。

判据：
  ① 复位横幅含 `PWM: PA6/TIM3_CH1 @<freq> Hz, duty=0%`（引脚/频率/上电占空比都在）
  ② 发 `dNN` 回显 `[PWM] duty = NN%`（命令通路真的通）
  ③ 发 `?` 回显带 `(PA6/TIM3_CH1 @20 kHz; ...)` 的信息行
  ④ 收工发 `d0` 回显 `duty = 0%`（不留残余占空比，除非 --keep-duty）

另有 `--led-test` 模式（第 ③ 步 LED 目视验收，功率侧悬空）：
  循环发 `d100 -> d40 -> d0`，每档保持几秒，**由你的眼睛看模块指示灯**
  （亮 / 变暗 / 灭）。这一步灯是唯一的判据，工具只保证「命令确实送到」并回显确认。
  灯不亮就把 SIG 从 PA6 挪到同一行的 PA7 再跑一次。

退出码：0 = 全过；1 = 有用例失败；2 = 环境/参数错误。

用法：
  python check_pwm_cmd.py --port COM5
  python check_pwm_cmd.py --port COM5 --duty 60 --keep-duty
  python check_pwm_cmd.py --port COM5 --led-test              # 第 ③ 步，盯灯
  python check_pwm_cmd.py --port COM5 --led-test --cycles 5 --hold 2
  python check_pwm_cmd.py --port COM5 --set 100               # 停在 d100，用万用表量 ≈3.3 V
  python check_pwm_cmd.py --port COM5 --set 40                # 停在 d40，量 ≈1.3 V
  python check_pwm_cmd.py --port COM5 --set 0                 # 停在 d0，量 0 V
"""
import argparse
import os
import re
import subprocess
import sys
import time

DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 115200
DEFAULT_DUTY = 40
DEFAULT_CLI = r"D:\MeSTM32CubePR\bin\STM32_Programmer_CLI.exe"

# 复位后等多久收横幅（模式 0 下 10 Hz 数据流会一直来，所以是按时间收，不是按行收）
BANNER_WINDOW_S = 4.0
ECHO_WINDOW_S = 1.5

BANNER_RE = re.compile(r"PWM:\s*PA6/TIM3_CH1\s*@(\d+)\s*Hz,\s*duty=(\d+)%")
ECHO_RE = re.compile(r"\[PWM\]\s*duty\s*=\s*(\d+)%")
INFO_RE = re.compile(r"\[PWM\]\s*duty\s*=\s*(\d+)%\s*\((PA6/TIM3_CH1[^)]*)\)")

# ---- LED 目视验收（第 ③ 步）用的步骤表 ----------------------------------
# 顺序有讲究：从最亮开始，中间插一个 40%，最后归零 —— 有对照才能排除
# 「灯本来就常亮 / 本来就不亮」这类误判。
LED_STEPS = ((100, "最亮"), (40, "明显变暗（约四成亮度）"), (0, "灭"))


def led_plan(cycles=3, hold=3.0):
    """返回 [(duty, 说明, 保持秒数), ...] —— 纯函数，便于自测。"""
    return [(duty, note, float(hold))
            for _ in range(max(1, int(cycles)))
            for duty, note in LED_STEPS]


def parse_banner(text):
    """从任意文本里抓横幅行 -> (freq_hz, duty_pct)；抓不到返回 (None, None)。"""
    m = BANNER_RE.search(text or "")
    if not m:
        return (None, None)
    return (int(m.group(1)), int(m.group(2)))


def parse_echo(text, want):
    """文本里是否出现 `[PWM] duty = <want>%`；返回命中的百分比或 None。

    只认「行首或空白 + [PWM]」的形式，避免 d40 的回显里数字被相邻数据行污染。
    """
    for line in (text or "").splitlines():
        m = ECHO_RE.search(line.strip())
        if m and int(m.group(1)) == int(want):
            return int(m.group(1))
    return None


def parse_info(text):
    """抓 `?` 的信息行 -> (duty_pct, 括号里的说明) 或 (None, None)。"""
    m = INFO_RE.search(text or "")
    if not m:
        return (None, None)
    return (int(m.group(1)), m.group(2))


def find_cli(explicit=None):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    if os.path.exists(DEFAULT_CLI):
        return DEFAULT_CLI
    return None


def reset_target(cli, logger=print):
    """用 ST-Link 软复位目标板。返回 True/False，不抛异常。"""
    try:
        r = subprocess.run([cli, "-c", "port=SWD", "-rst"],
                           capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=90)
    except Exception as exc:
        logger("   [!!] 复位失败：%r" % (exc,))
        return False
    out = (r.stdout or "") + (r.stderr or "")
    return "MCU Reset" in out or "reset is performed" in out.lower()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="第 ⑦ 步 PWM 命令通路验收（不需要接电机、不需要电池）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT, help="串口（默认 {}）".format(DEFAULT_PORT))
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率（默认 %d）" % DEFAULT_BAUD)
    ap.add_argument("--duty", type=int, default=DEFAULT_DUTY,
                    help="用来验回显的占空比，1..100（默认 %d）" % DEFAULT_DUTY)
    ap.add_argument("--cli", default=None, help="STM32_Programmer_CLI.exe 的完整路径")
    ap.add_argument("--no-reset", action="store_true",
                    help="不复位（跳过判据 ①；用于只想验命令通路的场合）")
    ap.add_argument("--keep-duty", action="store_true",
                    help="收工不归零（默认会发 d0，避免留下残余占空比）")
    ap.add_argument("--set", type=int, default=None, metavar="NN",
                    help="把占空比设为 NN%% (0..100) 并**保持**（不归零），用于配合万用表量 SIG 电压")
    ap.add_argument("--led-test", action="store_true",
                    help="第 ③ 步 LED 目视验收：循环发 d100/d40/d0，你盯模块指示灯（功率侧悬空）")
    ap.add_argument("--cycles", type=int, default=3, help="LED 验收循环轮数（默认 3）")
    ap.add_argument("--hold", type=float, default=3.0, help="LED 每档保持秒数（默认 3.0）")
    args = ap.parse_args(argv)

    if args.set is not None and not (0 <= args.set <= 100):
        print("[x] --set 必须在 0..100", file=sys.stderr)
        return 2
    if args.set is None and (not args.led_test) and not (1 <= args.duty <= 100):
        print("[x] --duty 必须在 1..100", file=sys.stderr)
        return 2

    try:
        import serial
    except ImportError:
        print("[x] 没装 pyserial（用工程自带的 venv：python -m pip install pyserial）",
              file=sys.stderr)
        return 2

    cli = find_cli(args.cli)
    if (not args.no_reset) and (args.set is None) and cli is None:
        print("[x] 找不到 STM32_Programmer_CLI.exe，请用 --cli 指定或用 --no-reset",
              file=sys.stderr)
        return 2

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as exc:
        print("[x] 打不开 %s：%r" % (args.port, exc), file=sys.stderr)
        return 2

    print("[i] 已打开 %s @%d" % (args.port, args.baud))

    def pump(seconds):
        """单线程读：不另起线程，异常直接抛出来（别让观察手段静默死掉）。"""
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < seconds:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
            else:
                time.sleep(0.01)
        return buf.decode("utf-8", "replace")

    # ---------- 单档保持模式（--set）：配合万用表量电压 ----------
    # 看灯是有歧义的（灯到底挂 SIG 还是 VCC、亮度差异看不看得出），
    # 量电压没有歧义：20 kHz 会被万用表积分成平均值 = 3.3 V × 占空比。
    if args.set is not None:
        got = None
        try:
            ser.write(("d%d\n" % args.set).encode())
            ser.flush()
            text = pump(ECHO_WINDOW_S)
            got = parse_echo(text, args.set)
            print("[%s] 已发 'd%d' -> 回显 duty=%s%%（**保持**这个档位，不发 d0）"
                  % ("PASS" if got == args.set else "FAIL", args.set, got))
            print("    期望万用表读数 ≈ %.2f V（= 3.3 V × %d%%，20 kHz 被表积分成平均值）"
                  % (3.3 * args.set / 100.0, args.set))
            print("    量法：黑表笔接 GND，红表笔接模块 SIG 针（或同网络的 PA6 焊盘），直流电压档")
            # 🔴 关键提醒：0% 与 100% 都是**直流电平**，证明不了 PWM 存在。
            #    main.c 的 App_SetDuty 用 CCR=(ARR+1)*pct/100 ⇒ pct=100 → CCR=4200>ARR=4199 → 恒高；
            #    pct=0 → CCR=0 → 恒低。引脚被钉死在高/低也读出同样的值，所以这两档**不能证伪**。
            if args.set == 100:
                print("    ⚠️ 但 100% 是**恒高**（CCR=ARR+1，引脚不翻转）—— 3.3 V 只说明「没被钉低」，")
                print("       **证明不了 20 kHz 方波存在**。要证明方波请量中间档：`--set 40` → ≈1.3 V。")
            elif args.set == 0:
                print("    ⚠️ 0% 是**恒低** —— 读 0 V 同样只是「没被钉高」。")
                print("       决定性数字在中间档：`--set 40` → ≈1.3 V（恒定引脚给不出这个值）。")
            else:
                print("    ✅ 这是**中间档**，读数 ≈1.3 V 量级只可能来自方波 ⇒ 这一档是 PWM 的决定性判据。")
        finally:
            ser.close()
        return 0 if got == args.set else 1

    # ---------- LED 目视验收模式（第 ③ 步，功率侧全悬空） ----------
    # 灯亮不亮只能用眼睛看，所以这里把节奏放慢、每一步都打在屏幕上，
    # 由用户的眼睛做判据；工具只负责「把命令确实送到」并回显确认。
    if args.led_test:
        steps = led_plan(args.cycles, args.hold)
        print("[i] LED 目视验收：共 %d 步，每步保持 %.1f s" % (len(steps), args.hold))
        print("    请盯模块上的指示灯（只接 SIG+GND，不接电机、不接电池）")
        if not args.no_reset:
            proc = subprocess.Popen([cli, "-c", "port=SWD", "-rst"],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            text = pump(BANNER_WINDOW_S)
            proc.wait(timeout=60)
            freq, duty0 = parse_banner(text)
            if freq is None:
                print("[!!] 复位后没看到 PWM 横幅 —— 板子里不是这一版固件？")
            else:
                print("[i] 横幅：@%d Hz, duty=%d%%（上电应为 0%%）" % (freq, duty0))
        try:
            for i, (duty, note, hold) in enumerate(steps, 1):
                ser.write(("d%d\n" % duty).encode())
                ser.flush()
                text = pump(hold)
                got = parse_echo(text, duty)
                ok = (got == duty)
                print("[%s] 第 %d/%d 步  d%-3d -> 灯应该%s   (回显 duty=%s%%, 命令%s)"
                      % (time.strftime("%H:%M:%S"), i, len(steps), duty, note, got,
                         "已确认送达" if ok else "未回显！"))
            if not args.keep_duty:
                ser.write(b"d0\n")
                ser.flush()
                pump(ECHO_WINDOW_S)
                print("[i] 已发 d0 收尾归零")
        finally:
            ser.close()
        print("")
        print("=" * 58)
        print("请把你**看到**的现象回我三条：")
        print("  ① d100 时，模块指示灯亮吗？")
        print("  ② d40 时，比 d100 暗吗？")
        print("  ③ d0 时，灭了吗？")
        print("（若 ① 就不亮：把 SIG 从 PA6 挪到同一行的 PA7，再跑一次）")
        print("=" * 58)
        return 0

    results = []
    try:
        # ---------- ① 复位横幅 ----------
        if args.no_reset:
            print("[i] 跳过复位（--no-reset）")
        else:
            # 🔴 先开串口（上面已开）、再复位：一个字节都不漏
            proc = subprocess.Popen([cli, "-c", "port=SWD", "-rst"],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            text = pump(BANNER_WINDOW_S)
            proc.wait(timeout=60)
            freq, duty0 = parse_banner(text)
            ok1 = (freq is not None) and (duty0 == 0)
            print("[%s] ① 复位横幅：@%s Hz, duty=%s%%  期望 duty=0%%"
                  % ("PASS" if ok1 else "FAIL", freq, duty0))
            if not ok1:
                for line in text.splitlines():
                    if "PWM" in line or "boot" in line:
                        print("      原文: " + line.strip())
            results.append(("① 复位横幅 duty=0%", ok1))

        # ---------- ② 发 dNN ----------
        cmd = ("d%d\n" % args.duty).encode()
        ser.write(cmd)
        ser.flush()
        text = pump(ECHO_WINDOW_S)
        got = parse_echo(text, args.duty)
        ok2 = got == args.duty
        print("[%s] ② 发 %r -> 回显 [PWM] duty = %s%%"
              % ("PASS" if ok2 else "FAIL", cmd.decode().strip(), got))
        results.append(("② 命令回显 duty=%d%%" % args.duty, ok2))

        # ---------- ③ 发 ? ----------
        ser.write(b"?\n")
        ser.flush()
        text = pump(ECHO_WINDOW_S)
        duty_info, note = parse_info(text)
        ok3 = (duty_info == args.duty) and bool(note)
        print("[%s] ③ 发 '?' -> duty=%s%%, 说明=%r"
              % ("PASS" if ok3 else "FAIL", duty_info, note))
        results.append(("③ 信息行（含引脚/频率）", ok3))

        # ---------- ④ 归零 ----------
        if args.keep_duty:
            print("[i] --keep-duty：保留 duty=%d%% 不收工归零" % args.duty)
        else:
            ser.write(b"d0\n")
            ser.flush()
            text = pump(ECHO_WINDOW_S)
            got0 = parse_echo(text, 0)
            ok4 = got0 == 0
            print("[%s] ④ 收工归零 -> duty = %s%%" % ("PASS" if ok4 else "FAIL", got0))
            results.append(("④ 收工归零 duty=0%", ok4))
    finally:
        ser.close()

    print("")
    print("=" * 58)
    for name, ok in results:
        print("  %-26s %s" % (name, "PASS" if ok else "FAIL"))
    npass = sum(1 for _, ok in results if ok)
    print("=" * 58)
    print("结果：%d / %d 通过" % (npass, len(results)))
    return 0 if npass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
