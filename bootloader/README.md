# bootloader —— OTA Bootloader 工程（S2 已完成）

> 📍 **本目录是 `stm32-edge-ai-node` 仓库里的第二个 CubeIDE 工程**，不是 App 的子模块。
> 布局对齐参考仓库 `D:\refs\STM32F407VGT6-FOTA-with-DualSlot`（`Bootloader/` + `Application/` 平级）。

---

## 1. 它是什么

上电后**第一段运行的代码**。职责只有四件：

1. 读参数区（元数据）——判断该跳哪个槽；
2. 需要升级时，从 **USART1** 收新固件写进 Slot B；
3. 校验（CRC32）后置标志、复位；
4. 跳转到合法的那一槽。

**Flash 分区表（方案乙，决策 D21）** —— 唯一权威是 `MVP/docs/plan/STM32边缘AI项目-OTA实施方案.md` §5，
本工程把它落成代码：`Core/Inc/bl_config.h`（带编译期一致性断言）。

| 区域 | 起始 | 大小 | 扇区 |
|---|---|---|---|
| Bootloader（本工程） | `0x08000000` | 32 KB | S0+S1 |
| 参数区（元数据，双副本 + CRC） | `0x08008000` | 16 KB | S2 |
| App A（运行槽） | `0x0800C000` | 336 KB | S3–S6 |
| App B（备用槽，收新固件） | `0x08060000` | 384 KB | S7–S9 |
| 日志 / 数据 | `0x080C0000` | 256 KB | S10+S11 |

---

## 2. 当前进度：**S2**（推演稿 §7）

| 项 | 结果 |
|---|---|
| 命令行编译 | ✅ **0 error / 0 warning**，`make clean && make all` 全量重编 |
| `.isr_vector` VMA | ✅ `0x08000000`（`objdump -h` 实测） |
| 向量表内容 | ✅ 初始 MSP `0x20020000`（RAM 顶）+ Reset 向量 `0x08000C61`（奇数、落在本区） |
| Flash 占用 | ✅ **14 144 B ≈ 13.81 KB / 32 KB（43.1 %，余量 56.9 %）** |
| RAM 占用 | ✅ `data 104 + bss 1 976 = 2 080 B` / 128 KB |
| App 工程影响 | ✅ **零改动**（`git status` 只显示新增 `bootloader/`） |
| 链接脚本 | ✅ `STM32F407VGTX_FLASH.ld` 的 `FLASH LENGTH = 32K`（App 是 `1024K`） |

> ⚠️ 以上是 **Debug（`-O0`）** 的体积。Release（`-Os`）会明显更小 —— 但 Bootloader 本来就跑在启动关键路径上，
> 「Debug 版也塞得下 32 KB」是更强的结论，所以以 Debug 数字为准。

**本阶段代码刻意只做**：横幅打印（含分区表全表）+ **只读**槽探测 + 心跳循环。
**不写 Flash、不跳转** ⇒ 任何时刻都可整目录丢弃，板上跑的仍是原 App。

---

## 3. 怎么编译（不需要 IDE、不需要硬件）

用用户级技能 `stm32-cubeide-cli-build`：

```bash
python ~/.workbuddy/skills/stm32-cubeide-cli-build/scripts/build.py \
  . --config Debug --clean \
  --cubeide-root "D:/mestm32/STM32CubeIDE_2.2.0"
```

核对地址（S2 的判据之一，**不要只看"编译成功"**）：

```bash
arm-none-eabi-objdump -h Debug/bootloader.elf | head -8   # .isr_vector 的 VMA 必须是 08000000
```

---

## 4. 与 App 工程的差异（只有这些，别让它们漂移）

| 维度 | App（`../`） | Bootloader（本目录） |
|---|---|---|
| `.ld` 的 `FLASH LENGTH` | `1024K` | **`32K`**（超出即链接报错，自带预算守卫） |
| 外设 | I2C1 / TIM2 / TIM3 / USART1 / USART3 | **只有 USART1 + LED(PA1)** |
| HAL 模块 | 18 个 | **14 个**（去掉 i2c / i2c_ex / tim / tim_ex） |
| `-u _printf_float` | 开（要打 `%.3f`） | **关**（省 ≈10.5 KB） |
| 链接的库 | `-L Core/Src -lneai` | **无**（`.cproject` 与 `Debug/objects.mk` 都已清空） |
| `printf` 落点 | 环形队列（保 1 kHz 节拍） | **阻塞直发**（无实时节拍，少一类竞态） |
| HardFault | `while(1)` | **PA1 快闪**（与 1 Hz 心跳区分 ⇒ 一眼分清"跑飞"和"没烧进去"） |
| 时钟树 | HSE 8 MHz → 168 MHz | **逐字相同**（刻意） |

---

## 5. 施工阶段纪律（照抄推演稿 §7，**不许跳**）

```
S1 分区表定稿 ✅  →  S2 建工程+编译 ✅  →  S3 元数据+跳转  →  S4 App 双配置
  →  S5 烧 Slot A（⚠️ 最危险）  →  S6 串口升级  →  S7 坏包拒收  →  S8 回滚
```

🔴 **S5 之后 App 不再位于 `0x08000000`**，再也不能"ST-Link 直接烧一个新 App 就让它跑起来"。
因此本仓库根 `.gitignore` 之外，**手上必须常备两份救砖件**（已备好，见 `D:\STM32Projects\_tmp\`）：

| 救砖件 | 用途 |
|---|---|
| `rescue_m1_mode0_20261016.elf` | 用 ST-Link 烧回 `0x08000000`，恢复成"没有 Bootloader 的原始状态" |
| `STM32F407VGTX_FLASH.orig_1M.ld` | `0x08000000 / 1024K` 原版链接脚本，随时能重编一份"直烧就能跑"的固件 |

---

## 6. 🔴 已知的文档勘误（施工中发现，勿按原稿执行）

推演稿 §7 的 **S3 判据原文**是「只跳 `0x08000000`（即现在的 App）」+「上电后能正常进入现有 App（串口仍打 `M1` 横幅）」。

**这两句互斥、且都做不到**：

1. **跳回自己**：Bootloader 按分区表就占 `0x08000000`（S0+S1）。跳 `0x08000000` = 跳进自身入口 → 立刻重入，永远进不了 App。
2. **覆盖 App**：把 Bootloader 烧到 `0x08000000` 这件事本身就会**覆盖掉**原先跑在那里的 App ⇒ "串口仍打 M1 横幅"在物理上不可能发生。

**修正后的 S3**（判据改为「不跳」侧，见 `RUNBOOK.md` R13）：

- 逻辑：读元数据（空白 → 写默认 + 自愈）→ 校验 Slot A 镜像合法性 → 合法才跳 `0x0800C000`；不合法则**不跳**，进安全等待（串口周期打印 + LED 慢闪，证明"活着且没锁死"）。
- 判据：① 串口出 Bootloader 横幅（版本 + 构建时间）；② 元数据自愈动作可读；③ **Slot A 尚未编程 ⇒ 明确打印 invalid 并停在等待循环**；④ **板上不再有 M1 横幅 —— 这是预期结果，不是故障**。
- 「能正常进入 App、串口打 M1 横幅」这条判据**移到 S5**（App 搬到 Slot A 之后才可观测）。
- 回退：ST-Link 烧回 `_tmp\rescue_m1_mode0_20261016.elf` 即恢复。

---

## 7. 相关文档

- 施工顺序 / 分区表推演：`MVP/docs/plan/STM32边缘AI项目-OTA分区表推演.md`（§6 分区表、§7 顺序、§9 验证手段）
- 分区表**数值权威**：`MVP/docs/plan/STM32边缘AI项目-OTA实施方案.md` §5
- 操作步骤（烧录 / 接线 / 救砖）：`MVP/docs/context/RUNBOOK.md` **R13**
- 决策记录：`MVP/docs/context/DECISIONS.md` D21（分区表）、D23（Bootloader 作为仓库内独立工程）
