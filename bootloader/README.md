# bootloader —— OTA Bootloader 工程（**S3 已完成**）

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
| 参数区（元数据，记录日志 + CRC32） | `0x08008000` | 16 KB | S2 |
| App A（运行槽） | `0x0800C000` | 336 KB | S3–S6 |
| App B（备用槽，收新固件） | `0x08060000` | 384 KB | S7–S9 |
| 日志 / 数据 | `0x080C0000` | 256 KB | S10+S11 |

---

## 2. 当前进度：**S3**（推演稿 §7）

| 项 | 结果 |
|---|---|
| 命令行编译 | ✅ **0 error / 0 warning**，`make clean && make all` 全量重编（25 个目标文件） |
| `.isr_vector` VMA | ✅ `0x08000000`（`objdump -h` 实测） |
| 向量表内容 | ✅ 初始 MSP `0x20020000`（RAM 顶）+ Reset 向量 `0x08001D41`（奇数、落在本区） |
| Flash 占用 | ✅ **24 012 B ≈ 23.45 KB / 32 KB（73.3 %）**，最高占用地址 `0x08005DD4` |
| ↳ 距参数区起点 | ✅ 还差 **8 748 B ≈ 8.54 KB** 才碰到 `0x08008000`（32 K 链接上限即是硬护栏） |
| RAM 占用 | ✅ `data 140 + bss 1 988 = 2 128 B`（其中 1 536 B 是 `.ld` 预留的 heap+stack） |
| App 工程影响 | ✅ **零改动**（App 全量重编 `42 240 + 528 = 42 768 B` / RAM 3 976 B，与施工前**逐字节一致**） |

> ⚠️ 以上是 **Debug（`-O0`）** 的体积。Release（`-Os`）会明显更小 —— 但 Bootloader 本来就跑在启动关键路径上，
> 「Debug 版也塞得下 32 KB」是更强的结论，所以以 Debug 数字为准。

> 🔴 **预算提醒（给 S6 的）**：S2→S3 用掉 9 868 B，其中我们自己的 5 个模块合计 7 985 B
> （`bl_boot` 3 293 / `bl_meta` 2 682 / `bl_flash` 1 409 / `bl_jump` 601）。
> 余量 8.54 KB。S6（串口收包协议 + 写 Slot B）也吃这一块，
> **但不会悄悄撑破**：`.ld` 的 `FLASH LENGTH = 32K` 会在**链接期直接报错**。
> 真到那天，退路依次是：① 本工程改 `-Os`；② 精简日志文案（`.rodata` 占 3 364 B）。

---

## 3. S3 的代码结构（5 个模块，职责单一）

```
Core/Src/
  main.c      最小初始化（GPIO + USART1）→ BL_Run()
  bl_boot.c   编排：横幅 → 分区表自检 → CRC 自检 → 读/自愈元数据 → 决策 → 跳或等
  bl_flash.c  【新】扇区层：地址↔扇区号派生 / 擦除 / 编程 / 分区表机器自检
  bl_meta.c   【新】元数据层：CRC32 + 顺序追加日志 + 自愈 + 写后读回校验
  bl_jump.c   【新】跳转层：镜像校验 + 外设 DeInit + 跳转
Core/Inc/  同名头文件 + bl_config.h（分区表唯一来源）
```

三层拆开是为了**后面几步能直接复用**：`bl_flash.c` 的擦/写就是 S6 写 Slot B 要用的，
`bl_jump.c` 的校验/跳转就是 S8 回滚要用的。

### S3 启动时序（串口上能看到的）

```
横幅(v0.3 + 构建时间 + SYSCLK + 分区表)
  └─ flash map verify   ← 逐个分区：由地址反推的扇区号 == 期望值？+ 是否整数个完整扇区？
  └─ crc32 selftest     ← "123456789" -> 0xCBF43926 ?（元数据完整性全靠它，先证明它是对的）
  └─ metadata region raw head + scan 统计
  └─ 无有效记录 ⇒ self-heal（擦 S2 + 写默认值）⇒ 写后读回校验 ⇒ 再扫一次
  └─ slot probe A / B   ← 只读
  └─ boot decision      ← 合法就跳；不合法 ⇒ safe wait（LED 1 Hz 慢闪 + 串口命令）
```

### 安全等待模式下的串口命令（S6 会在此扩展）

| 键 | 作用 |
|---|---|
| `?` | 帮助 |
| `m` | **重写元数据 + 读回校验**（判据②的"随取随验"入口，不必重新上电） |
| `r` | 重新扫描元数据 + 打印两个槽状态 |

等待时**每秒重看一眼目标槽**：S6 收到新镜像后可以**原地跳走，不需要复位**。

---

## 4. 🔴 S3 期间发现的两处「照抄参考仓库会出事」

### 4.1 「双副本交替写」在单扇区里**做不到** ⇒ 必须做成顺序日志

推演稿 §4.3 要求「照抄它的元数据结构，但**必须加 CRC + 双副本交替写**（它没有 CRC）」。

**实测推演后的结论**：F4 的**擦除粒度是整个扇区**，而参数区 16 KB **恰好等于一个扇区**（S2）。
⇒ 「擦掉旧副本、写新副本」这个动作**根本执行不了**：擦一个副本必然把另一个也擦掉，
中间出现"零份有效记录"的窗口 —— 而那个窗口正是双副本想消除的东西。

**正确实现 = 顺序追加日志**：

- 新记录写在扇区内**第一个空白（`0xFFFFFFFF`）位置**，**不需要擦除**；
- 读的时候扫全扇区，取 `seq` 最大且 CRC 正确的那条；
- 掉电在写记录中途 → 该条 CRC 不过 → 自动回退到上一条 ⇒ **掉电安全**；
- 128 条写满才擦一次 ⇒ 擦除窗口被摊薄 128 倍；
- 真在擦除窗口掉电 → 下次上电走自愈 → 写默认值 → 默认值 = **从 Slot A 启动**
  ⇒ 落到"已知可用的那份固件"，属于**失效安全**而非失效危险。

语义上仍然是"多副本 + 交替写"，且**任意时刻至少有一份有效**。

> 📌 通用价值：**"双副本"这类词要先问一句"擦除/写入的最小单位是多少"**。
> 在 NOR Flash 上，任何"就地更新一条记录"的方案都必须先回答扇区问题。

### 4.2 跳转前**必须放开 `PRIMASK`** ⇒ 否则 App 会静默假死

参考仓库（以及 ST 的多数例程）跳转前是 `__disable_irq()` 收尾，然后直接跳。
**对它那个 App 没问题** —— 那个 App 根本没用中断。

**我们的 App 不行**：它靠 **SysTick**（`HAL_Delay` / `HAL_GetTick`）和 **TIM2 的 1 kHz 中断**活着。
而 CPU 复位后 `PRIMASK = 0`，App 的 `main()` 里**没有** `__enable_irq()`
（已 grep 核对：`../Core/Src/main.c` 只在 `Error_Handler` 里出现 `__disable_irq()`）。

⇒ 带着 `PRIMASK = 1` 跳进去，App 会：时钟配好、外设配好、然后**卡在第一个 `HAL_Delay()` 里永远不动**，
串口一行不出、LED 不闪。**现场表现是"App 根本没起来"**，会把你引向"是不是固件坏了"，
而真凶在 Bootloader 的最后两条指令里。

本工程的 `bl_jump.c` 因此按这个顺序跳：

```
DeInit 外设 → __disable_irq() → 关 SysTick → 清空 NVIC 全部使能位与挂起位
  → SCB->VTOR = appAddr → __DSB/__ISB
  → __set_PRIMASK(0)    ← 🔴 参考仓库没有这一步
  → MSR MSP + BX（内联汇编，保证换栈与跳转之间不碰栈）
```

> ⚠️ 此时**已经没有任何中断源被使能**，所以放开 `PRIMASK` 不会让中断立刻打进来 ——
> 顺序不能颠倒（先改 `VTOR` 再放开、却又留着中断源，就会用 App 的向量表跑 Bootloader 的上下文）。

---

## 5. 怎么编译（不需要 IDE、不需要硬件）

用用户级技能 `stm32-cubeide-cli-build`：

```bash
python ~/.workbuddy/skills/stm32-cubeide-cli-build/scripts/build.py \
  . --config Debug --clean \
  --cubeide-root "D:/mestm32/STM32CubeIDE_2.2.0"
```

核对地址（**不要只看"编译成功"**）：

```bash
ARM=<CubeIDE>/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin
$ARM/arm-none-eabi-objdump -h Debug/bootloader.elf | head -8   # .isr_vector VMA 必须 = 08000000
$ARM/arm-none-eabi-objcopy -O binary Debug/bootloader.elf /tmp/bl.bin
# bl.bin 的前 8 字节 = 初始 MSP + Reset 向量（后者必须是奇数）
# 另：nm 里应有 BL_Crc32 / BL_MetaWrite / BL_MetaScan / BL_JumpToApp / BL_FlashMapVerify —— 
#     若被 --gc-sections 收走，说明"编译过了但其实没链进去"
```

> 🔴 **改了 `Core/Src/*.c` 的增删**（新加文件），必须同步更新
> `Debug/objects.list` + `Debug/Core/Src/subdir.mk`。本工程的清单由
> `D:\STM32Projects\_tmp\gen_bl_manifests.py` **模板整体重建**（原因见 R13.1 的 LF/CRLF 坑）。
> `build.py` 的 `sync_objects_list()` **只补不漏**。

---

## 6. 与 App 工程的差异（只有这些，别让它们漂移）

| 维度 | App（`../`） | Bootloader（本目录） |
|---|---|---|
| `.ld` 的 `FLASH LENGTH` | `1024K` | **`32K`**（超出即链接报错，自带预算守卫） |
| 外设 | I2C1 / TIM2 / TIM3 / USART1 / USART3 | **只有 USART1 + LED(PA1)** |
| HAL 模块 | 18 个 | **14 个**（去掉 i2c / i2c_ex / tim / tim_ex） |
| `-u _printf_float` | 开（要打 `%.3f`） | **关**（省 ≈10.5 KB）⇒ **本工程不许出现 `%f`** |
| 链接的库 | `-L Core/Src -lneai` | **无**（`.cproject` 与 `Debug/objects.mk` 都已清空） |
| `printf` 落点 | 环形队列（保 1 kHz 节拍） | **阻塞直发**（无实时节拍，少一类竞态） |
| HardFault | `while(1)` | **PA1 快闪**（与 1 Hz 慢闪区分 ⇒ 一眼分清"跑飞"和"在等你"） |
| 时钟树 | HSE 8 MHz → 168 MHz | **逐字相同**（刻意） |

> 🔴 **`%f` 静默失效的坑（RISKS R13）**：本工程特意没开 `-u _printf_float`，
> newlib-nano 遇到 `%f` **不报错、不警告，直接打印 `0.0000`**。
> `bl_meta.c` 里打印标定系数因此写成「乘 1000 后按整数打印」——
> 这不是美观问题，是"看起来对但全是 0"的问题。

---

## 7. 施工阶段纪律（照抄推演稿 §7，**不许跳**）

```
S1 分区表定稿 ✅  →  S2 建工程+编译 ✅  →  S3 元数据+校验+跳转 ✅  →  S4 App 双配置
  →  S5 烧 Slot A（⚠️ 最危险）  →  S6 串口升级  →  S7 坏包拒收  →  S8 回滚
```

🔴 **一旦把 Bootloader 烧进 `0x08000000`，原先跑在那儿的 App 就被覆盖了** ——
这是 **`S3` 就会发生**的事（不是 `S5`）。所以**上板前**必须常备两份救砖件：

| 救砖件 | 路径 | 用途 |
|---|---|---|
| 原始 App 固件（mode 0，`0x08000000` / 1024K 版） | `D:\STM32Projects\_tmp\rescue_m1_mode0_20261016.elf` | ST-Link 烧回 `0x08000000`，恢复成"没有 Bootloader 的原始状态" |
| 原版链接脚本 | `D:\STM32Projects\_tmp\STM32F407VGTX_FLASH.orig_1M.ld` | 随时能重编一份"直烧就能跑"的固件 |

```bash
"D:\MeSTM32CubePR\bin\STM32_Programmer_CLI.exe" -c port=SWD \
  -w "D:\STM32Projects\_tmp\rescue_m1_mode0_20261016.elf" -v -rst
```

---

## 8. 🔴 已知的文档勘误（施工中发现，勿按原稿执行）

推演稿 §7 的 **S3 判据原文**是「只跳 `0x08000000`（即现在的 App）」+「上电后能正常进入现有 App（串口仍打 `M1` 横幅）」。

**这两句互斥、且都做不到**：

1. **跳回自己**：Bootloader 按分区表就占 `0x08000000`（S0+S1，32 KB）。跳 `0x08000000` = 跳进自身复位入口 → 立刻重入，永远进不了 App。
2. **覆盖 App**：把 Bootloader 烧到 `0x08000000` 这个动作本身就会**覆盖掉**原先在那里运行的 App ⇒ "串口仍打 `M1` 横幅"在物理上不可能成立。

**修正后的 S3**（判据改为「不跳」侧，见 `RUNBOOK.md` **R13.3**）：

- 逻辑：读元数据（空白 → 写默认 + 自愈）→ 校验槽内镜像合法性（栈顶落 `0x20000000–0x20020000` 且 8 字节对齐、复位向量为奇数且落在本槽）→ **合法才跳 `0x0800C000`**；不合法则**不跳**，进安全等待（LED 1 Hz 慢闪 + 串口周期打印）。
- 判据：① 出 Bootloader 横幅；② 元数据自愈可读（写→读回→CRC 一致）；③ **Slot A 尚未编程 ⇒ 明确打印 `slot A invalid -> stay in bootloader`，不跳**；④ **板上不再有 `M1` 横幅 —— 这是预期结果，不是故障**。
- 「进入 App、串口打 `M1` 横幅」这条判据**移到 S5**（App 搬到 Slot A 之后才可观测）。
- 回退：ST-Link 烧回 `rescue_m1_mode0_20261016.elf` 即恢复。

---

## 9. 相关文档

- 施工顺序 / 分区表推演：`MVP/docs/plan/STM32边缘AI项目-OTA分区表推演.md`（§6 分区表、§7 顺序、§9 验证手段、§11 增量复核）
- 分区表**数值权威**：`MVP/docs/plan/STM32边缘AI项目-OTA实施方案.md` §5
- **操作步骤（烧录 / 接线 / 救砖 / S3 判据勘误）**：`MVP/docs/context/RUNBOOK.md` **R13**
- 决策记录：`MVP/docs/context/DECISIONS.md` D21（分区表）、D23（Bootloader 作为仓库内独立工程）、**D24（元数据做成顺序日志 + 跳转须放开 PRIMASK）**
- 参考仓库速读：`MVP/docs/plan/REF-F4-NANOEDGE.md` 与 `REF-REPOS-P1-2.md`
