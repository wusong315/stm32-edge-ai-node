# bootloader —— OTA Bootloader 工程（**S7 已完成：串口升级 + 坏包拒收真机闭环**）

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

## 2. 当前进度：**`S8` 完成 ⇒ `S2`–`S8` 全部收口**（推演稿 §7）

| 项 | 结果 |
|---|---|
| 命令行编译 | ✅ **0 error / 0 warning**，`make clean && make all` 全量重编（26 个目标文件） |
| `.isr_vector` VMA | ✅ `0x08000000`（`objdump -h` 实测） |
| 向量表内容 | ✅ 初始 MSP `0x20020000`（RAM 顶）+ Reset 向量（奇数、落在本区） |
| Flash 占用 | 🔴 **32 344 B ≈ 31.59 KB / 32 KB（98.7 %）**（`text 32 204 + data 140`） |
| ↳ 余量 | 🔴 **只剩 424 B** 就碰到参数区 `0x08008000`（`FLASH LENGTH = 32K` 仍是硬护栏） |
| RAM 占用 | ✅ `data 140 + bss 3 012 = 3 152 B`（含 `.ld` 预留的 heap 512 + stack 1 024；**收包缓冲 1 024 B 必须是 `static`**） |
| 串口升级（`S6`） | 🎉 **2026-10-16 一次跑通，6/6 判据全过**：42 帧 / 42 788 B / 4.1 s / 10.2 KB·s⁻¹；读完回 CRC32 `MATCH`；ST-Link 把 Slot B 读回**逐字节相同**；复位后 `active_slot=B` 并跳到 B |
| 坏包拒收（`S7`） | 🎉 **2026-10-16 跑通，5/5 判据全过**：`frame 0 crc16 FAIL -> NAK` ×8 → `gave up -> abort` → **不置标志、不提交** → `slot B became INVALID … -> NOT jumping` → `JUMP -> slot A … entry=0x0800E4C1`。🔴 同批修掉两个真问题（`--corrupt` 没造出坏帧 / 窗口后沿用陈旧判断） |
| 回滚（`S8`） | 🎉 **2026-10-16 三项全过**：① **镜像级 CRC32** 判据 ⇒ `--corrupt 20` **7/7**（`image crc32 MISMATCH: got 0x69F3D04F declared 0x612D7DCE over 42 788 B` → `NOT jumping` → `JUMP -> slot A`，**`RISKS` R22 闭环**）；② **复位计数回滚** ⇒ 砖镜像 **8/8**（`trial failed 3/3 (app_version 9) -> ROLLBACK slot B -> A`，零人工干预）；③ 正常升级 **8/8**；另有 `k` 确认实测 |

> ⚠️ 以上是 **Debug（`-O0`）** 的体积。Release（`-Os`）会明显更小 —— 但 Bootloader 本来就跑在启动关键路径上，
> 「Debug 版也塞得下 32 KB」是更强的结论，所以以 Debug 数字为准。

> 🔴 **预算账（`RISKS` R21 的实测答案）**：`S3`→`S6` 用掉 **+5 744 B**，其中 **约 2.6 KB 是 `.rodata`（日志文案）**；
> `S7` 的 Fix A 再吃 **132 B**；**`S8`（镜像级 CRC32 + 回滚）再吃 2 456 B**（`.rodata` 5 928 → **7 140 B**）。
> 也就是说「多打印几行」和「多写一个函数」是同一量级的代价，**日志不是免费的**。
> 🔴 **余量只剩 424 B** —— 连"多打印三行"都装不下。**功能面到此为止**；
> 真要再加东西，退路依次是：① 本工程改 `-Os`（**仍未用过，依然完好** —— 且这是**下一个动作**）；② 精简日志文案；③ 极端情况才动分区表。

---

## 3. 代码结构（6 个模块，职责单一）

```
Core/Src/
  main.c      最小初始化（GPIO + USART1）→ BL_Run()
  bl_boot.c   编排：横幅 → 分区表自检 → 两个 CRC 自检 → 读/自愈元数据
              → 升级提交（update_pending→active_slot）→ **试运行计数/回滚(trial)**
              → 决策 → 上电升级窗口 → 跳或等
  bl_flash.c  扇区层：地址↔扇区号派生 / 擦除 / 编程 / 分区表机器自检
  bl_meta.c   元数据层：CRC32 + 顺序追加日志 + 自愈 + 写后读回校验 + 镜像描述子
  bl_jump.c   跳转层：镜像校验（**头 8 字节 + 整镜像 CRC32**）+ 外设 DeInit + 跳转
  bl_ota.c    【S6】串口升级：帧层 CRC16 + 文本握手 + 停等收帧
              + 写 Slot B + 读回整镜像 CRC32 + 置 update_pending + **写描述子**
Core/Inc/  同名头文件 + bl_config.h（分区表唯一来源）+ bl_ota.h（**帧格式唯一来源**）
test/brick/ 【S8】「启动即复位」诱饵镜像（**只用来验回滚**，不参与任何构建）
```

三层拆开是为了**每一步都能直接复用**：`bl_flash.c` 的擦/写被 `bl_ota.c` 直接用；
`bl_jump.c` 的校验被 `bl_ota.c` 的"写完再判能不能跳"和 `bl_boot.c` 的回退分支共用。

### 启动时序（串口上能看到的）

```
横幅(v0.6 + 构建时间 + SYSCLK + 分区表)
  └─ flash map verify   ← 逐个分区：由地址反推的扇区号 == 期望值？+ 是否整数个完整扇区？
  └─ crc32 selftest     ← "123456789" -> 0xCBF43926 ?（元数据完整性全靠它）
  └─ crc16 selftest     ← "123456789" -> 0xBB3D ?（升级帧层完整性全靠它）
  └─ metadata region raw head + scan 统计
  └─ 无有效记录 ⇒ self-heal（擦 S2 + 写默认值）⇒ 写后读回校验 ⇒ 再扫一次
  └─ update_pending=1 ⇒ 复验 Slot B ⇒ 提交（active_slot→B、清标志、**trial=1**）→ **重新扫一遍元数据**
  └─ 【S8】trial=1 ⇒ boot_attempts+1 写回（**达上限就改 active_slot 回另一个槽**）
  └─ boot decision      ← 合法就跳（**跳之前先开 2 s 升级窗口**）；不合法 ⇒ safe wait
```

### 串口命令（`?` 可随时看）

| 键 | 作用 |
|---|---|
| `?` | 帮助 |
| **`u`** | **进入串口升级模式**（`S6`）—— 上电窗口里、或安全等待模式里都能用 |
| **`k`** | 【`S8`】**确认当前槽通过试运行**（`trial=0`、`boot_attempts=0`）—— 见下方"回滚" |
| `m` | **重写元数据 + 读回校验**（"写→读回→CRC 一致"的随取随验入口，不必重新上电） |
| `r` | 重新扫描元数据 + 打印两个槽状态 |

> 🔴 **`u` 只在上电后那 2 s 窗口里喊得到**（`BL_OTA_WINDOW_MS`）。原因：只要目标槽合法，
> `BL_BootDecision()` 几毫秒就跳走了，PC 根本插不进来 —— **没有这个窗口，串口升级这条路就不存在**。
> 上位机 `tools/ota_send.py` 的做法是**一直喊**（每 200 ms 一次）并让 ST-Link 复位一次去撞这个窗口。

### 串口升级（`S6`/`S7`）的协议与三条纪律

**帧格式的权威在 `Core/Inc/bl_ota.h` 的头部注释**，实现与"为什么这么设计"在 `Core/Src/bl_ota.c` 的头部注释；
操作步骤与实测记录在 `MVP/docs/context/RUNBOOK.md` **R13.10 / R13.11**。这里只留最该记住的三条：

1. **必须停等**（一帧一个 `'K'`）：F4 擦/写期间 CPU 取不了指，而 UART 的 RX 寄存器只有 **1 个字节** ⇒
   流式灌必然**静默丢字节**。停等把"设备忙"变成"主机在等"，零流控绕开。
2. **收数据期间一个字节都不许打印**：`printf` 阻塞 TX 好几毫秒，同样会丢。
3. **两层校验各管一件事**：逐帧 `CRC16` 管**传输**（坏帧进不了 Flash，`NAK` 让主机重发）；
   收完再从 Flash **读回整段重算 `CRC32`** 管**落盘**。只有后者过才置 `update_pending`。

> ⚠️ **升级目标槽当前固定是 Slot B**，且必须发 **SlotB 链接版镜像**（`SlotB/stm32-edge-ai-node.elf` → `objcopy -O binary`）。
> 一旦 `active_slot=B`，中途失败的升级会污染**正在运行的槽**。
>
> ✅ **"写了一半的槽仍会跳进去"这个缺陷已在 `S8` 修掉**（`RISKS` R22 闭环）。两层判据各管一件事：
> `BL_ImageCheck()` 只读**头 8 字节**（够快，但"擦一半/写一半"的槽**向量表往往完好**）；
> `BL_ImageCheckCrc()` 再加一层 —— 元数据里存着**主机声明的 CRC32 与长度**，启动前**从 Flash 读回整段重算**再比。
> 实测（`--corrupt 20`）：`image crc32 MISMATCH: got 0x69F3D04F declared 0x612D7DCE over 42 788 B` → `NOT jumping` → `JUMP -> slot A`。
> 详见 `RUNBOOK.md` **R13.13**。

### 回滚（`S8`）：靠"复位次数"兜住"合法但跑不起来"的镜像

`S8` 要兜的不是"内容坏掉"（那有 CRC32 管），而是"**内容完好、向量表合法、但一跑就崩**" —— 这种镜像能过所有静态校验，
只有跑起来才知道不行。解法是**试运行计数**：

| 字段 | 含义 |
|---|---|
| `trial` | 1 = 这个槽还没被确认过（刚提交上来） |
| `boot_attempts` | 试运行期间已经启动过几次 |
| `max_boot_attempts` | 上限，默认 **3** |

- **提交**升级时：`active_slot→B`、`trial=1`、`boot_attempts=0`；
- **每次上电**：`trial==1` ⇒ `boot_attempts+1` 写回；到 `>= max` ⇒ **`active_slot` 改回另一个槽、`trial=0`、计数清零**；
- 🔴 **回滚只改一个字节的 `active_slot`，不擦任何 Flash** —— 旧槽原样还在，这是"回滚"能成立的物理原因；
- **确认**：跑稳了发一个 `k`（`trial=0`、计数清零）。`S8` 实测：

```
[BL] trial boot 1/3 → 2/3 → 3/3            ← 砖镜像每次上电就把自己复位
[BL] 🔴 trial failed 3/3 (app_version 9) -> ROLLBACK slot B -> A
[BL] JUMP -> slot A … entry=0x0800E4C1
```

🔴 **已知局限（`DECISIONS.md` D30 ①，刻意不做）**：**App 起来后不会自动确认**。
⇒ **每次正常升级完要发一个 `k`**，否则连续复位 3 次就自动回滚（即使新固件完全正常）。
正解（App 回写 `trial=0`）要碰参数区 + 三个 Build Configuration，按取舍留作局限。

**为什么"复位次数"就是够用的计数**：能正常跑的固件**不会自己复位**；"启动即崩"的固件**必然不断复位**
⇒ 计数器**必然爬满** ⇒ **必然回滚**。这就是"零 App 改动也能做回滚"的全部理由。

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
# 另：nm 里应有 BL_Crc32 / BL_MetaWrite / BL_MetaScan / BL_JumpToApp / BL_FlashMapVerify /
#     BL_OtaRun / BlRecvFrame / BL_TryCommitUpdate / BL_OtaSelfTest ——
#     若被 --gc-sections 收走，说明"编译过了但其实没链进去"
```

> 🔴 **`bl_ota.c` 不能放进 `-O0` 就随便写的名单里**：它有一处**只有真机会暴露**的陷阱 ——
> `uint8_t buf[1024]` 这种局部数组会直接吃光栈（`_Min_Stack_Size = 0x400`）。
> 收包缓冲**必须**是 `static uint32_t wbuf[BL_OTA_CHUNK/4]`（既避开栈，又保证 `HAL_FLASH_Program` 要的 4 字节对齐）。

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
S1 分区表定稿 ✅  →  S2 建工程+编译 ✅  →  S3 元数据+校验+跳转 ✅  →  S4 App 双配置 ✅
  →  S5 烧 Slot A（⚠️ 最危险）✅  →  S6 串口升级 ✅  →  S7 坏包拒收 ✅  →  S8 回滚
```

> ✅ **`S7` 已于 2026-10-16 跑通（5/5）**：设备侧**确实不用新代码**（逐帧 CRC16 → `NAK` → 重传 → 用尽即中止且不置标志），
> 改动只有一处 —— **上电升级窗口返回后重新判一次目标槽**（否则会沿用窗口前的旧判断，跳进刚被擦掉的槽）。
> 🔴 **`S8`（回滚）的关键不是 `boot_attempts`，而是"镜像级判据"**：`BL_ImageCheck()` 只看头 8 字节，
> 拦不住"写了一半但向量表完好"的槽。正解 = `BL_OtaRun()` 成功时把握手里声明的 CRC32 写进元数据，
> 启动决策**读回整槽重算**再决定跳不跳。`boot_attempts` / `max_boot_attempts` 字段与提交时的清零已就位。
> 完整规格与实测（含两条反例）见 `MVP/docs/context/RUNBOOK.md` **R13.12**。

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
- **操作步骤（烧录 / 接线 / 救砖 / 各步判据勘误）**：`MVP/docs/context/RUNBOOK.md` **R13**
- **`S6` 串口升级：协议规格 + 三条纪律 + 已知局限**：`RUNBOOK.md` **R13.10**
- **`S6` 实测记录（编译预算 / 完整时序 / 四条独立取证 / 回退路径）**：`RUNBOOK.md` **R13.11**
- **`S7` 规格 + 实测（坏包拒收 5/5 / 残余缺陷 / 三条修复）**：`RUNBOOK.md` **R13.12**
- 上位机：`tools/ota_send.py`（`--selftest` 固定向量 / `--dry-run` / `--corrupt N` / `--stlink-reset`；
  ⚠️ `--corrupt` 是**在算完 CRC 之后再改字节**，别改回去 —— 先改 payload 会得到自洽帧，等于没测）
- 决策记录：`MVP/docs/context/DECISIONS.md` D21（分区表）、D23（Bootloader 作为仓库内独立工程）、
  D24（元数据做成顺序日志）、**D25（跳转须放开 PRIMASK）**、D26（本工程禁 `%f`）、D27（槽间差异是三项）、
  **D28（`S6` 串口升级协议与实现）**、**D29（`S7`：坏帧怎么造 / 窗口后必须重判目标槽 / 残余归 `S8`）**
- 参考仓库速读：`MVP/docs/plan/REF-F4-NANOEDGE.md` 与 `REF-REPOS-P1-2.md`
