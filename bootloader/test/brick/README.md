# brick —— 「启动即复位」镜像（`S8` 回滚验收用的诱饵）

## 它是什么

一个 **540 字节**的固件，内容只有一件事：**上电立刻请求复位，然后死循环**。

```c
__attribute__((section(".isr_vector"), used))
const uint32_t g_vectors[2] = { 0x20020000u, (uint32_t)(uintptr_t)&Reset_Handler };

void Reset_Handler(void) { *(volatile uint32_t *)0xE000ED0Cu = 0x05FA0004u; for(;;){} }
```

`0xE000ED0C` 是 `SCB->AIRCR`，写 `0x05FA0004` = `SYSRESETREQ`。

## 为什么需要它

`S8` 要验的是「**一个跑不起来的镜像，会不会被自动切走**」。这个诱饵的特别之处在于：

| 属性 | 值 | 意义 |
|---|---|---|
| 向量表（头 8 字节） | 合法（MSP `0x20020000` / Reset `0x08060009`） | 能过 `BL_ImageCheck()` 的头校验 |
| 整镜像 CRC32 | 与元数据里声明的**完全一致** | 能过 `S8` 的 `BL_ImageCheckCrc()` |
| 实际行为 | 复位 → 复位 → 复位 … | **只有「复位计数」能兜住** |

⇒ 它专门用来证明 **`BL_HandleTrial()` 的计数回滚是唯一能兜住"合法但跑不起来"的手段**。
（相比之下 `ota_send.py --corrupt N` 造的是「内容坏掉」的镜像，验的是**判据**；两者验的不是同一件事。）

## 怎么编

```bash
cd D:/STM32Projects/stm32-edge-ai-node

GCC=$(ls D:/mestm32/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/*gnu-tools*/tools/bin/arm-none-eabi-gcc.exe | head -1)
OC=$(dirname "$GCC")/arm-none-eabi-objcopy.exe

"$GCC" -mcpu=cortex-m4 -mthumb -O0 -nostdlib -nostartfiles \
  -T bootloader/test/brick/brick.ld bootloader/test/brick/brick.c \
  -o /tmp/brick.elf
"$OC" -O binary /tmp/brick.elf /tmp/brick.bin
```

## 判据（编完先自查，别拿去烧之前才发现不对）

| 项 | 期望值 |
|---|---|
| 文件大小 | **540 B** |
| `zlib.crc32` | **`0xDB666DE1`** |
| 头 8 字节 | `00 00 02 20`（MSP `0x20020000`）+ `09 00 06 08`（Reset `0x08060009`，**奇数 = Thumb**） |
| Reset 是否落在槽内 | `0x08060009` ∈ [`0x08060000`, `0x080C0000`) ✅ |

```bash
D:/STM32Projects/pc-venv/Scripts/python.exe -c "import zlib;d=open('/tmp/brick.bin','rb').read();print(len(d), hex(zlib.crc32(d)), d[:8].hex())"
# 540 0xdb666de1 0000022009000608
#                 MSP=0x20020000  Reset=0x08060009（最低位 1 = Thumb，奇数）
```

🔴 **必须用 `-O0`**：`-O1` / `-Os` / `-Og` 会产出 **536 B / `0xD7E09B81`**（编译器把死循环优化掉了一条指令）。
行为一样能复位，但**大小与 CRC32 会与 `RUNBOOK.md` R13.13 记录的实测值对不上** —— 那份记录的可信度就来自"逐字节可复现"。

## 怎么用

```bash
# 用正常的串口 OTA 把它灌进 Slot B（不是 ST-Link！这样才能顺带验"提交"那步）
D:/STM32Projects/pc-venv/Scripts/python.exe tools/ota_send.py \
  --bin /tmp/brick.bin --port COM5 --ver 9 --stlink-reset
```

`--ver 9` 是为了让回滚日志一眼能认出「坏的是哪一版」：
`[BL] trial failed 3/3 (app_version 9) -> ROLLBACK slot B -> A`。

然后**不要碰板子**，连续复位 4 次（或直接拔插一次电源、再等它自己复位）即可看到回滚。
⚠️ **回滚之后要恢复**：重跑一次正常升级，**再发一个 `k`**（否则下次复位 3 次又会滚回 A）——
见 `RUNBOOK.md` R13.13 第六节。

## 与主工程的边界

- 这个目录**不参与** App / Bootloader 的任何构建（`bootloader/Debug/subdir.mk` 只扫 `Core/`，本目录不在编译清单里）；
- 它**不需要** STM32 HAL，`-nostdlib -nostartfiles` 直接出裸镜像；
- 唯一的产物是那个 `.bin`，**不要把它烧到 `0x08000000`**（那是 Bootloader 的位置）。
