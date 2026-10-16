/**
  ******************************************************************************
  * @file    bl_boot.c
  * @brief   Bootloader 主流程。
  *
  * @note    🔴 施工阶段（推演稿 §7）：**本文件当前只到 S2 的范围**。
  *          S2 判据 = 「命令行编译 0 error/0 warning + `.isr_vector` VMA 落在
  *          `0x08000000` + 体积 < 32 KB」。所以这里只有：
  *            ① 打印横幅（含分区表全表 —— 让"地址表编译期自检通过"可见）
  *            ② **只读**探测两个槽当前是否为空（为 S5 做准备，零副作用）
  *            ③ 心跳循环（证明时钟树 / 串口 / LED 都活着）
  *          元数据读写、CRC32、DeInit 与跳转**留到 S3**。
  *
  * 🔴 S2 **绝不写 Flash、绝不跳转** —— 这样它在任何时刻都可以被丢弃，
  *    而板上跑的还是原来那份 App。
  ******************************************************************************
  */
#include "main.h"
#include "bl_boot.h"
#include "bl_config.h"

#include <stdio.h>

/* =====================================================================
 * 版本号 —— 升级判定的第一个抓手。
 * 🔴 以后排查"到底烧进去的是哪一版"时，这一行是唯一可信的锚：
 *    别靠文件时间戳，那玩意儿在复制/解压时会变。 */
#define BL_VERSION_MAJOR    (0U)
#define BL_VERSION_MINOR    (2U)    /* S2：第 2 次施工（S1 = 纸上定案） */

/* =====================================================================
 * 一、槽探测（**只读**）
 * =====================================================================
 * 判据取"向量表首字是否为 0xFFFFFFFF"：擦除后的 Flash 全是 0xFF，
 * 而未编程的槽就是这个状态。这是最省事、又不会误判的空白判据。
 *
 * ⚠️ 这里**故意不做"镜像是否合法"的完整校验**（栈顶落在 RAM、复位向量为奇数
 *    且落在本槽区间）—— 那是 S3 的事，且完整校验必须配合"跳到哪一槽"的决策，
 *    拆开写只会让 S2 提前引入不该有的复杂度。 */
static uint8_t BL_SlotLooksBlank(uint32_t addr)
{
  return (*(volatile uint32_t *)addr == 0xFFFFFFFFUL) ? 1U : 0U;
}

static void BL_ReportSlot(const char *tag, uint32_t addr, uint32_t size)
{
  uint32_t first = *(volatile uint32_t *)addr;
  uint32_t reset = *(volatile uint32_t *)(addr + 4UL);

  printf("  %s @0x%08lX (%4lu KB): first=0x%08lX reset=0x%08lX -> %s\r\n",
         tag,
         (unsigned long)addr,
         (unsigned long)(size / 1024UL),
         (unsigned long)first,
         (unsigned long)reset,
         (BL_SlotLooksBlank(addr) != 0U) ? "BLANK (未编程)" : "programmed");
}

/* =====================================================================
 * 二、横幅
 * ===================================================================== */
static void BL_PrintBanner(void)
{
  printf("\r\n\r\n");
  printf("=== stm32-ota-bootloader v%u.%u ===\r\n",
         (unsigned)BL_VERSION_MAJOR, (unsigned)BL_VERSION_MINOR);
  /* 构建时间戳同样有用：OTA 只会替换 App，Bootloader 是长期不动的，
   * 这一行能立刻回答"现在板上跑的是哪次编的 Bootloader"。 */
  printf("built " __DATE__ " " __TIME__ ", SYSCLK = %lu Hz\r\n",
         (unsigned long)HAL_RCC_GetSysClockFreq());

  printf("flash map (scheme B / D21) -- 数值见 Core/Inc/bl_config.h:\r\n");
  printf("  bootloader   0x%08lX  %4lu KB  S0+S1\r\n",
         (unsigned long)BL_BOOT_ADDR, (unsigned long)(BL_BOOT_SIZE / 1024UL));
  printf("  metadata     0x%08lX  %4lu KB  S2   (double copy, CRC)\r\n",
         (unsigned long)BL_META_ADDR, (unsigned long)(BL_META_SIZE / 1024UL));
  printf("  slot A (run) 0x%08lX  %4lu KB  S3..S6\r\n",
         (unsigned long)BL_SLOT_A_ADDR, (unsigned long)(BL_SLOT_A_SIZE / 1024UL));
  printf("  slot B (new) 0x%08lX  %4lu KB  S7..S9\r\n",
         (unsigned long)BL_SLOT_B_ADDR, (unsigned long)(BL_SLOT_B_SIZE / 1024UL));
  printf("  log / data   0x%08lX  %4lu KB  S10+S11\r\n",
         (unsigned long)BL_LOG_ADDR, (unsigned long)(BL_LOG_SIZE / 1024UL));

  printf("[BL] slot probe (read-only):\r\n");
  BL_ReportSlot("slot A", BL_SLOT_A_ADDR, BL_SLOT_A_SIZE);
  BL_ReportSlot("slot B", BL_SLOT_B_ADDR, BL_SLOT_B_SIZE);
  BL_ReportSlot("meta  ", BL_META_ADDR, BL_META_SIZE);
}

/* =====================================================================
 * 三、S2 的等待循环
 * =====================================================================
 * 🔴 这里**不是**最终的升级等待模式，只是"证明它活着"。
 *    真正的升级模式（收包 / 校验 / 写 Slot B）在 S6 落地。
 *    心跳用 LED（PA1 / D2，低电平点亮）1 Hz 翻转 + 每秒一行串口 ——
 *    两条独立信道同时给证据，避免"串口没输出"时无法区分"没跑"和"串口挂了"。 */
static void BL_IdleLoop(void)
{
  uint32_t t = 0U;

  printf("[BL] S2 skeleton -- no metadata, no jump, no flash write yet.\r\n");
  printf("[BL] next step S3: read metadata + decide slot (see RUNBOOK R13).\r\n");

  for (;;)
  {
    HAL_Delay(1000U);
    t++;
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    printf("[BL] alive, t=%lu s\r\n", (unsigned long)t);
  }
}

/* =====================================================================
 * 四、入口
 * ===================================================================== */
void BL_Run(void)
{
  BL_PrintBanner();
  BL_IdleLoop();
  /* 不会执行到这里：BL_IdleLoop() 是死循环。留这一行只为满足"函数不返回"的直觉。 */
}
