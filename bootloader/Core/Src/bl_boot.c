/**
  ******************************************************************************
  * @file    bl_boot.c
  * @brief   Bootloader 主流程。
  *
  * @note    🔴 施工阶段（推演稿 §7）：**本文件当前到 S3 的范围**。
  *
  *          S3 = "读元数据 → 校验镜像 → 合法才跳 / 不合法就留在原地"。
  *          🔴 与推演稿原文的差别（重要，见 `RUNBOOK.md` R13.3）：
  *          原稿写"只跳 0x08000000（即现在的 App）"，且判据是"串口仍打 M1 横幅"。
  *          **这两句都做不到**：① 跳 0x08000000 = 跳进自身复位入口（Bootloader 就占那儿）；
  *          ② 把 Bootloader 烧到 0x08000000 这个动作本身就把原来的 App 覆盖了。
  *          ⇒ 判据改到"**不跳**"这一侧：Slot A 此时还是空白，所以正确的现象是
  *            **明确打印 invalid 并停住**，而板上不再有 M1 横幅是**预期**。
  *
  *          S3 仍然**不写除了参数区以外**的任何 Flash、不碰 App 槽。
  ******************************************************************************
  */
#include "main.h"
#include "bl_boot.h"
#include "bl_config.h"
#include "bl_flash.h"
#include "bl_jump.h"
#include "bl_meta.h"

#include <stdio.h>

/* 🔴 `huart1` 是 `main.c` 里的文件级全局变量，**没有**在 `main.h` 里 extern
 *    （CubeMX 的既有习惯：谁用谁自己 extern —— 参考 App 的 `stm32f4xx_it.c`
 *    第 65 行也是这么写的）。照这个惯例办，别去改 CubeMX 会重新生成的 main.h。 */
extern UART_HandleTypeDef huart1;

/* =====================================================================
 * 版本号 —— 升级判定的第一个抓手。
 * 🔴 以后排查"到底烧进去的是哪一版"时，这一行是唯一可信的锚：
 *    别靠文件时间戳，那玩意儿在复制/解压时会变。 */
#define BL_VERSION_MAJOR    (0U)
#define BL_VERSION_MINOR    (3U)    /* S3：加元数据 + 校验 + 跳转 */

/* =====================================================================
 * 槽表 —— 一处定义，后面只按 id 取
 * ===================================================================== */
typedef struct
{
  const char *name;
  uint32_t    addr;
  uint32_t    size;
} BL_Slot_t;

static const BL_Slot_t BL_Slots[2] =
{
  { "slot A", BL_SLOT_A_ADDR, BL_SLOT_A_SIZE },
  { "slot B", BL_SLOT_B_ADDR, BL_SLOT_B_SIZE },
};

static void BL_SafeWait(const BL_Meta_t *m, int haveMeta);

/* =====================================================================
 * 一、横幅
 * ===================================================================== */
static void BL_PrintBanner(void)
{
  printf("\r\n\r\n");
  printf("=== stm32-ota-bootloader v%u.%u (S3: metadata + verify + jump) ===\r\n",
         (unsigned)BL_VERSION_MAJOR, (unsigned)BL_VERSION_MINOR);
  /* 构建时间戳同样有用：OTA 只会替换 App，Bootloader 是长期不动的，
   * 这一行能立刻回答"现在板上跑的是哪次编的 Bootloader"。 */
  printf("built " __DATE__ " " __TIME__ ", SYSCLK = %lu Hz\r\n",
         (unsigned long)HAL_RCC_GetSysClockFreq());

  printf("flash map (scheme B / D21) -- 数值唯一来源 Core/Inc/bl_config.h:\r\n");
  printf("  bootloader   0x%08lX  %4lu KB  S%lu+S%lu\r\n",
         (unsigned long)BL_BOOT_ADDR, (unsigned long)(BL_BOOT_SIZE / 1024UL),
         (unsigned long)BL_BOOT_SECTOR, (unsigned long)(BL_BOOT_SECTOR + 1UL));
  printf("  metadata     0x%08lX  %4lu KB  S%lu      (record log + CRC32)\r\n",
         (unsigned long)BL_META_ADDR, (unsigned long)(BL_META_SIZE / 1024UL),
         (unsigned long)BL_META_SECTOR);
  printf("  slot A (run) 0x%08lX  %4lu KB  S%lu..S%lu\r\n",
         (unsigned long)BL_SLOT_A_ADDR, (unsigned long)(BL_SLOT_A_SIZE / 1024UL),
         (unsigned long)BL_SLOT_A_SECTOR, (unsigned long)BL_SLOT_A_SECTOR + 3UL);
  printf("  slot B (new) 0x%08lX  %4lu KB  S%lu..S%lu\r\n",
         (unsigned long)BL_SLOT_B_ADDR, (unsigned long)(BL_SLOT_B_SIZE / 1024UL),
         (unsigned long)BL_SLOT_B_SECTOR, (unsigned long)BL_SLOT_B_SECTOR + 2UL);
  printf("  log / data   0x%08lX  %4lu KB  S%lu+S%lu\r\n",
         (unsigned long)BL_LOG_ADDR, (unsigned long)(BL_LOG_SIZE / 1024UL),
         (unsigned long)BL_LOG_SECTOR, (unsigned long)(BL_LOG_SECTOR + 1UL));
  printf("  meta record: %lu B x %lu (= %lu KB)\r\n",
         (unsigned long)BL_META_REC_SIZE,
         (unsigned long)BL_META_REC_COUNT,
         (unsigned long)(BL_META_SIZE / 1024UL));
}

/* =====================================================================
 * 二、槽探测（**只读**，无副作用）
 * ===================================================================== */
static BL_ImgStatus BL_ReportSlot(uint8_t id, uint32_t *spOut, uint32_t *rvOut)
{
  const BL_Slot_t *s = &BL_Slots[id];
  uint32_t sp = 0U;
  uint32_t rv = 0U;
  BL_ImgStatus st = BL_ImageCheck(s->addr, s->size, &sp, &rv);

  printf("[BL] %s @0x%08lX (%lu KB): first=0x%08lX reset=0x%08lX -> %s\r\n",
         s->name, (unsigned long)s->addr, (unsigned long)(s->size / 1024UL),
         (unsigned long)sp, (unsigned long)rv, BL_ImgStatusStr(st));

  if (spOut != NULL) { *spOut = sp; }
  if (rvOut != NULL) { *rvOut = rv; }
  return st;
}

/* =====================================================================
 * 三、安全等待模式（没有可启动镜像时停在这里）
 * =====================================================================
 * 🔴 这里**不是**最终的升级模式 —— 真正的收包/校验/写 Slot B 在 S6 落地。
 *    S3 的这一段只做三件事：证明"活着且没锁死"、可被 PC 端问话、
 *    以及**每秒重看一眼槽**（S6 收到新镜像后可以原地跳走，不用复位）。
 *
 *    LED 用 **1 Hz 慢闪**，与 HardFault 的"快闪"明确区分：
 *    现场不看串口，光看灯就能分辨"正常等待"还是"崩了"。 */
static void BL_HandleCmd(uint8_t ch)
{
  BL_Meta_t cur;

  switch (ch)
  {
    case '?':
      printf("[BL] commands: m = rewrite metadata (readback verified)\r\n");
      printf("[BL]           r = rescan metadata + both slots\r\n");
      printf("[BL]           ? = this help\r\n");
      break;

    /* 🔴 'm' 是判据②的"随取随验"入口：把"写→读回→CRC 一致"整条链路
     *    再走一遍并逐项打印。这样即使过了首次上电（自愈已跑过），
     *    任何时候都能重新取证，而不是"只有第一次上电才看得到"。 */
    case 'm':
    {
      int have = BL_MetaScan(&cur);
      if (have == 0)
      {
        BL_MetaDefaults(&cur);
        printf("[BL] no valid metadata -> writing defaults\r\n");
      }
      if (BL_MetaWrite(&cur) != 0)
      {
        BL_Meta_t chk;
        if (BL_MetaScan(&chk) != 0)
        {
          BL_MetaPrint(&chk, "rescan");
        }
      }
      break;
    }

    case 'r':
    {
      int have = BL_MetaScan(&cur);
      BL_MetaPrintRaw();
      if (have != 0)
      {
        BL_MetaPrint(&cur, "meta");
      }
      (void)BL_ReportSlot(BL_SLOT_ID_A, NULL, NULL);
      (void)BL_ReportSlot(BL_SLOT_ID_B, NULL, NULL);
      break;
    }

    case '\r':
    case '\n':
      break;

    default:
      printf("[BL] unknown cmd 0x%02X (press ? for help)\r\n", (unsigned)ch);
      break;
  }
}

static void BL_SafeWait(const BL_Meta_t *m, int haveMeta)
{
  uint32_t half = 0U;
  uint8_t  want = (haveMeta != 0) ? m->active_slot : (uint8_t)BL_SLOT_ID_A;

  if (want > (uint8_t)BL_SLOT_ID_B)
  {
    want = (uint8_t)BL_SLOT_ID_A;   /* 字段被写坏时的兜底 */
  }

  printf("\r\n[BL] ===== safe wait mode: no bootable image =====\r\n");
  printf("[BL] LED 1 Hz slow blink on PA1/D2 (HardFault would be a FAST blink).\r\n");
  printf("[BL] serial commands: ? = help, m = rewrite metadata, r = rescan\r\n");
  printf("[BL] re-checking %s every 1 s; will jump as soon as a valid image shows up.\r\n",
         BL_Slots[want].name);

  for (;;)
  {
    uint8_t ch = 0U;

    half++;
    HAL_Delay(500U);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);          /* 500 ms 翻转 = 1 Hz 慢闪 */

    if ((half % 4U) == 0U)                               /* 每 2 s 一行，不刷屏 */
    {
      printf("[BL] safe, t=%lu s\r\n", (unsigned long)(half / 2U));
    }

    /* 非阻塞收 1 字节：Timeout = 0 ⇒ 没数据立即返回 HAL_TIMEOUT，不会卡住心跳。 */
    if (HAL_UART_Receive(&huart1, &ch, 1U, 0U) == HAL_OK)
    {
      BL_HandleCmd(ch);
    }

    if ((half % 2U) == 0U)                               /* 每秒重新审视一次 */
    {
      uint32_t sp = 0U;
      uint32_t rv = 0U;
      if (BL_ImageCheck(BL_Slots[want].addr, BL_Slots[want].size, &sp, &rv) == BL_IMG_OK)
      {
        printf("[BL] %s became valid while waiting -> jumping\r\n", BL_Slots[want].name);
        BL_JumpToApp(BL_Slots[want].addr, sp, rv);
      }
    }
  }
}

/* =====================================================================
 * 四、启动决策
 * =====================================================================
 * 顺序：先看元数据指定的槽（active_slot）；它不合法则看**另一个**槽
 * （回滚/降级用得上，S8 会在这里加 boot_attempts 计数）；两个都不行才停。
 *
 * 🔴 故意**不**在这里自作主张改元数据：S3 阶段任何"自动切换"都会让
 *    "到底哪一步改了状态"变得说不清。写元数据的时机只有两个：
 *    自愈（空白片）与 S6/S8 的显式升级流程。 */
static void BL_BootDecision(int haveMeta, const BL_Meta_t *m)
{
  uint8_t want  = (haveMeta != 0) ? m->active_slot : (uint8_t)BL_SLOT_ID_A;
  uint8_t other;
  uint8_t pass;

  if (want > (uint8_t)BL_SLOT_ID_B)
  {
    want = (uint8_t)BL_SLOT_ID_A;
  }
  other = (uint8_t)(want ^ 1U);

  printf("[BL] boot decision: active_slot=%c (metadata %s)\r\n",
         (want == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A',
         (haveMeta != 0) ? "valid" : "absent -> assuming A");

  for (pass = 0U; pass < 2U; pass++)
  {
    uint8_t id = (pass == 0U) ? want : other;
    uint32_t sp = 0U;
    uint32_t rv = 0U;
    BL_ImgStatus st;

    if ((pass == 1U) && (haveMeta == 0))
    {
      /* 元数据是自愈出来的默认值（= A）时，不必再试 B —— 那不是"回滚"，
       * 只是"另一块同样没编程的空白"。少打印一行噪音。 */
      break;
    }

    st = BL_ReportSlot(id, &sp, &rv);
    if (st == BL_IMG_OK)
    {
      if (pass == 1U)
      {
        printf("[BL] NOTE: falling back to %s because %s is not bootable\r\n",
               BL_Slots[id].name, BL_Slots[want].name);
      }
      printf("[BL] JUMP -> %s @0x%08lX : VTOR=0x%08lX MSP=0x%08lX entry=0x%08lX\r\n",
             BL_Slots[id].name, (unsigned long)BL_Slots[id].addr,
             (unsigned long)BL_Slots[id].addr, (unsigned long)sp, (unsigned long)rv);
      printf("[BL] UART1 + PA1 handed over; banner below comes from the App, not from here.\r\n");
      BL_JumpToApp(BL_Slots[id].addr, sp, rv);
    }
  }

  printf("[BL] slot A invalid -> stay in bootloader (no jump).\r\n");
  printf("[BL] 🔴 this is the EXPECTED result at S3: slot A is still blank until S5.\r\n");
  printf("[BL] 🔴 absence of the App('M1') banner is expected, NOT a failure.\r\n");
  BL_SafeWait(m, haveMeta);
}

/* =====================================================================
 * 五、入口
 * ===================================================================== */
void BL_Run(void)
{
  BL_Meta_t meta;
  int haveMeta;

  BL_PrintBanner();
  (void)BL_FlashMapVerify();     /* 运行期分区表自检（编译期自检看不到"扇区号写错"） */
  (void)BL_CrcSelfTest();        /* 元数据的完整性全靠它，先证明它是对的 */

  /* ---- 元数据：一次扫描定乾坤 ----------------------------------------
   * 🔴 顺序刻意如此：**先扫（只读）→ 打印原始头几字 → 才决定要不要写**。
   *    这样"凭什么认为它无效"在串口上是看得见的（原始字 + 扫描统计），
   *    而不是只留一句"已自愈"。 */
  haveMeta = BL_MetaScan(&meta);
  BL_MetaPrintRaw();

  if (haveMeta == 0)
  {
    if (BL_MetaSelfHeal() != 0)
    {
      /* 自愈之后再扫一次：这一步是判据②的正面证据 ——
       * 元数据不是"以为写进去了"，而是**从 Flash 重新读出来**的。 */
      haveMeta = BL_MetaScan(&meta);
      if (haveMeta != 0)
      {
        BL_MetaPrint(&meta, "after self-heal");
      }
    }
    else
    {
      printf("[BL] SELF-HEAL FAILED -- metadata unavailable (continuing read-only)\r\n");
    }
  }
  else
  {
    BL_MetaPrint(&meta, "metadata");
  }

  BL_BootDecision(haveMeta, &meta);

  for (;;) { }   /* BL_BootDecision 不返回；这里只是形式上的收尾 */
}
