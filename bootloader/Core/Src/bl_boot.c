/**
  ******************************************************************************
  * @file    bl_boot.c
  * @brief   Bootloader 主流程。
  *
 * @note    🔴 施工阶段（推演稿 §7）：**本文件当前到 S6 的范围**。
 *
 *          S3 = "读元数据 → 校验镜像 → 合法才跳 / 不合法就留在原地"。
 *          ✅ S5 已真机闭环（把 Slot A 版 App 烧到 `0x0800C000`，跳过去运行正常）。
 *          S6 = "串口收新镜像 → 写 Slot B → 校验 → 置标志 → 复位 → 提交安装"。
 *               协议与实现都在 `bl_ota.c`；本文件只负责**编排**：
 *               ① 上电留一段升级窗口（否则 Slot A 合法就秒跳，PC 没机会插进来）；
 *               ② 复位后若发现 `update_pending=1`，先"提交"再决定跳哪个槽。
 *
 *          🔴 S3 时期与推演稿原文的差别（重要，见 `RUNBOOK.md` R13.3）：
 *          原稿写"只跳 0x08000000（即现在的 App）"，且判据是"串口仍打 M1 横幅"。
 *          **这两句都做不到**：① 跳 0x08000000 = 跳进自身复位入口（Bootloader 就占那儿）；
 *          ② 把 Bootloader 烧到 0x08000000 这个动作本身就把原来的 App 覆盖了。
 *          ⇒ 判据改到"**不跳**"这一侧：Slot A 此刻还是空白，所以正确的现象是
 *            **明确打印 invalid 并停住**，而板上不再有 M1 横幅是**预期**。
 *
 *          🔴 写元数据的时机现在有**三个**，别再多加：
 *            ① 自愈（空白片）；② S6 的"置标志"（`bl_ota.c`）；
 *            ③ S6 的"提交"（本文件 `BL_TryCommitUpdate()`，复位后只跑一次）。
 *            任何"自动切换"都必须落在这三处，否则"到底哪一步改了状态"就说不清了。
 ******************************************************************************
 */
#include "main.h"
#include "bl_boot.h"
#include "bl_config.h"
#include "bl_flash.h"
#include "bl_jump.h"
#include "bl_meta.h"
#include "bl_ota.h"

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
#define BL_VERSION_MINOR    (4U)    /* S3：元数据 + 校验 + 跳转；S5：真机闭环；S6：串口升级 */

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
static void BL_HandleCmd(uint8_t ch);
static void BL_WaitUpdateWindow(void);

/* =====================================================================
 * 一、横幅
 * ===================================================================== */
static void BL_PrintBanner(void)
{
  printf("\r\n\r\n");
  printf("=== stm32-ota-bootloader v%u.%u (S6: serial OTA -> slot B) ===\r\n",
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
      printf("[BL] commands: u = enter serial update mode (S6)\r\n");
      printf("[BL]           m = rewrite metadata (readback verified)\r\n");
      printf("[BL]           r = rescan metadata + both slots\r\n");
      printf("[BL]           ? = this help\r\n");
      break;

    /* 🔴 升级入口。`BL_OtaRun()` 返回 1 = 镜像已写进 Slot B 且校验通过、
     *    `update_pending` 已置位 ⇒ **必须复位**，让启动流程去"提交"。
     *    直接在这里复位（而不是返回一个标志让上层处理）是刻意的：
     *    两个入口（上电窗口 / 安全等待）都会调到这个 switch，
     *    复位只写一处，就不可能出现"某条路径忘了复位"。 */
    case 'u':
    case 'U':
    {
      if (BL_OtaRun() != 0)
      {
        printf("[BL] update staged in slot B -> NVIC_SystemReset() now\r\n");
        HAL_Delay(50U);                    /* 把最后一句日志放完再复位 */
        NVIC_SystemReset();
      }
      /* 没升级成功：什么都不改，回到调用者（该跳跳、该等等）。 */
      break;
    }

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

/* =====================================================================
 * 三·一、上电升级窗口（S6 才有的东西）
 * =====================================================================
 * 🔴 **为什么必须有**：只要目标槽里有合法镜像，`BL_BootDecision()` 会在
 *    上电后**几毫秒内**就跳走 —— PC 根本来不及说话，`u` 命令永远收不到，
 *    串口升级这条路等于不通。所以"跳之前先等一小会儿"不是花哨功能，
 *    而是 S6 **能不能用**的前提。
 *
 * 代价：每次上电多等 BL_OTA_WINDOW_MS。调试期完全可接受。
 * 期间收到别的字符照样走命令表（`?` / `m` / `r` 都能用），
 * 收到 `u` 就进升级、并且**立刻结束窗口**（成功则内部会复位，
 * 失败就继续正常启动，不让用户卡在这一步）。 */
static void BL_WaitUpdateWindow(void)
{
  uint32_t waited = 0U;

  printf("[BL] update window %u ms: send 'u' to enter serial OTA (any other key = normal boot)\r\n",
         (unsigned)BL_OTA_WINDOW_MS);

  while (waited < BL_OTA_WINDOW_MS)
  {
    uint8_t ch = 0U;

    HAL_Delay(BL_OTA_WINDOW_POLL_MS);
    waited += BL_OTA_WINDOW_POLL_MS;

    /* 非阻塞（Timeout = 0）：窗口是"限时"的，不能被一个字节卡住。 */
    if (HAL_UART_Receive(&huart1, &ch, 1U, 0U) == HAL_OK)
    {
      BL_HandleCmd(ch);                    /* 'u' 在这里会跑完整套升级流程 */
      if ((ch == 'u') || (ch == 'U'))
      {
        return;                            /* 试过一次就够了，别再让用户等 */
      }
    }
  }

  printf("[BL] window closed (no 'u') -> boot\r\n");
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
  printf("[BL] serial commands: ? = help, u = serial OTA, m = rewrite metadata, r = rescan\r\n");
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
 * 🔴 故意**不**在这里自作主张改元数据：任何"自动切换"都会让
 *    "到底哪一步改了状态"变得说不清。写元数据的时机只有三处：
 *    自愈（空白片）、S6 的显式升级（置标志）、以及下面的"提交"。
 */

/* ---------------------------------------------------------------------
 * 四·一、提交上一次串口升级（S6）
 * ---------------------------------------------------------------------
 * 复位后只做一次：`update_pending=1` 且 **Slot B 确实现已可启动** ⇒
 * 把 `active_slot` 切到 B、清标志、清 boot_attempts，并**重新从 Flash 读回**。
 *
 * 🔴 为什么"置标志"和"提交"要分成两次写、中间隔一个复位：
 *    这样"新镜像写好了"和"下次从哪起"是两个独立的事实，各自在串口上留痕。
 *    掉电夹在中间也无害 —— 标志已置、镜像已校验，复位后照样提交。
 * 🔴 为什么 Slot B 不合法时要**清标志**而不是干等：
 *    否则每次上电都重复报同一句话，而且万一以后有人把 A 也弄坏了，
 *    就会陷在"标志一直在、B 又不能用"的状态里。清掉即回到已知可用路径。 */
static int BL_TryCommitUpdate(int haveMeta, const BL_Meta_t *m)
{
  BL_Meta_t n;
  uint32_t  sp = 0U;
  uint32_t  rv = 0U;

  if ((haveMeta == 0) || (m->update_pending == 0U))
  {
    return 0;
  }

  printf("[BL] update_pending=1 -> verify slot B before install\r\n");

  if (BL_ImageCheck(BL_SLOT_B_ADDR, BL_SLOT_B_SIZE, &sp, &rv) != BL_IMG_OK)
  {
    printf("[BL] slot B is NOT bootable -> drop the pending flag, keep %c\r\n",
           (m->active_slot == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A');
    n = *m;
    n.update_pending = 0U;
    if (BL_MetaWrite(&n) != 1)
    {
      printf("[BL] WARN: could not clear update_pending (will retry next boot)\r\n");
    }
    return 0;
  }

  n = *m;
  n.active_slot    = (uint8_t)BL_SLOT_ID_B;
  n.update_pending = 0U;
  n.boot_attempts  = 0U;

  printf("[BL] install: active_slot %c -> B, update_pending 1 -> 0\r\n",
         (m->active_slot == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A');

  if (BL_MetaWrite(&n) != 1)
  {
    printf("[BL] install meta write FAILED -> staying on the old slot\r\n");
    return 0;
  }
  return 1;
}

static void BL_BootDecision(int haveMeta, const BL_Meta_t *m, int justInstalled)
{
  uint8_t want  = (haveMeta != 0) ? m->active_slot : (uint8_t)BL_SLOT_ID_A;
  uint8_t other;
  uint8_t pass;
  int     windowDone = 0;

  if (want > (uint8_t)BL_SLOT_ID_B)
  {
    want = (uint8_t)BL_SLOT_ID_A;
  }
  other = (uint8_t)(want ^ 1U);

  printf("[BL] boot decision: active_slot=%c (metadata %s)%s\r\n",
         (want == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A',
         (haveMeta != 0) ? "valid" : "absent -> assuming A",
         (justInstalled != 0) ? " [just installed from slot B]" : "");

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
      /* 🔴 上电升级窗口：只在"确实有东西可启动"时才开。
       *    刚提交完升级（justInstalled）那次**跳过** —— 那时候主机
       *    正在读升级后的日志，不该再把它拽回升级流程。 */
      if ((windowDone == 0) && (justInstalled == 0))
      {
        windowDone = 1;
        BL_WaitUpdateWindow();
      }

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

  printf("[BL] neither slot is bootable -> stay in bootloader (no jump).\r\n");
  printf("[BL] send 'u' within the update window, or press 'u' here, to flash a new image.\r\n");
  BL_SafeWait(m, haveMeta);
}

/* =====================================================================
 * 五、入口
 * ===================================================================== */
void BL_Run(void)
{
  BL_Meta_t meta;
  int haveMeta;
  int installed = 0;

  BL_PrintBanner();
  (void)BL_FlashMapVerify();     /* 运行期分区表自检（编译期自检看不到"扇区号写错"） */
  (void)BL_CrcSelfTest();        /* 元数据的完整性全靠它，先证明它是对的 */
  (void)BL_OtaSelfTest();        /* 升级通道的完整性靠帧层 CRC16，同理先证明它是对的 */

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

  /* ---- S6：先把"上一次串口升级"提交掉，再决定跳哪个槽 ------------------
   * 🔴 顺序不能反：`BL_BootDecision()` 看的是 `meta.active_slot`，
   *    而提交会改它 ⇒ 提交完**必须重新扫一遍**，否则决策用的是旧值。 */
  installed = BL_TryCommitUpdate(haveMeta, &meta);
  if (installed != 0)
  {
    haveMeta = BL_MetaScan(&meta);
    if (haveMeta != 0)
    {
      BL_MetaPrint(&meta, "after install");
    }
  }

  BL_BootDecision(haveMeta, &meta, installed);

  for (;;) { }   /* BL_BootDecision 不返回；这里只是形式上的收尾 */
}
