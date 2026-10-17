/**
  ******************************************************************************
  * @file    bl_boot.c
  * @brief   Bootloader 主流程。
  *
 * @note    🔴 施工阶段（推演稿 §7）：**本文件当前到 S8 的范围**。
 *
 *          S3 = "读元数据 → 校验镜像 → 合法才跳 / 不合法就留在原地"。
 *          ✅ S5 已真机闭环（把 Slot A 版 App 烧到 `0x0800C000`，跳过去运行正常）。
 *          S6 = "串口收新镜像 → 写 Slot B → 校验 → 置标志 → 复位 → 提交安装"。
 *               协议与实现都在 `bl_ota.c`；本文件只负责**编排**：
 *               ① 上电留一段升级窗口（否则 Slot A 合法就秒跳，PC 没机会插进来）；
 *               ② 复位后若发现 `update_pending=1`，先"提交"再决定跳哪个槽。
 *          ✅ S7 已真机闭环（坏包拒收；并修掉"窗口之后沿用陈旧判断"那个跳转缺陷）。
 *          S8 = "回滚"：两件事
 *               ① **镜像级 CRC32 判据** —— `BL_ImageCheck()` 只看头 8 字节，
 *                  拦不住"写了一半但向量表完好"的槽（S7 实测被它坑到板子静默）。
 *                  现在把 OTA 结束时声明并读回验过的 CRC32 记进元数据
 *                  （`BL_ImgDesc_t`），跳之前**读回整段重算**再决定 —— 闭合 RISKS R22。
 *               ② **试运行计数回滚** —— `commit` 时把新槽标成 `trial=1`；
 *                  之后每次复位进这个槽就 `boot_attempts++`；一旦超过
 *                  `max_boot_attempts` 就**自动切回旧槽**（"启动即崩"的固件
 *                  会不断复位 ⇒ 计数器必然爬满 ⇒ 必然回滚）。
 *                  确认方式（本步）= 串口命令 `k`（清 trial + 计数器）。
 *                  ⚠️ 遗留（有意）：**App 自己起来后自动确认**这一环还没做 ⇒
 *                     正常跑着的新固件若被人工断电 `max_boot_attempts` 次，
 *                     会被误判成"起不来"而回滚。发 `k` 即可消除该状态。
 *
 *          🔴 S3 时期与推演稿原文的差别（重要，见 `RUNBOOK.md` R13.3）：
 *          原稿写"只跳 0x08000000（即现在的 App）"，且判据是"串口仍打 M1 横幅"。
 *          **这两句都做不到**：① 跳 0x08000000 = 跳进自身复位入口（Bootloader 就占那儿）；
 *          ② 把 Bootloader 烧到 0x08000000 这个动作本身就把原来的 App 覆盖了。
 *          ⇒ 判据改到"**不跳**"这一侧：Slot A 此刻还是空白，所以正确的现象是
 *            **明确打印 invalid 并停住**，而板上不再有 M1 横幅是**预期**。
 *
 *          🔴 写元数据的时机现在有**五处**，别再多加：
 *            ① 自愈（空白片）；② S6 的"置标志"（`bl_ota.c`）；
 *            ③ S6 的"提交"（本文件 `BL_TryCommitUpdate()`，复位后只跑一次）；
 *            ④ S8 的"试运行计数 / 回滚"（本文件 `BL_HandleTrial()`）；
 *            ⑤ S8 的"串口确认"（本文件 `BL_HandleCmd()` 的 `k`）。
 *            任何"自动切换"都必须落在这五处，否则"到底哪一步改了状态"就说不清了。
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
#define BL_VERSION_MINOR    (6U)    /* S3 元数据+校验+跳转；S5 真机闭环；S6 串口升级；S7 坏包拒收；S8 镜像级判据 + 试运行计数回滚 */

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
static int  BL_HandleTrial(const BL_Meta_t *m);
static void BL_BootDecision(int haveMeta, const BL_Meta_t *m, int justInstalled);

/* =====================================================================
 * 一、横幅
 * ===================================================================== */
static void BL_PrintBanner(void)
{
  printf("\r\n\r\n");
  printf("=== stm32-ota-bootloader v%u.%u (S8: image-level CRC32 + rollback) ===\r\n",
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
 * =====================================================================
 * 🔴 S8 起改成走 `BL_ImageCheckCrc()`：除了头 8 字节，还要拿元数据里的
 *    **镜像描述子**（`img[id]`）读回整段重算 CRC32。`m == NULL` 或
 *    `m->img[id].len == 0` 时退回仅头校验，并由 `bl_jump.c` 明说一句
 *    "no crc32 descriptor" —— 让"为什么这个槽没被拦"在串口上可查。 */
static BL_ImgStatus BL_ReportSlot(uint8_t id, const BL_Meta_t *m,
                                  uint32_t *spOut, uint32_t *rvOut)
{
  const BL_Slot_t *s = &BL_Slots[id];
  uint32_t sp = 0U;
  uint32_t rv = 0U;
  uint32_t dCrc = 0U;
  uint32_t dLen = 0U;
  BL_ImgStatus st;

  if (m != NULL)
  {
    dCrc = m->img[id].crc32;
    dLen = m->img[id].len;
  }

  st = BL_ImageCheckCrc(s->addr, s->size, dCrc, dLen, &sp, &rv, 1);

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
      printf("[BL]           k = confirm the trial image (clear trial + boot_attempts)\r\n");
      printf("[BL]           m = rewrite metadata (readback verified)\r\n");
      printf("[BL]           r = rescan metadata + both slots\r\n");
      printf("[BL]           ? = this help\r\n");
      break;

    /* 🔴 S8 的"确认"入口。`trial=1` 的槽**每次复位进来**都会让
     *    `boot_attempts` 加一，超过 `max_boot_attempts` 就自动回滚。
     *    新固件跑起来看着没问题，就发一个 `k` 把 trial 与计数器归零 ——
     *    否则日后正常的人工断电也会被算成"试启动失败"，白白回滚。
     *    ⚠️ 将来的正式做法是 **App 起来后自己回来确认**（本步有意没做，
     *       理由见 `DECISIONS.md` D30：动 App 工程要改三个构建配置）。 */
    case 'k':
    case 'K':
    {
      BL_Meta_t n;
      int have = BL_MetaScan(&n);

      if (have == 0)
      {
        printf("[BL] k: no valid metadata -> nothing to confirm\r\n");
      }
      else if ((n.trial == 0U) && (n.boot_attempts == 0U))
      {
        printf("[BL] k: already confirmed (trial=0, boot_attempts=0)\r\n");
      }
      else
      {
        printf("[BL] k: confirming slot %c (trial %u->0, boot_attempts %u->0)\r\n",
               (n.active_slot == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A',
               (unsigned)n.trial, (unsigned)n.boot_attempts);
        n.trial         = 0U;
        n.boot_attempts = 0U;
        if (BL_MetaWrite(&n) != 1)
        {
          printf("[BL] k: confirm write FAILED -> still unconfirmed\r\n");
        }
      }
      break;
    }

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
      (void)BL_MetaPrintRaw();
      if (have != 0)
      {
        BL_MetaPrint(&cur, "meta");
      }
      (void)BL_ReportSlot(BL_SLOT_ID_A, &cur, NULL, NULL);
      (void)BL_ReportSlot(BL_SLOT_ID_B, &cur, NULL, NULL);
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
  /* 🔴 S8：安全等待里也要用**镜像级**判据 —— 否则"半截镜像"在这条路径上
   *    照样会被当成"变合法了"而跳走（S7 实测过的坑，只是入口不同）。 */
  const BL_Meta_t *mp = (haveMeta != 0) ? m : NULL;
  BL_ImgStatus lastSt = BL_IMG_BLANK;   /* 只在状态变化时打印，免得每秒刷一行 */

  if (want > (uint8_t)BL_SLOT_ID_B)
  {
    want = (uint8_t)BL_SLOT_ID_A;   /* 字段被写坏时的兜底 */
  }

  printf("\r\n[BL] ===== safe wait mode: no bootable image =====\r\n");
  printf("[BL] LED 1 Hz slow blink on PA1/D2 (HardFault would be a FAST blink).\r\n");
  printf("[BL] serial commands: ? = help, u = serial OTA, k = confirm, m = rewrite meta, r = rescan\r\n");
  printf("[BL] re-checking %s every 1 s (header + crc32); will jump as soon as it is valid.\r\n",
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
      uint32_t dCrc = 0U;
      uint32_t dLen = 0U;
      BL_ImgStatus st;

      if (mp != NULL)
      {
        dCrc = mp->img[want].crc32;
        dLen = mp->img[want].len;
      }

      /* verbose = 0：轮询不打印细节，由下面这行"状态变了才说"接管。 */
      st = BL_ImageCheckCrc(BL_Slots[want].addr, BL_Slots[want].size,
                            dCrc, dLen, &sp, &rv, 0);

      if (st != lastSt)
      {
        printf("[BL] %s recheck -> %s\r\n", BL_Slots[want].name, BL_ImgStatusStr(st));
        lastSt = st;
      }

      if (st == BL_IMG_OK)
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

  /* 🔴 S8：这里的判据从"头 8 字节"升级成**头 8 字节 + 整镜像 CRC32**。
   *    理由：提交意味着"从此以后从 B 启动"，是**最不能错的一步**；
   *    而 B 刚被擦过、重写过，恰恰是最可能出现"半截镜像"的地方。 */
  if (BL_ImageCheckCrc(BL_SLOT_B_ADDR, BL_SLOT_B_SIZE,
                       m->img[BL_SLOT_ID_B].crc32, m->img[BL_SLOT_ID_B].len,
                       &sp, &rv, 1) != BL_IMG_OK)
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
  /* 🔴 S8：新装的槽一律先当"试运行"—— 接下来每次复位进来都会累加
   *    `boot_attempts`，爬满 `max_boot_attempts` 就自动切回旧槽。
   *    用串口 `k` 确认（或将来由 App 自己确认）才解除。 */
  n.trial          = 1U;

  printf("[BL] install: active_slot %c -> B, update_pending 1 -> 0, trial=1\r\n",
         (m->active_slot == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A');

  if (BL_MetaWrite(&n) != 1)
  {
    printf("[BL] install meta write FAILED -> staying on the old slot\r\n");
    return 0;
  }
  return 1;
}

/* ---------------------------------------------------------------------
 * 四·二、试运行计数与回滚（S8）
 * ---------------------------------------------------------------------
 * 调用时机：**提交之后、启动决策之前** —— 必须在决策之前，因为回滚会改
 * `active_slot`，而决策读的就是它。
 *
 * 🔴 为什么"计数 = 复位次数"这件事就够用，不需要 App 主动上报心跳：
 *    一个能正常跑的新固件**不会自己复位**；而"启动即崩"的固件会不断复位
 *    ⇒ 计数器必然爬满 ⇒ 必然回滚。
 *    反过来，健康设备一直跑着，计数器停在 1 ⇒ **永远不会误回滚**。
 *    这就是"零 App 改动也能做回滚"的全部理由。
 * ⚠️ 代价（有意保留，理由见文件头）：**确认之前人手断电** `max_boot_attempts`
 *    次，会被当成"起不来"而回滚。发一个串口 `k` 即解除该状态；
 *    正式做法是 App 起来后自己确认（本步没做，见 `DECISIONS.md` D30）。
 * 🔴 回滚**只改 `active_slot`，不擦任何 Flash** —— 旧槽的原样内容还在，
 *    这正是"回滚"能成立的物理原因。
 *
 * @retval 1 = 元数据已被改并写盘（调用方需重新扫描后再决策）；0 = 没动
 */
static int BL_HandleTrial(const BL_Meta_t *m)
{
  BL_Meta_t n;
  uint8_t   other;

  if ((m == NULL) || (m->trial == 0U))
  {
    return 0;
  }

  other = (uint8_t)(m->active_slot ^ 1U);
  if (other > (uint8_t)BL_SLOT_ID_B)
  {
    other = (uint8_t)BL_SLOT_ID_A;      /* 字段被写坏时的兜底 */
  }

  n = *m;

  if (m->boot_attempts >= m->max_boot_attempts)
  {
    printf("[BL] trial failed %u/%u (app_version %lu) -> ROLLBACK slot %c -> %c\r\n",
           (unsigned)m->boot_attempts, (unsigned)m->max_boot_attempts,
           (unsigned long)m->app_version,
           (m->active_slot == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A',
           (other == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A');

    n.active_slot   = other;
    n.trial         = 0U;
    n.boot_attempts = 0U;

    if (BL_MetaWrite(&n) != 1)
    {
      printf("[BL] ROLLBACK meta write FAILED -> will retry next boot\r\n");
      return 0;
    }
    printf("[BL] ROLLBACK recorded; booting slot %c instead\r\n",
           (other == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A');
    return 1;
  }

  n.boot_attempts = (uint8_t)(m->boot_attempts + 1U);
  printf("[BL] trial boot %u/%u on slot %c -- unconfirmed, send 'k' to confirm\r\n",
         (unsigned)n.boot_attempts, (unsigned)m->max_boot_attempts,
         (m->active_slot == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A');

  if (BL_MetaWrite(&n) != 1)
  {
    printf("[BL] WARN: could not persist boot_attempts (rollback may be delayed)\r\n");
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

    st = BL_ReportSlot(id, m, &sp, &rv);
    if (st == BL_IMG_OK)
    {
      /* 🔴 上电升级窗口：只在"确实有东西可启动"时才开。
       *    刚提交完升级（justInstalled）那次**跳过** —— 那时候主机
       *    正在读升级后的日志，不该再把它拽回升级流程。 */
      if ((windowDone == 0) && (justInstalled == 0))
      {
        windowDone = 1;
        BL_WaitUpdateWindow();

        /* ============================================================
         * 🔴 S7 修正：窗口里可能**跑过一整套 OTA**。必须在这里重新判一次。
         * ============================================================
         * 升级成功不会走到这行（`BL_OtaRun()` 返回 1 后内部就 `NVIC_SystemReset()`
         * 了）；但**升级失败**（坏帧重传用尽 / 擦除失败 / 镜像 CRC32 不符 /
         * 主机中途断线）会正常返回 —— 而那时目标槽**已经先被擦掉**（擦除发生在
         * 收数据之前），甚至可能只写了一半。
         *
         * 不改的话后果是：`st` / `sp` / `rv` 是**窗口之前**读的旧值，
         * 下面那句 `JUMP` 会照样执行 ⇒ 明明刚打印了
         * "image REJECTED ... -> stay in bootloader"，下一行却跳进那个坏槽
         * （2026-10-16 实测踩到：Slot B 被擦成空白后仍按旧的 MSP/入口跳，
         *  极端情况就是拿 `MSP=0xFFFFFFFF` 进 App ⇒ 必 HardFault）。
         *
         * 修正：**重新读一次**，不合法就 `continue` 去试另一个槽；
         * 两个都不行自然落到下面那句 "neither slot is bootable" ⇒ 进安全等待。
         *
         * ⚠️ 残留（留给 `S8`）：`BL_ImageCheck()` 只看**头 8 字节**（初始 MSP +
         *    Reset 向量），所以"擦了一半、但向量表完好"的槽**仍会被判成合法**。
         *    要根治必须让启动决策能校验**整镜像**（把声明的 CRC32 记进元数据、
         *    启动时读回重算），那是 `S8` 回滚判据的一部分。 */
        st = BL_ReportSlot(id, m, &sp, &rv);
        if (st != BL_IMG_OK)
        {
          printf("[BL] %s became INVALID during the update window -> NOT jumping\r\n",
                 BL_Slots[id].name);
          continue;
        }
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

  /* ---- S8：试运行计数 / 回滚 ------------------------------------------
   * 🔴 必须夹在"提交"与"决策"之间：提交把它标成 `trial=1`，这里决定
   *    "这次还试不试得起"，**回滚会改 `active_slot`** —— 所以它跑完之后
   *    一定要重扫，否则决策会拿着刚被自己改掉的旧值去跳。 */
  if (BL_HandleTrial((haveMeta != 0) ? &meta : NULL) != 0)
  {
    haveMeta = BL_MetaScan(&meta);
    if (haveMeta != 0)
    {
      BL_MetaPrint(&meta, "after trial");
    }
  }

  BL_BootDecision(haveMeta, &meta, installed);

  for (;;) { }   /* BL_BootDecision 不返回；这里只是形式上的收尾 */
}
