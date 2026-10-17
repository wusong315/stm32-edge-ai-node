/**
  ******************************************************************************
  * @file    bl_meta.c
  * @brief   参数区（元数据）实现 —— CRC32 + 顺序追加日志 + 自愈。
  ******************************************************************************
  */
#include "main.h"
#include "bl_config.h"
#include "bl_flash.h"
#include "bl_meta.h"

#include <stdio.h>
#include <string.h>

/* =====================================================================
 * 一、CRC32
 * =====================================================================
 * 位运算版（无 256 项常量表）：省下 1 KB Flash，代价是每字节 8 次移位。
 * 元数据一条 124 B，一次扫描最多 128 条 ≈ 127 KB ≈ 100 万次移位 ——
 * 在 168 MHz 上不到 10 ms，而这件事一上电只做一次。**用不着查表换速度。** */
uint32_t BL_Crc32(const uint8_t *p, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;
  uint32_t b;

  if (p == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint32_t)p[i];
    for (b = 0U; b < 8U; b++)
    {
      /* 反射式：右移一位，若移出的位是 1 则异或多项式。
       * (crc & 1) ? 0xEDB88320 : 0 —— 写成无分支形式。 */
      crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(0UL - (crc & 1UL)));
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

int BL_CrcSelfTest(void)
{
  /* 🔴 这是 CRC-32/ISO-HDLC 的标准检查值（与 zlib.crc32 一致）。
   *    期望值先在 PC 上用 Python 的 zlib 算过，不是抄来的。 */
  static const char v[] = "123456789";
  uint32_t got      = BL_Crc32((const uint8_t *)v, 9U);
  uint32_t expected = 0xCBF43926UL;
  int ok = (got == expected);

  printf("[BL] crc32 selftest: \"123456789\" -> 0x%08lX (expect 0x%08lX) %s\r\n",
         (unsigned long)got, (unsigned long)expected, ok ? "PASS" : "FAIL");
  return ok;
}

/* =====================================================================
 * 二、扫描状态缓存
 * =====================================================================
 * 🔴 用"文件级静态变量"而不是把状态塞进每条记录：这些量是**一次扫描的产物**，
 *    不是要掉电保存的东西。让 BL_MetaWrite 每次先重新扫描，就不会出现
 *    "缓存过期 → 把新记录写到已被占用的位置" 这类自己骗自己的错误。 */
static uint32_t g_validCount   = 0U;                  /* 有效记录条数 */
static uint32_t g_newestSeq    = 0U;                  /* 最新记录的 seq */
static uint32_t g_firstFreeOff = BL_META_SIZE;        /* 首个空白记录偏移；= 区大小表示"没有空位" */
static int      g_haveAny      = 0;                   /* 是否有过有效记录 */

/* ---------------------------------------------------------------------
 * 读一条记录到 RAM（Flash 是内存映射的，直接读即可）。
 * 🔴 用 memcpy 而不是结构体指针强转：结构体成员访问会引入编译器
 *    的对齐假设，而 Flash 里的记录**未必**是我们这一版布局写的
 *    （可能是上一版固件留下的、也可能是残留 App 机器码）。
 *    按字节搬进本地对齐的变量里，再逐字段判定，最安全。 */
static void BL_MetaReadRec(uint32_t off, BL_Meta_t *dst)
{
  memcpy((void *)dst, (const void *)(BL_META_ADDR + off), (size_t)BL_META_REC_SIZE);
}

/* 判定一条记录是否"有效"：魔数 + 版本 + CRC 三段全过才算。
 * ⚠️ crc_span 也要先验：被打断的写入可能把 crc_span 本身写成垃圾，
 *    若直接拿它当长度算 CRC 就变成"用垃圾校验垃圾"。 */
static int BL_MetaRecIsValid(const BL_Meta_t *m)
{
  if (m->magic != BL_META_MAGIC)
  {
    return 0;
  }
  if (m->struct_version != (uint16_t)BL_META_STRUCT_VERSION)
  {
    return 0;
  }
  if (m->crc_span != (uint16_t)BL_META_CRC_SPAN)
  {
    return 0;
  }
  return (BL_Crc32((const uint8_t *)m, BL_META_CRC_SPAN) == m->crc32) ? 1 : 0;
}

/* 该位置是否从未被编程（擦除态）。 */
static int BL_MetaRecIsErased(uint32_t off)
{
  return (*(volatile uint32_t *)(BL_META_ADDR + off) == 0xFFFFFFFFUL) ? 1 : 0;
}

int BL_MetaScan(BL_Meta_t *out)
{
  uint32_t off;
  BL_Meta_t rec;
  BL_Meta_t best;

  g_validCount   = 0U;
  g_newestSeq    = 0U;
  g_firstFreeOff = BL_META_SIZE;   /* 先假设"写满了"，扫到空位再改 */
  g_haveAny      = 0;

  memset((void *)&best, 0, sizeof(best));

  for (off = 0U; (off + BL_META_REC_SIZE) <= BL_META_SIZE; off += BL_META_REC_SIZE)
  {
    /* 首个空白位置 = 下一条记录的落点（追加写无需擦除的关键） */
    if ((g_firstFreeOff == BL_META_SIZE) && (BL_MetaRecIsErased(off) != 0))
    {
      g_firstFreeOff = off;
    }

    BL_MetaReadRec(off, &rec);
    if (BL_MetaRecIsValid(&rec) != 0)
    {
      g_validCount++;
      /* seq 取最大者；相等时"靠后的位置"胜出（同一序号理论上不会出现，
       * 但真出现了，物理上后写的更可信）。 */
      if ((g_haveAny == 0) || (rec.seq >= g_newestSeq))
      {
        best       = rec;
        g_newestSeq = rec.seq;
        g_haveAny  = 1;
      }
    }
  }

  if (out != NULL)
  {
    *out = best;
  }
  return g_haveAny;
}

/* =====================================================================
 * 三、默认值 / 打印
 * ===================================================================== */
void BL_MetaDefaults(BL_Meta_t *m)
{
  if (m == NULL)
  {
    return;
  }
  memset((void *)m, 0, sizeof(BL_Meta_t));

  m->magic            = BL_META_MAGIC;
  m->struct_version   = (uint16_t)BL_META_STRUCT_VERSION;
  m->crc_span         = (uint16_t)BL_META_CRC_SPAN;
  m->seq              = 0U;                 /* BL_MetaWrite 会覆盖 */
  m->active_slot      = (uint8_t)BL_SLOT_ID_A;   /* 出厂 = 从 Slot A 启动 */
  m->update_pending   = 0U;
  m->boot_attempts    = 0U;
  m->max_boot_attempts = 3U;                /* S8：连试 3 次不成就回滚 */
  m->app_version      = 0U;
  m->neai_model_ver   = 0U;
  m->calib_k[0] = 1.0f; m->calib_k[1] = 1.0f; m->calib_k[2] = 1.0f;  /* 未标定 = 恒等 */
  m->calib_b[0] = 0.0f; m->calib_b[1] = 0.0f; m->calib_b[2] = 0.0f;
  /* reserved[] / pad[] 保持 0（不是 0xFF）：CRC 会覆盖它们，
   * 且 0 比 0xFF 更容易在 dump 里看出"这条记录是新写的"。 */
}

void BL_MetaPrint(const BL_Meta_t *m, const char *tag)
{
  if (m == NULL)
  {
    return;
  }
  printf("[BL] %s: seq=%lu active_slot=%c update_pending=%u "
         "boot_attempts=%u/%u app_ver=%lu neai_ver=%lu\r\n",
         tag,
         (unsigned long)m->seq,
         (m->active_slot == (uint8_t)BL_SLOT_ID_B) ? 'B' : 'A',
         (unsigned)m->update_pending,
         (unsigned)m->boot_attempts,
         (unsigned)m->max_boot_attempts,
         (unsigned long)m->app_version,
         (unsigned long)m->neai_model_ver);

  /* 🔴 这里**绝不能**写 `%.4f`：本工程刻意没开 `-u _printf_float`
   *    （省约 10.5 KB Flash），newlib-nano 遇到 `%f` 会**静默打印 0.0000**，
   *    不报错、不警告 —— 正是 RISKS R13 记的那个坑。
   *    折中办法：乘 1000 后用整数打印（FPU 做乘法，printf 只处理整数）。 */
  printf("[BL] %s: calib k(x1000)=[%ld %ld %ld] b(x1000)=[%ld %ld %ld] crc=0x%08lX\r\n",
         tag,
         (long)(m->calib_k[0] * 1000.0f), (long)(m->calib_k[1] * 1000.0f),
         (long)(m->calib_k[2] * 1000.0f),
         (long)(m->calib_b[0] * 1000.0f), (long)(m->calib_b[1] * 1000.0f),
         (long)(m->calib_b[2] * 1000.0f),
         (unsigned long)m->crc32);

  /* 🔴 `S8` 的镜像描述子是"拒收半截镜像"的唯一依据，必须一眼看得见：
   *    `len=0` 就是"这一槽没有描述子 ⇒ 只能做仅头校验"这个事实的物证。 */
  printf("[BL] %s: trial=%u img A=%luB/0x%08lX  B=%luB/0x%08lX\r\n",
         tag,
         (unsigned)m->trial,
         (unsigned long)m->img[BL_SLOT_ID_A].len,
         (unsigned long)m->img[BL_SLOT_ID_A].crc32,
         (unsigned long)m->img[BL_SLOT_ID_B].len,
         (unsigned long)m->img[BL_SLOT_ID_B].crc32);
}

void BL_MetaPrintRaw(void)
{
  const volatile uint32_t *p = (const volatile uint32_t *)BL_META_ADDR;
  uint32_t i;

  printf("[BL] metadata region raw head @0x%08lX (was App code before we erased it):\r\n",
         (unsigned long)BL_META_ADDR);
  for (i = 0U; i < 4U; i++)
  {
    printf("     +%02lu: 0x%08lX%s\r\n",
           (unsigned long)(i * 4UL),
           (unsigned long)p[i],
           (p[i] == 0xFFFFFFFFUL) ? "  (erased)" : "");
  }
  printf("[BL] metadata scan: valid_records=%lu  newest_seq=%lu  first_free_off=%lu/%lu\r\n",
         (unsigned long)g_validCount,
         (unsigned long)g_newestSeq,
         (unsigned long)((g_firstFreeOff >= BL_META_SIZE) ? BL_META_SIZE : g_firstFreeOff),
         (unsigned long)BL_META_SIZE);
}

/* =====================================================================
 * 四、写入（追加 + 读回校验）
 * ===================================================================== */
int BL_MetaWrite(const BL_Meta_t *m)
{
  BL_Meta_t rec;
  BL_Meta_t back;
  uint32_t off;
  int erased = 0;

  if (m == NULL)
  {
    return 0;
  }

  /* 每次都重新扫描：宁可多花几毫秒，也不要拿着过期的"空位偏移"去写。 */
  (void)BL_MetaScan(NULL);

  rec = *m;
  rec.magic          = BL_META_MAGIC;
  rec.struct_version = (uint16_t)BL_META_STRUCT_VERSION;
  rec.crc_span       = (uint16_t)BL_META_CRC_SPAN;
  /* 新序号 = 当前最新 + 1；空白片则从 0 开始。 */
  rec.seq            = (g_haveAny != 0) ? (g_newestSeq + 1UL) : 0UL;
  rec.crc32          = BL_Crc32((const uint8_t *)&rec, BL_META_CRC_SPAN);

  off = g_firstFreeOff;
  if ((off >= BL_META_SIZE) || ((off + BL_META_REC_SIZE) > BL_META_SIZE))
  {
    /* 扇区写满了（128 条）—— 擦一次，从头再来。
     * 🔴 这是**唯一**的擦除窗口：这段时间里参数区没有任何有效记录。
     *    风险可接受的两个理由：
     *      ① 概率极低（每 128 次写才碰上一次）；
     *      ② 若真在此掉电，下次上电会走自愈 → 写默认值 → **默认值 = 从 Slot A 启动**，
     *         也就是"回到已知可用的那份固件"，属于**失效安全**而不是失效危险。 */
    printf("[BL] metadata sector full -> erase S%lu and restart log\r\n",
           (unsigned long)BL_META_SECTOR);
    if (BL_FlashEraseSector(BL_META_SECTOR) != 0)
    {
      return 0;
    }
    erased = 1;
    off    = 0U;
  }

  if (BL_FlashProgramWords(BL_META_ADDR + off, (const uint32_t *)&rec,
                           BL_META_REC_SIZE / 4UL) != 0)
  {
    return 0;
  }

  /* ---- 读回校验：判据②"写→读回→CRC 一致"就靠这三步 ---------------- */
  BL_MetaReadRec(off, &back);

  {
    int magicOk = (back.magic == BL_META_MAGIC);
    int seqOk   = (back.seq == rec.seq);
    int crcOk   = (BL_Crc32((const uint8_t *)&back, BL_META_CRC_SPAN) == back.crc32);
    int same    = (memcmp((const void *)&back, (const void *)&rec,
                          (size_t)BL_META_CRC_SPAN) == 0) ? 1 : 0;

    printf("[BL] meta write @+%lu%s: magic %s  seq=%lu %s  crc %s  bytewise %s\r\n",
           (unsigned long)off,
           erased ? " (after erase)" : "",
           magicOk ? "OK" : "FAIL",
           (unsigned long)back.seq,
           seqOk ? "OK" : "FAIL",
           crcOk ? "OK" : "FAIL",
           same ? "IDENTICAL" : "DIFFER");

    if ((magicOk == 0) || (seqOk == 0) || (crcOk == 0) || (same == 0))
    {
      printf("[BL] meta write VERIFY FAILED -- do not trust metadata\r\n");
      return 0;
    }
  }

  printf("[BL] meta write OK (readback verified)\r\n");
  return 1;
}

int BL_MetaSelfHeal(void)
{
  BL_Meta_t d;

  BL_MetaDefaults(&d);
  printf("[BL] metadata invalid/blank -> self-heal: write factory defaults\r\n");
  return BL_MetaWrite(&d);
}
