/**
  ******************************************************************************
  * @file    bl_ota.c
  * @brief   串口升级（S6）实现 —— 协议规格见 `bl_ota.h` 头部注释。
  *
  * @note    🔴 三条纪律，改这个文件之前先读一遍：
  *
  *          ① **收数据期间不许打印**。`printf` 走的是阻塞发送，一帧日志
  *             几十字节就要占用 TX 好几毫秒；而 UART 的接收寄存器只有
  *             1 个字节 —— 主机若在这几毫秒里还在灌，就是**静默丢字节**。
  *             所以：只在"说完话、主机会停下来等"的时刻打印
  *             （应答 'K' **之前**、擦除**之后**、以及整个流程结束之后）。
  *
  *          ② **本文件不许出现 `%f`**。本工程刻意没开 `-u _printf_float`
  *             （省约 10.5 KB），newlib-nano 遇到 `%f` 会**静默打印 0.0000**，
  *             不报错不警告 —— 见 RISKS R13 / RUNBOOK R13.6.6。
  *
  *          ③ **不许用大局部数组**。链接脚本 `_Min_Stack_Size = 0x400`
  *             （1 KB），1024 字节的收包缓冲**必须**是 `static`。
  *
  *          🔴 另外**不用 `sscanf`**：它会把 newlib 里一大坨东西拉进来
  *             （1 KB 级），而我们要解析的只有"十进制 + 8 位十六进制"这一种
  *             形状。手写三个小函数（约 150 B）更省也更可控。
  ******************************************************************************
  */
#include "main.h"
#include "bl_config.h"
#include "bl_flash.h"
#include "bl_jump.h"
#include "bl_meta.h"
#include "bl_ota.h"

#include <stdio.h>

/* 与 bl_boot.c 同样的惯例：`huart1` 没有在 `main.h` 里 extern，谁用谁自己声明。 */
extern UART_HandleTypeDef huart1;

/* =====================================================================
 * 一、CRC16（CRC-16/ARC）—— 帧层校验
 * =====================================================================
 * 位运算版，与 `bl_meta.c` 的 CRC32 同一套路数（省掉 256 项常量表）。
 * 一帧 1029 字节 ≈ 8 千次移位，在 168 MHz 上**几十微秒**，完全不用查表。 */
static uint16_t BlCrc16(uint16_t crc, const uint8_t *p, uint32_t len)
{
  uint32_t i;
  uint32_t b;

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)p[i];
    for (b = 0U; b < 8U; b++)
    {
      /* 反射式：右移一位，移出位为 1 则异或 0xA001。写成无分支形式。 */
      crc = (uint16_t)((crc >> 1) ^ (uint16_t)(0xA001U & (uint16_t)(0U - (crc & 1U))));
    }
  }
  return crc;
}

int BL_OtaSelfTest(void)
{
  /* 🔴 期望值先在 PC 上用 Python 算过（与 zlib 无关的另一族：CRC-16/ARC）。 */
  static const char v[] = "123456789";
  uint16_t got      = BlCrc16(0U, (const uint8_t *)v, 9U);
  uint16_t expected = (uint16_t)0xBB3DU;
  int ok = (got == expected);

  printf("[BL] crc16 selftest (frame layer): \"123456789\" -> 0x%04X (expect 0x%04X) %s\r\n",
         (unsigned)got, (unsigned)expected, ok ? "PASS" : "FAIL");
  return ok;
}

/* =====================================================================
 * 二、小工具
 * ===================================================================== */

/** 发一个单字节应答（'K' / 'N' / 'E'）。 */
static void BlSendToken(uint8_t t)
{
  (void)HAL_UART_Transmit(&huart1, &t, 1U, 100U);
}

/** 十进制解析。返回 1 = 至少有一位数字；指针停在下一位非数字处。 */
static int BlParseDec(const char **pp, uint32_t *out)
{
  const char *p = *pp;
  uint32_t v = 0U;
  uint32_t n = 0U;

  while ((*p >= '0') && (*p <= '9'))
  {
    if (n >= 9U)                    /* 9 位十进制封顶：防溢出，也防主机发神经 */
    {
      return 0;
    }
    v = (v * 10U) + (uint32_t)(*p - '0');
    p++;
    n++;
  }
  if (n == 0U)
  {
    return 0;
  }
  *pp  = p;
  *out = v;
  return 1;
}

/** 恰好 8 位十六进制解析（CRC32 定长，少了多了都不认）。 */
static int BlParseHex8(const char **pp, uint32_t *out)
{
  const char *p = *pp;
  uint32_t v = 0U;
  uint32_t n;

  for (n = 0U; n < 8U; n++)
  {
    char     c = *p;
    uint32_t d;

    if ((c >= '0') && (c <= '9'))       { d = (uint32_t)(c - '0'); }
    else if ((c >= 'a') && (c <= 'f'))  { d = (uint32_t)(c - 'a') + 10U; }
    else if ((c >= 'A') && (c <= 'F'))  { d = (uint32_t)(c - 'A') + 10U; }
    else                                { return 0; }

    v = (v << 4) | d;
    p++;
  }
  *pp  = p;
  *out = v;
  return 1;
}

/** 跳过字段之间的分隔符（`,` `:` `=` 空格都收，主机写哪种都行）。 */
static const char *BlSkipSep(const char *p)
{
  while ((*p == ',') || (*p == ':') || (*p == '=') || (*p == ' ')) { p++; }
  return p;
}

/* =====================================================================
 * 三、读一行（起始行）
 * =====================================================================
 * 🔴 起始的 CR/LF 必须吃掉：上电窗口里主机在**连续喊** `u\n`，
 *    等它看到提示停下时，管道里通常还漂着 1~2 个 `\n` 或半个 `u\n`。
 *    如果第一行直接拿这个残渣去解析，就会"看起来什么都没发"而误判失败。
 *    ⇒ 行首空行跳过 + 下面最多重读 BL_OTA_START_TRIES 行（见 BlReadStart）。 */
static int BlReadLine(char *out, uint32_t cap)
{
  uint32_t n = 0U;
  uint8_t  b;

  for (;;)
  {
    if (HAL_UART_Receive(&huart1, &b, 1U, BL_OTA_LINE_TIMEOUT_MS) != HAL_OK)
    {
      return 0;                     /* 超时：主机不在 / 链路断了 */
    }
    if ((b == (uint8_t)'\r') || (b == (uint8_t)'\n'))
    {
      if (n == 0U)
      {
        continue;                   /* 行首空行 ⇒ 跳过 */
      }
      break;
    }
    if ((n + 1U) >= cap)
    {
      return 0;                     /* 超长 ⇒ 当成坏行 */
    }
    out[n] = (char)b;
    n++;
  }
  out[n] = '\0';
  return 1;
}

/** 解析 `s<LEN>,<CRC32HEX>[,<VER>]`。返回 1 = 形状合法。 */
static int BlParseStart(const char *line, uint32_t *len, uint32_t *crc, uint32_t *ver)
{
  const char *p = line;

  if ((*p != 's') && (*p != 'S'))
  {
    return 0;
  }
  p++;

  if (BlParseDec(&p, len) != 1)
  {
    return 0;
  }
  p = BlSkipSep(p);
  if (BlParseHex8(&p, crc) != 1)
  {
    return 0;
  }

  *ver = 0U;
  p = BlSkipSep(p);
  if ((*p >= '0') && (*p <= '9'))
  {
    if (BlParseDec(&p, ver) != 1)
    {
      return 0;
    }
  }

  /* 尾部只允许空白 / 行尾 —— 有多余东西说明这行不是我们认识的形状。 */
  while ((*p == ' ') || (*p == '\t') || (*p == '\r') || (*p == '\n'))
  {
    p++;
  }
  return (*p == '\0') ? 1 : 0;
}

/* =====================================================================
 * 四、收一帧（并回 'K' / 'N'）
 * =====================================================================
 * 🔴 这里用一次 `HAL_UART_Receive(..., n, 超时)` 读整块，而不是逐字节循环：
 *    一帧最多 1031 字节、115200 下约 90 ms，而超时给 3000 ms（33 倍余量）。
 *    逐字节循环要多绕 HAL 一千次（每帧多花约 5 ms CPU），没有收益。
 * ⚠️ 注意 HAL 的超时是**整段**超时：中途断了就直接返回 HAL_TIMEOUT，
 *    于是我们不假装知道"收了多少"——**整帧作废、NAK、让主机重发**才是正确姿势。 */
static int BlRecvFrame(uint8_t expectSeq, uint8_t *payload, uint32_t maxLen, uint32_t *outLen)
{
  uint8_t  hdr[BL_OTA_HDR_LEN];
  uint8_t  crcb[BL_OTA_CRC_LEN];
  uint32_t len;
  uint16_t got;
  uint16_t want;

  /* ---- 1) 帧头 ---------------------------------------------------- */
  if (HAL_UART_Receive(&huart1, hdr, BL_OTA_HDR_LEN, BL_OTA_FRAME_TIMEOUT_MS) != HAL_OK)
  {
    printf("[OTA] frame hdr timeout -> abort\r\n");
    return 0;
  }
  if ((hdr[0] != (uint8_t)BL_OTA_SOF0) || (hdr[1] != (uint8_t)BL_OTA_SOF1))
  {
    printf("[OTA] bad SOF (%02X %02X) -> abort\r\n", (unsigned)hdr[0], (unsigned)hdr[1]);
    return 0;
  }
  if (hdr[2] != expectSeq)
  {
    /* 停等协议下这不常发生；真发生了说明主机和我们不在同一个"第几帧"上。
     * NAK 让主机重发本帧，我们不推进任何状态。 */
    printf("[OTA] seq mismatch: got %u want %u -> NAK\r\n",
           (unsigned)hdr[2], (unsigned)expectSeq);
    BlSendToken((uint8_t)BL_OTA_NAK);
    return 0;
  }

  len = (uint32_t)hdr[3] | ((uint32_t)hdr[4] << 8);
  if ((len == 0U) || (len > maxLen))
  {
    printf("[OTA] bad frame len %lu (max %lu) -> NAK\r\n",
           (unsigned long)len, (unsigned long)maxLen);
    BlSendToken((uint8_t)BL_OTA_NAK);
    return 0;
  }

  /* ---- 2) 数据 + 校验（这一段中间**一个字都不打印**）---------------- */
  if (HAL_UART_Receive(&huart1, payload, (uint16_t)len, BL_OTA_FRAME_TIMEOUT_MS) != HAL_OK)
  {
    printf("[OTA] frame payload timeout -> abort\r\n");
    return 0;
  }
  if (HAL_UART_Receive(&huart1, crcb, BL_OTA_CRC_LEN, BL_OTA_FRAME_TIMEOUT_MS) != HAL_OK)
  {
    printf("[OTA] frame crc timeout -> abort\r\n");
    return 0;
  }

  /* CRC 从零重算整帧（含 SOF 与 seq）—— 不增量累积，图的是"每帧独立自证"：
   * 万一有一帧的簿记错了，不会一路污染到最后一帧才发现。 */
  got  = BlCrc16(0U, hdr, BL_OTA_HDR_LEN);
  got  = BlCrc16(got, payload, len);
  want = (uint16_t)crcb[0] | ((uint16_t)crcb[1] << 8);

  if (got != want)
  {
    printf("[OTA] frame %u crc16 FAIL got=0x%04X want=0x%04X -> NAK (expect retransmit)\r\n",
           (unsigned)expectSeq, (unsigned)got, (unsigned)want);
    BlSendToken((uint8_t)BL_OTA_NAK);
    return 0;
  }

  *outLen = len;
  return 1;
}

/* =====================================================================
 * 五、主流程
 * ===================================================================== */
int BL_OtaRun(void)
{
  /* 🔴 必须是 static（栈只有 1 KB），且必须是 uint32_t 数组 ——
   *    `BL_FlashProgramWords()` 要一个 4 字节对齐的字指针，
   *    而 `uint8_t buf[1024]` 的对齐**语言上不保证**。 */
  static uint32_t wbuf[BL_OTA_CHUNK / 4U];
  uint8_t *buf = (uint8_t *)wbuf;

  char     line[BL_OTA_LINE_MAX];
  uint32_t len     = 0U;
  uint32_t wantCrc = 0U;
  uint32_t ver     = 0U;
  uint32_t got     = 0U;
  uint32_t seq     = 0U;
  uint32_t sector  = BL_SLOT_B_SECTOR;
  uint32_t secBeg  = 0U;
  uint32_t secSize = 0U;
  uint32_t nsec    = 0U;
  uint32_t i;
  uint32_t try;
  uint32_t rbCrc;
  uint32_t sp = 0U;
  uint32_t rv = 0U;
  BL_ImgStatus st;
  BL_Meta_t    m;
  int          have;

  printf("\r\n[OTA] === serial update mode (S6) ===\r\n");
  printf("[OTA] target: slot B @0x%08lX, %lu B (S%lu..S%lu)\r\n",
         (unsigned long)BL_SLOT_B_ADDR, (unsigned long)BL_SLOT_B_SIZE,
         (unsigned long)BL_SLOT_B_SECTOR, (unsigned long)(BL_SLOT_B_SECTOR + 2UL));
  printf("[OTA] frame: A5 5A | seq:1 | len:2 LE | payload | crc16:2 LE (ARC; hdr+payload) | ack 'K'/'N'\r\n");
  printf("[OTA] send: s<len>,<crc32hex>[,<ver>]   e.g. s42788,612d7dce,2\r\n");

  /* ---- 1) 起始行（容忍主机在窗口里喊 'u' 留下的残渣）---------------- */
  for (try = 0U; try < BL_OTA_START_TRIES; try++)
  {
    if (BlReadLine(line, BL_OTA_LINE_MAX) != 1)
    {
      printf("[OTA] no start line (timeout) -> abort, back to wait mode\r\n");
      return 0;
    }
    if (BlParseStart(line, &len, &wantCrc, &ver) == 1)
    {
      break;
    }
    printf("[OTA] not a start line: \"%s\" (ignored)\r\n", line);
  }
  if (try >= BL_OTA_START_TRIES)
  {
    printf("[OTA] too many bad start lines -> abort\r\n");
    return 0;
  }

  printf("[OTA] start: len=%lu crc32=0x%08lX ver=%lu\r\n",
         (unsigned long)len, (unsigned long)wantCrc, (unsigned long)ver);

  /* ---- 2) 长度预检（在擦任何东西**之前**做）------------------------- */
  if ((len < 256U) || (len > BL_SLOT_B_SIZE))
  {
    printf("[OTA] length out of range (256..%lu) -> abort\r\n",
           (unsigned long)BL_SLOT_B_SIZE);
    return 0;
  }

  if (BL_FlashSectorRange(sector, &secBeg, &secSize) != 0)
  {
    printf("[OTA] cannot resolve slot B sector %lu -> abort\r\n", (unsigned long)sector);
    return 0;
  }
  nsec = (len + secSize - 1UL) / secSize;

  /* ---- 3) 擦除 —— **只擦镜像覆盖到的扇区** --------------------------
   * 🔴 与 ST-Link 的 `-w`（不加 `-e all`）是同一个原则：
   *    只擦"这次要用到的那几个"，槽的上界（S7..S9）是**最坏情况**。
   *    这样 41.79 KB 的镜像其实只需擦 1 个扇区。 */
  printf("[OTA] erasing S%lu..S%lu (%lu x %lu KB) -- a few seconds, do not touch\r\n",
         (unsigned long)sector, (unsigned long)(sector + nsec - 1UL),
         (unsigned long)nsec, (unsigned long)(secSize / 1024UL));
  for (i = 0U; i < nsec; i++)
  {
    if (BL_FlashEraseSector(sector + i) != 0)
    {
      printf("[OTA] erase S%lu FAILED -> abort\r\n", (unsigned long)(sector + i));
      return 0;
    }
    printf("[OTA] erased S%lu\r\n", (unsigned long)(sector + i));
  }
  printf("[OTA] erased OK; send %lu frame(s) of <=%lu B, one 'K' per frame\r\n",
         (unsigned long)((len + BL_OTA_CHUNK - 1UL) / BL_OTA_CHUNK),
         (unsigned long)BL_OTA_CHUNK);

  /* ---- 4) 停等收帧 → 逐帧写 Flash ---------------------------------- */
  while (got < len)
  {
    uint32_t want = len - got;
    uint32_t gotLen = 0U;
    int      ok = 0;

    if (want > BL_OTA_CHUNK)
    {
      want = BL_OTA_CHUNK;
    }

    for (try = 0U; try < BL_OTA_MAX_RETRY; try++)
    {
      if (BlRecvFrame((uint8_t)seq, buf, want, &gotLen) == 1)
      {
        ok = 1;
        break;
      }
    }
    if (ok == 0)
    {
      printf("[OTA] frame %lu gave up after %lu tries -> abort\r\n",
             (unsigned long)seq, (unsigned long)BL_OTA_MAX_RETRY);
      return 0;
    }
    if (gotLen != want)
    {
      printf("[OTA] frame %lu len %lu != expected %lu -> abort\r\n",
             (unsigned long)seq, (unsigned long)gotLen, (unsigned long)want);
      return 0;
    }

    /* 末帧的尾巴补 0xFF 到 4 字节倍数（目标区刚擦过，写 0xFF 等于不写）。
     * 只补缓冲里**用得到的那几个字节**，不动别处。 */
    for (i = gotLen; (i & 3UL) != 0UL; i++)
    {
      buf[i] = 0xFFU;
    }

    if (BL_FlashProgramWords(BL_SLOT_B_ADDR + got, wbuf, (gotLen + 3UL) / 4UL) != 0)
    {
      printf("[OTA] program FAILED at +%lu -> abort\r\n", (unsigned long)got);
      return 0;
    }

    got += gotLen;
    seq++;

    /* 进度：每 8 帧一行（≈8 KB）+ 收尾一行。**在回 'K' 之前打**，
     * 此刻主机正等着应答、不会灌数据，所以打印是安全的（见文件头纪律①）。 */
    if (((seq & 7UL) == 0UL) || (got >= len))
    {
      printf("[OTA] %lu/%lu B written\r\n", (unsigned long)got, (unsigned long)len);
    }

    BlSendToken((uint8_t)BL_OTA_ACK);
  }

  printf("[OTA] receive complete: %lu B in %lu frames\r\n",
         (unsigned long)got, (unsigned long)seq);

  /* ---- 5) 🔴 读回校验：从 Flash 把整段再读一遍，重算 CRC32 -------------
   * 这是"写 → 读回 → 校验"在**镜像级**的应用，一次覆盖两类失败：
   *   · 传输错了（虽然逐帧 CRC16 已经拦了一道）
   *   · 写 Flash 错了（页写失败、地址算错、扇区没擦干净……）
   * ⚠️ 只有这一条过了才置标志 —— 否则 Bootloader 绝不会去跳 Slot B。 */
  rbCrc = BL_Crc32((const uint8_t *)BL_SLOT_B_ADDR, len);
  printf("[OTA] readback crc32 @0x%08lX: 0x%08lX (declared 0x%08lX) -> %s\r\n",
         (unsigned long)BL_SLOT_B_ADDR,
         (unsigned long)rbCrc, (unsigned long)wantCrc,
         (rbCrc == wantCrc) ? "MATCH" : "MISMATCH");

  if (rbCrc != wantCrc)
  {
    printf("[OTA] image REJECTED (slot B content != sent file) -> stay in bootloader\r\n");
    printf("[OTA] note: slot B now holds a partial/garbage image; it will NOT be booted.\r\n");
    return 0;
  }

  /* ---- 6) 再校验一次"能不能跳"（栈顶 / 复位向量）--------------------- */
  st = BL_ImageCheck(BL_SLOT_B_ADDR, BL_SLOT_B_SIZE, &sp, &rv);
  printf("[OTA] slot B image check: first=0x%08lX reset=0x%08lX -> %s\r\n",
         (unsigned long)sp, (unsigned long)rv, BL_ImgStatusStr(st));
  if (st != BL_IMG_OK)
  {
    printf("[OTA] image REJECTED (not bootable) -> stay in bootloader\r\n");
    return 0;
  }

  /* ---- 7) 置标志（**不**直接改 active_slot）-------------------------
   * 🔴 刻意分两步：这里只写"Slot B 有一份已校验、待安装的镜像"，
   *    真正把 active_slot 切到 B 是**复位之后**启动流程的"提交"步骤
   *    （`bl_boot.c` 的 BL_TryCommitUpdate）。
   *    好处：串口上能清楚看到"置标志"与"提交"是**两条不同的元数据记录**，
   *    而不是"一台机器说它自己改好了"。也为 S8 的 boot_attempts 留了位置。 */
  have = BL_MetaScan(&m);
  if (have == 0)
  {
    printf("[OTA] no valid metadata -> start from factory defaults\r\n");
    BL_MetaDefaults(&m);
  }
  m.update_pending = 1U;
  m.app_version    = ver;
  m.boot_attempts  = 0U;

  if (BL_MetaWrite(&m) != 1)
  {
    printf("[OTA] staged meta write FAILED -> abort (slot B will NOT be booted)\r\n");
    return 0;
  }

  /* 重新**从 Flash 读出来**再打印：证明标志真的落盘了，不是内存里的幻觉。 */
  if (BL_MetaScan(&m) != 0)
  {
    BL_MetaPrint(&m, "staged");
  }
  printf("[OTA] staged: update_pending=1 app_version=%lu active_slot untouched\r\n",
         (unsigned long)ver);
  printf("[OTA] done -> rebooting to install\r\n");
  return 1;
}
