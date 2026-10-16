/**
  ******************************************************************************
  * @file    bl_flash.c
  * @brief   Flash 扇区层实现。
  ******************************************************************************
  */
#include "main.h"
#include "bl_config.h"
#include "bl_flash.h"

#include <stdio.h>

/* =====================================================================
 * 一、地址 → 扇区号
 * =====================================================================
 * F4 的扇区表（1 MB 型号）：
 *   S0  : 0x08000000 + 0x0000 0000, 16 KB
 *   S1  : 0x08000000 + 0x0000 4000, 16 KB
 *   S2  : 0x08000000 + 0x0000 8000, 16 KB
 *   S3  : 0x08000000 + 0x0000 C000, 16 KB
 *   S4  : 0x08000000 + 0x0001 0000, 64 KB   ← 从这里开始变大
 *   S5..S11: 0x0800 0000 + 0x0002 0000 起，每档 128 KB
 *
 * 🔴 不要写成 "addr / 16K" —— S4 之后立刻错。 */
uint32_t BL_FlashSectorOf(uint32_t addr)
{
  uint32_t off;

  if ((addr < BL_FLASH_BASE) || (addr >= (BL_FLASH_BASE + BL_FLASH_TOTAL)))
  {
    return 0xFFFFFFFFUL;
  }
  off = addr - BL_FLASH_BASE;

  if (off < 0x00010000UL)                 /* S0..S3：前 64 KB 里每 16 KB 一个扇区 */
  {
    return off / 0x00004000UL;
  }
  if (off < 0x00020000UL)                 /* S4：一个 64 KB 扇区 */
  {
    return 4UL;
  }
  return 5UL + ((off - 0x00020000UL) / 0x00020000UL);   /* S5..S11：每 128 KB 一个 */
}

int BL_FlashSectorRange(uint32_t sector, uint32_t *begin, uint32_t *size)
{
  if ((begin == NULL) || (size == NULL))
  {
    return -1;
  }

  if (sector < 4UL)
  {
    *begin = BL_FLASH_BASE + (sector * 0x00004000UL);
    *size  = 0x00004000UL;
  }
  else if (sector == 4UL)
  {
    *begin = BL_FLASH_BASE + 0x00010000UL;
    *size  = 0x00010000UL;
  }
  else if (sector <= 11UL)
  {
    *begin = BL_FLASH_BASE + 0x00020000UL + ((sector - 5UL) * 0x00020000UL);
    *size  = 0x00020000UL;
  }
  else
  {
    return -1;
  }
  return 0;
}

/* =====================================================================
 * 二、擦除 / 编程
 * ===================================================================== */
int BL_FlashEraseSector(uint32_t sector)
{
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0xFFFFFFFFUL;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    printf("[BL] flash unlock failed (erase sector %lu)\r\n", (unsigned long)sector);
    return -1;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
  eraseInit.Banks        = FLASH_BANK_1;          /* F407 单 bank，显式写出免得踩 H7 的坑 */
  eraseInit.Sector       = sector;
  eraseInit.NbSectors    = 1U;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7–3.6 V，按字编程 */

  if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    printf("[BL] erase sector %lu FAILED (sectorError=0x%08lX)\r\n",
           (unsigned long)sector, (unsigned long)sectorError);
    return -1;
  }

  (void)HAL_FLASH_Lock();
  return 0;
}

int BL_FlashProgramWords(uint32_t addr, const uint32_t *src, uint32_t nwords)
{
  uint32_t i;

  if ((src == NULL) || ((addr % 4UL) != 0UL))
  {
    return -1;
  }

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    printf("[BL] flash unlock failed (program @0x%08lX)\r\n", (unsigned long)addr);
    return -1;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  for (i = 0U; i < nwords; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (i * 4UL), src[i]) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      printf("[BL] program FAILED @0x%08lX (word %lu)\r\n",
             (unsigned long)(addr + (i * 4UL)), (unsigned long)i);
      return -1;
    }
  }

  (void)HAL_FLASH_Lock();
  return 0;
}

/* =====================================================================
 * 三、分区表机器自检
 * ===================================================================== */
typedef struct
{
  const char *name;
  uint32_t    addr;
  uint32_t    size;
  uint32_t    expectSector;
} BL_MapEntry_t;

int BL_FlashMapVerify(void)
{
  static const BL_MapEntry_t map[] =
  {
    { "bootloader", BL_BOOT_ADDR,   BL_BOOT_SIZE,   BL_BOOT_SECTOR   },
    { "metadata  ", BL_META_ADDR,   BL_META_SIZE,   BL_META_SECTOR   },
    { "slot A    ", BL_SLOT_A_ADDR, BL_SLOT_A_SIZE, BL_SLOT_A_SECTOR },
    { "slot B    ", BL_SLOT_B_ADDR, BL_SLOT_B_SIZE, BL_SLOT_B_SECTOR },
    { "log/data  ", BL_LOG_ADDR,    BL_LOG_SIZE,    BL_LOG_SECTOR    },
  };
  uint32_t i;
  int ok = 1;

  printf("[BL] flash map verify (addr -> derived sector):\r\n");

  for (i = 0U; i < (sizeof(map) / sizeof(map[0])); i++)
  {
    const BL_MapEntry_t *e = &map[i];
    uint32_t sec     = BL_FlashSectorOf(e->addr);
    int      secOk   = (sec == e->expectSector);
    int      rangeOk = 1;
    uint32_t a       = e->addr;
    uint32_t remain  = e->size;
    uint32_t cs      = sec;
    uint32_t nsec    = 0U;

    /* 🔴 分区必须"恰好由整数个完整扇区构成" —— 逐扇区向前走一遍：
     *    每一步的地址都必须正好是当前扇区的起点，且扇区不能大于剩余长度。
     *    为什么必须查：擦除粒度是扇区，"半个扇区"是查不出来的，
     *    一旦某分区只占半个扇区，擦它就会**连带擦掉邻居**。 */
    while (remain > 0U)
    {
      uint32_t b = 0U;
      uint32_t s = 0U;

      if (BL_FlashSectorRange(cs, &b, &s) != 0)
      {
        rangeOk = 0;
        break;
      }
      if ((a != b) || (s > remain))
      {
        rangeOk = 0;
        break;
      }
      a += s;
      remain -= s;
      cs++;
      nsec++;
    }

    printf("  %s 0x%08lX +%5lu KB  S%-2lu..S%-2lu (%lu 个)  expect S%-2lu %s | whole sectors %s\r\n",
           e->name,
           (unsigned long)e->addr,
           (unsigned long)(e->size / 1024UL),
           (unsigned long)sec,
           (unsigned long)(cs - 1UL),
           (unsigned long)nsec,
           (unsigned long)e->expectSector,
           secOk   ? "OK  " : "FAIL",
           rangeOk ? "OK" : "FAIL");

    if (!secOk || !rangeOk)
    {
      ok = 0;
    }
  }

  printf("[BL] flash map verify: %s\r\n", ok ? "PASS" : "FAIL");
  return ok;
}
