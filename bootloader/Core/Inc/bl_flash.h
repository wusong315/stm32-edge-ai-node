/**
  ******************************************************************************
  * @file    bl_flash.h
  * @brief   Flash 扇区层：地址 ↔ 扇区号、擦除、编程、分区表机器自检。
  *
  * @note    🔴 为什么单独一层：`HAL_FLASHEx_Erase` 要的是**扇区号**，而我们手上
  *          只有地址。F4 的扇区**不等大**（S0–S3 各 16 KB / S4 一个 64 KB /
  *          S5–S11 各 128 KB），所以"地址 ÷ 扇区大小"这种算法是错的。
  *          把换算写在一处、并用 `BL_FlashMapVerify()` 在**每次上电**时打印
  *          逐个分区的扇区号，可以让"分区表记错"这类错误在第一秒就现形。
  *
  *          S6（串口升级写 Slot B）会直接复用本层的擦/写函数。
  ******************************************************************************
  */
#ifndef __BL_FLASH_H
#define __BL_FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  由地址反推扇区号（F4 的非等大扇区表）。
  * @param  addr 落在 0x08000000..0x08100000 内的地址
  * @retval 扇区号 0..11；地址越界时返回 0xFFFFFFFF
  */
uint32_t BL_FlashSectorOf(uint32_t addr);

/**
  * @brief  取某个扇区的起始地址与大小。
  * @param  sector 扇区号 0..11
  * @param  begin  输出：起始地址
  * @param  size   输出：该扇区大小（字节）
  * @retval 0 成功 / -1 扇区号非法
  */
int BL_FlashSectorRange(uint32_t sector, uint32_t *begin, uint32_t *size);

/**
  * @brief  擦除一个扇区（整个扇区，无部分擦除）。
  * @note   调用方负责保证该扇区确实属于自己（参数区 / Slot B）。
  * @retval 0 成功 / -1 失败（失败原因已 printf）
  */
int BL_FlashEraseSector(uint32_t sector);

/**
  * @brief  从 RAM 逐字编程到 Flash（调用方保证目标区已擦除）。
  * @param  addr   目标地址，必须 4 字节对齐
  * @param  src    源缓冲区（按 32 位字计）
  * @param  nwords 字数
  * @retval 0 成功 / -1 失败
  */
int BL_FlashProgramWords(uint32_t addr, const uint32_t *src, uint32_t nwords);

/**
  * @brief  🔴 分区表"机器自检"：逐个分区核对
  *         (1) 由地址反推的扇区号 == bl_config.h 里写死的期望值；
  *         (2) 分区 = 整数个完整扇区（起止都落在扇区边界）。
  * @note   这是 S2 那条"编译期加法自检"的**运行期升级版**：编译期只能证明
  *         纸面加法对，证明不了"扇区号写错了"。
  * @retval 1 全部通过 / 0 有不一致（已逐条 printf）
  */
int BL_FlashMapVerify(void);

#ifdef __cplusplus
}
#endif

#endif /* __BL_FLASH_H */
