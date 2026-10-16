/**
  ******************************************************************************
  * @file    bl_boot.h
  * @brief   Bootloader 主流程入口。
  ******************************************************************************
  */
#ifndef __BL_BOOT_H
#define __BL_BOOT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Bootloader 主流程：打印横幅 → 分区表机器自检 → CRC 自检 → 读元数据（空白则自愈）
 * → 校验镜像 → 合法才跳 App，否则停在安全等待模式。
 * 🔴 本函数被设计成**不返回** —— 它要么跳进 App，要么停在升级/等待循环里。 */
void BL_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __BL_BOOT_H */
