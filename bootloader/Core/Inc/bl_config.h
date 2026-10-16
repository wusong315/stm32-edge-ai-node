/**
  ******************************************************************************
  * @file    bl_config.h
  * @brief   OTA 分区表 —— **分区表落进代码的唯一来源**。
  *
  * @note    数值权威：`MVP/docs/plan/STM32边缘AI项目-OTA实施方案.md` §5（方案乙）
  *          定案记录：`DECISIONS.md` D21（2026-10-11 用户批准）
  *
  * 🔴 为什么地址表要写成宏 + 编译期断言，而不是散在 .ld / main.c / 注释里：
  *    推演稿 §5 指出同一组地址要在**四处**保持一致（分区表 / .ld 的 FLASH ORIGIN /
  *    App 的 SCB->VTOR / system_stm32f4xx.c 的 VECT_TAB_OFFSET）。四处里改错任何
  *    一处的症状都是"跑飞"或"乱 HardFault"，很难查。把这张表放在一个头文件里，
  *    再用 _Static_assert 在**编译期**挡住"加不到 1 MB"和"起点没落在扇区边界"这两类
  *    最容易犯的算术错误 —— 比上板之后拿示波器找快得多。
  ******************************************************************************
  */
#ifndef __BL_CONFIG_H
#define __BL_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 一、Flash 总览
 * ===================================================================== */
#define BL_FLASH_BASE           (0x08000000UL)
#define BL_FLASH_TOTAL          (1024UL * 1024UL)   /* F407VGT6 = 1 MB */

/* =====================================================================
 * 二、分区表（方案乙，D21）
 * =====================================================================
 * 区域            起始            大小       覆盖扇区
 * Bootloader      0x08000000      32 KB      S0+S1
 * 参数区(元数据)  0x08008000      16 KB      S2        ← 双副本交替写
 * App A（运行槽） 0x0800C000     336 KB      S3..S6
 * App B（备用槽） 0x08060000     384 KB      S7..S9    ← 接收新固件
 * 日志/数据       0x080C0000     256 KB      S10+S11
 *                                                   合计 1024 KB
 */
#define BL_BOOT_ADDR            (BL_FLASH_BASE + 0x00000000UL)
#define BL_BOOT_SIZE            (32UL * 1024UL)

#define BL_META_ADDR            (BL_FLASH_BASE + 0x00008000UL)
#define BL_META_SIZE            (16UL * 1024UL)

#define BL_SLOT_A_ADDR          (BL_FLASH_BASE + 0x0000C000UL)
#define BL_SLOT_A_SIZE          (336UL * 1024UL)

#define BL_SLOT_B_ADDR          (BL_FLASH_BASE + 0x00060000UL)
#define BL_SLOT_B_SIZE          (384UL * 1024UL)

#define BL_LOG_ADDR             (BL_FLASH_BASE + 0x000C0000UL)
#define BL_LOG_SIZE             (256UL * 1024UL)

/* =====================================================================
 * 三、扇区粒度
 * =====================================================================
 * F4 的 Flash 扇区**不是等大的**（S0–S3 各 16 KB；S4 一个 64 KB；S5–S11 各 128 KB）。
 * 但所有扇区起点都是 16 KB 的整数倍 ⇒ 只要分区起点能整除 16 KB，就必然落在
 * 某个扇区边界上 —— 这个判据比逐个列举扇区表简单，且等价。
 *
 * 🔴 为什么必须对齐扇区：HAL_FLASHEx_Erase 的擦除粒度就是"扇区"，
 *    没有"擦 500 字节"这种操作。起点不对齐 ⇒ 擦 A 区会连带擦掉 B 区。 */
#define BL_SECTOR_GRAIN         (16UL * 1024UL)

/* =====================================================================
 * 四、App 侧要跟着改的两处（推演稿 §5 的第 3、4 处）
 * =====================================================================
 * App 工程（S4 起）要为两个槽建两个 Build Configuration，区别只有两个 -D 宏：
 *   SlotA: -DUSER_VECT_TAB_ADDRESS -DVECT_TAB_OFFSET=0xC000
 *   SlotB: -DUSER_VECT_TAB_ADDRESS -DVECT_TAB_OFFSET=0x60000
 * 这里把数值也定义一份，方便 Bootloader 做校验时对照（避免两处各写一变）。
 * 🔴 两处必须相等：system_stm32f4xx.c 的 VECT_TAB_OFFSET 与 App 运行时的
 *    SCB->VTOR 都取自这里对应的槽地址。 */
#define BL_SLOT_A_VECT_TAB_OFF (0xC000UL)           /* = SLOT_A_ADDR - FLASH_BASE */
#define BL_SLOT_B_VECT_TAB_OFF (0x60000UL)          /* = SLOT_B_ADDR - FLASH_BASE */

/* =====================================================================
 * 五、编译期自检（推演稿 §9 第一行判据的机器版：纸上加法交给编译器做）
 * ===================================================================== */

/* ① 各段首尾相接、无缝无叠 */
_Static_assert(BL_BOOT_ADDR + BL_BOOT_SIZE == BL_META_ADDR,
               "Bootloader 区与参数区之间有缝隙或重叠");
_Static_assert(BL_META_ADDR + BL_META_SIZE == BL_SLOT_A_ADDR,
               "参数区与 App A 之间有缝隙或重叠");
_Static_assert(BL_SLOT_A_ADDR + BL_SLOT_A_SIZE == BL_SLOT_B_ADDR,
               "App A 与 App B 之间有缝隙或重叠");
_Static_assert(BL_SLOT_B_ADDR + BL_SLOT_B_SIZE == BL_LOG_ADDR,
               "App B 与日志区之间有缝隙或重叠");

/* ② 刚好用满 1 MB */
_Static_assert(BL_LOG_ADDR + BL_LOG_SIZE == BL_FLASH_BASE + BL_FLASH_TOTAL,
               "分区表合计不等于 1 MB");

/* ③ 每个分区起点都落在扇区边界上 */
_Static_assert((BL_BOOT_ADDR % BL_SECTOR_GRAIN) == 0UL, "Bootloader 区起点未对齐扇区");
_Static_assert((BL_META_ADDR % BL_SECTOR_GRAIN) == 0UL, "参数区起点未对齐扇区");
_Static_assert((BL_SLOT_A_ADDR % BL_SECTOR_GRAIN) == 0UL, "App A 起点未对齐扇区");
_Static_assert((BL_SLOT_B_ADDR % BL_SECTOR_GRAIN) == 0UL, "App B 起点未对齐扇区");
_Static_assert((BL_LOG_ADDR % BL_SECTOR_GRAIN) == 0UL, "日志区起点未对齐扇区");

/* ④ 槽的向量表偏移与槽地址自洽（防止只改一处） */
_Static_assert(BL_SLOT_A_VECT_TAB_OFF == (BL_SLOT_A_ADDR - BL_FLASH_BASE),
               "Slot A 的 VECT_TAB_OFFSET 与槽地址不一致");
_Static_assert(BL_SLOT_B_VECT_TAB_OFF == (BL_SLOT_B_ADDR - BL_FLASH_BASE),
               "Slot B 的 VECT_TAB_OFFSET 与槽地址不一致");

#ifdef __cplusplus
}
#endif

#endif /* __BL_CONFIG_H */
