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

/* =====================================================================
 * 六、扇区号（S3 新增）
 * =====================================================================
 * ⚠️ 上面第三节只保证了"起点落在扇区边界"，但 HAL_FLASHEx_Erase 要的是
 *    **扇区号**，不是地址。扇区号由地址反推（F4 扇区不等大，见 bl_flash.c 的
 *    BL_FlashSectorOf），这里把"应该推出来的值"写死一份，两边对照 ——
 *    这样"地址算错"和"扇区表记错"会互相咬住，不会一起错过去。
 *
 * 🔴 为什么需要这个对照：擦除粒度是扇区。若把参数区的扇区号写成 3，
 *    第一次自愈就会把 **App A 的头 16 KB 擦掉** —— 而且擦完还"看起来正常"
 *    （元数据写成功、打印一切正常），直到 S5 烧完 App 才发作。 */
#define BL_BOOT_SECTOR          (0UL)       /* 0x08000000 */
#define BL_META_SECTOR          (2UL)       /* 0x08008000 */
#define BL_SLOT_A_SECTOR        (3UL)       /* 0x0800C000 */
#define BL_SLOT_B_SECTOR        (7UL)       /* 0x08060000 */
#define BL_LOG_SECTOR           (10UL)      /* 0x080C0000 */

/* =====================================================================
 * 七、参数区记录格式（S3 新增）
 * =====================================================================
 * 🔴 为什么参数区是「记录日志」而不是"两个固定副本"（这是一个实测出来的更正）：
 *    文档里原话是"双副本交替写"。但**擦除粒度是整个扇区**，而参数区 16 KB
 *    = 恰好一个扇区（S2）⇒ "擦掉旧副本、写新副本"这个动作**做不到**
 *    （擦一个副本必然把另一个也擦掉，中间出现"零份有效"的窗口）。
 *
 *    正确实现 = **顺序追加日志**：新记录写在扇区内第一个空白（0xFFFFFFFF）位置，
 *    **不需要擦除**；读的时候扫全扇区，取 seq 最大且 CRC 正确的那条。
 *    - 掉电在写记录的中途 → 该条 CRC 不过 → 自动回退到上一条 ⇒ 掉电安全；
 *    - 记录写满整个扇区才擦一次（128 次写才擦 1 次）⇒ 擦除窗口被摊薄 128 倍。
 *    语义上仍然是"多副本 + 交替写"，且任意时刻至少有一份有效。 */
#define BL_META_REC_SIZE        (128UL)     /* 每条记录 128 B */
#define BL_META_REC_COUNT       (BL_META_SIZE / BL_META_REC_SIZE)  /* = 128 条 */
#define BL_META_CRC_OFF         (BL_META_REC_SIZE - 4UL)           /* crc32 落在末字 */
#define BL_META_CRC_SPAN        (BL_META_CRC_OFF)                  /* CRC 覆盖整条记录 */

_Static_assert(BL_META_SIZE % BL_META_REC_SIZE == 0UL,
               "参数区大小不能被记录大小整除（最后一条记录会跨界）");
_Static_assert((BL_META_REC_SIZE % 4UL) == 0UL,
               "记录大小必须是 4 的整数倍（Flash 按字编程）");
_Static_assert(BL_META_REC_COUNT >= 8UL,
               "参数区记录的条数太少，擦除次数会变得频繁");

#ifdef __cplusplus
}
#endif

#endif /* __BL_CONFIG_H */
