/**
  ******************************************************************************
  * @file    bl_meta.h
  * @brief   参数区（元数据）—— 升级状态与标定系数的掉电保存。
  *
  * @note    📍 权威：`MVP/docs/plan/STM32边缘AI项目-OTA分区表推演.md`
  *          §4.3（照抄哪三件事）+ §11.3（要预留哪些字段）。
  *
  *          🔴 与参考仓库的两处**刻意不同**（推演稿 §4.3 明确要求）：
  *            ① 参考仓库只有 `magic`，撞对了就认 —— 我们**加 CRC32**；
  *            ② 参考仓库是"单副本 + 撞 magic 就重写" —— 我们做成
  *               **顺序追加日志（多副本，任意时刻 ≥1 份有效）**。
  *               为什么不是字面上的"两个固定副本"：扇区擦除粒度是 16 KB，
  *               参数区恰好一个扇区 ⇒ "擦旧副本"必然连新副本一起擦掉。
  *               详见 `bl_config.h` 第七节与 `bootloader/README.md`。
  ******************************************************************************
  */
#ifndef __BL_META_H
#define __BL_META_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 一、记录内的魔数
 * =====================================================================
 * 0x544F4F42 小端存储后的字节序列是 'B' 'O' 'O' 'T' ——
 * 用十六进制 dump 看参数区时一眼就能认出"这是元数据，不是残留的 App 机器码"。 */
#define BL_META_MAGIC           (0x544F4F42UL)
#define BL_META_STRUCT_VERSION  (1U)

/* 槽标识 */
#define BL_SLOT_ID_A            (0U)
#define BL_SLOT_ID_B            (1U)

/**
  * @brief  一条元数据记录（= 参数区里的一条日志）。
  * @note   🔴 本结构体的**内存布局就是 Flash 布局**，由 bl_config.h 里的
  *         `BL_META_REC_SIZE` / `BL_META_CRC_OFF` 与本文件末尾的 `_Static_assert`
  *         双向咬死：改字段忘了改记录大小，**编译期就会报错**。
  *         这样就不会出现"字段挪了位置、旧固件读新记录读出垃圾"这种事。
  */
typedef struct
{
  uint32_t magic;             /* @  0  必须 == BL_META_MAGIC            */
  uint16_t struct_version;    /* @  4  结构体版本，为将来加字段留后路      */
  uint16_t crc_span;          /* @  6  CRC 覆盖的字节数（= 124）          */
  uint32_t seq;               /* @  8  单调递增序号：扫描时取最大者        */
  uint8_t  active_slot;       /* @ 12  0 = Slot A，1 = Slot B           */
  uint8_t  update_pending;    /* @ 13  1 = Slot B 有已校验待安装的镜像     */
  uint8_t  boot_attempts;     /* @ 14  本次新镜像已尝试启动次数（S8 用）   */
  uint8_t  max_boot_attempts; /* @ 15  超过即回滚（S8 用）               */
  uint32_t app_version;       /* @ 16  运行槽内 App 的版本号              */
  uint32_t neai_model_ver;    /* @ 20  片上再训练的模型版本（推演稿 §11.3）*/
  float    calib_k[3];        /* @ 24  6 姿态静态标定的增益 k（3 轴）      */
  float    calib_b[3];        /* @ 36  6 姿态静态标定的零偏 b（3 轴）      */
  uint8_t  reserved[64];      /* @ 48  推演稿 §11.3 要求的预留区（64 B）   */
  uint8_t  pad[12];           /* @112  对齐垫：把 crc32 顶到记录最后一字   */
  uint32_t crc32;             /* @124  CRC32，覆盖 [0,124)              */
} BL_Meta_t;

/* =====================================================================
 * 二、接口
 * ===================================================================== */

/** @brief CRC32（反射、多项式 0xEDB88320、初值/异或 0xFFFFFFFF）。
  *        自检向量：BL_Crc32("123456789", 9) == 0xCBF43926。 */
uint32_t BL_Crc32(const uint8_t *p, uint32_t len);

/** @brief 用内置向量自检 CRC 实现，打印 PASS/FAIL。返回 1 = 通过。
  * @note  🔴 为什么值得花这几十字节：元数据的完整性**全靠** CRC。
  *        若 CRC 实现有错，"校验通过"就变成了随机事件 —— 而这种错误
  *        在板子上表现出来是"偶发回滚"，几乎不可能靠现场现象定位。 */
int BL_CrcSelfTest(void);

/** @brief 填一份出厂默认值（不带 seq / CRC，交给 BL_MetaWrite 补）。 */
void BL_MetaDefaults(BL_Meta_t *m);

/** @brief 扫描整个参数区，取 seq 最大且 CRC 正确的那条。
  * @retval 1 找到有效记录（副本写入 out）；0 一条都没有
  * @note   同时刷新内部缓存（有效条数 / 首个空位偏移 / 最新 seq），
  *         供 BL_MetaWrite 使用。 */
int BL_MetaScan(BL_Meta_t *out);

/** @brief 打印"参数区原始头几个字" + 扫描统计。
  * @note  首次把 Bootloader 烧进去时，参数区里躺着的是**被覆盖了一半的旧 App 机器码**，
  *        这一行能直接看见"魔数不匹配"的证据，而不是只看到一句"自愈完成"。 */
void BL_MetaPrintRaw(void);

/** @brief 把一条记录追加写进参数区（必要时先擦整个扇区），并**读回校验**。
  * @param  m 要写入的内容（seq 与 CRC 由本函数填，调用方不用管）
  * @retval 1 写入且读回一致；0 失败
  * @note   🔴 "写→读回→CRC 一致"三步都在函数里，所以**判据②是自证的**：
  *         串口说 OK 就意味着这三步真的走通了，不是"我以为写进去了"。 */
int BL_MetaWrite(const BL_Meta_t *m);

/** @brief 无有效记录时自愈：写一份默认值。返回 1 = 自愈成功。 */
int BL_MetaSelfHeal(void);

/** @brief 解码打印一条记录的全部字段。 */
void BL_MetaPrint(const BL_Meta_t *m, const char *tag);

#ifdef __cplusplus
}
#endif

/* =====================================================================
 * 三、布局咬合（编译期）
 * ===================================================================== */
#ifndef __cplusplus
#include <stddef.h>
#include "bl_config.h"
_Static_assert(sizeof(BL_Meta_t) == BL_META_REC_SIZE,
               "BL_Meta_t 的大小与 BL_META_REC_SIZE 不一致（改了字段没改记录大小）");
_Static_assert(offsetof(BL_Meta_t, crc32) == BL_META_CRC_OFF,
               "crc32 没有落在记录最后一字（pad[] 需要调整）");
#endif

#endif /* __BL_META_H */
