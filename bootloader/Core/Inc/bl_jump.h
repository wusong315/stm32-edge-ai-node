/**
  ******************************************************************************
  * @file    bl_jump.h
  * @brief   镜像校验 + 跳转。
  *
  * @note    📍 权威：推演稿 §4.3（跳转四件事）+ §11.4（跳转前的 DeInit 清单）。
  ******************************************************************************
  */
#ifndef __BL_JUMP_H
#define __BL_JUMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 镜像合法性判定结果（带原因，便于串口直接说清"为什么不能跳"）。 */
typedef enum
{
  BL_IMG_OK = 0,      /* 可以跳 */
  BL_IMG_BLANK,       /* 首字 = 0xFFFFFFFF：这一槽还没编程 */
  BL_IMG_BAD_SP,      /* 初始栈顶不在 RAM 范围 / 未对齐 */
  BL_IMG_BAD_RV,      /* 复位向量不是奇数（无 Thumb 位）或不在本槽区间 */
  BL_IMG_BAD_CRC      /* 🔴 S8：镜像级 CRC32 与元数据里声明的值不符
                       *    = "写了一半 / 内容坏了但向量表完好" —— 只有它能拦住 */
} BL_ImgStatus;

/**
  * @brief  校验某个槽里的镜像是否可跳。
  * @param  addr 槽起始地址
  * @param  size 槽大小（用来界定"复位向量必须落在本槽内"）
  * @param  sp   输出：读到的初始栈顶（可为 NULL）
  * @param  rv   输出：读到的复位向量（可为 NULL）
  * @retval 见 BL_ImgStatus
  * @note   判据只有三条，但每条都对应一种真实的失败：
  *         🔴 "栈顶落在 RAM" —— 空片/半写镜像会让栈顶是 0 或 0xFFFFFFFF，
  *            直接跳过去会立刻 HardFault，而且**看不出是 Bootloader 的错**；
  *         🔴 "复位向量是奇数" —— Cortex-M 的向量最低位必须是 1（Thumb 态），
  *            偶数说明这不是一个合法向量表；F4 的硬件在跳转时**不会替你检查**。
  *         ⚠️ 这三条**只读头 8 字节** ⇒ 拦不住"写了一半但向量表完好"的槽。
  *            要拦那种必须用 BL_ImageCheckCrc()。
  */
BL_ImgStatus BL_ImageCheck(uint32_t addr, uint32_t size, uint32_t *sp, uint32_t *rv);

/**
  * @brief  在 BL_ImageCheck() 之上再加一道**镜像级 CRC32**（S8）。
  * @param  dCrc 元数据里声明的整镜像 CRC32（= `zlib.crc32`）
  * @param  dLen 元数据里声明的镜像长度；**0 = 没有描述子** ⇒ 退回仅头校验
  * @param  verbose 1 = 把判据细节（算出来的 CRC / 声明值 / 退回仅头校验）
  *                 打印出来；0 = 只返回结果、一个字不打印
  * @retval BL_IMG_OK / BL_IMG_BAD_CRC（头 8 字节那几种原因照样原样透出）
  * @note   🔴 这就是"写 → 读回 → 校验"纪律在镜像级的落点，也是本工程
  *         唯一能拦住下面这个场景的手段：
  *           OTA 只写了前 20 帧就被判失败 ⇒ 槽里是半截镜像，但**向量表完好**
  *           ⇒ BL_ImageCheck() 判 valid ⇒ 跳进去 ⇒ App 跑飞、板子静默。
  *         ⚠️ 代价是**要读回整段镜像**（本工程镜像 ≈ 41.8 KB）：
  *           位运算版 CRC32 在 168 MHz 上约 8 次移位/字节 ⇒ 约 4 ms。
  *           只在**跳之前**做一次，完全可接受；换来的是"绝不跳进坏槽"。
  *         ⚠️ `dLen > size` 视为描述子不可信 ⇒ 同样报 BAD_CRC（宁可拒收）。
  *         ⚠️ `verbose = 0` 是给"安全等待里每秒轮询一次"用的 —— 那种地方
  *           一秒钟印一行会把串口刷满，而状态没变时重复的话没有信息量。
  */
BL_ImgStatus BL_ImageCheckCrc(uint32_t addr, uint32_t size,
                              uint32_t dCrc, uint32_t dLen,
                              uint32_t *sp, uint32_t *rv, int verbose);

/** @brief 把 BL_ImgStatus 变成一句人能读的话。 */
const char *BL_ImgStatusStr(BL_ImgStatus st);

/**
  * @brief  关掉自己开过的一切外设，然后跳到 App。
  * @note   🔴 **本函数不返回**（noreturn）。跳转前做四件事（推演稿 §4.3）：
  *         `__disable_irq()` → `SCB->VTOR = appAddr` → `__set_MSP(栈顶)` →
  *         跳 `*(appAddr+4)`；外加推演稿 §11.4 要求的**外设 DeInit**。
  *         ⚠️ 第五件事是参考仓库没做、而我们必须做的：**放开 PRIMASK**，
  *            否则 App 的 SysTick 永不触发 → `HAL_Delay` 永久卡死。
  *            详见 bl_jump.c 里的长注释。
  */
void BL_JumpToApp(uint32_t addr, uint32_t sp, uint32_t rv) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* __BL_JUMP_H */
