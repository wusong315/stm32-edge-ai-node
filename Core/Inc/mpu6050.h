/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 六轴惯性传感器驱动（寄存器直读版）
  *
  * 为什么自己写驱动：
  *   MPU6050 是 InvenSense（现 TDK）的器件，不是 ST 自家传感器，
  *   因此**没有 ST 官方的 stmdev 驱动栈**可用（参考工程的 hts221_reg.h 那种用不上），
  *   只能按数据手册直接读写寄存器。
  *
  * 设计约定（与项目口径一致）：
  *   - 加速度 ±2g  → 16384 LSB/g
  *   - 陀螺   ±250°/s → 131 LSB/(°/s)
  *   - DLPF_CFG = 1     → 加速度带宽约 184Hz，配 1kHz 采样（Nyquist 500Hz）
  *   - I2C 400kHz 下一次连读 6 字节约 0.2ms → **绝不能在 1ms 定时中断里调用本驱动**
  ******************************************************************************
  */

#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * 器件地址与身份标识
 * -------------------------------------------------------------------------*/
#define MPU6050_ADDR_7BIT       (0x68U)          /* AD0 = 0 */
#define MPU6050_ADDR_HAL        (MPU6050_ADDR_7BIT << 1)   /* 0xD0，HAL 要 8 位地址 */
#define MPU6050_WHO_AM_I_VALUE  (0x68U)          /* WHO_AM_I 寄存器的期望回读值 */

/* ---------------------------------------------------------------------------
 * 寄存器地址（只列本项目用到的）
 * -------------------------------------------------------------------------*/
#define MPU6050_REG_SMPLRT_DIV      (0x19U)   /* 采样率分频（只分频陀螺，不影响加速度） */
#define MPU6050_REG_CONFIG          (0x1AU)   /* 低通滤波 DLPF_CFG / 外部同步 */
#define MPU6050_REG_GYRO_CONFIG     (0x1BU)   /* 陀螺量程 FS_SEL */
#define MPU6050_REG_ACCEL_CONFIG    (0x1CU)   /* 加速度量程 AFS_SEL */
#define MPU6050_REG_INT_PIN_CFG     (0x37U)   /* 中断引脚行为 */
#define MPU6050_REG_INT_ENABLE      (0x38U)   /* 中断使能（DATA_RDY） */
#define MPU6050_REG_INT_STATUS      (0x3AU)   /* 中断标志 */
#define MPU6050_REG_ACCEL_XOUT_H    (0x3BU)   /* 加速度起始寄存器（后续寄存器自动自增） */
#define MPU6050_REG_PWR_MGMT_1      (0x6BU)   /* 电源管理 1：SLEEP 位在此 */
#define MPU6050_REG_PWR_MGMT_2      (0x6CU)   /* 电源管理 2：各轴待机 */
#define MPU6050_REG_WHO_AM_I        (0x75U)   /* 身份寄存器，固定回 0x68 */

/* ---------------------------------------------------------------------------
 * 量程与灵敏度
 * -------------------------------------------------------------------------*/
#define MPU6050_ACCEL_FS_2G_PA     (16384.0f)  /* ±2g  → LSB/g   */
#define MPU6050_GYRO_FS_250_PA     (131.0f)    /* ±250°/s → LSB/(°/s) */

/* 加速度连读长度：AX/AY/AZ 各 2 字节 */
#define MPU6050_ACCEL_BYTES        (6U)
/* 全量连读长度：加速度6 + 温度2 + 陀螺6 */
#define MPU6050_ALL_BYTES          (14U)

/* I2C 单次操作的超时（毫秒）。1ms 采样节拍下不能给大超时，否则会拖死主循环。 */
#define MPU6050_I2C_TIMEOUT_MS     (5U)

/* ---------------------------------------------------------------------------
 * 数据结构
 * -------------------------------------------------------------------------*/

/* 原始值（有符号 16 位，大端在寄存器里，本结构体中已还原为整数） */
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
} mpu6050_accel_raw_t;

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
} mpu6050_all_raw_t;

/* 物理量（单位：g / °C / °/s） */
typedef struct
{
  float ax;
  float ay;
  float az;
  float temp_c;
  float gx;
  float gy;
  float gz;
} mpu6050_data_t;

typedef enum
{
  MPU6050_OK        = 0,
  MPU6050_ERR_PROBE = 1,   /* WHO_AM_I 回读不符，器件没应答 */
  MPU6050_ERR_I2C   = 2    /* I2C 读写失败 */
} mpu6050_status_t;

/* ---------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------*/

/* 初始化：探测 WHO_AM_I → 解 SLEEP → 配 DLPF / 量程。
   返回 MPU6050_OK 才算器件真的在线。 */
mpu6050_status_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

/* 单次读 WHO_AM_I（不依赖 Init 是否已调用，可用于上电自检） */
mpu6050_status_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *id);

/* 连读 6 字节加速度（推荐在 1kHz 路径使用：一次事务拿到同一时刻的 XYZ） */
mpu6050_status_t MPU6050_ReadAccel(mpu6050_accel_raw_t *out);

/* 连读 14 字节（加速度 + 温度 + 陀螺），用于调试观察或后续扩展 */
mpu6050_status_t MPU6050_ReadAll(mpu6050_all_raw_t *out);

/* 原始值 → 物理量 */
void MPU6050_AccelToG(const mpu6050_accel_raw_t *raw, float *ax, float *ay, float *az);
void MPU6050_AllToPhysical(const mpu6050_all_raw_t *raw, mpu6050_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
