/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 驱动实现（I2C 寄存器直读）
  *
  * 参考来源：ST 官方 datalogger 工程的 platform_read/platform_write 写法，
  * 但**去掉了 ST MEMS 专有的 `reg |= 0x80` 连续读标志** ——
  * MPU6050 读连续寄存器本来就会自动地址自增，没有这个位。
  * 逐条对照见 MVP/docs/context/REF-F4-NANOEDGE.md
  ******************************************************************************
  */

#include "mpu6050.h"

/* 本驱动只服务一个器件，用文件级静态句柄保存 I2C 绑定，避免层层传参。 */
static I2C_HandleTypeDef *s_hi2c = NULL;

/* WHO_AM_I 探测参数：上电后器件内部稳定需要时间，因此给足重试窗口。
   ⚠️ 与参考工程的区别：参考工程用 `do{...}while(id != expected)` 死等，
   器件不在线时会**永久卡死**。这里改成**有限重试 + 返回错误码**，
   让上层能明确报"传感器没找到"，而不是黑屏卡住 —— 调试期这个差别很大。 */
#define MPU6050_PROBE_ATTEMPTS   (20U)
#define MPU6050_PROBE_DELAY_MS   (10U)

/* ---------------------------------------------------------------------------
 * I2C 底层读写
 * -------------------------------------------------------------------------*/

static HAL_StatusTypeDef mpu_reg_write(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(s_hi2c, MPU6050_ADDR_HAL, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1,
                           MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mpu_reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(s_hi2c, MPU6050_ADDR_HAL, reg,
                          I2C_MEMADD_SIZE_8BIT, buf, len,
                          MPU6050_I2C_TIMEOUT_MS);
}

/* MPU6050 的输出寄存器是**大端**（高字节在前），必须按 (hi<<8)|lo 还原。 */
static int16_t mpu_be16(const uint8_t *p)
{
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ---------------------------------------------------------------------------
 * 上电自检
 * -------------------------------------------------------------------------*/

mpu6050_status_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *id)
{
  if (hi2c == NULL || id == NULL)
  {
    return MPU6050_ERR_I2C;
  }
  s_hi2c = hi2c;

  if (mpu_reg_read(MPU6050_REG_WHO_AM_I, id, 1) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }
  return (*id == MPU6050_WHO_AM_I_VALUE) ? MPU6050_OK : MPU6050_ERR_PROBE;
}

/* ---------------------------------------------------------------------------
 * 初始化
 * -------------------------------------------------------------------------*/

mpu6050_status_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t  id    = 0;
  uint8_t  tries = 0;

  if (hi2c == NULL)
  {
    return MPU6050_ERR_I2C;
  }
  s_hi2c = hi2c;

  /* ---- 1. 探测器件：反复读 WHO_AM_I 直到读到 0x68（或耗尽重试） ---- */
  do
  {
    HAL_Delay(MPU6050_PROBE_DELAY_MS);
    if (mpu_reg_read(MPU6050_REG_WHO_AM_I, &id, 1) == HAL_OK && id == MPU6050_WHO_AM_I_VALUE)
    {
      break;
    }
    tries++;
  } while (tries < MPU6050_PROBE_ATTEMPTS);

  if (id != MPU6050_WHO_AM_I_VALUE)
  {
    return MPU6050_ERR_PROBE;   /* 接线 / 上拉 / 供电 / AD0 问题，上层报错即可 */
  }

  /* ---- 2. 解除休眠。上电默认 SLEEP=1，不做这一步后面读什么都是 0 ---- */
  if (mpu_reg_write(MPU6050_REG_PWR_MGMT_1, 0x00U) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }
  HAL_Delay(10);   /* 唤醒后等内部时钟切换稳定 */

  /* ---- 3. 采样率分频 = 0。⚠️ 该寄存器只分频**陀螺**；
            加速度计的输出率恒为 1kHz，与它无关。
            所以本项目的 TIM2 1kHz 采样与器件天然自洽。 ---- */
  if (mpu_reg_write(MPU6050_REG_SMPLRT_DIV, 0x00U) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }

  /* ---- 4. 数字低通滤波 DLPF_CFG = 1（加速度带宽约 184Hz，陀螺约 188Hz）
            配 1kHz 采样：Nyquist 500Hz > 184Hz，不会混叠；
            同时滤波能削掉电机电流噪声。 ---- */
  if (mpu_reg_write(MPU6050_REG_CONFIG, 0x01U) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }

  /* ---- 5. 量程：加速度 ±2g（AFS_SEL=0）→ 16384 LSB/g ---- */
  if (mpu_reg_write(MPU6050_REG_ACCEL_CONFIG, 0x00U) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }

  /* ---- 6. 量程：陀螺 ±250°/s（FS_SEL=0）→ 131 LSB/(°/s)
            首版不做姿态解算，但先配上，免得以后回来改。 ---- */
  if (mpu_reg_write(MPU6050_REG_GYRO_CONFIG, 0x00U) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }

  /* 本项目用 TIM2 定时采样，**不使用器件的 DATA_RDY 中断**，
     故 INT_ENABLE / INT_PIN_CFG 保持默认（关闭）即可，无需配置。 */

  return MPU6050_OK;
}

/* ---------------------------------------------------------------------------
 * 读数据
 * -------------------------------------------------------------------------*/

mpu6050_status_t MPU6050_ReadAccel(mpu6050_accel_raw_t *out)
{
  uint8_t buf[MPU6050_ACCEL_BYTES];

  if (s_hi2c == NULL || out == NULL)
  {
    return MPU6050_ERR_I2C;
  }

  /* 一次连读 6 字节而不是分 3 次读 2 字节：
     分次读会跨采样时刻，三轴不来自同一瞬间 → 波形会被"撕裂"。 */
  if (mpu_reg_read(MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_ACCEL_BYTES) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }

  out->ax = mpu_be16(&buf[0]);
  out->ay = mpu_be16(&buf[2]);
  out->az = mpu_be16(&buf[4]);
  return MPU6050_OK;
}

mpu6050_status_t MPU6050_ReadAll(mpu6050_all_raw_t *out)
{
  uint8_t buf[MPU6050_ALL_BYTES];

  if (s_hi2c == NULL || out == NULL)
  {
    return MPU6050_ERR_I2C;
  }

  /* 0x3B 起连读 14 字节 = 加速度6 + 温度2 + 陀螺6（寄存器地址自动自增） */
  if (mpu_reg_read(MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_ALL_BYTES) != HAL_OK)
  {
    return MPU6050_ERR_I2C;
  }

  out->ax   = mpu_be16(&buf[0]);
  out->ay   = mpu_be16(&buf[2]);
  out->az   = mpu_be16(&buf[4]);
  out->temp = mpu_be16(&buf[6]);
  out->gx   = mpu_be16(&buf[8]);
  out->gy   = mpu_be16(&buf[10]);
  out->gz   = mpu_be16(&buf[12]);
  return MPU6050_OK;
}

/* ---------------------------------------------------------------------------
 * 换算
 * -------------------------------------------------------------------------*/

void MPU6050_AccelToG(const mpu6050_accel_raw_t *raw, float *ax, float *ay, float *az)
{
  if (raw == NULL)
  {
    return;
  }
  if (ax != NULL) { *ax = (float)raw->ax / MPU6050_ACCEL_FS_2G_PA; }
  if (ay != NULL) { *ay = (float)raw->ay / MPU6050_ACCEL_FS_2G_PA; }
  if (az != NULL) { *az = (float)raw->az / MPU6050_ACCEL_FS_2G_PA; }
}

void MPU6050_AllToPhysical(const mpu6050_all_raw_t *raw, mpu6050_data_t *out)
{
  if (raw == NULL || out == NULL)
  {
    return;
  }
  out->ax     = (float)raw->ax / MPU6050_ACCEL_FS_2G_PA;
  out->ay     = (float)raw->ay / MPU6050_ACCEL_FS_2G_PA;
  out->az     = (float)raw->az / MPU6050_ACCEL_FS_2G_PA;
  /* MPU6050 温度换算（数据手册给出的经验公式） */
  out->temp_c = ((float)raw->temp / 340.0f) + 36.53f;
  out->gx     = (float)raw->gx / MPU6050_GYRO_FS_250_PA;
  out->gy     = (float)raw->gy / MPU6050_GYRO_FS_250_PA;
  out->gz     = (float)raw->gz / MPU6050_GYRO_FS_250_PA;
}
