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

/* 🔴 调用 HAL 之前先自己看 BUSY 标志（2026-10-18，修 200 ms 停顿的**真因**）。
 *
 * 坑的机理：F4 的 HAL 在 `HAL_I2C_Mem_Read/Write` 开头会等 BUSY 位清零，而
 * 等待上限是 HAL 内部**写死的** `I2C_TIMEOUT_BUSY_FLAG = 25 ms`，
 * **完全不理会我们传进去的 5 ms / 2 ms 超时**。
 * 于是"上一笔传输被 EMI 打断、总线停在 BUSY"时，每一次尝试都要**干等 25 ms**。
 *
 * 实测对账（`app_ver=7`，`d100`，60 s）：6 次停顿，最坏 198 ms、期间只失败 **8** 次
 * ⇒ 8 × 25 ms = 200 ms，**严丝合缝**。也解释了为什么把超时 5→2 ms 一点用没有
 * （每失败成本 13.7 → 13.0 ms 不动）。
 *
 * 本工程是**单主机、单线程**，进到这里的时刻不可能有别人在传数据，
 * 所以 BUSY=1 只能意味着"上一笔被打断、外设还卡在传输态"。
 * 这时最该做的是**立刻返回失败**，让上层（`App_OnTick` 的②级）去补时钟把它解开，
 * 而不是在 HAL 里盲等 25 ms —— 25 ms 是 25 个采样点。 */
static HAL_StatusTypeDef mpu_wait_idle(void)
{
  /* ⚠️ 不能"看到 BUSY 就立刻失败"：一笔传输刚发完 STOP 时，BUSY 位还会**滞留几微秒**
   *    （HAL 自己在下一笔开头就是靠等它清零来跨过这段）。所以先自旋等一小会儿。
   *    2026-10-18 踩过：第一版写成"立即返回 HAL_BUSY"，结果**读全好、写全挂**
   *    （`step=1`：紧跟在读之后的 `PWR_MGMT_1` 写被误杀），传感器再也上不了线。
   * 取 ≈50 µs 上限：既跨得过正常的 STOP 滞留，又比 HAL 写死的 25 ms 快 500 倍。
   * 自旋 2000 次 ≈ 2000×4 周期 / 168 MHz ≈ 48 µs。 */
  uint32_t spin = 2000U;

  while (spin-- != 0U)
  {
    if (__HAL_I2C_GET_FLAG(s_hi2c, I2C_FLAG_BUSY) == RESET)
    {
      return HAL_OK;
    }
  }
  return HAL_BUSY;
}

static HAL_StatusTypeDef mpu_reg_write(uint8_t reg, uint8_t value)
{
  if (mpu_wait_idle() != HAL_OK)
  {
    return HAL_BUSY;
  }
  return HAL_I2C_Mem_Write(s_hi2c, MPU6050_ADDR_HAL, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1,
                           MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mpu_reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (mpu_wait_idle() != HAL_OK)
  {
    return HAL_BUSY;
  }
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

/* 器件已被确认在线之后的那一段配置（解 SLEEP → 配采样率/带宽/量程）。
 * ⚠️ 单独抽出来的唯一原因：冷启动（MPU6050_Init）与掉线重连（MPU6050_Reinit）
 *    的**探测策略不同**，但配置序列必须**逐字一致**——
 *    否则重连后器件停在别的 DLPF / 量程上，数据会静默变形，而波形看着"正常"。 */
/* 诊断用：0 = 没失败；1..5 = mpu_configure 的第几步写挂了；6 = 探测阶段。
 * 只在 Debug 排查时读（ST-Link / 串口打印），量产无副作用（每次失败只写一个字节）。 */
volatile uint8_t g_mpu_fail_step = 0U;
/* HAL 的 ErrorCode（bit1 AF / bit2 BERR / bit3 ARLO / bit8 TIMEOUT...），
   失败时一起抓，能一眼分清"没人应答"还是"总线仲裁丢了"还是"超时"。 */
volatile uint8_t g_mpu_fail_hal  = 0U;
static mpu6050_status_t mpu_configure(void);

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

  return mpu_configure();
}

mpu6050_status_t MPU6050_Reinit(I2C_HandleTypeDef *hi2c)
{
  uint8_t id = 0;

  if (hi2c == NULL)
  {
    return MPU6050_ERR_I2C;
  }
  s_hi2c = hi2c;

  /* 🔴 只探 1 次，而且**连那 10 ms 的预延时都省掉**。
   *    重连路径上每一毫秒都是采样点：失败就 5 ms 内给结论，
   *    让上层按退避节奏再来，而不是在这里死等 200–300 ms。
   *    器件此刻是真的坏还是仅仅被 EMI 打了一下，由上层用"连续失败次数"判，
   *    不该由这一层用长延时替它扛。 */
  if (mpu_reg_read(MPU6050_REG_WHO_AM_I, &id, 1) != HAL_OK)
  {
    g_mpu_fail_step = 7U;   /* 7 = 重连探测的 I2C 读本身就失败（含 BUSY 短路） */
    return MPU6050_ERR_I2C;
  }
  if (id != MPU6050_WHO_AM_I_VALUE)
  {
    g_mpu_fail_step = 6U;
    return MPU6050_ERR_PROBE;
  }
  g_mpu_fail_step = 0U;

  return mpu_configure();
}

static mpu6050_status_t mpu_configure(void)
{
  /* ---- 2. 解除休眠。上电默认 SLEEP=1，不做这一步后面读什么都是 0 ---- */
  if (mpu_reg_write(MPU6050_REG_PWR_MGMT_1, 0x00U) != HAL_OK)
  {
    g_mpu_fail_step = 1U;
    g_mpu_fail_hal = (uint8_t)(HAL_I2C_GetError(s_hi2c) & 0xFFU);
    return MPU6050_ERR_I2C;
  }
  HAL_Delay(10);   /* 唤醒后等内部时钟切换稳定 */

  /* ---- 3. 采样率分频 = 0。⚠️ 该寄存器只分频**陀螺**；
            加速度计的输出率恒为 1kHz，与它无关。
            所以本项目的 TIM2 1kHz 采样与器件天然自洽。 ---- */
  if (mpu_reg_write(MPU6050_REG_SMPLRT_DIV, 0x00U) != HAL_OK)
  {
    g_mpu_fail_step = 2U;
    g_mpu_fail_hal = (uint8_t)(HAL_I2C_GetError(s_hi2c) & 0xFFU);
    return MPU6050_ERR_I2C;
  }

  /* ---- 4. 数字低通滤波 DLPF_CFG = 0（加速度带宽 **260Hz**，延迟 0ms）
            🔴 这里必须是 0，不能用 1 —— 理由（见 RISKS.md R17）：
              MPU6050 加速度计带宽**最高只有 260Hz**，而本项目叶频是 300–540Hz，
              本来就已经在带宽之外。DLPF_CFG=1 只给到 184Hz，会把叶频整段再压低
              （300Hz 约 −8dB、540Hz 约 −19dB），而不平衡故障的主特征恰恰就是
              叶频幅值 → 信噪比和灵敏度都会掉。
              所以取**能用满的最大带宽 260Hz**，把能留的叶频能量留住。
            代价：抗混叠差一点（1kHz 采样、Nyquist 500Hz，260Hz 之后衰减较缓）。
              → 上板后**必须先用频谱验证叶频是否真的可见**，不可见的对策见 R17。 ---- */
  if (mpu_reg_write(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG) != HAL_OK)
  {
    g_mpu_fail_step = 3U;
    g_mpu_fail_hal = (uint8_t)(HAL_I2C_GetError(s_hi2c) & 0xFFU);
    return MPU6050_ERR_I2C;
  }

  /* ---- 5. 量程：加速度 ±2g（AFS_SEL=0）→ 16384 LSB/g ---- */
  if (mpu_reg_write(MPU6050_REG_ACCEL_CONFIG, 0x00U) != HAL_OK)
  {
    g_mpu_fail_step = 4U;
    g_mpu_fail_hal = (uint8_t)(HAL_I2C_GetError(s_hi2c) & 0xFFU);
    return MPU6050_ERR_I2C;
  }

  /* ---- 6. 量程：陀螺 ±250°/s（FS_SEL=0）→ 131 LSB/(°/s)
            首版不做姿态解算，但先配上，免得以后回来改。 ---- */
  if (mpu_reg_write(MPU6050_REG_GYRO_CONFIG, 0x00U) != HAL_OK)
  {
    g_mpu_fail_step = 5U;
    g_mpu_fail_hal = (uint8_t)(HAL_I2C_GetError(s_hi2c) & 0xFFU);
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
