/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* =====================================================================
 * 应用配置（口径与依据见 MVP/docs/context/DECISIONS.md D15）
 * =====================================================================
 * 三档输出模式是**编译期开关**而不是运行期切换，理由：
 *   ① 模式 2 的 1 kHz 二进制流与模式 0/1 的人可读文字**不能共存** ——
 *      串口里混着二进制帧和文字，PC 端根本没法解析；
 *   ② 编译期剔除没用的分支，既省 Flash/RAM，也避免“未使用变量”告警。
 * 用 #ifndef 包住是为了能用 -DAPP_OUTPUT_MODE=n 在命令行覆盖，
 * 从而在无硬件时把三档都编译一遍做语法/告警检查。 */

/* 输出模式（三档，先跑通再往上加）：
 *   0 = 调试文本 10 Hz —— 只打印原始整数，用来**验证接线和数据对不对**。
 *       第一次上板就用这一档：静置时 az 应该接近 +16384（1g）。
 *   1 = 文本模式 200 Hz —— 每 TEXT_DECIM 个采样点输出一行，给人看 / 画波形。
 *   2 = 二进制帧模式 1 kHz —— 原速采集，攒够一帧整包发出，用来**采数据集**。 */
#ifndef APP_OUTPUT_MODE
#define APP_OUTPUT_MODE     (0)
#endif

#define SAMPLES_PER_FRAME   (32U)     /* 二进制模式每帧点数 */
#define TEXT_DECIM          (5U)      /* 1000 / 5   = 200 Hz */
#define DEBUG_DECIM         (100U)    /* 1000 / 100 =  10 Hz */

/* 串口发送队列（环形缓冲）大小，单位字节。
 * 1024 足够吸收一个二进制帧（197 字节）的突发。 */
#define TXQ_SIZE            (1024U)

/* 🔴 主循环单次最多往串口写多少字节 —— **这个数不能随便加大**。
 * 4 字节 @115200 ≈ 0.35 ms；再加上一次 I2C 读约 0.2 ms ≈ 0.55 ms，
 * 仍然小于 1 ms 采样周期，所以主循环绝不会因为发串口而错过采样节拍。
 * 详见 App_PumpUart() 上方的注释。 */
#define TXQ_CHUNK           (4U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* =====================================================================
 * 采样节拍（由 TIM2 的 1 kHz 中断推进）
 * =====================================================================
 * 分工铁律：中断里**只加一个数**，所有重活（I2C 读、串口发）都在主循环。
 * 原因见 USER CODE BEGIN 4 的注释。 */
static volatile uint32_t g_tick        = 0;   /* 中断里自增的“拍子”计数 */
static uint32_t          g_tick_seen   = 0;   /* 主循环已消费到第几拍 */
static uint32_t          g_sample_lost = 0;   /* 主循环没跟上、被跳过的拍数 */
static uint8_t           g_sensor_ok   = 0;   /* MPU6050 是否已探测到 */

/* =====================================================================
 * 串口发送环形队列
 * =====================================================================
 * 为什么需要它：115200 波特率下 1 字节约 87 us，一帧 198 字节要 17 ms 才发得完，
 * 而采样周期只有 1 ms。若在主循环里直接阻塞式发整帧，必丢大量采样点。
 * 所以：主循环只把字节“塞进队列”（微秒级、不阻塞），
 * 再由 App_PumpUart() 每次只吐 TXQ_CHUNK 个字节慢慢往串口倒。
 *
 * 队列只在主循环这一条上下文里读写（没有中断/DMA 碰它），
 * 因此无需加锁、无需 volatile —— 别被“环形队列”三个字吓住。 */
static uint8_t  g_txq[TXQ_SIZE];
static uint16_t g_txq_head = 0;               /* 写入位置（生产者 = 主循环） */
static uint16_t g_txq_tail = 0;               /* 读出位置（消费者 = App_PumpUart） */
static uint32_t g_tx_overflow = 0;            /* 队列满而丢弃的字节数 */

#if (APP_OUTPUT_MODE == 2)
/* =====================================================================
 * 以下只服务于「二进制帧模式」，其它模式下根本不会编译进来
 * （用条件编译而非运行时判断：既省 RAM，也避免未使用变量告警）
 * ===================================================================== */

/* 二进制帧格式（小端）：
 *   偏移      长度    内容
 *   0         2       同步头 0xAA 0x55
 *   2         2       帧序号 uint16（掉帧一眼看出）
 *   4         1       点数   uint8（= SAMPLES_PER_FRAME）
 *   5         N*6     数据：每点 ax,ay,az 各 int16
 *   5+N*6     1       校验和 = 前面所有字节之和 & 0xFF
 * 整帧 198 字节；PC 端解析脚本按同一张表读（tools/parse_vib_frame.py）。 */
#define FRAME_HDR0      (0xAAU)                    /* 同步头 1 */
#define FRAME_HDR1      (0x55U)                    /* 同步头 2 */
#define FRAME_OFF_SEQ   (2U)                       /* 帧序号偏移 */
#define FRAME_OFF_N     (4U)                       /* 点数偏移 */
#define FRAME_OFF_DATA  (5U)                       /* 数据起始偏移 */
#define FRAME_DATA_LEN  (SAMPLES_PER_FRAME * 6U)   /* 每点 3 轴 × int16 */
#define FRAME_TOTAL     (FRAME_OFF_DATA + FRAME_DATA_LEN + 1U)
static uint8_t s_frame[FRAME_TOTAL];

/* 三轴各存一列（而不是交错成一个数组）：
 * 将来接 NanoEdge AI 时要按轴整列 memcpy 进输入缓冲区，这样最省事。 */
static int16_t  g_frame_ax[SAMPLES_PER_FRAME];
static int16_t  g_frame_ay[SAMPLES_PER_FRAME];
static int16_t  g_frame_az[SAMPLES_PER_FRAME];
static uint16_t g_frame_idx     = 0;   /* 当前帧已填点数 */
static uint16_t g_frame_seq     = 0;   /* 帧序号 */
static uint32_t g_frame_dropped = 0;   /* 因丢拍而作废的半帧数 */
#endif /* APP_OUTPUT_MODE == 2 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void     App_TxqPush(const uint8_t *p, uint16_t n);
static void     App_PumpUart(void);
static void     App_OnTick(void);
static void     App_EnsureSensor(void);
#if (APP_OUTPUT_MODE == 2)
static uint16_t App_TxqFree(void);
static void     App_EmitFrame(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =====================================================================
 * 一、串口输出链路
 * ===================================================================== */

/* printf() 的最终落点。
 *
 * CubeMX 生成的 syscalls.c 里 _write() 是 weak 的，它会逐字符调用 __io_putchar()，
 * 所以我们**只需在这里定义 __io_putchar**；
 * 千万不要再自己定义一个 _write（那是多余的，容易和 syscalls.c 打架）。
 *
 * 本函数只负责“把字符塞进队列”，不碰串口硬件，因此微秒级返回：
 * 即使在 1 kHz 采样路径里调用 printf，也不会拖慢采样节拍。 */
int __io_putchar(int ch)
{
  uint8_t b = (uint8_t)ch;
  App_TxqPush(&b, 1U);
  return ch;
}

/* 把 n 个字节放入发送队列；放不下就丢弃并计数（不阻塞、不死等）。
 * 取舍原则：宁可丢日志，也绝不丢采样节拍。 */
static void App_TxqPush(const uint8_t *p, uint16_t n)
{
  uint16_t i;

  for (i = 0U; i < n; i++)
  {
    uint16_t next = (uint16_t)((g_txq_head + 1U) & (TXQ_SIZE - 1U));
    if (next == g_txq_tail)
    {
      g_tx_overflow += (uint32_t)(n - i);   /* 余下的全丢，立刻返回 */
      return;
    }
    g_txq[g_txq_head] = p[i];
    g_txq_head = next;
  }
}

/* 每次从队列搬最多 TXQ_CHUNK 个字节到串口，然后立刻返回。
 *
 * TXQ_CHUNK = 4 是算过账的：4 字节 @115200 ≈ 0.35 ms，
 * 加上一次 I2C 连读的约 0.2 ms，合计 0.55 ms < 1 ms 采样周期，
 * 因此主循环永远不会因为“发串口”而错过采样拍。
 * 想加大这个数之前，请先把这笔账重新算一遍。 */
static void App_PumpUart(void)
{
  uint16_t n = 0U;
  uint8_t  chunk[TXQ_CHUNK];

  while ((n < TXQ_CHUNK) && (g_txq_tail != g_txq_head))
  {
    chunk[n] = g_txq[g_txq_tail];
    g_txq_tail = (uint16_t)((g_txq_tail + 1U) & (TXQ_SIZE - 1U));
    n++;
  }

  if (n > 0U)
  {
    /* 阻塞发送，但只有 4 字节；正常情况下几十微秒即返回，超时给 100 ms 兜底 */
    (void)HAL_UART_Transmit(&huart1, chunk, n, 100U);
  }
}

#if (APP_OUTPUT_MODE == 2)
/* 队列里还能塞多少字节（故意留一格不用，用来区分“空”和“满”） */
static uint16_t App_TxqFree(void)
{
  uint16_t used = (uint16_t)((g_txq_head - g_txq_tail) & (TXQ_SIZE - 1U));
  return (uint16_t)(TXQ_SIZE - 1U - used);
}
#endif

/* =====================================================================
 * 二、上电自检 / 掉线重连
 * ===================================================================== */

/* 探测 MPU6050。硬件没插好时每秒重试一次并在串口打印提示，
 * 方便第一次上板时对着终端排查接线（这是最能省时间的几行代码）。 */
static void App_EnsureSensor(void)
{
  static uint8_t  s_first = 1U;
  static uint32_t s_last  = 0U;
  mpu6050_status_t st;

  if (g_sensor_ok != 0U)
  {
    return;
  }
  /* 首次立即探测；之后每 1000 拍（=1 秒）重试一次 */
  if ((s_first == 0U) && ((uint32_t)(g_tick - s_last) < 1000U))
  {
    return;
  }
  s_first = 0U;
  s_last  = g_tick;

  st = MPU6050_Init(&hi2c1);
  if (st == MPU6050_OK)
  {
    g_sensor_ok = 1U;
    printf("[OK] MPU6050 online (WHO_AM_I=0x68), 1kHz sampling starts\r\n");
  }
  else
  {
    uint8_t id = 0U;
    (void)MPU6050_WhoAmI(&hi2c1, &id);   /* 顺便把实际读到的 id 打出来 */
    printf("[WAIT] MPU6050 not found (id=0x%02X). Check wiring: "
           "VCC=3V3 GND SDA=PB7 SCL=PB6 AD0=GND\r\n", (unsigned)id);
  }
}

/* =====================================================================
 * 三、采样分派：每次新拍到，按当前输出模式处理一点
 * ===================================================================== */

/* 只处理“最新”那一拍；若主循环没跟上（pending > 1），把丢掉的拍数记下来。
 * 模式 2 下还要额外作废当前半帧 —— 有断层的数据不能当作连续序列喂给 AI。 */
static void App_OnTick(void)
{
  uint32_t pending = g_tick - g_tick_seen;

  if (pending == 0U)
  {
    return;                      /* 还没有新拍，立刻返回（主循环高频空转） */
  }
  if (pending > 1U)
  {
    g_sample_lost += (pending - 1U);
#if (APP_OUTPUT_MODE == 2)
    if (g_frame_idx > 0U)
    {
      g_frame_idx = 0U;
      g_frame_dropped++;
    }
#endif
  }
  g_tick_seen = g_tick;

  if (g_sensor_ok == 0U)
  {
    return;                      /* 传感器没在线，交给 App_EnsureSensor 去重连 */
  }

#if (APP_OUTPUT_MODE == 0)
  /* -------------------------------------------------------------------
   * 模式 0：调试文本 10 Hz —— 只打印原始整数
   * -------------------------------------------------------------------
   * 第一次上板就用这一档！判断标准：
   *   静置时 az ≈ +16384（= +1 g）、ax/ay ≈ 0；
   *   把板子翻过来，az 应变成 ≈ -16384。
   * 若三轴恒为 0 或恒为 -1，基本可断定 I2C 没通（接线/上拉/AD0）。 */
  if ((g_tick % DEBUG_DECIM) != 0U)
  {
    return;
  }
  {
    mpu6050_all_raw_t all;
    if (MPU6050_ReadAll(&all) != MPU6050_OK)
    {
      printf("[ERR] I2C read failed, sensor offline\r\n");
      g_sensor_ok = 0U;
      return;
    }
    printf("t=%8lu | ax=%6d ay=%6d az=%6d | T=%6d | gx=%6d gy=%6d gz=%6d"
           " | lost=%lu ovf=%lu\r\n",
           (unsigned long)g_tick,
           (int)all.ax, (int)all.ay, (int)all.az, (int)all.temp,
           (int)all.gx, (int)all.gy, (int)all.gz,
           (unsigned long)g_sample_lost, (unsigned long)g_tx_overflow);
  }
  /* 10 Hz 打印 → 5 Hz 闪烁，一眼看出程序在跑 */
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

#elif (APP_OUTPUT_MODE == 1)
  /* -------------------------------------------------------------------
   * 模式 1：文本波形 200 Hz —— 每 5 拍输出一行三轴 g 值
   * -------------------------------------------------------------------
   * 🔴 %.3f 需要链接时加 -u _printf_float，否则小数部分会被**静默丢掉**
   *    （不报错、不警告，只是打印出 0.000）。见 DECISIONS D15 / RUNBOOK R6.2。
   * 带宽：每行约 22 字节 × 200 行/s ≈ 4.4 KB/s，115200（≈11.5 KB/s）够用。 */
  if ((g_tick % TEXT_DECIM) != 0U)
  {
    return;
  }
  {
    mpu6050_accel_raw_t raw;
    float ax, ay, az;

    if (MPU6050_ReadAccel(&raw) != MPU6050_OK)
    {
      g_sensor_ok = 0U;
      return;
    }
    MPU6050_AccelToG(&raw, &ax, &ay, &az);
    printf("%.3f,%.3f,%.3f\r\n", (double)ax, (double)ay, (double)az);
  }
  /* 每秒打一行以 # 开头的状态注释；PC 端脚本遇到 '#' 直接跳过，
   * 这样既能看到健康度，也不会打断数据列。 */
  if ((g_tick % 1000U) < TEXT_DECIM)
  {
    printf("# lost=%lu ovf=%lu\r\n",
           (unsigned long)g_sample_lost, (unsigned long)g_tx_overflow);
  }

#elif (APP_OUTPUT_MODE == 2)
  /* -------------------------------------------------------------------
   * 模式 2：二进制帧 1 kHz —— 原速采集，攒够一帧整包发出
   * -------------------------------------------------------------------
   * 🔴 这一档**进入稳定出帧后不再输出任何文字**：串口里一旦混入文本，
   *    帧同步头 0xAA 0x55 就可能被误判/错位。
   *    （开机横幅与自检提示属于例外，它们出现在出帧之前；PC 端解析器
   *      靠「同步头 + 校验和 + 序号连续」三重校验，足以拒绝任何误同步。）
   *    想看 lost/dropped 计数请用 IDE 的 Live Watch 看 g_sample_lost /
   *    g_frame_dropped / g_tx_overflow 这三个变量。
   * 带宽：198 字节/帧 × 31.25 帧/s ≈ 6.2 KB/s < 11.5 KB/s，留有余量。 */
  {
    mpu6050_accel_raw_t raw;

    if (MPU6050_ReadAccel(&raw) != MPU6050_OK)
    {
      g_sensor_ok = 0U;
      return;
    }
    g_frame_ax[g_frame_idx] = raw.ax;
    g_frame_ay[g_frame_idx] = raw.ay;
    g_frame_az[g_frame_idx] = raw.az;
    g_frame_idx++;

    if (g_frame_idx >= SAMPLES_PER_FRAME)
    {
      App_EmitFrame();
      g_frame_idx = 0U;
    }
  }
#endif

}

#if (APP_OUTPUT_MODE == 2)
/* 打包一整帧（格式见 USER CODE BEGIN PV 的注释）并整帧入队。
 * 队列装不下就整帧丢弃，绝不写“半帧”出去 —— 半帧会彻底打乱 PC 端解析。 */
static void App_EmitFrame(void)
{
  uint16_t i;
  uint16_t off = FRAME_OFF_DATA;
  uint8_t  sum = 0U;

  s_frame[0] = FRAME_HDR0;
  s_frame[1] = FRAME_HDR1;
  s_frame[FRAME_OFF_SEQ]     = (uint8_t)(g_frame_seq & 0xFFU);
  s_frame[FRAME_OFF_SEQ + 1] = (uint8_t)((g_frame_seq >> 8) & 0xFFU);
  s_frame[FRAME_OFF_N]       = (uint8_t)SAMPLES_PER_FRAME;

  for (i = 0U; i < SAMPLES_PER_FRAME; i++)
  {
    uint16_t ax = (uint16_t)g_frame_ax[i];
    uint16_t ay = (uint16_t)g_frame_ay[i];
    uint16_t az = (uint16_t)g_frame_az[i];
    s_frame[off++] = (uint8_t)(ax & 0xFFU);
    s_frame[off++] = (uint8_t)((ax >> 8) & 0xFFU);
    s_frame[off++] = (uint8_t)(ay & 0xFFU);
    s_frame[off++] = (uint8_t)((ay >> 8) & 0xFFU);
    s_frame[off++] = (uint8_t)(az & 0xFFU);
    s_frame[off++] = (uint8_t)((az >> 8) & 0xFFU);
  }

  for (i = 0U; i < (uint16_t)(FRAME_TOTAL - 1U); i++)
  {
    sum = (uint8_t)(sum + s_frame[i]);
  }
  s_frame[FRAME_TOTAL - 1U] = sum;

  if (App_TxqFree() >= (uint16_t)FRAME_TOTAL)
  {
    App_TxqPush(s_frame, (uint16_t)FRAME_TOTAL);
    g_frame_seq++;
  }
  else
  {
    g_frame_dropped++;
  }
}
#endif /* APP_OUTPUT_MODE == 2 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ---- 开场横幅（全部走发送队列，立即返回，不阻塞启动）------------- */
  printf("\r\n\r\n=== stm32-edge-ai-node boot ===\r\n");
  printf("SYSCLK = %lu Hz, I2C1 = 400 kHz, TIM2 = 1 kHz\r\n",
         (unsigned long)HAL_RCC_GetSysClockFreq());
#if (APP_OUTPUT_MODE == 0)
  printf("mode 0: debug text @10 Hz (raw integers, for wiring check)\r\n");
#elif (APP_OUTPUT_MODE == 1)
  printf("mode 1: text waveform @200 Hz (unit = g)\r\n");
#elif (APP_OUTPUT_MODE == 2)
  printf("mode 2: binary frames @1 kHz, %u bytes/frame\r\n",
         (unsigned)FRAME_TOTAL);
#endif

  /* ---- 启动 1 kHz 采样节拍 ------------------------------------------
   * 🔴 必须是 Start_IT（带中断）而不是 Start：
   *    否则 HAL_TIM_PeriodElapsedCallback 永远不会被调用，g_tick 一格都不涨，
   *    现象是“程序跑起来了但什么都不输出”，很容易误判成接线问题。
   *    TIM2 的 NVIC 已在 HAL_TIM_Base_MspInit 里使能（见 stm32f4xx_hal_msp.c）。 */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* ---- 首次探测传感器（之后由主循环每秒重试）----------------------- */
  App_EnsureSensor();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 主循环三件事，按“越频繁越靠前”排：
     *   1) 倒串口 —— 每次只吐 4 字节，保证 1 ms 采样节拍不被拖垮；
     *   2) 处理采样 —— 有新拍才动，没新拍立刻返回；
     *   3) 传感器掉线重连 —— 没事时也立刻返回。
     * 设计上这里**没有 HAL_Delay / 阻塞等待**，主循环全速空转，
     * 因此采样抖动只取决于 I2C 与串口的实际耗时之和。 */
    App_PumpUart();
    App_OnTick();
    App_EnsureSensor();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* =====================================================================
 * 1 kHz 采样节拍中断：这里**只做一件事** —— 把拍子加一。
 * =====================================================================
 * 为什么不干脆在这里读 I2C？四条理由：
 *   1) 6 字节连读在 400 kHz 下约 0.2 ms，占掉 1 ms 周期的 20% CPU，
 *      而且中断执行期间会压住其它优先级更低的中断（包括 SysTick 计时）。
 *   2) HAL_I2C_Mem_Read 是**阻塞式**实现，在中断里调用阻塞函数是典型危险写法：
 *      一旦器件 NACK 或总线卡住，超时等待会把整个系统拖住。
 *   3) 中断服务程序越短越不容易出竞态；这里只有一句自增，天然安全。
 *   4) 参考工程（ST 官方 datalogger）也是这个套路：
 *      HAL_GPIO_EXTI_Callback 里只置一个 volatile 标志，重活全留给主循环。
 *
 * 于是形成本项目的一条铁律：
 *   **中断只举旗子，重活留给主循环。**
 * 主循环看到 g_tick 变了，才去读传感器、才去发串口。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    g_tick++;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
