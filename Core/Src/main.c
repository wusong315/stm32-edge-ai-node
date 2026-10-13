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
#include "neai_app.h"
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
 * 四档输出模式是**编译期开关**而不是运行期切换，理由：
 *   ① 模式 2 的 1 kHz 二进制流与模式 0/1/3 的人可读文字**不能共存** ——
 *      串口里混着二进制帧和文字，PC 端根本没法解析；
 *   ② 编译期剔除没用的分支，既省 Flash/RAM，也避免“未使用变量”告警。
 * 用 #ifndef 包住是为了能用 -DAPP_OUTPUT_MODE=n 在命令行覆盖，
 * 从而在无硬件时把四档都编译一遍做语法/告警检查。 */

/* 输出模式（四档，先跑通再往上加）：
 *   0 = 调试文本 10 Hz —— 只打印原始整数，用来**验证接线和数据对不对**。
 *       第一次上板就用这一档：静置时 az 应该接近 +16384（1g）。
 *   1 = 文本模式 200 Hz —— 每 TEXT_DECIM 个采样点输出一行，给人看 / 画波形。
 *   2 = 二进制帧模式 1 kHz —— 原速采集，攒够一帧整包发出，用来**采数据集**。
 *   3 = 在线推理模式 1 kHz —— 原速采集**直接喂给 NanoEdge AI 模型**（不往串口灌
 *       原始数据），串口只输出相似度与推理耗时。**这是最终产品的形态。**
 *       ⚠️ 依赖 `Core/Src/libneai.a` + `Core/Inc/NanoEdgeAI.h` 这一对库文件，
 *          集成与换库步骤见 `MVP/docs/context/RUNBOOK.md` R10。 */
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

/* =====================================================================
 * 第 ⑦ 步：PWM 限速通道（PA6 = TIM3_CH1）
 * =====================================================================
 * 为什么需要它（依据 R17）：MPU6050 的加速度计带宽上限只有 260 Hz，而 3 V 直驱时
 * 叶频 = 3 × 转频 = 3 × 136.7 = 410.2 Hz —— **落在器件带宽之外**，
 * 结果是"频率位置可信、绝对幅值不可信"。PWM 把转速压下来之后，
 * 1×/2×/3× 全部落进 260 Hz 以内，位置和幅值同时变可信。
 * 另一条降速路线（1 节电芯 + 空槽跨接 ≈ 1.5 V）已放弃 —— 见 RUNBOOK R6.4。
 *
 * 硬件前置（缺一不可，见 RUNBOOK R6.4 第 ⑦ 步）：
 *   ① 必须共地：IRF520N 模块 GND ↔ STM32 GND ↔ 电池盒负极
 *   ② 电机两端并续流二极管（1N4148/1N5819，**阴极接电机 +**）
 *   ③ 电机供电仍走电池盒（3 V）—— 🔴 **不要接 5V 排针**
 *
 * 参数：TIM3 挂在 APB1（84 MHz）→ 84e6 / (0+1) / (4199+1) = 20.0 kHz。
 *   取 20 kHz 是为了**避开可听噪声**；PSC 取 0 让占空比分辨率做到 1/4200。
 * 🔴 上电默认占空比 = 0（电机不转）—— 接线还没确认时上电不应该转起来。 */
#define APP_PWM_PSC          (0U)
#define APP_PWM_ARR          (4199U)     /* 84 MHz / 4200 = 20.0 kHz */
#define APP_PWM_DUTY_DEFAULT (0U)        /* 上电占空比（0..100，单位 %） */

/* 串口命令（走 USART1 = PA10，即已经接着的那根 CH340 线，**不需要加线**）：
 *   `dNN` + 回车 → 把占空比设为 NN%（0..100，超过按 100 处理）
 *   `?`   + 回车 → 回报当前占空比
 * 只用一个字母 + 最多 3 位十进制，够用且不引入 sscanf（省 Flash）。 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* =====================================================================
 * 采样节拍（由 TIM2 的 1 kHz 中断推进）
 * =====================================================================
 * 分工铁律：中断里**只加一个数**，所有重活（I2C 读、串口发）都在主循环。
 * 原因见 USER CODE BEGIN 4 的注释。 */
static volatile uint32_t g_tick        = 0;   /* 中断里自增的“拍子”计数 */
static uint32_t          g_tick_seen   = 0;   /* 主循环已消费到第几拍 */
static uint32_t          g_sample_lost = 0;   /* 稳态下主循环没跟上的拍数（启动期不计，见 App_SyncTickBaseline） */
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

/* =====================================================================
 * PWM 限速（第 ⑦ 步）—— 参数来历见 USER CODE BEGIN PD
 * =====================================================================
 * ⚠️ **这里的 TIM3 是手写的，不是 CubeMX 生成的**，这是有意的：
 *    本工程 `.cproject` 里有手工加的 `-L Core/Src -lneai` 与
 *    `nanoprintffloat`（`-u _printf_float`），让 CubeMX 重新 GENERATE CODE
 *    会把这些一起覆盖掉 —— 代价比手写这 40 行大得多。
 *    手写版全部落在 USER CODE 区内，**CubeMX 重新生成也丢不了**。
 *    将来若要在 CubeMX 里正式加 TIM3：请把下面这一整段（PV/0/2 三处）
 *    删掉，改用生成的 `htim3`，**别让两套初始化同时存在**（会重复配置同一个外设）。*/
static TIM_HandleTypeDef  htim3_pwm;                            /* 🔴 特意不叫 htim3 */
static uint8_t            g_pwm_duty = (uint8_t)APP_PWM_DUTY_DEFAULT;  /* 当前占空比 % */

/* ---- 串口命令接收状态（USART1 单字节中断）---------------------------
 * 中断里**只攒字符**，真正的动作（改占空比 / 打印）由主循环的 App_PollCmd() 执行。
 * 分工理由与采样节拍完全一样：中断里干的活越少越好。
 * 这几个变量跨中断与主循环 → 必须 volatile。 */
#define APP_CMD_NONE   (0U)
#define APP_CMD_DUTY   (1U)
#define APP_CMD_INFO   (2U)
static volatile uint8_t   s_rx_byte   = 0U;   /* HAL_UART_Receive_IT 的落点 */
static volatile uint8_t   s_cmd_kind  = APP_CMD_NONE;
static volatile uint8_t   s_cmd_val   = 0U;   /* 累积到的数值 */
static volatile uint8_t   s_cmd_digit = 0U;   /* 本行已收到几位数字（最多 3 位） */
static volatile uint8_t   s_cmd_ready = 0U;   /* 1 = 有一整条命令待主循环处理 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void     App_TxqPush(const uint8_t *p, uint16_t n);
static void     App_PumpUart(void);
static void     App_OnTick(void);
static void     App_EnsureSensor(void);
#if (APP_OUTPUT_MODE == 2)
static uint16_t App_TxqFree(void);
static void     App_EmitFrame(void);
#endif
static void     App_PwmInit(void);
static void     App_SetDuty(uint8_t pct);
static void     App_CmdRxStart(void);
static void     App_PollCmd(void);
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
 * 二、I2C 总线体检（只在探测失败时打印一次）
 * ===================================================================== */

/* 为什么需要它：`id=0x00` 这一个现象**同时**覆盖四种成因，而四者的修法没有交集 ——
 *   ① SDA/SCL 没接通（线掉了 / 插错孔）      → 查线
 *   ② SDA/SCL 被钉在 GND（短接 / 器件坏）    → 拔线二分定位
 *   ③ AD0 没接 GND，器件改在 0x69 上应答     → 把 AD0 接地
 *   ④ 器件本体或供电有问题                   → 换模块 / 换供电
 * 光盯着串口看 `id=0x00` 是分不出来的，人只能在四个方向上瞎试。下面三项检查
 * 把它的覆盖面切开，每一项都对应一条能判否的判据：
 *   A. 空闲电平 —— I2C 是**开漏**，读 IDR 拿到的就是线上真实电平；空闲时必须两条都为 1。
 *   B. 两个候选地址各探一次 —— 0x68(AD0=GND) / 0x69(AD0=VCC)。
 *   C. 全总线扫描 —— 确认总线上到底有没有**任何**器件在应答。
 * 结论由 A/B/C 的组合唯一推出。这是把"排错"从猜测变成判据的那一步。 */
static void App_I2cDiag(void)
{
  uint8_t          sda0, scl0, sda1, scl1;
  uint8_t          ack68, ack69, hits = 0U;
  uint8_t          id = 0U;
  uint8_t          a;
  uint32_t         err68;
  mpu6050_status_t wst;

  printf("\r\n[DIAG] ===== I2C1 总线体检 =====\r\n");

  /* ---- A. 软复位**前**的空闲电平（先取样，再复位）---- */
  sda0 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
  scl0 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;

  /* 上一轮失败的传输可能把外设留在"未发 STOP 的 BUSY 态"。
   * 🔴 不软复位的话，后面的探测会一律返回 HAL_BUSY ——
   *    读出来的"无应答"是**假阳性**，会把排查引向完全错误的方向。
   *    （这正是本项目踩过的坑：把"没数据"错判成接线问题。） */
  MX_I2C1_Init();

  sda1 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
  scl1 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;

  printf("[DIAG] A) 空闲电平 SDA(PB7)=%u SCL(PB6)=%u （空闲必须 1/1）\r\n",
         (unsigned)sda1, (unsigned)scl1);
  if ((sda0 != sda1) || (scl0 != scl1))
  {
    printf("[DIAG]    软复位前是 %u/%u → 复位后 %u/%u：是外设没释放总线，不是硬件短接\r\n",
           (unsigned)sda0, (unsigned)scl0, (unsigned)sda1, (unsigned)scl1);
  }

  /* ---- B. 两个候选地址各探一次（HAL 要 8 位地址 = 7bit << 1）---- */
  ack68 = (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR_HAL, 3U, 20U) == HAL_OK) ? 1U : 0U;
  err68 = hi2c1.ErrorCode;
  ack69 = (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(MPU6050_ADDR_HAL + 2U), 3U, 20U) == HAL_OK) ? 1U : 0U;
  printf("[DIAG] B) 探测 0x68(AD0=GND) ACK=%u | 0x69(AD0=VCC) ACK=%u | 0x68 ErrCode=0x%04lX\r\n",
         (unsigned)ack68, (unsigned)ack69, (unsigned long)err68);

  wst = MPU6050_WhoAmI(&hi2c1, &id);
  printf("[DIAG]    WHO_AM_I 状态=%u 回读=0x%02X（期望 0x68）\r\n",
         (unsigned)wst, (unsigned)id);

  /* ---- C. 全总线扫描：总线上到底有没有器件在应答 ---- */
  for (a = 0x08U; a < 0x78U; a++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)((uint16_t)a << 1), 1U, 5U) == HAL_OK)
    {
      printf("[DIAG] C) 总线应答 @7bit=0x%02X\r\n", (unsigned)a);
      hits++;
    }
  }
  printf("[DIAG] C) 扫描 0x08~0x77：共 %u 个应答\r\n", (unsigned)hits);

  /* ---- D. 判读：把四种成因收敛到一个 ---- */
  if ((sda1 == 0U) || (scl1 == 0U))
  {
    printf("[DIAG] D) >> 空闲时有一条线是低电平 → 该线被**钉在 GND**。\r\n");
    printf("[DIAG]      拔掉 MPU 那束线再复位：电平回到 1/1 → 短接在 MPU 侧；\r\n");
    printf("[DIAG]      仍是 0 → 短接在主板侧（相邻孔搭一起 / 焊锡桥）。\r\n");
  }
  else if (ack68 != 0U)
  {
    printf("[DIAG] D) >> 器件在 0x68 上应答，**接线是通的** → 问题在上拉/速率/时序，别再去动线。\r\n");
  }
  else if (ack69 != 0U)
  {
    printf("[DIAG] D) >> 器件在 **0x69** 上应答 → `AD0` 没接到 GND（悬空或被拉高）。把 AD0 接 GND 即可。\r\n");
  }
  else if (hits != 0U)
  {
    printf("[DIAG] D) >> 0x68/0x69 都不应答，但总线上**有别的器件** → SDA/SCL 是通的，问题在 MPU 本体或其供电。\r\n");
  }
  else
  {
    printf("[DIAG] D) >> 全总线**无任何应答** → SDA/SCL 至少有一条没接通。\r\n");
    printf("[DIAG]      重点核对 MPU 的 SDA→PB7、SCL→PB6（这两根最容易插错孔 / 插偏）。\r\n");
  }
  printf("[DIAG] ===== 体检结束 =====\r\n\r\n");
}

/* =====================================================================
 * 三、I2C 总线恢复（每次探测失败都做）
 * ===================================================================== */

/* 🔴 这一段是**实测踩出来的**，不是预防性设计。
 *
 * 现象：电机一转，MPU6050 掉线（`id=0x00`）；把电机停了、线也重插到供电灯亮起，
 *       固件**仍然**报 `[WAIT]`，直到按一次复位才恢复 —— 两读之间硬件没变，
 *       所以真正被锁死的是**固件侧的 I2C 外设状态**，不是接线。
 *
 * 掉线有两种，恢复动作不同，只做其中一种都会漏：
 *   ① 外设停在 BUSY / 上一笔传输没走完 STOP
 *      → HAL 拿到 BUSY 会**直接返回 HAL_BUSY，根本不去碰总线**。
 *        于是"每秒重试一次"的重连逻辑永远失败 —— 它重试的是同一个坏状态。
 *      → 软复位外设即可。
 *   ② 从机被"钉住"（在半个字节上被打断，SCL 松开后它仍把 SDA 拉低）
 *      → 此时 START 都发不出来，软复位也没用；必须在 SCL 上补最多 9 个时钟
 *        把从机的移位寄存器走完，再补一个 STOP 让它释放。
 *
 * ⚠️ 这是"掉线重连"代码最典型的缺口：**只重试、不恢复**。
 *    现场（尤其是 OTA 之后）没有人能按复位键，一次电磁扰动就永久失联。
 *    本项目的重连必须自愈，否则"预测性维护"这个卖点不成立。 */
static void App_I2cBusRecover(void)
{
  GPIO_InitTypeDef gi = {0};
  uint8_t          i;
  uint8_t          stuck;

  /* ① 外设级复位。
   * 用 DeInit + MX_I2C1_Init 两件套，**不能只调 MX_I2C1_Init**：
   * HAL_I2C_Init 只在 State==RESET 时才调 HAL_I2C_MspInit，而 MspInit
   * 才是把 PB6/PB7 配成 AF4 的地方。DeInit 把 State 置回 RESET 并释放引脚，
   * 下一次 Init 才会重新配 AF —— 少了这步，引脚会停在手动拉线的状态。 */
  (void)HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();

  /* ② 总线释放了吗？空闲时两条线都应为高（靠上拉）。 */
  stuck = ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) ||
           (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)) ? 1U : 0U;
  if (stuck == 0U)
  {
    return;                       /* 情形 ① 已解决，不用补时钟 */
  }

  /* ③ 补时钟：把 SCL/SDA 临时切成开漏输出，手动打脉冲。 */
  gi.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
  gi.Mode  = GPIO_MODE_OUTPUT_OD;
  gi.Pull  = GPIO_PULLUP;          /* 开漏得有上拉才回得到高电平 */
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gi);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);    /* 先释放 SDA */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_Delay(1);

  for (i = 0U; i < 9U; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
    {
      break;                      /* 从机已释放 SDA，不必再补 */
    }
  }

  /* ④ 补一个 STOP：SCL 高电平时把 SDA 由低拉高。 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1);

  /* ⑤ 引脚交还 I2C 外设（同样走 DeInit + Init，理由见 ①）。 */
  (void)HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();
}

/* =====================================================================
 * 四、上电自检 / 掉线重连
 * ===================================================================== */

/* 探测 MPU6050。硬件没插好时每秒重试一次并在串口打印提示，
 * 方便第一次上板时对着终端排查接线（这是最能省时间的几行代码）。 */
static void App_EnsureSensor(void)
{
  static uint8_t  s_first     = 1U;
  static uint8_t  s_diag_done = 0U;   /* 总线体检只打一次，避免每秒刷屏 */
  static uint32_t s_last      = 0U;
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
    s_diag_done = 0U;   /* 掉线后再次失败时，体检要能重跑 */
    printf("[OK] MPU6050 online (WHO_AM_I=0x68), 1kHz sampling starts\r\n");
  }
  else
  {
    uint8_t id = 0U;
    (void)MPU6050_WhoAmI(&hi2c1, &id);   /* 顺便把实际读到的 id 打出来 */
    printf("[WAIT] MPU6050 not found (id=0x%02X). Check wiring: "
           "VCC=3V3 GND SDA=PB7 SCL=PB6 AD0=GND\r\n", (unsigned)id);

    if (s_diag_done == 0U)
    {
      s_diag_done = 1U;
      App_I2cDiag();   /* 首次失败才体检：一次给出定因，不是每秒刷屏 */
    }

    /* 🔴 每次失败都要恢复总线，下一次重试才有机会成功。
     * 只重试不恢复 = 反复去读同一个坏掉的外设 → 永久失联。 */
    App_I2cBusRecover();
  }
}

/* =====================================================================
 * 五、采样分派：每次新拍到，按当前输出模式处理一点
 * ===================================================================== */

/* 把“主循环已消费到第几拍”对齐到当前拍，并把 lost 清零。
 *
 * 🔴 必须在进入主循环**之前**调用一次。
 *    App_EnsureSensor() 首次探测传感器要走 I2C（失败时还要重试），实测阻塞约 20 ms；
 *    这段启动期主循环根本还没开始跑，若不先对齐基线，App_OnTick() 第一次执行就会
 *    把这段启动耗时记成“丢了 21 拍”，看起来像稳态在丢帧。
 *
 * lost 的语义是「**稳态下**主循环没跟上采样节拍」，不含上电初始化期 ——
 * 上板验收时要的是“它能证明稳态零丢帧”，一个恒定的 21 只会增加误判成本。 */
static void App_SyncTickBaseline(void)
{
  g_tick_seen   = g_tick;
  g_sample_lost = 0U;
}

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

#elif (APP_OUTPUT_MODE == 3)
  /* -------------------------------------------------------------------
   * 模式 3：在线推理 1 kHz —— 把连续采样喂给 NanoEdge AI 模型
   * -------------------------------------------------------------------
   * 与模式 2 一样按 1 kHz 原速读传感器，但数据**不往串口发**，
   * 而是交给 neai_app 攒成一个窗口（DATA_INPUT_USER 点）后调用 learn()/detect()。
   *
   * 🔴 只有模式 3 会喂模型 —— 模式 0 是 10 Hz、模式 1 是 200 Hz，
   *    采样节奏都不对，喂进去只会污染模型，所以坚决不喂。
   *
   * 🔴 模型未就绪（init 失败 / 传感器掉线）时**自动跳过**，不阻塞采样节拍；
   *    原因会由 neai_app 打印，不会静默失败。
   *
   * 输出：事件行由 neai_app.c 按 APP_NEAI_PRINT_DECIM 节流打印；
   *      这里每秒再补一行汇总，当"程序还活着"的心跳用。 */
  {
    mpu6050_accel_raw_t raw;
    float               acc3[3];                       /* 传感器实际给的 3 轴 */
    float               xyz[NEAI_APP_NUM_AXES];        /* 模型要求轴数（换库自动跟随） */
    uint16_t            k;

    if (MPU6050_ReadAccel(&raw) != MPU6050_OK)
    {
      g_sensor_ok = 0U;
      return;
    }
    MPU6050_AccelToG(&raw, &acc3[0], &acc3[1], &acc3[2]);

    /* 把传感器的 3 轴映射到模型要求的轴数上。
     * 🔴 不能直接把 &acc3[1] / &acc3[2] 传给模型：占位库 AXIS_NUMBER=1，
     *    那样写会**越界读栈**。这里显式按 NEAI_APP_NUM_AXES 逐元素拷贝，
     *    少于 3 轴就当单轴模型用，多于 3 轴时余下补 0（本项目不会出现）。 */
    for (k = 0U; k < (uint16_t)NEAI_APP_NUM_AXES; k++)
    {
      xyz[k] = (k < 3U) ? acc3[k] : 0.0f;
    }

    /* 返回值这里不用：日志由 neai_app 自己按节流打印。
     * 将来做"异常告警点灯"时，在这里判 NEAI_PUSH_DETECTED + 相似度阈值即可。 */
    (void)NEAI_AppPushSample(xyz);
  }

  /* 每秒一行汇总（心跳 + 健康度） */
  if ((g_tick % 1000U) == 0U)
  {
    NEAI_AppPrintStats();
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

/* =====================================================================
 * 二、PWM 限速通道（第 ⑦ 步）—— PA6 = TIM3_CH1
 * =====================================================================
 * 为什么需要 PWM、20 kHz 怎么来的、硬件三项前置 —— 见 USER CODE BEGIN PD。
 * 为什么手写而不是让 CubeMX 生成 —— 见 USER CODE BEGIN PV。 */

/* 配置并启动 TIM3_CH1 的 PWM 输出。
 * 🔴 下面三件事在"CubeMX 生成"模式下由 MspInit 代劳，手写时**必须自己补**：
 *    ① **开 TIM3 时钟** —— HAL_TIM_PWM_Init() 自己不碰 RCC，而生成的
 *       stm32f4xx_hal_msp.c 里 HAL_TIM_Base_MspInit() 只认 TIM2（TIM3 不在 .ioc 里）。
 *       不开时钟的话所有寄存器写入都被丢弃、PA6 恒为低 —— 现象是"代码跑了但电机不转"，
 *       特别容易被误判成"MOS 坏了"。**这是本函数里最容易漏的一步。**
 *    ② **把 PA6 配成 AF2** —— 复位后 PA6 不是复用态，HAL 不会替你配。
 *    ③ 用宏 __HAL_RCC_TIM3_CLK_ENABLE()（零开销）而不是 HAL_RCC_...()。 */
static void App_PwmInit(void)
{
  GPIO_InitTypeDef   gpio = {0};
  TIM_OC_InitTypeDef oc   = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();          /* MX_GPIO_Init 已开过，重复调用无害 */

  /* PA6 → AF2（F407 上 AF2 = TIM3/TIM4/TIM5 的复用组）
   * ⚠️ PA6 的其它可能复用：AF0=SPI1、AF1=TIM13。这里要的是 **AF2 = TIM3_CH1**。
   * Speed 选 LOW 是**故意的**：20 kHz 根本用不着翻转速率，边沿越缓，
   * 向旁边那根 MPU6050 排线辐射的开关噪声越小 —— R6.4 记录的 I2C 掉线
   * 就发生在电机开关那一瞬（感性浪涌），降速之后这类耦合只会更容易发作。 */
  gpio.Pin       = GPIO_PIN_6;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &gpio);

  htim3_pwm.Instance               = TIM3;
  htim3_pwm.Init.Prescaler         = APP_PWM_PSC;
  htim3_pwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3_pwm.Init.Period            = APP_PWM_ARR;
  htim3_pwm.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3_pwm) != HAL_OK)
  {
    Error_Handler();
  }

  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 0U;                    /* 先给 0%，实际值由下面的 App_SetDuty 定 */
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3_pwm, &oc, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3_pwm, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  App_SetDuty((uint8_t)APP_PWM_DUTY_DEFAULT);
}

/* 设置占空比（0..100，单位 %）；超过 100 一律按 100 处理。
 * 换算 CCR = (ARR+1) × pct / 100 —— 用 ARR+1（而不是 ARR）是为了让 pct=100 时
 * CCR = 4200 > ARR = 4199：PWM 模式 1 下计数器永远追不上比较值 ⇒ 输出恒高，
 * 也就是**真正的 100%**（用 ARR 的话会留一个计数周期的低电平缺口）。 */
static void App_SetDuty(uint8_t pct)
{
  uint32_t ccr;

  if (pct > 100U)
  {
    pct = 100U;
  }
  g_pwm_duty = pct;
  ccr = (((uint32_t)APP_PWM_ARR + 1U) * (uint32_t)pct) / 100U;
  __HAL_TIM_SET_COMPARE(&htim3_pwm, TIM_CHANNEL_1, ccr);
}

/* 打开 USART1 的"收"：单字节中断，每收到 1 字节回调一次。
 * 🔴 这里是**唯一**手工打开 NVIC 的地方 —— `.ioc` 里没有配置 USART1 中断，
 *    所以 CubeMX 不会生成 HAL_NVIC_EnableIRQ(USART1_IRQn)。
 *    抢占优先级取 2（TIM2 = 0、USART3 = 1），保证 1 kHz 采样节拍永远最优先。 */
static void App_CmdRxStart(void)
{
  HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1U);
}

/* UART 收满一个字节的回调（**中断上下文** —— 别在这里 printf、别在这里改寄存器）。
 * 解析规则故意做得又笨又松，用串口工具手打也不会崩：
 *   'd' / 'D'                → 这是一条"设占空比"命令
 *   数字                      → 累积成数值（最多 3 位，防溢出）
 *   CR / LF / 空格 / , / ;    → 一条命令结束，置 s_cmd_ready 交主循环
 *   '?' / 'h' / 'H'          → 这是一条"查状态"命令
 *   其它                      → 忽略（顺手吃掉串口工具的 ANSI 转义残渣）
 * 🔴 处理完必须**重新装机**，否则下一次就再也不进中断了（经典坑）。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t b;

  if (huart->Instance != USART1)
  {
    return;                             /* 别的串口的回调，本项目不管 */
  }
  b = s_rx_byte;

  if ((b >= (uint8_t)'0') && (b <= (uint8_t)'9'))
  {
    if (s_cmd_kind == APP_CMD_NONE)
    {
      s_cmd_kind = APP_CMD_DUTY;        /* 光敲数字也当"设占空比" */
    }
    if (s_cmd_digit < 3U)
    {
      s_cmd_val   = (uint8_t)((s_cmd_val * 10U) + (uint8_t)(b - (uint8_t)'0'));
      s_cmd_digit = (uint8_t)(s_cmd_digit + 1U);
    }
  }
  else if ((b == (uint8_t)'d') || (b == (uint8_t)'D'))
  {
    s_cmd_kind = APP_CMD_DUTY;
  }
  else if ((b == (uint8_t)'?') || (b == (uint8_t)'h') || (b == (uint8_t)'H'))
  {
    s_cmd_kind = APP_CMD_INFO;
  }
  else if ((b == (uint8_t)'\r') || (b == (uint8_t)'\n') || (b == (uint8_t)' ') ||
           (b == (uint8_t)',')  || (b == (uint8_t)';'))
  {
    if (s_cmd_kind != APP_CMD_NONE)
    {
      s_cmd_ready = 1U;                 /* 交给主循环执行 */
    }
  }
  else
  {
    /* 忽略 */
  }

  (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1U);
}

/* 在主循环里执行"待处理命令"（没有命令时立刻返回，不占节拍）。
 * 🔴 **模式 2 下不打印任何字符** —— 那会把 0xAA 0x55 的二进制流搅乱。
 *    想知道命令有没有生效：看电机声音变了没有，或看 PC 端 1× 频率动没动。 */
static void App_PollCmd(void)
{
  uint8_t kind, val, digit;

  if (s_cmd_ready == 0U)
  {
    return;
  }

  /* 先把状态取走并清干净，再执行 —— 免得执行期间来了新命令被吞掉 */
  kind  = s_cmd_kind;
  val   = s_cmd_val;
  digit = s_cmd_digit;

  s_cmd_kind  = APP_CMD_NONE;
  s_cmd_val   = 0U;
  s_cmd_digit = 0U;
  s_cmd_ready = 0U;

  if ((kind == APP_CMD_DUTY) && (digit > 0U))
  {
    App_SetDuty(val);
#if (APP_OUTPUT_MODE != 2)
    printf("[PWM] duty = %u%%\r\n", (unsigned)g_pwm_duty);
#endif
  }
  else if (kind == APP_CMD_INFO)
  {
#if (APP_OUTPUT_MODE != 2)
    printf("[PWM] duty = %u%%  (PA6/TIM3_CH1 @20 kHz; cmd: dNN, NN = 0..100)\r\n",
           (unsigned)g_pwm_duty);
#endif
  }
  else
  {
    /* 空命令（只敲了个回车）或没带数字的 'd'：什么也不做 */
  }
}

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ---- 第 ⑦ 步：PWM 限速通道（PA6 = TIM3_CH1，上电占空比 0 = 电机不转）--
   * 🔴 必须在 USB/电池盒接好、MOS 接线确认之后才把占空比调上去；
   *    上电默认 0 是安全默认值，见 USER CODE BEGIN PD。 */
  App_PwmInit();

  /* ---- 打开 USART1 收中断：PC 可以发 "dNN" 实时改转速 ---------------
   * 用的是已经接好的 CH340（PA10），**不需要额外加线**。 */
  App_CmdRxStart();

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
#elif (APP_OUTPUT_MODE == 3)
  printf("mode 3: on-device inference @1 kHz (NanoEdge AI, similarity + us)\r\n");
#endif

  /* 第 ⑦ 步的 PWM 状态 —— 一行说清"在哪儿、多少 Hz、当前占空比"。
   * 频率这里**算出来**而不是写死，免得改了 PSC/ARR 之后横幅说假话。
   * 🔴 启动横幅全部堆在 1024 B 的发送队列里、此时主循环还没开始倒，
   *    所以这一行也要克制（超了会丢字节，看 ovf 计数）。 */
  printf("PWM: PA6/TIM3_CH1 @%lu Hz, duty=%u%% (USART1 cmd: dNN + Enter)\r\n",
         (unsigned long)(84000000UL /
                         (((uint32_t)APP_PWM_PSC + 1UL) * ((uint32_t)APP_PWM_ARR + 1UL))),
         (unsigned)g_pwm_duty);

  /* ---- 初始化 NanoEdge AI -------------------------------------------
   * 🔴 无论当前是哪个输出模式都初始化一次，有两个好处：
   *    ① 一上电就在串口打出 NEAI_ID —— 这是"当前链的是哪个库"的铁证
   *       （占位库的 ID 是 62e147...，换成自己的模型后必须变）；
   *    ② 模式 0/1/2 下没人喂数据，所以它**完全是惰性的**，不影响采样与点灯验证。
   * 若只想验证"没有 AI 的纯采集链路"，可用 -DAPP_NEAI_ENABLE=0 重编一版做对照。 */
  NEAI_AppInit();

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

  /* ---- 对齐采样基线（详见函数注释）---------------------------------
   * 上面那次探测会阻塞约 20 ms，必须在进主循环前把“已消费拍数”重新对齐，
   * 否则这 20 ms 会被记进 lost，让上板验收看到恒定的 lost=21。 */
  App_SyncTickBaseline();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 主循环四件事，按“越频繁越靠前”排：
     *   1) 倒串口 —— 每次只吐 4 字节，保证 1 ms 采样节拍不被拖垮；
     *   2) 处理采样 —— 有新拍才动，没新拍立刻返回；
     *   3) 传感器掉线重连 —— 没事时也立刻返回；
     *   4) 执行串口命令（第 ⑦ 步：改 PWM 占空比）—— 没命令时立刻返回。
     * 设计上这里**没有 HAL_Delay / 阻塞等待**，主循环全速空转，
     * 因此采样抖动只取决于 I2C 与串口的实际耗时之和。 */
    App_PumpUart();
    App_OnTick();
    App_EnsureSensor();
    App_PollCmd();

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
