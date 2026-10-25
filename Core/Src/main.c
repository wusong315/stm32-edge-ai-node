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
#include "mqtt_pkt.h"
#include "esp_at.h"
#include <stdio.h>
#include <string.h>
#include <math.h>          /* sqrtf —— 逐窗交流 RMS（判停机用） */
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
/* 🔴 **默认档已改为 3（产品档 = 1 kHz 在线推理）** —— 2026-10-22 配合云端启用。
 *    要"看原始数据"的诊断档（0 / 2），**两个开关一起动**：
 *      ① `Core/Inc/secrets.h` 里 `APP_CLOUD_ENABLE` 改 **0**（云端不参与编译）
 *      ② 再加 `-DAPP_OUTPUT_MODE=0`（或 2）覆盖这里的默认值
 *    只改 ② 不改 ① ⇒ 撞下面的 `#error`（云端只在模式 3 下有相似度可报）。 */
#ifndef APP_OUTPUT_MODE
#define APP_OUTPUT_MODE     (3)
#endif

#define SAMPLES_PER_FRAME   (32U)     /* 二进制模式每帧点数 */
#define TEXT_DECIM          (5U)      /* 1000 / 5   = 200 Hz */
#define DEBUG_DECIM         (100U)    /* 1000 / 100 =  10 Hz */

/* =====================================================================
 * 第 ⑧ 步：云端上报（MQTT over ESP8266 透传 —— 决策 D33 / D34）
 * =====================================================================
 * 开关与参数**全部**可以被 `Core/Inc/secrets.h` 覆盖（那个文件不入库，
 * 放 Wi-Fi 密码与设备密钥）。没有它时用下面的安全默认值 ——
 * `APP_CLOUD_ENABLE = 0` ⇒ 整个云端段落不参与编译，行为与第 ⑦ 步一字不差。
 *
 * 🔴 为什么**不**做成 `APP_OUTPUT_MODE == 4`
 * ---------------------------------------------------------------------
 * 云端是"采样/推理之上的一层"，和"串口往外输出什么"没有关系。做成独立开关后：
 *   · 最终产品 = `APP_OUTPUT_MODE=3` + `APP_CLOUD_ENABLE=1`
 *     —— 串口照样能看推理日志，云端同时上报；
 *   · 不必把模式 3 的采样代码复制一份（复制出来的两份代码迟早会走样）；
 *   · 想单独验证云端时也不用改模式。
 * 唯一约束：相似度只有模式 3 产生，所以两者必须配套（见下面的 #error）。
 */

/* ---- 本地私有参数（不入库）。没有这个文件也不影响编译 ------ */
#if defined(__has_include)
#  if __has_include("secrets.h")
#    include "secrets.h"
#  endif
#endif

#ifndef APP_CLOUD_ENABLE
#define APP_CLOUD_ENABLE      (0)      /* 1 = 启用云端上报（需要 secrets.h 或 -D） */
#endif
#ifndef CLOUD_WIFI_SSID
#define CLOUD_WIFI_SSID       ""       /* 空串 ⇒ 启动时跳过连接，模式 3 照常跑 */
#endif
#ifndef CLOUD_WIFI_PASS
#define CLOUD_WIFI_PASS       ""
#endif
#ifndef CLOUD_DEVICE_ID
#define CLOUD_DEVICE_ID       ""       /* 形如 6ad76a1d..._stm32-01（产品ID_NodeId） */
#endif
#ifndef CLOUD_SECRET
#define CLOUD_SECRET          ""       /* 注册设备时自填的 8~32 位密钥 */
#endif
#ifndef CLOUD_HOST
#define CLOUD_HOST            "f5d6c11092.st1.iotda-device.cn-north-4.myhuaweicloud.com"
#endif
#ifndef CLOUD_PORT
#define CLOUD_PORT            (1883)   /* 明文；8883 是 MQTTS，本项目不用 */
#endif
#ifndef AUTH_TS
#define AUTH_TS               "2026102205"   /* 鉴权 ts（10 位）。sig_type=0 时平台不校验内容 */
#endif

/* ---- 以下不是"可选项"，而是**实测出来的口径**，改之前先看注释 ---------- */
#define CLOUD_SERVICE_ID      "vibration"  /* 🔴 与物模型 service_id 逐字一致 */
#define CLOUD_KEEPALIVE_S     (120U)       /* 🔴 合法区间 30..1200；填 10 会 rc=0x02（实测） */
#define CLOUD_PERIOD_MS       (30000U)     /* 正常心跳 30 s → 2 880 条/天（配额 28.8 %） */
#define CLOUD_ALARM_MS        (60000U)     /* 异常持续中 60 s 一条 → 1 440 条/天 */
#define CLOUD_PING_MS         (60000U)     /* 空闲兜底 PINGREQ（keepalive 1.5 倍 = 180 s 才被踢） */
#define CLOUD_SIM_ALARM       (90U)        /* 相似度 < 90 判异常（与 R10.9 的 T14 口径一致） */
#define CLOUD_SIM_CLEAR       (95U)        /* > 95 才解除异常 —— 迟滞 5 分，防抖 */
#define CLOUD_MIN_GAP_MS      (5000U)      /* 任意两条上报之间至少 5 s（翻转也不例外） */

/* ---- 重连（2026-10-25 加）--------------------------------------------
 * 🔴 为什么以前不重连：`CWJAP` + `CIPSTART` + `CIPMODE` 全走 `wait_for()` 死等，
 *    最长 25 s。放在主循环里会把 1 kHz 采样整块吃掉（恒定几百点 lost）。
 *    代价是**链路一掉就永久离线** —— 2026-10-25 实测：平台显示"离线"、
 *    影子停在 `similarity=55`，而板子还在照常发（`puback` 冻结不动）。
 *
 * 现在的做法 = **非阻塞状态机**：每拍只推进一步，"阻塞"从设计里消失了。
 * 所以"建链必须在 TIM2 启动之前"那条老约束也就不再需要了。 */
/* ⚠️ 这一条故意用 `#ifndef` 包住：它是**唯一**一个"改了就能伪造掉线"的开关。
 *    自测重连时用 `--variant CLOUD_DOWN_MS=3000` 压到 3 s，就能在
 *    完全不动网线的前提下，把「判掉线 → 重连 → 重新上线」整条链跑一遍。 */
#ifndef CLOUD_DOWN_MS
#define CLOUD_DOWN_MS         (100000U)    /* 下行静默这么久 ⇒ 判掉线（正常最慢 60 s 一个 PINGRESP） */
#endif
#define CLOUD_MIN_ONLINE_MS   (20000U)     /* 刚上线这段时间不判掉线（防抖） */
#define CLOUD_BACKOFF_MIN_MS  (2000U)      /* 第一次失败等 2 s，之后翻倍 */
#define CLOUD_BACKOFF_MAX_MS  (60000U)
#define CLOUD_ESC_SILENCE_MS  (1100U)      /* `+++` 前后要求的静默 */
#define CLOUD_ESC_TRIES       (2U)         /* AT 无应答后最多再逃逸几次 */
#define CLOUD_BOOT_SETTLE_MS  (1500U)      /* 开机先让模块把手上的事做完，再去敲它 */
#define CLOUD_CONNACK_MS      (5000U)      /* 发完 CONNECT 等 CONNACK */

/* ---- 停机判定 + 判定去抖（2026-10-25 加）----------------------------
 * 🔴 停机为什么要单独判：训练集只有 `normal_d100`（电机在跑），**停机段属 OOD**
 *    （`T14` 实测 14.9 % 的停机窗口被判异常）。不挡住的话，机器一停云端就刷
 *    "异常"，看起来像误报，其实是模型根本没学过静止工况。
 *
 * 门槛 0.013 g 的依据（离线用 5 段已有数据算的「逐 128 窗三轴**交流** RMS」，
 * 脚本 `_tmp/still_gate.py`）：
 *     停机（正对照）   max 0.00913   p50 0.00674
 *     normal_d100      min 0.01936   p50 0.02163
 *     新电池两轮        min 0.05300 / 0.06139
 *     修好供电          min 0.03212
 *  ⇒ 0.013 落在「停机最高」与「额定最低」之间，**上下各留约 1.45 倍余量**。
 * ⚠️ 必须用**交流**（去直流）：重力在 az 上有约 1 g 的直流分量，
 *    直接用 RMS 的话停机和额定都会顶到 ~1 g，两边完全重合（这是最容易踩的坑）。 */
#ifndef CLOUD_STILL_RMS
#define CLOUD_STILL_RMS       (0.013f)
#endif

/* 判定去抖：连续 N 个窗口都想翻，才真的翻。
 *
 * N 是**实测选出来的**，不是拍的。`d100` 跑 120 s / 839 窗，逐窗统计「连续 <90」的串长：
 *     串长 1 窗 18 次 / 2 窗 5 次 / 3 窗 2 次
 * 用同一段数据离线模拟不同 N 造成的判定翻转次数：
 *     N=2 → 14 次    N=3 → 4 次    **N=4 → 0 次**
 * ⇒ 取 4 才能把「正常工况下的误报」清零。代价是判定延迟 +0.51 s
 *   （4 窗 × 128 ms），对"不平衡"这类**持续**故障毫无影响；
 *   ⚠️ 但**短暂冲击**更难检出（`T14` 的外部冲击 13.6 % 是单次瞬态，
 *      改 N 之后这个数字需要在 R10.10 重新测，别直接沿用）。 */
#ifndef CLOUD_DEBOUNCE_WIN
#define CLOUD_DEBOUNCE_WIN    (4U)
#endif


#if (APP_CLOUD_ENABLE) && (APP_OUTPUT_MODE != 3)
#error "APP_CLOUD_ENABLE=1 needs APP_OUTPUT_MODE=3. Fix one of them: (a) build with -DAPP_OUTPUT_MODE=3, or (b) set APP_CLOUD_ENABLE 0 in Core/Inc/secrets.h"
#endif

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

/* 🔴 2026-10-18：I2C 读失败的三级处理（EMI，RISKS R23）。
 * 背景：电机一转，电刷换向的 EMI 会让 MPU6050 偶发无应答。
 * 实测（新电池、d100）：30.7 s 窗口里 g_sample_lost +8 870 拍（29 %）、
 * g_frame_dropped +57 次，帧到达时间上出现 6 次 780–820 ms 的停采。
 *
 * 原逻辑「一次 5 ms 超时 = 判掉线」的代价太大：作废半帧 + 进退避重连。
 * 现在的三级阶梯（每一次都只针对"该层真正能判断的事"）：
 *   ① 连续 1–2 次失败 → 只丢这一拍（帧继续攒）；
 *   ② 连续 3–39 次     → 仍认为传感器在线，**就地自愈**：每 8 次插一次总线恢复；
 *   ③ 连续 40 次       → 才认账"掉线"（作废半帧 + 交给 App_EnsureSensor）。
 *
 * 🔴 ②这一级是 2026-10-18 加的关键修复。只做①（阈值 3 就判掉线）时，
 *    60 s 里仍有 5 次 862–891 ms 的停顿（累计缺 4.21 s）。算账发现
 *    最大的一块根本不是采样超时，而是**重连路径自身**：
 *    MPU6050_Init 的探测窗口 20 ×(10 ms 延时 + 5 ms 超时) ≈ 200–300 ms，
 *    一次失败探测 = 主循环盲等 200 ms 以上；而重连又要连续探测 2–3 次
 *    （2×200 + 退避 10+20+40）≈ 850 ms —— 与实测的 862/891 ms 严丝合缝。
 *    所以：① 改成用 MPU6050_Reinit（单次探测，≈5 ms）快速失败；
 *          ② 短期抖动根本不进退避重连，就地恢复。
 *
 * ⚠️ 取舍（诚实记录）：跳拍会让一帧的 32 个点**跨过若干 1 ms 的空洞**，
 *    原则上违反"不许跨空洞拼接"；但 1 ms 级的时基误差对 128 点窗口的频谱
 *    影响可忽略，而 800 ms 级的整段缺失是致命的 —— 两害相权取其轻。
 *    空洞仍在帧间隔信息里可见（PC 端 frame_timing 能看到 ≥3× 中位数的间隔）。 */
#define APP_I2C_FAIL_TOLERANCE   (3U)    /* ① 之内：只丢拍 */
#define APP_I2C_RECOVER_EVERY    (8U)    /* ② 之中：每 8 次连续失败恢复一次总线 */
#define APP_I2C_OFFLINE_AT       (40U)   /* ③ 阈值：连续 40 次才判掉线 */
static volatile uint8_t   s_i2c_fail  = 0U;   /* 连续读失败次数（成功即清零） */

/* 掉线/恢复的量化计数器。
 * 🔴 非 static：mode 2 下串口不许打文字，这些数是靠 ST-Link hotplug 读 RAM
 *    看**增量**的（`-r32 <addr> <字节数>`），所以符号要在 `.elf` 里 nm 得到。
 *    判"有没有丢"永远要找物理量，别信"校验错 0 / 序号断档 0"（已证伪两次）。 */
uint32_t g_i2c_fail_total   = 0U;   /* 单拍读失败累计（含被 ①② 消化掉的） */
uint32_t g_bus_recover      = 0U;   /* App_I2cBusRecover() 执行次数 */
uint32_t g_bus_stuck        = 0U;   /* 其中真的发现 SCL/SDA 被拉死的次数 */
uint32_t g_recon_attempts   = 0U;   /* 重连探测次数（每次 = 一次 Init/Reinit） */
uint32_t g_offline_events   = 0U;   /* 判掉线的次数（③ 触发） */

/* 「最长停顿解剖」——记录**最坏那一次**间隔里到底花在哪儿。
 * 为什么要它：2026-10-18 实测 60 s 有 8 次 ~200 ms 停顿、108 次失败，
 * 但按"每次失败 = 一个 I2C 超时（2 ms）"只该丢 0.2 s，实际丢了 1.41 s。
 * 差 7 倍 ⇒ 说明**大头不在"失败这一次调用"**，必须把停顿拆开看，
 * 否则就会靠猜去调参数（改超时 5→2 ms 已验证无效：每失败成本 13.7→13.0 ms 不变）。
 * 读法：`-r32 0x20000964 0x18`（地址见 `_tmp\bench60.py`）。 */
uint32_t g_gap_max_ms     = 0U;   /* 最坏停顿长度（拍 = ms） */
uint32_t g_gap_max_fails  = 0U;   /* 该停顿内的读失败次数 */
uint32_t g_gap_max_recover= 0U;   /* 该停顿内的总线恢复次数 */
uint32_t g_gap_cur_fails  = 0U;   /* 下面两个是"当前这一轮"正在累计的同类量 */
uint32_t g_gap_cur_recover= 0U;

#if (APP_CLOUD_ENABLE)
/* =====================================================================
 * 第 ⑧ 步：云端上报的运行态
 * =====================================================================
 * 全部放 bss 而不是栈：报文缓冲 386 B，主循环里不想动栈；
 * 而且这些变量要能用 IDE 的 Live Watch 直接看（调试时比翻日志快）。
 *
 * 🔴 为什么只留**一个** `s_pub`：CONNECT 与 PUBLISH 从不同时用
 *    （CONNECT 在启动阶段同步发完，之后才可能发 PUBLISH）。
 *    省 386 B RAM，代价是一句注释 —— 值。 */
static char       s_topic[96];       /* $oc/devices/<id>/sys/properties/report */
static mqtt_pkt_t s_pub;             /* 复用的报文缓冲（386 B） */
static uint16_t   s_pub_id = 1U;     /* QoS1 的 packet id，只用来和 PUBACK 对号 */

/* 云端连接状态机。**每个状态在 App_CloudStep() 里最多干一件小事**，
 * 绝不阻塞 —— 这是"重连也不能打断 1 kHz 采样"的实现方式。 */
typedef enum
{
  CS_WAIT = 0,      /* 退避等待（失败后） */
  CS_ESC_SIL,       /* 等线路真正静默，凑 `+++` 的前半个条件 */
  CS_ESC_SETTLE,    /* 已发 `+++`，等后半个静默 */
  CS_PING,          /* 发 AT */
  CS_ECHO,          /* 发 ATE0 */
  CS_JOIN,          /* 发 AT+CWJAP（最长 25 s，但**非阻塞**） */
  CS_TCP,           /* 发 AT+CIPSTART */
  CS_MODE,          /* 发 AT+CIPMODE=1 */
  CS_SEND,          /* 发 AT+CIPSEND，等 '>' */
  CS_CONNACK,       /* 已发 MQTT CONNECT，等 4 字节 CONNACK */
  CS_ONLINE         /* 管道就绪，正常上报 */
} cloud_state_t;

static struct
{
  uint8_t  connected;       /* 1 = CONNACK rc=0，管道可用 */
  uint8_t  have_verdict;    /* 1 = 已经报过至少一条（第一条不受节拍限制） */
  uint8_t  alarm;           /* 上一次**已上报**的判定：0 正常 / 1 异常 */
  uint32_t reports;         /* 成功发出去的条数 */
  uint32_t pubacks;         /* 收到平台 PUBACK 的次数（= 平台真收到了） */
  uint32_t errors;          /* 拼包/发送失败次数 */
  uint32_t last_report_ms;  /* 上一条上报的时刻 */
  uint32_t last_tx_ms;      /* 上一次"有字节出去"的时刻（keepalive 计时用） */
  /* --- 状态机（2026-10-25 加） --- */
  uint8_t  enabled;         /* 1 = secrets.h 配齐了，状态机才允许跑 */
  uint8_t  state;           /* cloud_state_t */
  uint8_t  link_lost;       /* 1 = 收到看不懂的字节（多半是模块吐的 CLOSED） */
  uint8_t  esc_tries;       /* 本轮 AT 失败后已逃逸几次 */
  uint32_t t_state;         /* 进入当前状态的时刻 */
  uint32_t t_esc;           /* 发出 `+++` 的时刻 */
  uint32_t t_online;        /* 进入 CS_ONLINE 的时刻 */
  uint32_t backoff;         /* 下一次重试前的等待 */
  uint32_t reconnects;      /* 重连次数 */
  uint32_t fails;           /* 建链失败次数 */
  uint32_t last_detect;     /* 上一次看到的 detect_calls（用来识别"新窗口"） */
  uint8_t  pend_cnt;        /* 去抖：连续几个窗口想翻转 */
} s_cloud;

/* 逐窗交流 RMS 的累加器（判"停机"用，见 CLOUD_STILL_RMS 的说明）。
 * 每拍 3 乘 6 加，1 kHz 下开销可忽略；窗口结束时开一次方（≈7.8 次/秒）。 */
static float    g_ac_sum[3];
static float    g_ac_sumsq[3];
static uint16_t g_ac_n   = 0U;
static float    g_ac_rms = 0.0f;      /* 最近一个窗口的三轴交流 RMS（单位 g） */
static uint32_t g_still_win = 0U;     /* 被判成"停机"的窗口数（调试用） */

#endif

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
#if (APP_CLOUD_ENABLE)
static void     App_CloudStart(void);
static void     App_CloudStep(uint32_t now);
static void     App_CloudGoWait(uint32_t now, uint8_t is_failure);
static void     App_CloudPoll(void);
static void     App_TxqDrain(uint32_t timeout_ms);
static void     App_CloudReport(uint8_t sim, uint8_t anomaly);
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
/* 约微秒级的忙等（不追求精确，够用就行）。
 * 为什么不能用 HAL_Delay：补时钟只需要 >1 µs 的电平建立时间，而 HAL_Delay(1)
 * 最少 1 ms —— 9 个时钟 × 2 次 + 收尾 STOP 3 次 = 21 ms，是 21 个采样点。
 * 实测（`app_ver=7`，60 s，`d100`）：6 次停顿**全部**走了补时钟分支，
 * 光这 21 次 HAL_Delay 就占了约 140 ms，是残余停顿里仅次于 HAL 的 25 ms 盲等的第二块。
 * 168 MHz 下 `us * 80` 次空循环 ≈ 1.9 × us 微秒（偏慢，正好留余量）。 */
static void App_ShortDelayUs(uint32_t us)
{
  volatile uint32_t n = us * 80U;
  while (n-- != 0U)
  {
    __NOP();
  }
}

static void App_I2cBusRecover(void)
{
  GPIO_InitTypeDef gi = {0};
  uint8_t          i;
  uint8_t          stuck;

  g_bus_recover++;

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

  g_bus_stuck++;

  /* ③ 补时钟：把 SCL/SDA 临时切成开漏输出，手动打脉冲。 */
  gi.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
  gi.Mode  = GPIO_MODE_OUTPUT_OD;
  gi.Pull  = GPIO_PULLUP;          /* 开漏得有上拉才回得到高电平 */
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gi);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);    /* 先释放 SDA */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  App_ShortDelayUs(5U);

  for (i = 0U; i < 9U; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    App_ShortDelayUs(5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    App_ShortDelayUs(5U);
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
    {
      break;                      /* 从机已释放 SDA，不必再补 */
    }
  }

  /* ④ 补一个 STOP：SCL 高电平时把 SDA 由低拉高。 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  App_ShortDelayUs(5U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  App_ShortDelayUs(5U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  App_ShortDelayUs(5U);

  /* ⑤ 引脚交还 I2C 外设（同样走 DeInit + Init，理由见 ①）。 */
  (void)HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();
}

/* =====================================================================
 * 四、上电自检 / 掉线重连
 * ===================================================================== */

/* 现在这个时刻**能不能往串口打文字**？
 *
 * 🔴 模式 2 一旦开始出帧就绝对不能打。被实测抓到的两个后果：
 *   ① `[DIAG]` 要扫 0x08~0x77 共 112 个地址（每个 5 ms 超时）——
 *      电机一转、I2C 偶发一次无应答，它就阻塞主循环约 **1 秒**，
 *      1 kHz 采样直接丢 **一千拍**；而丢拍在二进制流里**看不出来**
 *      （固件丢帧不推进 seq，PC 端永远是"序号断档 0 次"）。
 *   ② 这些文字还会被插进帧流中间，PC 端只能靠重同步丢掉 —— 实测一次
 *      60 秒录制因此丢了 3 872 个采样点（3.87 秒的假空洞）。
 *
 * 上电阶段（还没成功发出过一帧）仍然放开：那时正是最需要看接线提示的时候。 */
static uint8_t App_TextAllowed(void)
{
#if (APP_OUTPUT_MODE == 2)
  return (g_frame_seq == 0U) ? 1U : 0U;
#else
  return 1U;
#endif
}

/* 探测 MPU6050。硬件没插好时在串口打印提示，方便第一次上板时对着终端排查接线
 * （这是最能省时间的几行代码）。
 *
 * 🔴 重连节奏用**指数退避**：10 → 20 → 40 → 80 → 160 → **200（封顶）** 拍。
 *    原来写死"每 1000 拍（=1 秒）重试一次"，实测后果很严重：
 *    电机一转，I2C 偶发一次无应答 → 固件要**干等满 1 秒**才重试
 *    ⇒ 一次只要几十毫秒就能恢复的瞬时故障，直接丢掉 ~1000 个采样点（31 帧）。
 *    60 秒录制里发生 4 次 = 3 872 点凭空消失，且数据流毫无痕迹。
 *    退避让"抖一下就回来"，同时仍不会对真正坏死的总线按 1 kHz 高频硬刚
 *    （那样每次 5 ms 超时会把采样彻底拖死）。
 *
 * 🔴 封顶**必须是 200 拍、不能是 1000 拍**（2026-10-17 正对照实测后下调）。
 *    那次"拔掉 SDA 1 秒再插回"，从拔到恢复隔了约 4 秒 —— 因为退避已经爬到
 *    1000 拍那一档，插回去了还要干等最多 1 秒才轮到下一次试探。
 *    封顶 200 拍后：真正坏死的总线上每秒最多试 5 次（每次 ~20–40 ms 的
 *    BusRecover），对"反正没有数据可采"的停机期无害；而**恢复延迟 ≤200 ms**。
 *    代价/收益很清楚：停线期间多花 10~20 % 的循环时间，换回 800 ms 的数据。 */
static void App_EnsureSensor(void)
{
  static uint8_t  s_first     = 1U;
  static uint8_t  s_diag_done = 0U;   /* 总线体检只打一次，避免每秒刷屏 */
  static uint32_t s_last      = 0U;
  static uint32_t s_backoff   = 0U;   /* 下一次重试要等几拍（0 = 首次） */
  uint32_t wait;
  uint8_t  cold;                      /* 1 = 本次是上电后的第一次探测 */
  mpu6050_status_t st;

  if (g_sensor_ok != 0U)
  {
    return;
  }
  /* 首次立即探测；之后按退避间隔重试 */
  wait = (s_backoff == 0U) ? 10U : s_backoff;
  if ((s_first == 0U) && ((uint32_t)(g_tick - s_last) < wait))
  {
    return;
  }
  cold    = s_first;
  s_first = 0U;
  s_last  = g_tick;
  s_backoff = (wait >= 100U) ? 200U : (wait * 2U);
  if (s_backoff > 200U)
  {
    s_backoff = 200U;
  }

  /* 🔴 冷启动用 MPU6050_Init（20 次探测，上电后器件可能要 >100 ms 才应答）；
   *    掉线重连用 MPU6050_Reinit（**只探 1 次**）。
   *    差别是 200–300 ms 的盲等 —— 实测它就是 862/891 ms 停采的主项，
   *    详见 APP_I2C_FAIL_TOLERANCE 上方那段算账。 */
  g_recon_attempts++;
  st = (cold != 0U) ? MPU6050_Init(&hi2c1) : MPU6050_Reinit(&hi2c1);
  if (st == MPU6050_OK)
  {
    g_sensor_ok = 1U;
    s_diag_done = 0U;   /* 掉线后再次失败时，体检要能重跑 */
    s_backoff   = 0U;   /* 恢复正常，下次掉线仍然快速重试 */
    if (App_TextAllowed() != 0U)
    {
      printf("[OK] MPU6050 online (WHO_AM_I=0x68), 1kHz sampling starts\r\n");
    }
  }
  else
  {
    if (App_TextAllowed() != 0U)
    {
      uint8_t id = 0U;
      (void)MPU6050_WhoAmI(&hi2c1, &id);   /* 顺便把实际读到的 id 打出来 */
      printf("[WAIT] MPU6050 not found (id=0x%02X, st=%u step=%u err=0x%02X). "
             "Check wiring: VCC=3V3 GND SDA=PB7 SCL=PB6 AD0=GND\r\n",
             (unsigned)id, (unsigned)st, (unsigned)g_mpu_fail_step,
             (unsigned)g_mpu_fail_hal);
    }

    if ((s_diag_done == 0U) && (App_TextAllowed() != 0U))
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

#if (APP_CLOUD_ENABLE)
/* 一个 128 点窗口攒满时调一次：把累加器换算成「三轴交流 RMS」。
 *
 * 为什么是"方差相加再开方"而不是逐轴 RMS：
 *   ① 必须去掉直流（重力约 1 g），否则停机和额定都顶到 ~1 g，完全分不开；
 *   ② 方差和对安装朝向不变 ⇒ 换个姿态也不用重新标门槛。 */
static void App_AcRmsUpdate(void)
{
  float    var = 0.0f;
  uint16_t k;

  if (g_ac_n < 8U)
  {
    return;                     /* 半窗（掉线/重启）不算，免得门槛误判 */
  }
  for (k = 0U; k < 3U; k++)
  {
    float mean = g_ac_sum[k] / (float)g_ac_n;
    float ms   = g_ac_sumsq[k] / (float)g_ac_n;
    float v    = ms - (mean * mean);
    if (v > 0.0f)
    {
      var += v;                 /* 数值噪声可能让 v 微负，跳过即可 */
    }
  }
  g_ac_rms = sqrtf(var);

  g_ac_sum[0] = g_ac_sum[1] = g_ac_sum[2] = 0.0f;
  g_ac_sumsq[0] = g_ac_sumsq[1] = g_ac_sumsq[2] = 0.0f;
  g_ac_n = 0U;
}
#endif

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
    static uint32_t     s_ok_tick = 0U;   /* 上一次**成功采样**的时刻（解剖停顿用） */

    if (MPU6050_ReadAccel(&raw) != MPU6050_OK)
    {
      /* 三级阶梯，见 APP_I2C_FAIL_TOLERANCE 上方的整段说明。 */
      g_i2c_fail_total++;
      g_gap_cur_fails++;
      s_i2c_fail++;

      /* ① 抖动：只丢这一拍，帧继续攒 */
      if (s_i2c_fail < APP_I2C_FAIL_TOLERANCE)
      {
        return;
      }

      /* ② 传感器仍算在线，就地自愈 —— 每 8 次连续失败恢复一次总线。
       *    为什么敢就地恢复：App_I2cBusRecover() 在不"卡死"的常态下
       *    只做 DeInit+Init 加两次引脚读，几十微秒就返回；
       *    真正要补 9 个时钟时说明从机确实被钉住，那时更该补。 */
      if (s_i2c_fail < APP_I2C_OFFLINE_AT)
      {
        if ((s_i2c_fail % APP_I2C_RECOVER_EVERY) == 0U)
        {
          g_gap_cur_recover++;
          App_I2cBusRecover();
        }
        return;
      }

      /* ③ 连续 40 次仍不应答 → 认账掉线，交给 App_EnsureSensor 走退避重连 */
      s_i2c_fail = 0U;
      g_offline_events++;
      g_sensor_ok = 0U;
      /* 🔴 必须把已经攒了一半的帧**作废**。
       *    不作废的话，等传感器重连回来会接着往同一帧里塞：
       *    这一帧的 32 个点就**跨过一个真实空洞首尾相接**，而校验和、
       *    序号、物理量全部合法 ⇒ 数据看着完好，其实是一段假波形。
       *    这种错误无法在下游被发现，宁可少一帧（下游只是少一个窗口）。 */
      if (g_frame_idx > 0U)
      {
        g_frame_idx = 0U;
        g_frame_dropped++;
      }
      return;
    }
    s_i2c_fail = 0U;

    /* 「停顿解剖」：每次成功采样时结算上一轮间隔，只记录最坏那一次。
     * 间隔用 g_tick 差值（1 拍 = 1 ms）而不是 HAL_GetTick —— 两者同源，
     * 但 g_tick 就是采样时基本身，口径与 g_sample_lost 完全一致。 */
    {
      uint32_t gap = g_tick - s_ok_tick;
      if (gap > g_gap_max_ms)
      {
        g_gap_max_ms      = gap;
        g_gap_max_fails   = g_gap_cur_fails;
        g_gap_max_recover = g_gap_cur_recover;
      }
      s_ok_tick         = g_tick;
      g_gap_cur_fails   = 0U;
      g_gap_cur_recover = 0U;
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
#if (APP_CLOUD_ENABLE)
    /* 顺手累加"逐窗交流 RMS"的原料（判停机用，见 CLOUD_STILL_RMS）。
     * 就在喂模型这一个点上做，不额外开一路采样，也不影响喂进去的数据。 */
    {
      uint16_t k2;
      for (k2 = 0U; k2 < 3U; k2++)
      {
        g_ac_sum[k2]   += xyz[k2];
        g_ac_sumsq[k2] += xyz[k2] * xyz[k2];
      }
      g_ac_n++;
    }
    {
      uint8_t r = NEAI_AppPushSample(xyz);
      if ((r == NEAI_PUSH_DETECTED) || (r == NEAI_PUSH_LEARNED))
      {
        App_AcRmsUpdate();      /* 窗口刚好结束 ⇒ 算一次 */
      }
    }
#else
    (void)NEAI_AppPushSample(xyz);
#endif
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

/* UART 出错回调 —— 必须处理，否则模块会"突然变哑"。
 *
 * 🔴 触发场景：ORE（溢出）。115200 下一个字节 87 µs，主循环任何 >87 µs 的停顿
 *    都可能让 DR 被覆盖。HAL 在 ORE 后会调用 AbortReceive 把接收**停掉**，
 *    此后 RxCpltCallback 再也不进来，表象就是"ESP 没反应了"。
 *    处理办法 = 清标志 + 重新装机。
 * 2026-10-25 加：配合 esp_at.c 的中断收包，PUBACK 才收得到。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_CLEAR_OREFLAG(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  if (huart->Instance == USART3)
  {
    esp_rx_restart();                   /* 寄存器级重新打开 RXNEIE */
    return;
  }
  (void)HAL_UART_Receive_IT(huart, (uint8_t *)&s_rx_byte, 1U);
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

#if (APP_CLOUD_ENABLE)
/* =====================================================================
 * 三、云端上报（第 ⑧ 步）
 * =====================================================================
 * 分工（D34 方案 A）：
 *   esp_at.c   —— 一根网线：连 Wi-Fi / 建 TCP / 进透传，**不解析任何 MQTT**
 *   mqtt_pkt.c —— 只拼字节：CONNECT / PUBLISH / PINGREQ，**不碰串口**
 *   本段       —— 决定"什么时候发、发什么、发完怎么记账"
 * 这样分层的好处：10.23 上板后如果连不上，可以把"报文拼错"单独排除掉
 * （PC 上已经逐字节对拍过，见 `_tmp/hosttest/test_mqtt.c`），只剩接线与 AT。
 *
 * 上报节拍（依据 R14.9 的配额换算，**别凭感觉改**）
 * ---------------------------------------------------------------------
 *   免费单元 10 000 条/天 ≈ 0.116 条/s，于是：
 *      1 s/条 → 86 400 条/天（超 8.6 倍）🔴 超配额不报错，只**限流**，
 *                                现象是"数据突然不动了"，很像固件卡死
 *     30 s/条 →  2 880 条/天（28.8 %）   正常心跳
 *     60 s/条 →  1 440 条/天（14.4 %）   异常持续中，比正常还省
 *   策略：**状态变化立刻发**（演示时"一有异常秒出"），持续异常降频到 60 s。
 *   最坏情况（一直在异常 + 抖边界）也只是 1 440 条的量级，离 10 000 很远。
 */

/* 把发送队列里的字节全部倒到 USART1 上（最多等 timeout_ms）。
 *
 * 🔴 为什么云端连接前**必须**先做这一步：
 *    printf() 只是把字节塞进 1024 B 的队列，真正往串口倒字节的
 *    App_PumpUart() 在**主循环**里。而 esp_join() 最长阻塞 25 s，
 *    这期间主循环一动不动 ⇒ 队列很快塞满 ⇒ 之后所有日志被丢弃。
 *    丢掉的恰好是 "CONNACK rc=0x00 还是 0x04" 这种最关键的一行，
 *    串口上看起来就是"连到一半没声了"，极难判断走到哪一步。 */
static void App_TxqDrain(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();

  while ((g_txq_head != g_txq_tail) && ((HAL_GetTick() - t0) < timeout_ms))
  {
    App_PumpUart();
  }
}

/* 发一条 properties/report（QoS1，平台会回 PUBACK）。
 *
 * payload 只有两个属性，与物模型逐字对应：
 *   {"services":[{"service_id":"vibration","properties":
 *     {"similarity":92,"verdict":"normal"}}]}
 *
 * 🔴 **不带 `ts`**：物模型里的 `ts` 属性已于 2026-10-22 删除（D33），
 *    数据时间交给平台内建的 `eventTime`。带着它不会报错，但平台直接忽略，
 *    反而容易让人误以为"时间是我发的"。 */
static void App_CloudReport(uint8_t sim, uint8_t anomaly)
{
  static char js[160];
  int         len;
  uint16_t    plen;

  len = snprintf(js, sizeof js,
                 "{\"services\":[{\"service_id\":\"%s\",\"properties\":"
                 "{\"similarity\":%u,\"verdict\":\"%s\"}}]}",
                 CLOUD_SERVICE_ID, (unsigned)sim,
                 (anomaly != 0U) ? "anomaly" : "normal");
  if ((len <= 0) || (len >= (int)sizeof js))
  {
    s_cloud.errors++;
    return;                                  /* 理论上不可能：字段都是定长小数字 */
  }

  plen = mqtt_encode_publish(&s_pub, s_topic, (const uint8_t *)js,
                             (uint16_t)len, 1U, s_pub_id);
  if (plen == 0U)
  {
    s_cloud.errors++;
    return;
  }
  s_pub_id++;                                /* 0 会被当成非法 id，翻转后跳过 0 */
  if (s_pub_id == 0U)
  {
    s_pub_id = 1U;
  }

  if (esp_transparent_send(s_pub.buf, plen) != (int)plen)
  {
    s_cloud.errors++;
    return;
  }

  s_cloud.reports++;
  s_cloud.last_report_ms = HAL_GetTick();
  s_cloud.last_tx_ms     = s_cloud.last_report_ms;
  s_cloud.alarm          = anomaly;
  s_cloud.have_verdict   = 1U;

  printf("[CLOUD] #%lu sim=%u %s (puback=%lu)\r\n",
         (unsigned long)s_cloud.reports, (unsigned)sim,
         (anomaly != 0U) ? "ANOMALY" : "normal",
         (unsigned long)s_cloud.pubacks);
}

/* 跨次调用的收包暂存区。
 *
 * 🔴 **必须保留半包**：串口是字节流，PUBACK 的 4 个字节完全可能分两次到达。
 *    2026-10-25 上板实测（v17）就是这样：一次读到 `40 02`、下一次读到 `00 00`。
 *    旧代码看到"不够 4 字节"直接 return，把已经收到的 2 字节**扔掉**，
 *    下一轮于是把 `0x00` 当成报文头 ⇒ 整条字节流错位，
 *    表现为 `puback` 永远停在 1，并且不停打印"收到无法识别的字节 0x02 / 0x00"。
 *    现在改成"收进暂存区 → 按定长切 → 只消费整条到齐的部分"。 */
#define CLOUD_RX_CAP   (32u)
static uint8_t  s_rxb[CLOUD_RX_CAP];
static uint16_t s_rxn = 0u;

/* 把已经到达的字节收下来（**非阻塞**，timeout = 0）。
 *
 * 透传模式下收到的就是裸 MQTT 报文，只会是这三种之一：
 *   PUBACK   `40 02 xx xx`   —— QoS1 上报的回执，说明平台真收到了
 *   PINGRESP `D0 00`         —— 心跳应答
 *   CONNACK  `20 02 ..`      —— 只可能出现在刚连上时（这里当异常处理）
 * 我们没有订阅任何 topic，所以不会有 PUBLISH 进来。
 * 🔴 只认这几种定长报文，**不实现**通用的剩余长度解析 —— 省 Flash，也避免
 *    "为了通用而写错"：真收到看不懂的字节就清缓冲，不猜。 */
static void App_CloudDrainRx(void)
{
  int      n;
  uint8_t  type;
  uint16_t rl;
  uint16_t need;
  uint16_t k;

  /* ① 收：接到暂存区尾部。收满就先不读，剩下的下一轮再说（不丢数据） */
  if (s_rxn < CLOUD_RX_CAP)
  {
    n = esp_transparent_recv(&s_rxb[s_rxn],
                             (uint16_t)(CLOUD_RX_CAP - s_rxn), 0U);
    if (n > 0)
    {
      s_rxn = (uint16_t)(s_rxn + (uint16_t)n);
    }
  }

  /* ② 解：只消费"整条到齐"的报文，半包留在缓冲区里等下一轮补齐 */
  while (s_rxn >= 2u)
  {
    type = (uint8_t)(s_rxb[0] & 0xF0U);
    rl   = (uint16_t)(s_rxb[1] & 0x7FU);   /* 三种报文 RL 都 < 128 ⇒ 单字节编码 */

    if (rl > 4u)
    {
      /* 我们只订阅了三种定长报文，RL 不可能超过 4 —— 说明已经错位了。
       * 🔴 在 ONLINE 状态下，最常见的成因是**模块自己吐了一行文本**：
       *    TCP 被对端关掉时 ESP8266 会打 `CLOSED`，那 6 个字节在透传里
       *    和 MQTT 数据混在一起，`0x43`('C') 被当成类型、`0x4C`('L') 被当成
       *    长度 = 76 ⇒ 落到这里。所以这里不只是"清缓冲"，而是**当成掉线信号**
       *    交给上层去重连（`s_cloud.link_lost`）。 */
      s_cloud.link_lost = 1U;
      s_rxn = 0u;
      return;
    }

    need = (uint16_t)(2u + rl);
    if (s_rxn < need)
    {
      return;                              /* 半包：等下一轮 */
    }

    switch (type)
    {
      case 0x40U:                          /* PUBACK —— 平台真收到了 */
        s_cloud.pubacks++;
        break;

      case 0xD0U:                          /* PINGRESP */
        break;

      case 0x20U:                          /* CONNACK（不该在这时候出现） */
        printf("[CLOUD] 意外收到 CONNACK rc=0x%02X\r\n",
               (unsigned)s_rxb[3]);
        break;

      default:                             /* 看不懂就不猜，清空重同步 */
        s_cloud.link_lost = 1U;            /* 同上：多半是模块吐的文本，不是 MQTT */
        s_rxn = 0u;
        return;
    }

    /* 消费掉这一条，剩下的左移（最多 32 字节，代价可忽略） */
    for (k = need; k < s_rxn; k++)
    {
      s_rxb[k - need] = s_rxb[k];
    }
    s_rxn = (uint16_t)(s_rxn - need);
  }
}

/* =====================================================================
 * 云端连接状态机（2026-10-25）
 * =====================================================================
 * 契约：**App_CloudStep() 里每个状态一进入就立刻返回**，不等待任何应答。
 * 这样主循环每轮只花几微秒，1 kHz 采样一点都不会被吃掉。
 */

/* 初始化：只把状态机摆到起点，**不建链**（建链交给状态机一步步做）
 *
 * 🔴 2026-10-25 之前的写法是"开机一次性阻塞建链"（最长 30 s），
 *    并且刻意**不做运行期重连**。代价是链路一掉就永久离线，
 *    平台上看就是"设备一直离线"，而板子本地跑得好好的（`RISKS` R26）。
 *    改成状态机后，这条老约束"建链必须在 HAL_TIM_Base_Start_IT 之前"
 *    也一起作废了 —— 没有阻塞，放在哪里都不会吃采样。 */
static void App_CloudStart(void)
{
  /* ---- 参数没配齐就别连 --------------------------------------------
   * 空字符串会让 AT 变成 `AT+CWJAP="",""`，模块回 ERROR，
   * 现场看起来像"模块坏了"，其实只是没填 secrets.h。
   * 🔴 **逐个报是哪一个没填** —— 一句笼统的"未配置"会让人把三行都翻一遍。 */
  if (CLOUD_WIFI_SSID[0] == '\0')
  {
    printf("[CLOUD] 跳过：CLOUD_WIFI_SSID 没填（Core/Inc/secrets.h）\r\n");
    return;
  }
  if (CLOUD_DEVICE_ID[0] == '\0')
  {
    printf("[CLOUD] 跳过：CLOUD_DEVICE_ID 没填（Core/Inc/secrets.h）\r\n");
    return;
  }
  if (CLOUD_SECRET[0] == '\0')
  {
    printf("[CLOUD] 跳过：CLOUD_SECRET 没填（Core/Inc/secrets.h）\r\n");
    return;
  }

  esp_init(&huart3);
  (void)snprintf(s_topic, sizeof s_topic,
                 "$oc/devices/%s/sys/properties/report", CLOUD_DEVICE_ID);

  printf("[CLOUD] 目标 %s:%d（非阻塞建链，掉线自动重连）\r\n",
         CLOUD_HOST, (int)CLOUD_PORT);
  App_TxqDrain(500U);        /* 开机阶段 1 kHz 节拍还没启动，短暂等是安全的 */

  /* 🔴 起点是 CS_WAIT 而不是 CS_ESC_SIL：给模块 1.5 s 把它正在做的事做完。
   *    直接进 ESC_SIL 的话，模可能在开机瞬间正吐残留，"静默"判定会被它自己
   *    的字节不断推迟，或者反过来太早发 `+++` 被当数据吞掉 ——
   *    实测代价是浪费一整轮重连（12 s）。 */
  s_cloud.state     = (uint8_t)CS_WAIT;
  s_cloud.t_state   = HAL_GetTick();
  s_cloud.backoff   = CLOUD_BOOT_SETTLE_MS;
  s_cloud.esc_tries = 0U;
  s_cloud.enabled   = 1U;    /* 🔴 只有走到这里才允许状态机跑 */
  esp_set_transparent(0);    /* 我们并不知道模块真实状态，一律当"不在透传"处理 */
}

/**
 * @brief 进入退避等待（建链失败 or 运行期掉线）
 * @param is_failure 1 = 建链失败（失败计数 + 退避翻倍）；0 = 掉线（固定短退避）
 */
static void App_CloudGoWait(uint32_t now, uint8_t is_failure)
{
  s_cloud.connected = 0U;
  s_cloud.link_lost = 0U;

  if (is_failure != 0U)
  {
    s_cloud.fails++;
    if (s_cloud.backoff == 0U)
    {
      s_cloud.backoff = CLOUD_BACKOFF_MIN_MS;
    }
    else
    {
      s_cloud.backoff *= 2U;
      if (s_cloud.backoff > CLOUD_BACKOFF_MAX_MS)
      {
        s_cloud.backoff = CLOUD_BACKOFF_MAX_MS;
      }
    }
    printf("[CLOUD] 建链失败（累计 %lu 次），%lu ms 后重试\r\n",
           (unsigned long)s_cloud.fails, (unsigned long)s_cloud.backoff);
  }
  else
  {
    s_cloud.backoff = CLOUD_BACKOFF_MIN_MS;
  }

  s_cloud.state   = (uint8_t)CS_WAIT;
  s_cloud.t_state = now;
}

/**
 * @brief 推进一步。**每拍只做一件事，绝不等待。**
 *
 * 状态迁移：
 *   WAIT ──退避到点──> ESC_SIL ──静默够──> ESC_SETTLE ──静默够──> PING
 *   PING ──OK──> ECHO ──> JOIN ──> TCP ──> MODE ──> SEND ──> CONNACK ──> ONLINE
 *   任何一步 FAIL/TIMEOUT ──> WAIT（退避重来）
 */
static void App_CloudStep(uint32_t now)
{
  int st;

  switch (s_cloud.state)
  {
    case CS_WAIT:
      if ((now - s_cloud.t_state) < s_cloud.backoff)
      {
        break;
      }
      esp_set_transparent(0);
      s_cloud.esc_tries = 0U;
      s_cloud.state     = (uint8_t)CS_ESC_SIL;
      s_cloud.t_state   = now;
      break;

    case CS_ESC_SIL:
      /* `+++` 的前半个条件：线路**真的**静默 ≥1.1 s（不是盲等 —— 模块可能
       * 正在吐复位前排队的残留，那段"忙"会把 3 个 `+` 当数据吞掉）。
       * 5 s 还等不到静默就硬发一次，多一次机会不亏。 */
      if ((esp_silence_ms() >= CLOUD_ESC_SILENCE_MS) ||
          ((now - s_cloud.t_state) >= 5000U))
      {
        esp_rx_flush();
        esp_send_plus3();
        s_cloud.t_esc = now;
        s_cloud.state = (uint8_t)CS_ESC_SETTLE;
      }
      break;

    case CS_ESC_SETTLE:
      if ((now - s_cloud.t_esc) >= CLOUD_ESC_SILENCE_MS)
      {
        esp_rx_flush();
        (void)esp_async_ping();
        s_cloud.state   = (uint8_t)CS_PING;
        s_cloud.t_state = now;
      }
      break;

    case CS_PING:
      st = esp_async_poll();
      if (st == (int)ESP_ASYNC_RUNNING)
      {
        break;
      }
      if (st == (int)ESP_ASYNC_OK)
      {
        if (s_cloud.esc_tries > 0U)
        {
          printf("[CLOUD] ✅ +++ 逃逸成功，模块回到指令模式\r\n");
        }
        (void)esp_async_echo_off();
        s_cloud.state = (uint8_t)CS_ECHO;
        break;
      }
      /* AT 不通 ⇒ 再逃逸一次；逃逸用尽仍不通 ⇒ 退避重来 */
      if (s_cloud.esc_tries < CLOUD_ESC_TRIES)
      {
        s_cloud.esc_tries++;
        printf("[CLOUD] AT 无应答，第 %u 次 +++ 逃逸重试\r\n",
               (unsigned)s_cloud.esc_tries);
        s_cloud.state   = (uint8_t)CS_ESC_SIL;
        s_cloud.t_state = now;
      }
      else
      {
        printf("[CLOUD] ESP 无应答 —— 查 PB10/PB11 是否交叉、共地、3V3、"
               "模块是否上电；以及 D1 mini 的 USB 是否还插着\r\n");
        App_CloudGoWait(now, 1U);
      }
      break;

    case CS_ECHO:
      st = esp_async_poll();
      if (st == (int)ESP_ASYNC_RUNNING)
      {
        break;
      }
      /* ATE0 失败不致命（只是日志吵一倍），继续往下走 */
      (void)esp_async_join(CLOUD_WIFI_SSID, CLOUD_WIFI_PASS);
      s_cloud.state = (uint8_t)CS_JOIN;
      break;

    case CS_JOIN:
      st = esp_async_poll();
      if (st == (int)ESP_ASYNC_RUNNING)
      {
        break;
      }
      if (st == (int)ESP_ASYNC_OK)
      {
        printf("[ESP] Wi-Fi 已连接\r\n");
        (void)esp_async_tcp_open(CLOUD_HOST, (uint16_t)CLOUD_PORT);
        s_cloud.state = (uint8_t)CS_TCP;
      }
      else
      {
        printf("[CLOUD] 连 Wi-Fi 失败 —— 查 SSID/密码、路由器是否 2.4 GHz\r\n");
        App_CloudGoWait(now, 1U);
      }
      break;

    case CS_TCP:
      st = esp_async_poll();
      if (st == (int)ESP_ASYNC_RUNNING)
      {
        break;
      }
      if (st == (int)ESP_ASYNC_OK)
      {
        (void)esp_async_cipmode();
        s_cloud.state = (uint8_t)CS_MODE;
      }
      else
      {
        printf("[CLOUD] TCP 连接失败 —— 查域名拼写、端口 1883\r\n");
        App_CloudGoWait(now, 1U);
      }
      break;

    case CS_MODE:
      st = esp_async_poll();
      if (st == (int)ESP_ASYNC_RUNNING)
      {
        break;
      }
      if (st == (int)ESP_ASYNC_OK)
      {
        (void)esp_async_cipsend();
        s_cloud.state = (uint8_t)CS_SEND;
      }
      else
      {
        printf("[CLOUD] AT+CIPMODE=1 失败\r\n");
        App_CloudGoWait(now, 1U);
      }
      break;

    case CS_SEND:
      st = esp_async_poll();
      if (st == (int)ESP_ASYNC_RUNNING)
      {
        break;
      }
      if (st != (int)ESP_ASYNC_OK)
      {
        printf("[CLOUD] 进透传失败（CIPSEND 没等到 '>'）\r\n");
        App_CloudGoWait(now, 1U);
        break;
      }
      /* ---- 进透传成功：此后写出去的字节**就是** MQTT 报文 ---------- */
      esp_set_transparent(1);
      esp_rx_flush();              /* 抽掉 "OK"/">" 之类的残留 */
      {
        char cid[80];
        char pw[65];

        if (mqtt_auth_build(CLOUD_DEVICE_ID, CLOUD_SECRET, AUTH_TS, 0U,
                            cid, sizeof cid, pw, sizeof pw) != 0)
        {
          printf("[CLOUD] 鉴权参数非法（device_id 或 ts 长度不对）\r\n");
          App_CloudGoWait(now, 1U);
          break;
        }
        if (mqtt_encode_connect(&s_pub, cid, CLOUD_DEVICE_ID, pw,
                                CLOUD_KEEPALIVE_S) == 0U)
        {
          printf("[CLOUD] CONNECT 拼包失败（缓冲不够）\r\n");
          App_CloudGoWait(now, 1U);
          break;
        }
        if (esp_transparent_send(s_pub.buf, s_pub.len) != (int)s_pub.len)
        {
          printf("[CLOUD] CONNECT 发送失败\r\n");
          App_CloudGoWait(now, 1U);
          break;
        }
      }
      s_rxn           = 0U;
      s_cloud.state   = (uint8_t)CS_CONNACK;
      s_cloud.t_state = now;
      break;

    case CS_CONNACK:
    {
      int n = esp_transparent_recv(&s_rxb[s_rxn],
                                   (uint16_t)(CLOUD_RX_CAP - s_rxn), 0U);
      if (n > 0)
      {
        s_rxn = (uint16_t)(s_rxn + (uint16_t)n);
      }
      if (s_rxn >= 4U)
      {
        int rc = mqtt_connack_rc(s_rxb, s_rxn);
        if (rc == 0)
        {
          s_cloud.connected      = 1U;
          s_cloud.link_lost      = 0U;
          s_cloud.backoff        = 0U;
          s_cloud.fails          = 0U;
          s_cloud.esc_tries      = 0U;
          s_cloud.have_verdict   = 0U;   /* 重连后让第一条立刻发，便于确认 */
          s_cloud.pend_cnt       = 0U;   /* 去抖从头开始 */
          s_cloud.last_detect    = NEAI_AppStats()->detect_calls;
          s_cloud.last_tx_ms     = now;
          s_cloud.last_report_ms = now;
          s_cloud.t_online       = now;
          s_cloud.state          = (uint8_t)CS_ONLINE;
          printf("[CLOUD] ✅ CONNACK rc=0x00，管道就绪"
                 "（上报节拍 %u s / 异常 %u s，阈值 sim<%u）\r\n",
                 (unsigned)(CLOUD_PERIOD_MS / 1000U),
                 (unsigned)(CLOUD_ALARM_MS / 1000U),
                 (unsigned)CLOUD_SIM_ALARM);
        }
        else
        {
          printf("[CLOUD] CONNACK 失败 rc=%d（0x%02X）—— 见 RUNBOOK R14.2 ①\r\n",
                 rc, (unsigned)s_rxb[3]);
          App_CloudGoWait(now, 1U);
        }
        s_rxn = 0U;
      }
      else if ((now - s_cloud.t_state) >= CLOUD_CONNACK_MS)
      {
        printf("[CLOUD] 等 CONNACK 超时（平台没回）\r\n");
        App_CloudGoWait(now, 1U);
      }
      break;
    }

    case CS_ONLINE:
    default:
      break;
  }
}

/* 主循环里调，**非阻塞、微秒级**。
 *
 * 🔴 这里任何一次阻塞都会直接变成采样丢点（lost 计数），所以：
 *    · 重连也走状态机（`App_CloudStep` 每拍只推进一步）；
 *    · 接收用 timeout=0，靠环形缓冲预判，不做 1 ms 忙等；
 *    · 连不上就静默降级（云端不影响采样与推理，这是本项目一贯的取舍）。 */
static void App_CloudPoll(void)
{
  const neai_app_stats_t *st;
  uint32_t now;
  uint32_t period;
  uint8_t  sim;
  uint8_t  alarm;

  now = HAL_GetTick();

  /* ---- ① 先推状态机（建链 / 重连都在这里，非阻塞） ---- */
  if (s_cloud.enabled == 0U)
  {
    return;                        /* 没配 secrets.h / 参数不全 ⇒ 整块不跑 */
  }
  App_CloudStep(now);

  if (s_cloud.state != (uint8_t)CS_ONLINE)
  {
    return;                        /* 没连上就只跑状态机 */
  }

  /* ---- ② 收包（PUBACK / PINGRESP，或模块吐的 CLOSED） ---- */
  App_CloudDrainRx();

  /* ---- ③ 掉线判据 A：字节流里出现了 MQTT 解析不了的东西 ----
   * 最典型的就是模块自己打的 `CLOSED`（TCP 被对端关掉）。
   * 这个信号比"静默超时"快得多，所以优先用它。 */
  if (s_cloud.link_lost != 0U)
  {
    s_cloud.reconnects++;
    printf("[CLOUD] ⚠ 检测到链路中断（模块 CLOSED / 字节流错乱）"
           "→ 自动重连 #%lu\r\n", (unsigned long)s_cloud.reconnects);
    App_CloudGoWait(now, 0U);
    return;
  }

  /* ---- ④ 掉线判据 B：下行静默太久 ----
   * 正常情况下最慢 60 s 也会来一个 PINGRESP，所以 100 s 一个字节都没有
   * 就说明管道已经单向死了（我们还能写、但收不到）—— 那正是 2026-10-25
   * 现场看到的样子：`puback` 冻结不动、平台判离线。 */
  if (((now - s_cloud.t_online) >= CLOUD_MIN_ONLINE_MS) &&
      (esp_silence_ms() >= CLOUD_DOWN_MS))
  {
    s_cloud.reconnects++;
    printf("[CLOUD] ⚠ 下行静默 %lu ms 一个字节都没有 → 判掉线，自动重连 #%lu\r\n",
           (unsigned long)esp_silence_ms(), (unsigned long)s_cloud.reconnects);
    App_CloudGoWait(now, 0U);
    return;
  }

  st = NEAI_AppStats();
  if (st->detect_calls == 0U)
  {
    /* 还没跑完第一次推理（含开机学习期约 37 s）—— 此时的 last_similarity
     * 还是 0，报上去就是一条假的"极度异常"。宁可不报。 */
    return;
  }

  sim   = st->last_similarity;

  /* 🔴 迟滞（hysteresis）：进异常用 90，退异常要回到 95 以上。
   *    只用一个门槛时，相似度在 90 附近抖动会让判定每个窗口翻一次 ——
   *    2026-10-25 上板实测：60 s 内发了 **75 条**上报（按 30 s 节拍应该是 2 条），
   *    既打爆链路也打爆平台配额，日志还被刷得看不见别的。
   *    迟滞 + 下面的最小间隔一起解决这个问题。 */
  /* ① 先按迟滞算出"这一窗想给什么判定" */
  if (s_cloud.alarm != 0U)
  {
    alarm = (sim > CLOUD_SIM_CLEAR) ? 0U : 1U;
  }
  else
  {
    alarm = (sim < CLOUD_SIM_ALARM) ? 1U : 0U;
  }

  /* ② 停机覆盖：机器根本没转 ⇒ 一律不判异常，并且**不参与去抖**。
   *
   * 🔴 训练集只有 `normal_d100`（电机在跑），停机段属 OOD（`T14`：14.9 %
   *    的停机窗口会被判异常）。不挡住的话，机器一停云端就刷"异常"，
   *    看着像误报，其实是模型没见过静止工况。 */
  if (g_ac_rms < CLOUD_STILL_RMS)
  {
    alarm = 0U;
    g_still_win++;
    s_cloud.pend_cnt = 0U;          /* 停机不算"想翻转" */
  }

  /* ③ 判定去抖：连续 N 个窗口都想翻，才真的翻。
   *
   * 🔴 为什么需要：`d100` 下实测仍有 0.8 % 的**孤立单窗尖峰**（min 50），
   *    而「连续 <90」的最长连串只有 **1 窗**。每个尖峰会造成"翻出 + 翻回"
   *    两条上报 ⇒ 90 s 里 8 条（4 尖峰 × 2），云上看就是 normal/anomaly 乱跳。
   *    N=2 就能把这些全部消掉，代价只是检测延迟 +0.13 s。 */
  {
    uint8_t changed = 0U;

    if ((s_cloud.have_verdict != 0U) &&
        (st->detect_calls != s_cloud.last_detect))
    {
      s_cloud.last_detect = st->detect_calls;      /* 有新窗口了，才判一次 */
      if (alarm != s_cloud.alarm)
      {
        s_cloud.pend_cnt++;
        if (s_cloud.pend_cnt >= CLOUD_DEBOUNCE_WIN)
        {
          s_cloud.alarm  = alarm;                  /* 确认翻转 */
          s_cloud.pend_cnt = 0U;
          changed        = 1U;
        }
      }
      else
      {
        s_cloud.pend_cnt = 0U;
      }
    }
    alarm = s_cloud.alarm;     /* 未确认前一律沿用上一次已上报的判定 */

    /* ④ 第一条：立刻发，让平台一上电就有数 */
    if (s_cloud.have_verdict == 0U)
    {
      App_CloudReport(sim, alarm);
      return;
    }

    /* ⑤ 判定**确认**翻转 → 尽快发（演示时"异常秒出"靠这一条），
     *    但受最小间隔约束：不设下限的话"秒出"会变成"每秒一条"。 */
    if ((changed != 0U) &&
        ((now - s_cloud.last_report_ms) >= CLOUD_MIN_GAP_MS))
    {
      App_CloudReport(sim, alarm);
      return;
    }
  }

  /* ⑥ 持续同一判定 → 按节拍发（正常 30 s / 异常 60 s） */
  period = (alarm != 0U) ? CLOUD_ALARM_MS : CLOUD_PERIOD_MS;
  if ((now - s_cloud.last_report_ms) >= period)
  {
    App_CloudReport(sim, alarm);
    return;
  }

  /* ⑦ 兜底心跳：万一长时间没发任何东西（比如相似度一直没更新），
   *    keepalive 到期会被平台踢（实测 ≈1.5 倍 keepalive）。 */
  if ((now - s_cloud.last_tx_ms) >= CLOUD_PING_MS)
  {
    uint16_t plen = mqtt_encode_pingreq(&s_pub);
    if ((plen > 0U) && (esp_transparent_send(s_pub.buf, plen) == (int)plen))
    {
      s_cloud.last_tx_ms = now;
    }
  }
}

#endif /* APP_CLOUD_ENABLE */

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
#if (APP_CLOUD_ENABLE)
  /* 这一行是"云端到底开没开、往哪儿发、多久发一次"的唯一现场证据 ——
   * 联调时先看它，比翻代码快。 */
  printf("cloud: IoTDA %s:%d, up %u s / alarm %u s, service=%s\r\n",
         CLOUD_HOST, (int)CLOUD_PORT,
         (unsigned)(CLOUD_PERIOD_MS / 1000U), (unsigned)(CLOUD_ALARM_MS / 1000U),
         CLOUD_SERVICE_ID);
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

#if (APP_CLOUD_ENABLE)
  /* ---- 第 ⑧ 步：把云端状态机摆到起点（**不建链**）-------------------
   * 2026-10-25 起建链改成**非阻塞状态机**，在 App_CloudPoll() 里每拍推进一步。
   * 所以这里不再有"必须放在 TIM2 启动之前"的约束（那条约束原本是为了
   * 避免 25 s 阻塞被记成 lost）。放在这里只是为了让日志顺序好看。
   * 失败/掉线都不影响采样与推理 —— 云端只是"多一条数据出口"。 */
  App_CloudStart();
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

  /* ---- 对齐采样基线（详见函数注释）---------------------------------
   * 上面那次探测会阻塞约 20 ms，必须在进主循环前把“已消费拍数”重新对齐，
   * 否则这 20 ms 会被记进 lost，让上板验收看到恒定的 lost=21。 */
  App_SyncTickBaseline();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 主循环五件事，按“越频繁越靠前”排：
     *   1) 倒串口 —— 每次只吐 4 字节，保证 1 ms 采样节拍不被拖垮；
     *   2) 处理采样 —— 有新拍才动，没新拍立刻返回；
     *   3) 传感器掉线重连 —— 没事时也立刻返回；
     *   4) 执行串口命令（第 ⑦ 步：改 PWM 占空比）—— 没命令时立刻返回；
     *   5) 云端上报（第 ⑧ 步）—— 非阻塞：收包用 timeout=0，
     *      只到 30 s 节拍才真的发一包（发一包 ≈ 200 字节 ≈ 17 ms，
     *      摊到 30 s 上是 0.06 %，对采样没有一点影响）。
     * 设计上这里**没有 HAL_Delay / 阻塞等待**，主循环全速空转，
     * 因此采样抖动只取决于 I2C 与串口的实际耗时之和。 */
    App_PumpUart();
    App_OnTick();
    App_EnsureSensor();
    App_PollCmd();
#if (APP_CLOUD_ENABLE)
    App_CloudPoll();
#endif

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
