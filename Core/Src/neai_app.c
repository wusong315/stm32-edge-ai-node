/* ============================================================================
 * neai_app.c — NanoEdge AI 应用层实现
 * ============================================================================
 * 说明见 neai_app.h。本文件刻意做到三件事：
 *   ① 尺寸全部取自库宏 → 换库零改动；
 *   ② 库返回码全部处理 → 不做"静默失败"；
 *   ③ 日志按节流打印 → 不挤掉 1 kHz 采样节拍。
 * ============================================================================ */

#include "neai_app.h"
#include "NanoEdgeAI.h"
#include "cycle_dwt.h"
#include "main.h"

#include <string.h>
#include <stdio.h>

/* 从库宏换算出本层的三个尺寸（改库自动跟着变） */
#define NEAI_NUM_AXES    (AXIS_NUMBER)
#define NEAI_WIN_SAMPLES (DATA_INPUT_USER)                 /* 一个窗口含多少个采样点 */
#define NEAI_BUF_FLOATS  (DATA_INPUT_USER * AXIS_NUMBER)   /* 交给库的 float 数组总长度 */

/* 编译期挡住"学习次数设得比库要求还少"这种错误（负长度数组必编译失败）。
 * 比 #warning 更硬：这种配置错误不该等到看日志才发现。 */
typedef char NEAI_learn_target_check
    [(APP_NEAI_LEARN_TARGET >= MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING) ? 1 : -1];

#if (APP_NEAI_ENABLE != 0)

/* ---------------------------------------------------------------------------
 * 状态
 * ------------------------------------------------------------------------- */
static float            s_buf[NEAI_BUF_FLOATS];  /* 交给库的输入缓冲区（axis-major 默认） */
static uint16_t         s_fill  = 0U;            /* 当前窗口已填采样点数 */
static uint8_t          s_ready = 0U;            /* init() 成功才置 1 */
static neai_app_stats_t s_st;                    /* 统计（供串口心跳与 Live Watch） */

/* 库返回码 → 可读名字，日志里直接看得懂 */
static const char *NEAI_StateName(enum neai_state st)
{
  switch (st)
  {
    case NEAI_OK:                                return "OK";
    case NEAI_INIT_FCT_NOT_CALLED:               return "INIT_NOT_CALLED";
    case NEAI_BOARD_ERROR:                       return "BOARD_ERROR";
    case NEAI_KNOWLEDGE_BUFFER_ERROR:            return "KNOWLEDGE_BUFFER_ERROR";
    case NEAI_NOT_ENOUGH_CALL_TO_LEARNING:       return "NOT_ENOUGH_CALL_TO_LEARNING";
    case NEAI_MINIMAL_RECOMMENDED_LEARNING_DONE: return "MINIMAL_RECOMMENDED_LEARNING_DONE";
    default:                                     return "UNKNOWN_ERROR";
  }
}

/* 把 DWT 周期数换算成微秒。
 * 🔴 用 HAL_RCC_GetHCLKFreq() 而不是硬编码 168000000：
 *    除数跟着实际主频走，将来改时钟树也不会算错（参考工程就是栽在硬编码上）。 */
static float NEAI_CyclesToUs(uint32_t cycles)
{
  return ((float)cycles * 1000000.0f) / (float)HAL_RCC_GetHCLKFreq();
}

#if (APP_NEAI_REMOVE_DC != 0)
/* 逐窗去直流：一窗内每轴的均值减掉。
 *
 * 🔴 必须是"填满整窗后两遍扫描"，不能改成逐点高通滤波器：
 *    PC 端 `to_neai_csv.py` 做的是 `seg - seg.mean(axis=1)`，只有两遍扫描
 *    与它**逐元素等价**；换成 IIR 高通就是另一个滤波器，训练与推理会不一致。
 *
 * 代价：每 128 ms 做 3×(128 加 + 128 减) ≈ 768 次浮点操作，相对 1 kHz 采样可忽略。
 * 布局：两种排列都要处理（AXIS_MAJOR=1 连续段；=0 跨步取）。 */
static void NEAI_RemoveWindowDc(float *buf)
{
  uint16_t axis, i;

  for (axis = 0U; axis < (uint16_t)NEAI_NUM_AXES; axis++)
  {
    float sum = 0.0f;
    float mean;

#if (APP_NEAI_AXIS_MAJOR != 0)
    {
      float *p = &buf[(uint32_t)axis * NEAI_WIN_SAMPLES];
      for (i = 0U; i < (uint16_t)NEAI_WIN_SAMPLES; i++) { sum += p[i]; }
      mean = sum / (float)NEAI_WIN_SAMPLES;
      for (i = 0U; i < (uint16_t)NEAI_WIN_SAMPLES; i++) { p[i] -= mean; }
    }
#else
    for (i = 0U; i < (uint16_t)NEAI_WIN_SAMPLES; i++)
    {
      sum += buf[((uint32_t)i * NEAI_NUM_AXES) + axis];
    }
    mean = sum / (float)NEAI_WIN_SAMPLES;
    for (i = 0U; i < (uint16_t)NEAI_WIN_SAMPLES; i++)
    {
      buf[((uint32_t)i * NEAI_NUM_AXES) + axis] -= mean;
    }
#endif
  }
}
#endif

/* 记一次错误：非 OK 返回码绝不静默丢弃 */
static void NEAI_NoteError(enum neai_state st)
{
  s_st.last_error = (uint32_t)st;
  s_st.error_count++;
  printf("[NEAI] *** library error: %s (%u)\r\n", NEAI_StateName(st), (unsigned)st);
}

void NEAI_AppReset(void)
{
  (void)memset(s_buf, 0, sizeof(s_buf));
  (void)memset(&s_st, 0, sizeof(s_st));
  s_st.min_similarity = 100U;   /* 最小值从 100 往下收 */
  s_fill  = 0U;
  s_ready = 0U;
}

void NEAI_AppInit(void)
{
  enum neai_state st;

  NEAI_AppReset();

  /* DWT 周期计数器：只用于测"一次推理多少微秒"，
   * 没有它推理照样跑，只是耗时统计会变成 0（不会报错）。 */
  KIN1_InitCycleCounter();
  KIN1_EnableCycleCounter();
  KIN1_ResetCycleCounter();

  /* 🔴 这一行是"到底链上了哪个库"的铁证：换库后 NEAI_ID 必须跟着变。
   *    若这里的 ID 还是 62e147... 就说明占位库没被换掉。 */
  printf("[NEAI] lib linked: ID=%s AXIS=%u WIN=%u MIN_ITER=%u\r\n",
         NEAI_ID,
         (unsigned)NEAI_NUM_AXES,
         (unsigned)NEAI_WIN_SAMPLES,
         (unsigned)MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING);

  st = neai_anomalydetection_init();
  if (st == NEAI_OK)
  {
    s_ready         = 1U;
    s_st.initialized = 1U;
    printf("[NEAI] init OK. learn target=%u windows, sensitivity=%.2f\r\n",
           (unsigned)APP_NEAI_LEARN_TARGET,
           (double)neai_anomalydetection_get_sensitivity());
  }
  else
  {
    /* init 失败**不阻断采集**：先把采样链路跑通，比"整个程序卡住"更有价值 */
    NEAI_NoteError(st);
    printf("[NEAI] AI disabled, sampling continues.\r\n");
  }
}

uint8_t NEAI_AppIsReady(void)
{
  return s_ready;
}

uint8_t NEAI_AppPushSample(const float *axis_g)
{
  enum neai_state st;
  uint16_t        i;

  if ((s_ready == 0U) || (axis_g == 0))
  {
    return NEAI_PUSH_NONE;
  }

  /* ---- 1) 存一个采样点进窗口 -------------------------------------- */
#if (APP_NEAI_AXIS_MAJOR != 0)
  /* 按轴优先（默认）：[ax0..axN-1][ay0..ayN-1][az0..azN-1] */
  for (i = 0U; i < NEAI_NUM_AXES; i++)
  {
    s_buf[((uint32_t)i * NEAI_WIN_SAMPLES) + s_fill] = axis_g[i];
  }
#else
  /* 按点交错：[ax0,ay0,az0][ax1,ay1,az1]... */
  for (i = 0U; i < NEAI_NUM_AXES; i++)
  {
    s_buf[((uint32_t)s_fill * NEAI_NUM_AXES) + i] = axis_g[i];
  }
#endif

  s_fill++;
  if (s_fill < (uint16_t)NEAI_WIN_SAMPLES)
  {
    return NEAI_PUSH_NONE;                 /* 窗口还没攒满 */
  }
  s_fill = 0U;                             /* 攒满 → 把这块缓冲整个交给库 */

#if (APP_NEAI_REMOVE_DC != 0)
  /* 🔴 必须在 learn/detect **之前**、整窗填满**之后**：两遍扫描去均值，
   *    与 PC 端训练集的口径逐元素等价（见 neai_app.h 里 APP_NEAI_REMOVE_DC 的说明）。*/
  NEAI_RemoveWindowDc(s_buf);
#endif

  /* ---- 2) 学习阶段 ------------------------------------------------ */
  if (s_st.learn_calls < APP_NEAI_LEARN_TARGET)
  {
    st = neai_anomalydetection_learn(s_buf);
    s_st.learn_calls++;

    if (st == NEAI_MINIMAL_RECOMMENDED_LEARNING_DONE)
    {
      /* 这不是错误：库在提示"已经学够了"，可以提前转入推理 */
      s_st.learned_enough = 1U;
      if ((s_st.learn_calls % APP_NEAI_PRINT_DECIM) == 0U)
      {
        printf("[NEAI] learn %u/%u -> library says: enough learning\r\n",
               (unsigned)s_st.learn_calls, (unsigned)APP_NEAI_LEARN_TARGET);
      }
      return NEAI_PUSH_LEARNED;
    }
    if (st != NEAI_OK)
    {
      NEAI_NoteError(st);
      return NEAI_PUSH_ERROR;
    }

    if (s_st.learn_calls >= APP_NEAI_LEARN_TARGET)
    {
      s_st.learned_enough = 1U;
      printf("[NEAI] learning done (%u windows). switching to DETECT mode.\r\n",
             (unsigned)s_st.learn_calls);
    }
    else if ((s_st.learn_calls % APP_NEAI_PRINT_DECIM) == 0U)
    {
      printf("[NEAI] learn %u/%u\r\n",
             (unsigned)s_st.learn_calls, (unsigned)APP_NEAI_LEARN_TARGET);
    }
    return NEAI_PUSH_LEARNED;
  }

  /* ---- 3) 推理阶段：测耗时 → 取相似度 ----------------------------- */
  {
    volatile uint32_t cycles;
    uint8_t           sim = 0U;
    float             us;

    KIN1_ResetCycleCounter();
    st = neai_anomalydetection_detect(s_buf, &sim);
    cycles = KIN1_GetCycleCounter();

    if (st != NEAI_OK)
    {
      NEAI_NoteError(st);
      return NEAI_PUSH_ERROR;
    }

    us = (cycles == 0U) ? 0.0f : NEAI_CyclesToUs((uint32_t)cycles);

    s_st.detect_calls++;
    s_st.last_similarity = sim;
    if (sim < s_st.min_similarity)
    {
      s_st.min_similarity = sim;
    }
    s_st.last_detect_us = us;
    if (us > s_st.max_detect_us)
    {
      s_st.max_detect_us = us;      /* 峰值耗时是面试/报告里的关键数字 */
    }

    if ((s_st.detect_calls % APP_NEAI_PRINT_DECIM) == 0U)
    {
      printf("[NEAI] sim=%3u  t=%4u.%01u us  (min=%3u max=%4u.%01u us)\r\n",
             (unsigned)sim,
             (unsigned)us, (unsigned)((us - (float)(unsigned)us) * 10.0f),
             (unsigned)s_st.min_similarity,
             (unsigned)s_st.max_detect_us,
             (unsigned)((s_st.max_detect_us - (float)(unsigned)s_st.max_detect_us) * 10.0f));
    }
    return NEAI_PUSH_DETECTED;
  }
}

void NEAI_AppPrintStats(void)
{
  printf("[NEAI] stat: init=%u learn=%u/%u detect=%lu sim=%u/min=%u t=%.1f/max%.1f us err=%lu(%lu)\r\n",
         (unsigned)s_st.initialized,
         (unsigned)s_st.learn_calls, (unsigned)APP_NEAI_LEARN_TARGET,
         (unsigned long)s_st.detect_calls,
         (unsigned)s_st.last_similarity, (unsigned)s_st.min_similarity,
         (double)s_st.last_detect_us, (double)s_st.max_detect_us,
         (unsigned long)s_st.last_error, (unsigned long)s_st.error_count);
}

const neai_app_stats_t *NEAI_AppStats(void)
{
  return &s_st;
}

#else  /* APP_NEAI_ENABLE == 0 ------------------------------------------------
        * 关掉 NanoEdge 时的空实现：保留同名接口，这样 main.c 的调用一行都不用改。
        * 用途：怀疑"是不是 AI 库引起的异常"时，用 -DAPP_NEAI_ENABLE=0 重编一版做对照。 */

void NEAI_AppReset(void)  { }

void NEAI_AppInit(void)
{
  printf("[NEAI] DISABLED at compile time (APP_NEAI_ENABLE=0)\r\n");
}

uint8_t NEAI_AppIsReady(void) { return 0U; }

uint8_t NEAI_AppPushSample(const float *axis_g)
{
  (void)axis_g;
  return NEAI_PUSH_NONE;
}

void NEAI_AppPrintStats(void) { }

const neai_app_stats_t *NEAI_AppStats(void) { return (const neai_app_stats_t *)0; }

#endif /* APP_NEAI_ENABLE */
