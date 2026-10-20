/* ============================================================================
 * neai_app.h — NanoEdge AI 应用层（本项目自己的胶水层）
 * ============================================================================
 *
 * 职责：把"采样得到的三轴 g 值"喂给 NanoEdge AI 库，并维护 learn / detect 的
 *       编排、相似度输出、推理耗时统计。**不碰传感器、不碰串口硬件**。
 *
 * 设计要点（都是为了"换库时零改动"）：
 *   1. 缓冲区尺寸**直接取 `NanoEdgeAI.h` 里的宏**（`DATA_INPUT_USER` × `AXIS_NUMBER`），
 *      而不是写死数字 —— 将来用 Studio 生成我们自己的模型（3 轴 / 128 点）时，
 *      只需替换 `NanoEdgeAI.h` + `libneai.a` 这一对文件，本文件一行都不用改。
 *   2. 库返回的错误码**不忽略**：任何非 OK 都记进统计并打印，避免"静默不工作"。
 *   3. 多轴缓冲区的排列方式（按轴还是按点）**留成显式开关**，
 *      因为这一点只有 Studio 生成的三轴库才能验证，占位单轴库无法验证。
 *
 * 依赖：`NanoEdgeAI.h`（库接口，Studio 生成）、`cycle_dwt.h`（DWT 周期计数，测推理耗时）
 * 相关：`MVP/docs/context/RUNBOOK.md` R10（集成与换库步骤）、`DECISIONS.md` D18（AD 主线）
 * ============================================================================ */

#ifndef __NEAI_APP_H__
#define __NEAI_APP_H__

#include <stdint.h>
#include <stdbool.h>

/* 取库的尺寸宏 */
#include "NanoEdgeAI.h"

/* ---------------------------------------------------------------------------
 * 🔴 库宏的**跨版本适配**（2026-10-19 换真库时实测踩到，两个版本名字不同）
 *
 *   NanoEdge AI Studio 4.x：`AXIS_NUMBER` / `DATA_INPUT_USER`
 *                           + `MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING`
 *                           + `init(void)` / `set_sensitivity()`
 *   NanoEdge AI Studio 5.x：`NEAI_INPUT_AXIS_NUMBER` / `NEAI_INPUT_SIGNAL_LENGTH`
 *                           没有 MIN_ITERATION 宏 —— **改由 learn() 返回
 *                           `NEAI_LEARNING_DONE` 告知"学够了"**
 *                           `init(bool use_pretrained)`，且**没有 set_sensitivity**
 *
 * 两代各映射一次 ⇒ 换库仍然只换 `NanoEdgeAI.h` + `libneai.a` 两个文件，
 * 本层与 `main.c` 一行都不用改（这正是 `neai_app.h` 从第一天起的设计目标）。
 * ------------------------------------------------------------------------- */
#if defined(NEAI_INPUT_AXIS_NUMBER) && defined(NEAI_INPUT_SIGNAL_LENGTH)
#  define NEAI_LIB_AXES                 (NEAI_INPUT_AXIS_NUMBER)
#  define NEAI_LIB_WIN                  (NEAI_INPUT_SIGNAL_LENGTH)
#  define NEAI_LIB_TELLS_WHEN_DONE      (1)    /* 5.x：靠 learn() 返回值判断 */
#  define NEAI_INIT_ARGS()              ((bool)APP_NEAI_USE_PRETRAINED)
#else
#  define NEAI_LIB_AXES                 (AXIS_NUMBER)
#  define NEAI_LIB_WIN                  (DATA_INPUT_USER)
#  define NEAI_LIB_TELLS_WHEN_DONE      (0)    /* 4.x：只有宏，得自己数 */
#  define NEAI_INIT_ARGS()              /* 4.x 的 init 无参 */
#endif

/* 对外暴露"当前模型要几个轴" —— 这样 main.c 不必自己去 include 库头文件，
 * 也就不会有人不小心把尺寸写死成 3。 */
#define NEAI_APP_NUM_AXES               (NEAI_LIB_AXES)

/* ---------------------------------------------------------------------------
 * 编译期开关：0 = NanoEdge 完全退出（退化成纯采集工程，用于定位"到底是不是 AI 的锅"）
 *             1 = 启用（默认）
 * 用 #ifndef 包住，便于命令行 -DAPP_NEAI_ENABLE=0 覆盖做对照构建。 */
#ifndef APP_NEAI_ENABLE
#define APP_NEAI_ENABLE                 (1)
#endif

/* 学习阶段的**上限**（注意：不是"目标次数"了）
 *
 * 🔴 语义在换 5.x 库时变了（2026-10-19）：
 *    5.x 的 `learn()` 自己会返回 `NEAI_LEARNING_DONE` —— 库说学够了就学够了，
 *    **不再需要我们去猜"该学多少次"**（旧版的 MIN_ITERATION 宏在 5.x 里已不存在）。
 *    所以本宏降级为**防死循环的硬上限**。实测参考：本库（ICM，128 点 × 3 轴）
 *    在 **290 次**左右返回 DONE ⇒ 290 × 0.128 s ≈ **37 s** 上板学习时间。
 *    4.x 老库没有这个返回值，那时它仍是"目标次数"（`D18` 的 4~5 倍口径）。 */
#ifndef APP_NEAI_LEARN_TARGET
#define APP_NEAI_LEARN_TARGET           (1000U)
#endif

/* `init()` 的参数：true = 直接使用 Studio 训练好的模型（本项目就是要这个；
 * 因为我们训练集是 2780 窗真实数据，片上再学只是"补一点本机差异"）。
 * 设 0 才是从零开始学 —— 那需要喂几千窗，今晚不采用。 */
#ifndef APP_NEAI_USE_PRETRAINED
#define APP_NEAI_USE_PRETRAINED         (1)
#endif

/* 打印节流：每 N 次 learn / detect 事件才往串口打一行（1 = 每次都打）。
 * 🔴 为什么要这个开关：窗口只有 16 点时，1 kHz 采样下 detect() 每秒会被调用 62 次，
 *    每次打一行会把串口（115200，约 11.5 KB/s）塞满并挤掉采样节拍。
 *    换成我们自己的 128 点窗口后大约每秒 7.8 次，此时设回 1 也不吵。 */
#ifndef APP_NEAI_PRINT_DECIM
#define APP_NEAI_PRINT_DECIM            (1U)
#endif

/* ---------------------------------------------------------------------------
 * 多轴缓冲区排列方式 —— ⚠️ 这一项占位库验证不了，必须等三轴库到手后定死
 *
 *   1 = 按轴优先（axis-major）：[ax0..axN-1, ay0..ayN-1, az0..azN-1]
 *   0 = 按点交错（sample-interleaved）：[ax0,ay0,az0, ax1,ay1,az1, ...]
 *
 * 默认取 1，是 ST 多轴例程的惯用排列；**但请以 Studio 生成我们自己的库时
 * 随库给出的说明为准**，若不符只需把这个宏改成 0，不必改任何其它代码。
 * （单轴库 AXIS_NUMBER==1 时两种写法完全等价，所以现在改与不改都测不出差别。） */
#ifndef APP_NEAI_AXIS_MAJOR
#define APP_NEAI_AXIS_MAJOR             (1)
#endif

/* ---------------------------------------------------------------------------
 * 逐窗去直流 —— 🔴 必须与 PC 端 `tools/to_neai_csv.py` 的口径**完全一致**
 *
 *   1 = 去（默认；对应 to_neai_csv **不加** --keep-dc）
 *   0 = 不去（只在训练集是 `--keep-dc` 生成时才设 0）
 *
 * 为什么默认必须去（2026-10-18 实测 seg01）：重力 + 装夹角度带来
 *   ax≈0.589 g / ay≈−0.018 g / az≈0.152 g 的直流，
 *   而振动本身的 σ 只有 0.017 / 0.011 / 0.022 g —— **直流是信号的 27 倍**。
 * 训练集去了、推理不去 ⇒ 每一窗都带一个 27 倍量级的固定偏移，
 * 模型必然把"正常"全判成异常（反之亦然）。**这个错不会报任何错，只会让
 * 上板后相似度恒为 0**，所以做成编译开关而不是"记得手动同步"。
 *
 * 代价：连带滤掉 1/(窗口时长) ≈ 1/(128 ms) = 7.8 Hz 以下的成分。
 *   真机转频 68–75 Hz、主振动 142–150 Hz、3× 205–226 Hz，**都不受影响**；
 *   顺带抑制温漂与电池压降引起的基线漂移，对异常检测是好事。
 * ------------------------------------------------------------------------- */
#ifndef APP_NEAI_REMOVE_DC
#define APP_NEAI_REMOVE_DC              (1U)
#endif

/* `NEAI_AppPushSample()` 的返回值 —— 让调用方（main.c）决定打不打日志 */
#define NEAI_PUSH_NONE                  (0U)   /* 还没攒满一个窗口 */
#define NEAI_PUSH_LEARNED               (1U)   /* 刚完成一次 learn()  */
#define NEAI_PUSH_DETECTED              (2U)   /* 刚完成一次 detect() */
#define NEAI_PUSH_ERROR                 (3U)   /* 库返回了非 OK */

typedef struct
{
  uint8_t  initialized;      /* init() 是否成功 */
  uint8_t  learned_enough;   /* 学习次数是否已达标 */
  uint32_t learn_calls;      /* 累计 learn() 调用次数 */
  uint32_t detect_calls;     /* 累计 detect() 调用次数 */
  uint8_t  last_similarity;  /* 最近一次相似度 0~100，**越大越像"正常"** */
  uint8_t  min_similarity;   /* 观测到的最小相似度 */
  float    last_detect_us;   /* 最近一次推理耗时（µs，DWT 实测） */
  float    max_detect_us;    /* 推理耗时峰值（面试要讲的数字） */
  uint32_t last_error;       /* 最近一次非 OK 的返回码（0 = 一直是 OK） */
  uint32_t error_count;      /* 累计出错次数 */
} neai_app_stats_t;

/* 初始化：调 neai_anomalydetection_init()，开 DWT 周期计数，打印一行身份信息。
 * 注意：会打印 NEAI_ID —— 那就是"当前链的是哪个库"的铁证（也是防呆：别把占位库当自己的模型）。*/
void    NEAI_AppInit(void);

/* 清空采样缓冲与统计（重新学习时用） */
void    NEAI_AppReset(void);

/* 库是否已初始化成功 */
uint8_t NEAI_AppIsReady(void);

/* 喂一个采样点。axis_g 指向 AXIS_NUMBER 个 float（本项目实际给 3 个：ax, ay, az，单位 g）。
 * 返回值见上面的 NEAI_PUSH_* 宏。 */
uint8_t NEAI_AppPushSample(const float *axis_g);

/* 打一行统计（建议每秒调一次，作为"程序还活着"的心跳） */
void    NEAI_AppPrintStats(void);

/* 取统计快照（只读指针，用于 IDE Live Watch） */
const neai_app_stats_t *NEAI_AppStats(void);

#endif /* __NEAI_APP_H__ */
