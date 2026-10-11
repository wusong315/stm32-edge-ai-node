/* ============================================================================
 * 🔴🔴🔴  占位文件（PLACEHOLDER）—— 不是我们自己的模型！  🔴🔴🔴
 * ============================================================================
 *
 * 本文件与同工程的 `Core/Src/libneai.a` **成对**，当前这一对是**借用**的：
 *
 *   来源 : ST 官方参考仓库 stm32ai-nanoedge-datalogger
 *          路径 Projects/NUCLEO-F401RE/NUCLEO-F401RE_HTS221/Core/{Inc/NanoEdgeAI.h, Src/libneai.a}
 *   身份 : 该例程给 **HTS221 温湿度传感器** 建的库
 *           →  NEAI_ID = "62e147cdec0a8efca99d1a8d"
 *           →  AXIS_NUMBER = 1、DATA_INPUT_USER = 16（单轴 16 点，与本项目无关）
 *
 * 借它的唯一目的：**在没有硬件、还没有我们自己的模型之前，
 * 先把「libneai.a 能被链接进 F407 工程 + API 调用能编译通过」这条链打通**，
 * 免得硬件到货后才在最要命的时候发现链接配置不对。
 *
 * ⚠️ 因此：
 *   · 这个库**不产生任何有意义的诊断结果**，绝不可用它去跑"异常检测"的结论；
 *   · `AXIS_NUMBER` / `DATA_INPUT_USER` / `NEAI_ID` **不是我们定的，是 Studio 定的**，
 *     将来必须由 NanoEdge AI Studio 训练出我们自己的模型后**整对替换**这两个文件；
 *   · 换库步骤见 `MVP/docs/context/RUNBOOK.md` R10（替换后重新编译，代码零改动）。
 *
 * ABI 兼容性已核实（2026-10-11）：该 .o 的属性为 v7E-M / Thumb-2 / VFPv4-D16 /
 * Tag_ABI_VFP_args = VFP registers（硬浮点），与本工程的
 * `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb` 逐项一致，可正常链接。
 * ============================================================================ */

/**
  ******************************************************************************
  * @file    NanoEdgeAI.h
  *
  * @brief   Header file of NanoEdgeAI library
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/

#ifndef __NANOEDGEAI_H__
#define __NANOEDGEAI_H__

/* Includes */
#include <stdint.h>

/* Define */
#define NEAI_ID "62e147cdec0a8efca99d1a8d"
#define AXIS_NUMBER 1
#define DATA_INPUT_USER 16
#define MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING 25

#ifndef __NEAI_STATE__
#define __NEAI_STATE__
enum neai_state {
    NEAI_OK = 0,
    NEAI_INIT_FCT_NOT_CALLED = 123,
    NEAI_BOARD_ERROR,
    NEAI_KNOWLEDGE_BUFFER_ERROR,
    NEAI_NOT_ENOUGH_CALL_TO_LEARNING, //This is a fail-safe to prevent users from learning one or even no signals.
    NEAI_MINIMAL_RECOMMENDED_LEARNING_DONE,
    NEAI_UNKNOWN_ERROR};
#endif

/* Function prototypes */
#ifdef __cplusplus
extern "C" {
#endif
	enum neai_state neai_anomalydetection_init(void);
	enum neai_state neai_anomalydetection_learn(float data_input[]);
	enum neai_state neai_anomalydetection_detect(float data_input[], uint8_t *similarity);
	enum neai_state neai_anomalydetection_set_sensitivity(float sensitivity);
	float neai_anomalydetection_get_sensitivity(void);
#ifdef __cplusplus
}
#endif

#endif

/* =============
Here some sample declaration added in your main program for the use of the NanoEdge AI library.
You can directly copy these declarations or modify the names.
* WARNING: respect the size of the buffer.

uint8_t similarity = 0; // Point to similarity (see argument of neai_anomalydetection_detect fct)
float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // Buffer of input values
*/
