/* =============
Copyright (c) 2026, STMicroelectronics

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions
  and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of
  conditions and the following disclaimer in the documentation and/or other materials provided
  with the distribution.

* Neither the name of the copyright holders nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific prior written
  permission.

*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER /
 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.*
*/


#ifndef NANOEDGEAI_H
#define NANOEDGEAI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* NEAI ID */
#define NEAI_ID "6ad5fc07c9f44b852848376b"

/* Input signal configuration */
#define NEAI_INPUT_SIGNAL_LENGTH 128
#define NEAI_INPUT_AXIS_NUMBER 3

/* NEAI State Enum */
enum neai_state {
	NEAI_OK = 0,
	NEAI_ERROR = 1,
	NEAI_NOT_INITIALIZED = 2,
	NEAI_INVALID_PARAM = 3,
	NEAI_NOT_SUPPORTED = 4,
	NEAI_LEARNING_DONE = 5,
	NEAI_LEARNING_IN_PROGRESS = 6
};


#ifdef __cplusplus
extern "C" {
#endif

/* ===== Anomaly Detection API ===== */
/**
 * @brief  Must be called at the beginning to initialize the anomaly detection model.
 * @param  use_pretrained [in]  Set to true to use a pretrained model, false to start learning
 *                              from scratch (pretrained model comes from NanoEdge AI Studio
 *                              and is embedded in the library).
 * @return NEAI_OK on success, error code otherwise.
 */
enum neai_state neai_anomalydetection_init(bool use_pretrained);

/**
 * @brief  Learn patterns from a new input sample.
 *         It can be called at any time after initialization.
 *         At the beginning, multiple calls to build the knowledge base of the model.
 *         Later as an additional learning step to complement the knowledge base.
 * @param  in [in]  Pointer to the input signal array
 *                  (size NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER).
 * @return NEAI_LEARNING_DONE when minimum learning calls are reached.
 *         NEAI_LEARNING_IN_PROGRESS if more learning calls are needed.
 *         Error code otherwise.
 */
enum neai_state neai_anomalydetection_learn(float *in);

/**
 * @brief  Perform anomaly detection on a new input sample by returning a similarity percentage.
 *         The mathematical distance between the incoming sample and the learned patterns.
 * @param  in         [in]   Pointer to the input signal array
 *                           (size NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER).
 * @param  similarity [out]  Pointer to the similarity percentage [0-100]
 *                           (100 means completely similar, 0 means completely different).
 * @return NEAI_OK on success.
 *         NEAI_LEARNING_IN_PROGRESS if the model needs more learning calls (minimum learning
 *                                   calls not reached).
 *         Error code otherwise.
 */
enum neai_state neai_anomalydetection_detect(float *in, uint8_t *similarity);

/**
 * @brief  Retrieve a pointer to the current learned knowledge and its size in bytes.
 *         The pointer is valid until the next call to neai_anomalydetection_init,
 *         neai_anomalydetection_learn or neai_anomalydetection_set_knowledge.
 *         The caller may copy the bytes to non-volatile storage and later restore them
 *         via neai_anomalydetection_set_knowledge.
 *         The knowledge blob is not portable across different builds (its size and
 *         layout depend on the model hyperparameters fixed at build time).
 * @param  knowledge_ptr  [out]  Pointer that will receive the address of the knowledge buffer.
 * @param  knowledge_size [out]  Pointer that will receive the size of the knowledge buffer (bytes).
 * @return NEAI_OK on success.
 *         NEAI_NOT_INITIALIZED if the model is not initialized.
 *         NEAI_INVALID_PARAM if either argument is NULL.
 */
enum neai_state neai_anomalydetection_get_knowledge(void **knowledge_ptr, size_t *knowledge_size);

/**
 * @brief  Restore previously saved knowledge bytes into the running model.
 *         The model must be initialized first; the existing knowledge will be overwritten.
 * @param  knowledge_ptr  [in]  Pointer to the bytes obtained from a prior get_knowledge call.
 * @param  knowledge_size [in]  Size of the bytes (must match the value returned by get_knowledge).
 * @return NEAI_OK on success.
 *         NEAI_NOT_INITIALIZED if the model is not initialized.
 *         NEAI_INVALID_PARAM if knowledge_ptr is NULL or knowledge_size does not match.
 */
enum neai_state neai_anomalydetection_set_knowledge(const void *knowledge_ptr, size_t knowledge_size);

/* ===== Common getter functions ===== */
/**
 * @brief  Get the NEAI identifier.
 * @return Pointer to a string containing the NEAI ID.
 */
char* neai_get_id(void);

/**
 * @brief  Get the input signal size (number of samples per axis).
 * @return Input signal size.
 */
int neai_get_input_signal_size(void);

/**
 * @brief  Get the number of input axes/channels.
 * @return Number of input axes.
 */
int neai_get_axis_number(void);


#ifdef __cplusplus
}
#endif

#endif /* NANOEDGEAI_H */


/* =============
Declarations to add to your main program to use the NanoEdge AI library.
You may copy-paste them directly or rename variables as needed.
WARNING: Respect the structures, types, and buffer sizes; only variable names may be changed.

enum neai_state state;   // Captures return states from NEAI functions
bool use_pretrained = false;   // Init function parameter: true = use pretrained model, false = learn from scratch
uint8_t similarity;   // Similarity percentage returned by detect function
float input_signal[NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER];   // Input signal buffer
============= */
