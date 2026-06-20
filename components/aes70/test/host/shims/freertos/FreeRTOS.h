/*
 * Minimal FreeRTOS shim for HOST unit tests (single-threaded, no RTOS).
 * Only the surface the AES70 object model / router touches is provided; the
 * mutex and queue are no-op stand-ins because the tests drive the router
 * directly on one thread.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef int           BaseType_t;
typedef unsigned int  UBaseType_t;
typedef uint32_t      TickType_t;
typedef void         *TaskHandle_t;

#define pdTRUE        1
#define pdFALSE       0
#define pdPASS        1
#define portMAX_DELAY 0xffffffffu
#define tskNO_AFFINITY 0x7fffffff
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

static inline void vTaskDelay(TickType_t t) { (void)t; }
static inline void vTaskDelete(TaskHandle_t t) { (void)t; }
