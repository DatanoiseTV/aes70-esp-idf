/* Host shim: task API (transport task is stubbed out, so this is unused). */
#pragma once
#include "FreeRTOS.h"

typedef enum { eRunning = 0, eReady, eBlocked, eSuspended, eDeleted, eInvalid } eTaskState;

static inline BaseType_t xTaskCreatePinnedToCore(void (*fn)(void *), const char *name,
                                                 uint32_t stack, void *arg,
                                                 UBaseType_t prio, TaskHandle_t *out,
                                                 BaseType_t core)
{
    (void)fn; (void)name; (void)stack; (void)arg; (void)prio; (void)core;
    if (out) *out = NULL;
    return pdPASS;
}
static inline eTaskState eTaskGetState(TaskHandle_t t) { (void)t; return eDeleted; }
