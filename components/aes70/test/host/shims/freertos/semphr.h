/* Host shim: recursive mutex as a no-op (tests are single-threaded). */
#pragma once
#include "FreeRTOS.h"

typedef void *SemaphoreHandle_t;

/* A non-NULL sentinel so creation-success checks pass. */
static int s_aes70_mutex_sentinel;

static inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void)
{
    return &s_aes70_mutex_sentinel;
}
static inline BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t s, TickType_t t)
{
    (void)s; (void)t; return pdTRUE;
}
static inline BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t s)
{
    (void)s; return pdTRUE;
}
static inline void vSemaphoreDelete(SemaphoreHandle_t s) { (void)s; }
