/* Host shim: queue stand-in. The tests call the router directly rather than
 * going through the application set-request queue, so receive always reports
 * empty and send is accepted and dropped. */
#pragma once
#include "FreeRTOS.h"

typedef void *QueueHandle_t;

static int s_aes70_queue_sentinel;

static inline QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t item_size)
{
    (void)len; (void)item_size; return &s_aes70_queue_sentinel;
}
static inline BaseType_t xQueueReceive(QueueHandle_t q, void *buf, TickType_t t)
{
    (void)q; (void)buf; (void)t; return pdFALSE;       /* always empty */
}
static inline BaseType_t xQueueSend(QueueHandle_t q, const void *buf, TickType_t t)
{
    (void)q; (void)buf; (void)t; return pdTRUE;
}
static inline void vQueueDelete(QueueHandle_t q) { (void)q; }
