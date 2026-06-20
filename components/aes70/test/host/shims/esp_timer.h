/* Host shim for esp_timer_get_time (monotonic microseconds). */
#pragma once
#include <stdint.h>

int64_t esp_timer_get_time(void);   /* defined in stubs.c */
