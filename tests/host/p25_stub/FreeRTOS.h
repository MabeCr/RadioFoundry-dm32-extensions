// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// Host stand-in for FreeRTOS.h, for p25ListenTest only: the heap calls the P25 listener makes (the test counts them)
#ifndef P25_STUB_FREERTOS_H
#define P25_STUB_FREERTOS_H

#include <stddef.h>

void *pvPortMalloc(size_t size);
void vPortFree(void *p);

#endif
