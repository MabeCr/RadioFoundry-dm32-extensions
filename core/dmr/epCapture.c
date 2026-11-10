// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * EP capture ring (epCapture.h). A plain array used as a ring buffer; no locking because every call comes from the
 * radio task, the same task that drives the debug link.
 */
#include "config.h"

#if CONFIG_EP_CAPTURE

#include <string.h>

#include "dmr/epCapture.h"

#ifndef CONFIG_EP_CAPTURE_DEPTH
#define CONFIG_EP_CAPTURE_DEPTH 24
#endif

static struct {
	epCaptureEntry_t ring[CONFIG_EP_CAPTURE_DEPTH];
	uint16_t head;		// next slot to fill
	uint16_t count;
	uint16_t overflow;
	bool on;
} s;

void epCaptureEnable(bool on)
{
	s.on = on;
	epCaptureReset();
}

bool epCaptureEnabled(void)
{
	return s.on;
}

static void push(const uint8_t *payload, uint8_t len, uint8_t voiceIndex, uint32_t slot, bool crcOk)
{
	if (!s.on) {
		return;
	}
	if (s.count == CONFIG_EP_CAPTURE_DEPTH) {
		s.overflow++;	// keep the oldest frames (the PI/IV material matters most): drop the new one
		return;
	}
	epCaptureEntry_t *e = &s.ring[(s.head + s.count) % CONFIG_EP_CAPTURE_DEPTH];

	e->slot = slot;
	e->voiceIndex = voiceIndex;
	e->crcOk = crcOk ? 1 : 0;
	e->len = len;
	memcpy(e->payload, payload, len);
	s.count++;
}

void epCapturePi(const uint8_t pi[DMR_PI_BYTES], uint32_t slot, bool crcOk)
{
	push(pi, DMR_PI_BYTES, 0xFF, slot, crcOk);
}

void epCaptureVoice(const uint8_t payload[DMR_VOICE_BYTES], uint8_t voiceIndex, uint32_t slot, bool crcOk)
{
	push(payload, DMR_VOICE_BYTES, voiceIndex, slot, crcOk);
}

void epCaptureInfo(uint16_t *count, uint16_t *overflow, uint16_t *capacity)
{
	*count = s.count;
	*overflow = s.overflow;
	*capacity = CONFIG_EP_CAPTURE_DEPTH;
}

size_t epCaptureDrain(epCaptureEntry_t *out, size_t max)
{
	size_t n = max < s.count ? max : s.count;

	for (size_t k = 0; k < n; k++) {
		out[k] = s.ring[(s.head + k) % CONFIG_EP_CAPTURE_DEPTH];
	}
	s.head = (uint16_t)((s.head + n) % CONFIG_EP_CAPTURE_DEPTH);
	s.count = (uint16_t)(s.count - n);
	return n;
}

void epCaptureReset(void)
{
	s.head = 0;
	s.count = 0;
	s.overflow = 0;
}

#endif
