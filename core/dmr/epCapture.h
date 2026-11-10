// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * A small ring buffer of the raw octets of encrypted DMR calls (PI headers and voice bursts, before any privacy
 * hook runs), drained over the debug link (DL_CMD_EP_CAPTURE, docs/spec/debuglink.md 3.25) for working out an
 * encryption scheme's recipe off the air on a bench computer. CONFIG_EP_CAPTURE, default off: this holds the
 * clear PI octets and the still-scrambled voice bits of a call someone else encrypted, so it stays opt-in and the
 * host link is the only way out.
 */

#ifndef RFOUNDRY_EPCAPTURE_H
#define RFOUNDRY_EPCAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dmr/call/dmr_call.h"	// DMR_PI_BYTES, DMR_VOICE_BYTES

// One entry: a PI header (voiceIndex 0xFF) or a voice burst's still-scrambled payload
typedef struct {
	uint32_t slot;		// the modem's slot counter when heard, 0 if the driver kept none
	uint8_t voiceIndex;	// 0-5 for a voice burst, 0xFF for a PI header
	uint8_t crcOk;
	uint8_t len;		// DMR_PI_BYTES or DMR_VOICE_BYTES
	uint8_t payload[DMR_VOICE_BYTES];
} epCaptureEntry_t;

#if CONFIG_EP_CAPTURE

// CONFIG_EP_CAPTURE_DEPTH entries (config/features.mk), each sizeof(epCaptureEntry_t) (≈34 bytes)

// Capture on/off. Turning it on clears the buffer and the counters (off by default: a reboot starts disabled)
void epCaptureEnable(bool on);
bool epCaptureEnabled(void);

// Called from the call stack (core/dmr/call/dmr_call.c) for every PI header and every voice burst of a call this
// radio can't decrypt. A no-op when disabled or the ring is full (the overflow counter still counts it)
void epCapturePi(const uint8_t pi[DMR_PI_BYTES], uint32_t slot, bool crcOk);
void epCaptureVoice(const uint8_t payload[DMR_VOICE_BYTES], uint8_t voiceIndex, uint32_t slot, bool crcOk);

// The entries currently held and how many were dropped for a full ring since the last reset/enable
void epCaptureInfo(uint16_t *count, uint16_t *overflow, uint16_t *capacity);

// Pop up to max entries (oldest first) into out; returns the number popped. Popped entries leave the ring
size_t epCaptureDrain(epCaptureEntry_t *out, size_t max);

// Clear the ring and the counters without changing whether capture is enabled
void epCaptureReset(void);

#else

static inline void epCaptureEnable(bool on) { (void)on; }
static inline bool epCaptureEnabled(void) { return false; }
static inline void epCapturePi(const uint8_t pi[DMR_PI_BYTES], uint32_t slot, bool crcOk)
{
	(void)pi;
	(void)slot;
	(void)crcOk;
}
static inline void epCaptureVoice(const uint8_t payload[DMR_VOICE_BYTES], uint8_t voiceIndex, uint32_t slot, bool crcOk)
{
	(void)payload;
	(void)voiceIndex;
	(void)slot;
	(void)crcOk;
}
static inline void epCaptureInfo(uint16_t *count, uint16_t *overflow, uint16_t *capacity)
{
	*count = 0;
	*overflow = 0;
	*capacity = 0;
}
static inline size_t epCaptureDrain(epCaptureEntry_t *out, size_t max)
{
	(void)out;
	(void)max;
	return 0;
}
static inline void epCaptureReset(void) {}

#endif

#endif
