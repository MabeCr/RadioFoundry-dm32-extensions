// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// Host tests for the EP capture ring (core/dmr/epCapture.c): enable/disable, FIFO order, overflow counting, and that
// disabled calls are a no-op. Built with CONFIG_EP_CAPTURE=1 and a small CONFIG_EP_CAPTURE_DEPTH so overflow is easy
// to hit (see the Makefile rule)
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "dmr/epCapture.h"

static uint8_t pat(uint8_t base, uint8_t len)
{
	(void)len;
	return base;
}

static void fill(uint8_t *buf, uint8_t len, uint8_t base)
{
	for (uint8_t i = 0; i < len; i++) {
		buf[i] = (uint8_t)(pat(base, len) + i);
	}
}

static void test_disabled_is_noop(void)
{
	uint8_t pi[DMR_PI_BYTES];

	fill(pi, DMR_PI_BYTES, 1);
	epCaptureEnable(false);
	epCapturePi(pi, 10, true);

	uint16_t count, overflow, capacity;

	epCaptureInfo(&count, &overflow, &capacity);
	assert(count == 0);
	assert(overflow == 0);
	assert(capacity == CONFIG_EP_CAPTURE_DEPTH);
	printf("test_disabled_is_noop: ok\n");
}

static void test_enable_clears(void)
{
	uint8_t pi[DMR_PI_BYTES];

	fill(pi, DMR_PI_BYTES, 2);
	epCaptureEnable(true);
	epCapturePi(pi, 1, true);

	uint16_t count, overflow, capacity;

	epCaptureInfo(&count, &overflow, &capacity);
	assert(count == 1);
	epCaptureEnable(true);	// re-enabling clears the ring (fresh capture, not a running total)
	epCaptureInfo(&count, &overflow, &capacity);
	assert(count == 0);
	assert(overflow == 0);
	printf("test_enable_clears: ok\n");
}

static void test_fifo_order_and_fields(void)
{
	epCaptureEnable(true);

	uint8_t pi[DMR_PI_BYTES];
	uint8_t voice[DMR_VOICE_BYTES];

	fill(pi, DMR_PI_BYTES, 0x10);
	epCapturePi(pi, 100, true);
	fill(voice, DMR_VOICE_BYTES, 0x20);
	epCaptureVoice(voice, 3, 101, false);

	uint16_t count, overflow, capacity;

	epCaptureInfo(&count, &overflow, &capacity);
	assert(count == 2);
	assert(overflow == 0);

	epCaptureEntry_t out[4];
	size_t n = epCaptureDrain(out, 4);

	assert(n == 2);
	assert(out[0].voiceIndex == 0xFF);
	assert(out[0].len == DMR_PI_BYTES);
	assert(out[0].slot == 100);
	assert(out[0].crcOk == 1);
	assert(memcmp(out[0].payload, pi, DMR_PI_BYTES) == 0);
	assert(out[1].voiceIndex == 3);
	assert(out[1].len == DMR_VOICE_BYTES);
	assert(out[1].slot == 101);
	assert(out[1].crcOk == 0);
	assert(memcmp(out[1].payload, voice, DMR_VOICE_BYTES) == 0);

	epCaptureInfo(&count, &overflow, &capacity);
	assert(count == 0);	// drained entries leave the ring
	printf("test_fifo_order_and_fields: ok\n");
}

static void test_overflow_keeps_oldest(void)
{
	epCaptureEnable(true);

	uint8_t voice[DMR_VOICE_BYTES];

	// one more than the ring holds (CONFIG_EP_CAPTURE_DEPTH, built small for this test)
	for (uint16_t i = 0; i < CONFIG_EP_CAPTURE_DEPTH + 1; i++) {
		fill(voice, DMR_VOICE_BYTES, (uint8_t)i);
		epCaptureVoice(voice, (uint8_t)(i % 6), i, true);
	}

	uint16_t count, overflow, capacity;

	epCaptureInfo(&count, &overflow, &capacity);
	assert(count == CONFIG_EP_CAPTURE_DEPTH);
	assert(overflow == 1);

	epCaptureEntry_t first;

	assert(epCaptureDrain(&first, 1) == 1);
	assert(first.slot == 0);	// the oldest frame survived; the newest one (dropped) did not displace it
	printf("test_overflow_keeps_oldest: ok\n");
}

static void test_partial_drain(void)
{
	epCaptureEnable(true);

	uint8_t voice[DMR_VOICE_BYTES];

	for (uint16_t i = 0; i < 3; i++) {
		fill(voice, DMR_VOICE_BYTES, (uint8_t)i);
		epCaptureVoice(voice, (uint8_t)i, i, true);
	}

	epCaptureEntry_t out[1];

	assert(epCaptureDrain(out, 1) == 1);
	assert(out[0].slot == 0);

	uint16_t count, overflow, capacity;

	epCaptureInfo(&count, &overflow, &capacity);
	assert(count == 2);

	epCaptureEntry_t rest[4];
	size_t n = epCaptureDrain(rest, 4);	// asking for more than remain is fine

	assert(n == 2);
	assert(rest[0].slot == 1);
	assert(rest[1].slot == 2);
	printf("test_partial_drain: ok\n");
}

int main(void)
{
	test_disabled_is_noop();
	test_enable_clears();
	test_fifo_order_and_fields();
	test_overflow_keeps_oldest();
	test_partial_drain();
	printf("All epCaptureTest tests passed!\n");
	return 0;
}
