// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * Host test for the P25 listener (core/p25/p25_listen.c): a fake board audio tap plays a synthetic control channel
 * (trunking signaling frames) to the radio-task tick, and the counters, the latest frame and the block ring are read
 * back. The heap is counted so a leak across start and stop shows.
 *
 *   make -C tests/host run-p25listen
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "p25/p25_block.h"
#include "p25/p25_listen.h"
#include "p25_synth.h"

static int failures = 0;

#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static int live_allocs;

void *pvPortMalloc(size_t size)
{
	live_allocs++;
	return malloc(size);
}

void vPortFree(void *p)
{
	live_allocs--;
	free(p);
}

static int16_t pcm[200000];
static size_t pcm_n, pcm_pos;
static unsigned starts, stops;
static bool refuse_start;

static bool fake_start(void)
{
	if (refuse_start)
		return false;
	starts++;
	return true;
}

static void fake_stop(void)
{
	stops++;
}

// The board buffers; each poll hands over what has arrived, here up to 200 samples (25 ms at 8 kHz)
static uint16_t fake_read(int16_t *buf, uint16_t max)
{
	size_t n = pcm_n - pcm_pos;

	if (n > 200)
		n = 200;
	if (n > max)
		n = max;
	memcpy(buf, pcm + pcm_pos, n * sizeof(int16_t));
	pcm_pos += n;
	return (uint16_t)n;
}

static const hal_afsk_t fake_afsk = { .rx_rate_hz = 8000, .rx_start = fake_start, .rx_read = fake_read, .rx_stop = fake_stop };

const board_t board = { .name = "fake", .afsk = &fake_afsk };

// Three trunking signaling frames of one to three blocks, six blocks in all
static void make_signal(void)
{
	static uint8_t dib[6000];
	size_t nd = 0;
	uint8_t pay[300];

	for (unsigned i = 0; i < 40; i++)
		dib[nd++] = rnd() & 3;
	for (unsigned f = 0; f < 3; f++) {
		unsigned np = 0;

		for (unsigned b = 0; b <= f; b++) {
			uint8_t in[12] = { 0 };

			in[0] = (uint8_t)((b == f ? 0x80 : 0) | 0x3A);	// last block flag, RFSS status broadcast
			in[2] = (uint8_t)(0x10 + f);
			in[3] = (uint8_t)b;
			uint16_t c = p25_crc16(in, 10);

			in[10] = c >> 8;
			in[11] = c & 0xFF;
			p25_block_encode(in, pay + np);
			np += 98;
		}
		nd += frame_dibits_p(dib + nd, 0x293, P25_DUID_TSDU, 20, pay, np);
	}
	pcm_n = synth(dib, nd, 8000, 0.3, 6000, 0, 0, pcm, sizeof pcm / sizeof pcm[0]);
	pcm_pos = 0;
}

int main(void)
{
	p25_listen_stats_t s;
	p25_listen_tsbk_t b[8];
	uint32_t t = 1000;

	make_signal();
	p25_listen_tick(false, t);
	CHECK(!p25_listen_running() && starts == 0, "no listening without an FM channel");
	for (unsigned i = 0; i < 80; i++, t += 25) {
		p25_listen_tick(true, t);
		if (i == 0)
			CHECK(p25_listen_running() && starts == 1 && live_allocs == 1, "started");
	}
	p25_listen_stats(&s);
	CHECK(s.running && s.samples == pcm_n, "samples %u of %u", (unsigned)s.samples, (unsigned)pcm_n);
	CHECK(s.nid_ok == 3 && s.duid[P25_DUID_TSDU] == 3, "frames %u tsdu %u", (unsigned)s.nid_ok, (unsigned)s.duid[P25_DUID_TSDU]);
	CHECK(s.tsbk_ok == 6 && s.tsbk_bad == 0, "blocks %u good %u bad", (unsigned)s.tsbk_ok, (unsigned)s.tsbk_bad);
	CHECK(s.last_nac == 0x293 && s.last_duid == P25_DUID_TSDU, "last frame nac %03X duid %X", s.last_nac, s.last_duid);
	CHECK(s.syncs >= 3 && s.peak > 10000, "syncs %u peak %u", (unsigned)s.syncs, (unsigned)s.peak);

	uint8_t n = p25_listen_drain(b, 8);

	CHECK(n == 6, "ring holds %u", n);
	static const uint8_t first[6] = { 0, 0, 1, 0, 1, 2 };	// the second argument byte: the block's place in its frame
	static const uint8_t arg0[6] = { 0x10, 0x11, 0x11, 0x12, 0x12, 0x12 };

	for (unsigned i = 0; i < n && i < 6; i++) {
		CHECK(b[i].crc_ok && b[i].raw[0] == (uint8_t)((i == 0 || i == 2 || i == 5 ? 0x80 : 0) | 0x3A) && b[i].raw[2] == arg0[i] &&
		      b[i].raw[3] == first[i], "block %u: crc %u raw %02X %02X %02X", i, b[i].crc_ok, b[i].raw[0], b[i].raw[2], b[i].raw[3]);
	}
	CHECK(p25_listen_drain(b, 8) == 0, "drained");

	// leaving FM stops the audio, frees the receiver, and keeps the totals
	p25_listen_tick(false, t);
	p25_listen_stats(&s);
	CHECK(!p25_listen_running() && stops == 1 && live_allocs == 0, "stopped: running %d stops %u allocs %d",
	      p25_listen_running(), stops, live_allocs);
	CHECK(!s.running && s.tsbk_ok == 6 && s.nid_ok == 3, "totals kept: %u blocks %u frames", (unsigned)s.tsbk_ok, (unsigned)s.nid_ok);

	// a refused start is tried again a second later, not at every poll
	refuse_start = true;
	t += 100;
	p25_listen_tick(true, t);
	p25_listen_tick(true, t + 500);
	p25_listen_stats(&s);
	CHECK(s.start_fails == 1 && !s.running && live_allocs == 0, "refused: %u fails, allocs %d", (unsigned)s.start_fails, live_allocs);
	refuse_start = false;
	p25_listen_tick(true, t + 1100);
	CHECK(p25_listen_running() && starts == 2, "retried after the wait");
	p25_listen_enable(false);
	p25_listen_tick(true, t + 1200);
	CHECK(!p25_listen_running() && live_allocs == 0, "disabled");
	p25_listen_enable(true);

	if (failures)
		printf("# FAILED %d\n", failures);
	else
		printf("# PASSED p25ListenTest\n");
	return failures != 0;
}
