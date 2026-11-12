// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// P25 Phase 1 receiver front end (docs/spec/p25.md 3): finds frames in FM discriminator audio and reports each one's
// NAC and DUID. Input is the demodulated audio a radio's FM receiver gives the CPU (any rate from 7200 to 48000 Hz; the
// 4800 baud clock is derived from `rate`). Symbols are read at 8 sub-symbol timing phases
// at once with a cubic interpolator; the frame sync word (all outer levels, so a DC offset cancels) is correlated on
// every phase, and the best phase at a hit is kept for that frame's dibits, so no clock loop is needed: every frame
// brings its own sync. Integer only; no RTOS, no board.
#ifndef RFOUNDRY_P25_RX_H
#define RFOUNDRY_P25_RX_H

#include <stddef.h>
#include "p25/p25.h"
#include "p25/p25_block.h"

#define P25_RX_PHASES		8u

typedef struct {
	p25_nid_t nid;
	uint32_t sample;	// input samples consumed when the NID ended
	uint16_t score_q8;	// the sync correlation, 256 = a perfect match
	int16_t dc;		// the audio's offset at the sync, in input units (a transmit frequency error shows here)
	uint16_t amp;		// the outer level's size above that offset: the deviation scale
} p25_frame_t;

typedef void (*p25_frame_fn)(const p25_frame_t *f, void *ctx);
// One TSBK of a trunking signaling data unit, `index` counting from 0 within the unit
typedef void (*p25_tsbk_fn)(const p25_tsbk_t *t, uint8_t index, const p25_frame_t *f, void *ctx);

typedef struct {
	uint32_t rate;
	uint32_t step_q16;	// input samples per symbol, Q16
	uint32_t ibase, fbase;	// the current symbol period's start: whole samples and Q16 fraction
	uint32_t n;		// samples consumed
	int16_t x[16];		// the last 16 input samples (ring by n)
	uint32_t k;		// symbol periods done
	int16_t hist[P25_RX_PHASES][32];	// each phase's interpolated values (ring by k)
	// locked on a sync: collecting the NID's dibits at one phase
	bool locked;
	uint8_t phase;
	int16_t dc, amp;
	uint16_t score_q8;
	uint16_t got;		// dibits after the sync, status symbols included
	uint8_t nid_n;
	uint64_t nid;
	bool in_blocks;		// the NID said TSDU: collecting 98-dibit blocks
	uint8_t blk_n, blk_idx;
	uint8_t blk[P25_BLOCK_DIBITS];
	p25_frame_t cur;
	p25_frame_fn fn;
	void *ctx;
	p25_tsbk_fn tsbk_fn;
	void *tsbk_ctx;
	uint32_t syncs, nid_ok, nid_bad, tsbk_ok, tsbk_bad;
	uint32_t duid[16];	// frames by DUID
} p25_rx_t;

// False when the rate can't be handled (outside 7200..48000 Hz)
bool p25_rx_init(p25_rx_t *r, uint32_t rate, p25_frame_fn fn, void *ctx);
// Also decode the blocks of trunking signaling data units (a TSDU holds one to three)
void p25_rx_set_tsbk(p25_rx_t *r, p25_tsbk_fn fn, void *ctx);
void p25_rx_feed(p25_rx_t *r, const int16_t *s, size_t n);

#endif
