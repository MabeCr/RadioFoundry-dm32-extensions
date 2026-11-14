// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// P25 listening (CONFIG_P25; docs/spec/p25.md 6): feeds the board's FM receive audio (hal_afsk.h rx_*) to the P25
// front end from the radio task while the radio receives an FM channel, and keeps counters and the latest control
// channel blocks for the debug link. Receive only: nothing here transmits or plays audio.
#ifndef RFOUNDRY_P25_LISTEN_H
#define RFOUNDRY_P25_LISTEN_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#if CONFIG_P25

#define P25_LISTEN_TSBKS	12u

typedef struct {
	uint32_t ms;		// when it was decoded (radio task time)
	uint8_t crc_ok, errors;
	uint8_t raw[12];	// header, manufacturer ID, 8 argument bytes, the 2 CRC bytes: as the trellis decode gave them
} p25_listen_tsbk_t;

typedef struct {
	bool running;		// the audio is being read now
	uint32_t samples, peak;
	uint32_t syncs, nid_ok, nid_bad, tsbk_ok, tsbk_bad;
	uint32_t duid[16];
	uint32_t start_fails;
	uint16_t last_nac;	// the latest frame's
	uint8_t last_duid, last_errors;
	int16_t last_dc;
	uint16_t last_amp, last_score_q8;
	uint16_t tsbk_overflow;	// blocks lost because the debug link didn't drain the ring
} p25_listen_stats_t;

// On by default; a switch for a later setting
void p25_listen_enable(bool on);
// Radio task, every poll: fm_rx = an FM channel is being received (not transmitting, not a hotspot)
void p25_listen_tick(bool fm_rx, uint32_t now_ms);
bool p25_listen_running(void);
void p25_listen_stats(p25_listen_stats_t *out);
// The oldest blocks not yet read, up to max; returns how many
uint8_t p25_listen_drain(p25_listen_tsbk_t *out, uint8_t max);

#else

static inline bool p25_listen_running(void) { return false; }

#endif

#endif
