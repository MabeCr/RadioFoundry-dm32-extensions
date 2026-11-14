// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// A synthetic P25 C4FM signal for the host tests: dibits to a raised cosine shaped waveform at any sample rate, timing
// offset, gain, DC offset and noise, and the frame layout (sync, NID, status symbols). Include once per test.
#ifndef P25_SYNTH_H
#define P25_SYNTH_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "p25/p25.h"

static uint32_t rng = 12345;
static inline uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng >> 8; }
static inline double gauss(void)
{
	double u = (rnd() + 1.0) / 16777217.0, v = (rnd() + 1.0) / 16777217.0;

	return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

static inline uint8_t p25_duid_pick(uint32_t r)
{
	static const uint8_t d[] = { P25_DUID_HDU, P25_DUID_TDU, P25_DUID_LDU1, P25_DUID_TSDU, P25_DUID_LDU2, P25_DUID_PDU,
				     P25_DUID_TDULC };

	return d[r % sizeof d];
}

// --- synthetic signal ---

#define SPS_MAX 12

// The transmit chain: dibits to levels, raised cosine (alpha 0.2) over +-4 symbols, sampled at `rate` with a timing offset
static inline size_t synth(const uint8_t *dib, size_t nd, uint32_t rate, double t0, double amp, double dc, double noise,
		    int16_t *out, size_t cap)
{
	size_t n = (size_t)((nd + 8) * rate / (double)P25_BAUD);
	const double alpha = 0.2;

	if (n > cap)
		n = cap;
	for (size_t i = 0; i < n; i++) {
		double t = (double)i * P25_BAUD / rate - t0;	// in symbols
		double v = 0;
		int c = (int)floor(t);

		for (int k = c - 4; k <= c + 5; k++) {
			if (k < 0 || (size_t)k >= nd)
				continue;
			double x = t - k, h;

			if (fabs(x) < 1e-9)
				h = 1;
			else {
				double den = 1 - pow(2 * alpha * x, 2);

				h = fabs(den) < 1e-9 ? M_PI / 4 * sin(M_PI * x) / (M_PI * x)
					: sin(M_PI * x) / (M_PI * x) * cos(M_PI * alpha * x) / den;
			}
			v += p25_dibit_level(dib[k]) * h;
		}
		v = v * amp + dc + noise * gauss();
		out[i] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
	}
	return n;
}

// One frame's dibits: sync, NID, then `extra` random dibits, status symbols at every 36th from the sync's first
static inline size_t frame_dibits_p(uint8_t *d, uint16_t nac, uint8_t duid, unsigned extra, const uint8_t *pay, unsigned npay)
{
	uint64_t nid = p25_nid_encode(nac, duid);
	size_t n = 0, pos = 0;
	uint8_t raw[24 + 32 + 400];
	size_t nr = 0;

	for (unsigned i = 0; i < P25_SYNC_DIBITS; i++)
		raw[nr++] = (P25_SYNC_BITS >> (46 - 2 * i)) & 3;
	for (unsigned i = 0; i < P25_NID_DIBITS; i++)
		raw[nr++] = (nid >> (62 - 2 * i)) & 3;
	for (unsigned i = 0; i < npay; i++)
		raw[nr++] = pay[i];
	for (unsigned i = 0; i < extra; i++)
		raw[nr++] = rnd() & 3;
	for (size_t i = 0; i < nr; i++, pos++) {
		if (pos % P25_STATUS_EVERY == P25_STATUS_EVERY - 1)
			d[n++] = rnd() & 3, pos++;
		d[n++] = raw[i];
	}
	return n;
}

static inline size_t frame_dibits(uint8_t *d, uint16_t nac, uint8_t duid, unsigned extra)
{
	return frame_dibits_p(d, nac, duid, extra, NULL, 0);
}


#endif
