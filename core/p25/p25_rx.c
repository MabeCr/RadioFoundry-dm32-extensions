// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// P25 receiver front end (p25_rx.h)
#include "p25/p25_rx.h"

#define SCORE_MIN_Q8	192	// 0.75 of a perfect sync fit (residual under a quarter of the outer level)

bool p25_rx_init(p25_rx_t *r, uint32_t rate, p25_frame_fn fn, void *ctx)
{
	if (rate < 7200 || rate > 48000)
		return false;
	*r = (p25_rx_t){ .rate = rate, .fn = fn, .ctx = ctx };
	r->step_q16 = (uint32_t)(((uint64_t)rate << 16) / P25_BAUD);
	return true;
}

// Catmull-Rom between a1 and a2, mu in Q16 (0..65535)
static int32_t interp(int32_t a0, int32_t a1, int32_t a2, int32_t a3, uint32_t mu)
{
	int64_t m = mu;
	int64_t c3 = 3 * (a1 - a2) + a3 - a0;
	int64_t c2 = 2 * a0 - 5 * a1 + 4 * a2 - a3;
	int64_t c1 = a2 - a0;
	int64_t t = (c3 * m) >> 16;

	t = ((t + c2) * m) >> 16;
	t = ((t + c1) * m) >> 16;
	return (int32_t)(a1 + (t >> 1));
}

static int sync_sign(unsigned i)	// sync dibit i (0 first) as +1 / -1
{
	return ((P25_SYNC_BITS >> (46 - 2 * i)) & 3) == 1 ? 1 : -1;	// 01 = +3, 11 = -3
}

static uint8_t slice(int32_t v, int32_t dc, int32_t amp)
{
	int32_t x = v - dc;
	int32_t th = amp * 2 / 3;

	if (x > th)
		return 1;	// +3
	if (x >= 0)
		return 0;	// +1
	if (x >= -th)
		return 2;	// -1
	return 3;		// -3
}

// Fit the sync's +-1 pattern to one phase's last 24 values: v = dc + amp * s. The sync is not balanced, so the offset
// is fitted with the scale rather than taken as the mean. Returns the fit in Q8 (256 = exact, from the residual
// against the outer level), 0 when the scale isn't positive
static int sync_score(const p25_rx_t *r, unsigned ph, int16_t *dc, int16_t *amp)
{
	const int32_t n = (int32_t)P25_SYNC_DIBITS;
	int32_t sv = 0, ss = 0;
	int64_t svs = 0;
	int v[P25_SYNC_DIBITS];

	for (unsigned i = 0; i < P25_SYNC_DIBITS; i++) {
		v[i] = r->hist[ph][(r->k - P25_SYNC_DIBITS + i) & 31];
		sv += v[i];
		ss += sync_sign(i);
		svs += sync_sign(i) * v[i];
	}
	int64_t a = ((svs * n - (int64_t)ss * sv)) / ((int64_t)n * n - (int64_t)ss * ss);

	if (a <= 0)
		return 0;
	int64_t o = ((int64_t)sv - a * ss) / n;
	int64_t res = 0;

	for (unsigned i = 0; i < P25_SYNC_DIBITS; i++) {
		int64_t e = v[i] - o - a * sync_sign(i);

		res += e < 0 ? -e : e;
	}
	*dc = (int16_t)o;
	*amp = (int16_t)(a > 32767 ? 32767 : a);
	int64_t sc = 256 - (res * 256) / (a * n);

	return sc < 0 ? 0 : (int)sc;
}

static void symbol_period(p25_rx_t *r)
{
	// the P phases of this period, then advance the period's start by one symbol
	for (unsigned j = 0; j < P25_RX_PHASES; j++) {
		uint32_t f = r->fbase + (uint32_t)(((uint64_t)r->step_q16 * j / P25_RX_PHASES));
		uint32_t i = r->ibase + (f >> 16);
		const int16_t *x = r->x;

		f &= 0xFFFF;
		r->hist[j][r->k & 31] = (int16_t)interp(x[(i - 1) & 15], x[i & 15], x[(i + 1) & 15], x[(i + 2) & 15], f);
	}
	r->k++;
	r->fbase += r->step_q16;
	r->ibase += r->fbase >> 16;
	r->fbase &= 0xFFFF;
	if (r->k < 32)
		return;

	if (r->locked) {
		uint8_t d = slice(r->hist[r->phase][(r->k - 1) & 31], r->dc, r->amp);

		// the 36th dibit from the sync's first is a status symbol: dibit 35 is NID dibit 11 + 24 sync dibits
		if (r->got != P25_STATUS_EVERY - P25_SYNC_DIBITS - 1) {
			r->nid = (r->nid << 2) | d;
		}
		r->got++;
		if (r->got == P25_NID_DIBITS + 1) {
			p25_frame_t f;

			r->locked = false;
			if (p25_nid_decode(r->nid, &f.nid)) {
				r->nid_ok++;
				r->duid[f.nid.duid]++;
				f.sample = r->n;
				f.score_q8 = r->score_q8;
				f.dc = r->dc;
				f.amp = (uint16_t)r->amp;
				if (r->fn)
					r->fn(&f, r->ctx);
			} else {
				r->nid_bad++;
			}
		}
		return;
	}

	// search: the best phase by correlation, kept only above the threshold
	int best = 0;
	unsigned bj = 0;
	int16_t bdc = 0, bamp = 0;

	for (unsigned j = 0; j < P25_RX_PHASES; j++) {
		int16_t dc, amp;
		int s = sync_score(r, j, &dc, &amp);

		if (s > best) {
			best = s;
			bj = j;
			bdc = dc;
			bamp = amp;
		}
	}
	if (best >= SCORE_MIN_Q8) {
		r->locked = true;
		r->phase = (uint8_t)bj;
		r->dc = bdc;
		r->amp = bamp;
		r->score_q8 = (uint16_t)best;
		r->got = 0;
		r->nid = 0;
		r->syncs++;
	}
}

void p25_rx_feed(p25_rx_t *r, const int16_t *s, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		r->x[r->n & 15] = s[i];
		r->n++;
		// a period reads up to a symbol's worth of samples past ibase, plus the interpolator's two
		while (r->n >= r->ibase + (r->step_q16 >> 16) + 4)
			symbol_period(r);
	}
}
