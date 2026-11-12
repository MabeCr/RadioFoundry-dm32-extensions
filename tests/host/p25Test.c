// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * Host tests for the P25 Phase 1 receiver core: the NID code and the sync/NID front end, against a synthetic C4FM
 * signal (raised cosine shaped, any sample rate, timing offset, noise, DC offset and gain).
 *
 *   make -C tests/host
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "p25/p25.h"
#include "p25/p25_rx.h"

static int failures = 0;

#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rng = 12345;
static uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng >> 8; }
static double gauss(void)
{
	double u = (rnd() + 1.0) / 16777217.0, v = (rnd() + 1.0) / 16777217.0;

	return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

static uint8_t p25_duid_pick(uint32_t r)
{
	static const uint8_t d[] = { P25_DUID_HDU, P25_DUID_TDU, P25_DUID_LDU1, P25_DUID_TSDU, P25_DUID_LDU2, P25_DUID_PDU,
				     P25_DUID_TDULC };

	return d[r % sizeof d];
}

static int popc(uint64_t v) { return __builtin_popcountll(v); }

static void test_nid_code(void)
{
	// every codeword is at least 23 bits from every other: spot check the weight of the encodings' differences
	int mind = 64;

	for (unsigned i = 0; i < 2000; i++) {
		uint64_t a = p25_nid_encode(rnd() & 0xFFF, p25_duid_pick(rnd()));
		uint64_t b = p25_nid_encode(rnd() & 0xFFF, p25_duid_pick(rnd()));

		if (a != b && popc(a ^ b) < mind)
			mind = popc(a ^ b);
	}
	CHECK(mind >= 22, "distance %d", mind);	// the 64th bit is an overall parity, the code itself has 23

	static const uint8_t duids[] = { P25_DUID_HDU, P25_DUID_TDU, P25_DUID_LDU1, P25_DUID_TSDU, P25_DUID_LDU2,
					 P25_DUID_PDU, P25_DUID_TDULC };

	for (unsigned t = 0; t < 400; t++) {
		uint16_t nac = rnd() & 0xFFF;
		uint8_t duid = duids[rnd() % sizeof(duids)];
		uint64_t w = p25_nid_encode(nac, duid);
		p25_nid_t n;

		CHECK(p25_nid_decode(w, &n) && n.nac == nac && n.duid == duid && n.errors == 0, "clean nac %03X", nac);
		for (unsigned e = 1; e <= 11; e++) {
			uint64_t bad = w;
			unsigned done = 0;

			while (done < e) {
				uint64_t bit = 1ull << (rnd() % 64);

				if (!(bad ^ w) || !((bad ^ w) & bit)) {	// distinct positions
					bad ^= bit;
					done++;
				}
			}
			memset(&n, 0, sizeof n);
			CHECK(p25_nid_decode(bad, &n) && n.nac == nac && n.duid == duid, "%u errors nac %03X duid %X", e, nac, duid);
		}
	}
	// ordinary random words are rejected: at most a sliver may land within reach of a codeword
	unsigned wrong = 0;

	for (unsigned t = 0; t < 200; t++) {
		p25_nid_t n;
		uint64_t r = ((uint64_t)rnd() << 40) ^ ((uint64_t)rnd() << 16) ^ rnd();

		if (p25_nid_decode(r, &n))
			wrong++;
	}
	CHECK(wrong < 20, "random words accepted: %u/200", wrong);
}

// --- synthetic signal ---

#define SPS_MAX 12

// The transmit chain: dibits to levels, raised cosine (alpha 0.2) over +-4 symbols, sampled at `rate` with a timing offset
static size_t synth(const uint8_t *dib, size_t nd, uint32_t rate, double t0, double amp, double dc, double noise,
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
static size_t frame_dibits(uint8_t *d, uint16_t nac, uint8_t duid, unsigned extra)
{
	uint64_t nid = p25_nid_encode(nac, duid);
	size_t n = 0, pos = 0;
	uint8_t raw[24 + 32 + 200];
	size_t nr = 0;

	for (unsigned i = 0; i < P25_SYNC_DIBITS; i++)
		raw[nr++] = (P25_SYNC_BITS >> (46 - 2 * i)) & 3;
	for (unsigned i = 0; i < P25_NID_DIBITS; i++)
		raw[nr++] = (nid >> (62 - 2 * i)) & 3;
	for (unsigned i = 0; i < extra; i++)
		raw[nr++] = rnd() & 3;
	for (size_t i = 0; i < nr; i++, pos++) {
		if (pos % P25_STATUS_EVERY == P25_STATUS_EVERY - 1)
			d[n++] = rnd() & 3, pos++;
		d[n++] = raw[i];
	}
	return n;
}

typedef struct { unsigned n; uint16_t nac[16]; uint8_t duid[16]; } got_t;

static void on_frame(const p25_frame_t *f, void *ctx)
{
	got_t *g = ctx;

	if (g->n < 16) {
		g->nac[g->n] = f->nid.nac;
		g->duid[g->n] = f->nid.duid;
	}
	g->n++;
}

static unsigned run_case(uint32_t rate, double t0, double amp, double dc, double noise, unsigned frames)
{
	static uint8_t dib[4000];
	static int16_t pcm[200000];
	size_t nd = 0;
	got_t g = { 0 };
	p25_rx_t rx;
	static const uint8_t seq[] = { P25_DUID_TSDU, P25_DUID_TSDU, P25_DUID_LDU1, P25_DUID_TDU };

	// leading silence of random dibits so the first sync isn't at the start
	for (unsigned i = 0; i < 40; i++)
		dib[nd++] = rnd() & 3;
	for (unsigned f = 0; f < frames; f++)
		nd += frame_dibits(dib + nd, 0x293, seq[f % 4], 60 + (rnd() % 40));
	size_t n = synth(dib, nd, rate, t0, amp, dc, noise, pcm, sizeof pcm / sizeof pcm[0]);

	if (!p25_rx_init(&rx, rate, on_frame, &g)) {
		CHECK(0, "init %u", rate);
		return 0;
	}
	// feed in odd sized chunks
	for (size_t i = 0; i < n;) {
		size_t c = 1 + rnd() % 97;

		if (c > n - i)
			c = n - i;
		p25_rx_feed(&rx, pcm + i, c);
		i += c;
	}
	unsigned ok = 0;

	for (unsigned i = 0; i < g.n && i < 16; i++)
		if (g.nac[i] == 0x293 && g.duid[i] == seq[i % 4])
			ok++;
	return ok;
}

static void test_rx(void)
{
	// timing: every offset across the symbol, at the radio's audio rate
	for (unsigned k = 0; k < 20; k++) {
		unsigned ok = run_case(8000, k / 20.0, 6000, 0, 0, 4);

		CHECK(ok == 4, "8 kHz offset %.2f: %u/4 frames", k / 20.0, ok);
	}
	// other rates
	static const uint32_t rates[] = { 7200, 9600, 16000, 24000, 48000 };

	for (unsigned r = 0; r < sizeof rates / sizeof rates[0]; r++) {
		unsigned ok = run_case(rates[r], 0.37, 6000, 0, 0, 4);

		CHECK(ok == 4, "%u Hz: %u/4", rates[r], ok);
	}
	// DC offset (a transmit frequency error) and a weak signal
	CHECK(run_case(8000, 0.3, 6000, 4000, 0, 4) == 4, "dc +4000");
	CHECK(run_case(8000, 0.3, 6000, -5000, 0, 4) == 4, "dc -5000");
	CHECK(run_case(8000, 0.3, 500, 0, 0, 4) == 4, "weak signal");
	// noise: outer level 18000 against a noise sigma of 2500 is about 14 dB per sample
	unsigned good = 0, total = 0;

	for (unsigned t = 0; t < 30; t++) {
		good += run_case(8000, (t % 10) / 10.0, 6000, 0, 2500, 4);
		total += 4;
	}
	CHECK(good * 100 >= total * 85, "noisy frames found: %u/%u", good, total);
	// no signal: noise alone finds nothing
	{
		static int16_t pcm[80000];
		got_t g = { 0 };
		p25_rx_t rx;

		for (unsigned i = 0; i < 80000; i++)
			pcm[i] = (int16_t)(3000 * gauss());
		p25_rx_init(&rx, 8000, on_frame, &g);
		p25_rx_feed(&rx, pcm, 80000);
		CHECK(g.n == 0, "frames from noise: %u (syncs %u)", g.n, (unsigned)rx.syncs);
	}
	p25_rx_t rx;

	CHECK(!p25_rx_init(&rx, 6000, NULL, NULL), "rate too low");
	CHECK(!p25_rx_init(&rx, 96000, NULL, NULL), "rate too high");
}

int main(void)
{
	test_nid_code();
	test_rx();
	if (failures)
		printf("# FAILED %d\n", failures);
	else
		printf("# PASSED p25Test\n");
	return failures != 0;
}
