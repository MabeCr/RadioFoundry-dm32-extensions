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
#include "p25_synth.h"
#include "p25/p25_rx.h"
#include "p25/p25_block.h"
#include "p25/p25_tsbk.h"

static int failures = 0;

#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

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

// --- data blocks ---

static void test_block(void)
{
	// the interleave: spot values of the inverse permutation as p25.rs lists it, through the code's own round trip
	uint8_t in[12], out[98], back[12];

	for (unsigned t = 0; t < 200; t++) {
		for (unsigned i = 0; i < 10; i++)
			in[i] = rnd();
		uint16_t c = p25_crc16(in, 10);

		in[10] = c >> 8;
		in[11] = c & 0xFF;
		p25_block_encode(in, out);
		CHECK(p25_block_decode(out, back) == 0 && !memcmp(in, back, 12), "clean block %u", t);
		// scattered channel errors: two single bit errors far apart
		uint8_t bad[98];

		memcpy(bad, out, 98);
		for (unsigned e = 0; e < 2; e++)
			bad[e * 49 + rnd() % 40] ^= 1u << (rnd() & 1);
		p25_block_decode(bad, back);
		CHECK(!memcmp(in, back, 12), "2 scattered bit errors, block %u", t);
	}
	// a consecutive burst of 6 dibits is spread by the interleaver
	unsigned burst_ok = 0;

	for (unsigned t = 0; t < 100; t++) {
		for (unsigned i = 0; i < 12; i++)
			in[i] = rnd();
		p25_block_encode(in, out);
		unsigned at = rnd() % 90;

		for (unsigned i = 0; i < 6; i++)
			out[at + i] ^= 1u << (rnd() & 1);
		p25_block_decode(out, back);
		burst_ok += !memcmp(in, back, 12);
	}
	CHECK(burst_ok >= 95, "bursts of 6 bit errors in 6 dibits fixed: %u/100", burst_ok);
	// every encoding is a different dibit string at least 3 channel bits apart (the code's free distance is larger)
	uint8_t a[98], b[98];

	memset(in, 0, 12);
	p25_block_encode(in, a);
	for (unsigned i = 0; i < 48; i++) {
		memset(in, 0, 12);
		in[i / 4] = 1 << (6 - 2 * (i % 4));
		p25_block_encode(in, b);
		unsigned dist = 0;

		for (unsigned j = 0; j < 98; j++)
			dist += __builtin_popcount(a[j] ^ b[j]);
		CHECK(dist >= 5, "single dibit input %u: distance %u", i, dist);
	}
}

typedef struct { unsigned n; p25_tsbk_t t[8]; uint8_t idx[8]; } tsbk_got_t;

static void on_tsbk(const p25_tsbk_t *t, uint8_t index, const p25_frame_t *f, void *ctx)
{
	tsbk_got_t *g = ctx;

	if (g->n < 8) {
		g->t[g->n] = *t;
		g->idx[g->n] = index;
	}
	g->n++;
}

// Frames of one to three blocks each; returns the number of blocks found whole
static unsigned run_tsbk(uint32_t rate, double t0, double amp, double dc, double noise)
{
	static uint8_t dib[6000];
	static int16_t pcm[200000];
	size_t nd = 0;
	uint8_t pay[300], want[3][12];
	unsigned wantn = 0, nblk_total = 0;
	tsbk_got_t g = { 0 };
	p25_rx_t rx;

	for (unsigned i = 0; i < 40; i++)
		dib[nd++] = rnd() & 3;
	for (unsigned f = 0; f < 3; f++) {
		unsigned nb = 1 + f;
		unsigned np = 0;

		for (unsigned b = 0; b < nb; b++) {
			uint8_t in[12];

			for (unsigned i = 0; i < 12; i++)
				in[i] = rnd();
			in[0] = (in[0] & 0x7F) | (b == nb - 1 ? 0x80 : 0);
			uint16_t c = p25_crc16(in, 10);

			in[10] = c >> 8;
			in[11] = c & 0xFF;
			if (f == 2 && b == 0)
				memcpy(want[0], in, 12);	// kept for a check of the field split
			p25_block_encode(in, pay + np);
			np += 98;
			nblk_total++;
		}
		nd += frame_dibits_p(dib + nd, 0x293, P25_DUID_TSDU, 20, pay, np);
		(void)wantn;
	}
	size_t n = synth(dib, nd, rate, t0, amp, dc, noise, pcm, sizeof pcm / sizeof pcm[0]);

	p25_rx_init(&rx, rate, NULL, NULL);
	p25_rx_set_tsbk(&rx, on_tsbk, &g);
	for (size_t i = 0; i < n;) {
		size_t c = 1 + rnd() % 211;

		if (c > n - i)
			c = n - i;
		p25_rx_feed(&rx, pcm + i, c);
		i += c;
	}
	unsigned ok = 0;

	for (unsigned i = 0; i < g.n && i < 8; i++)
		if (g.t[i].crc_ok)
			ok++;
	// the third frame's first block: fields split as laid out
	if (g.n >= 6 && g.t[3].crc_ok) {
		CHECK(g.t[3].lb == (want[0][0] >> 7) && g.t[3].protect == ((want[0][0] >> 6) & 1) &&
		      g.t[3].opcode == (want[0][0] & 0x3F) && g.t[3].mfid == want[0][1] && !memcmp(g.t[3].args, want[0] + 2, 8),
		      "tsbk field split");
	}
	CHECK(g.n <= 6 || noise > 0, "too many blocks: %u", g.n);
	// indexes count within each unit: 0 | 0 1 | 0 1 2
	if (noise == 0 && g.n == 6) {
		static const uint8_t ix[6] = { 0, 0, 1, 0, 1, 2 };

		for (unsigned i = 0; i < 6; i++)
			CHECK(g.idx[i] == ix[i], "block index %u is %u", i, g.idx[i]);
	}
	(void)nblk_total;
	return ok;
}

static void test_tsbk_rx(void)
{
	for (unsigned k = 0; k < 10; k++) {
		unsigned ok = run_tsbk(8000, k / 10.0, 6000, 0, 0);

		CHECK(ok == 6, "tsbk offset %.1f: %u/6 blocks", k / 10.0, ok);
	}
	CHECK(run_tsbk(24000, 0.4, 6000, 3000, 0) == 6, "tsbk 24 kHz with offset");
	unsigned good = 0;

	for (unsigned t = 0; t < 20; t++)
		good += run_tsbk(8000, (t % 10) / 10.0, 6000, 0, 2000);
	CHECK(good >= 20 * 6 * 70 / 100, "noisy blocks %u/120", good);
}

// --- messages (field layouts and the example vectors are from p25.rs's tests, MIT) ---

static void msg_of(const uint8_t b[12], p25_msg_t *m)
{
	p25_tsbk_t t = { .lb = b[0] >> 7, .protect = (b[0] >> 6) & 1, .opcode = b[0] & 0x3F, .mfid = b[1], .crc_ok = 1 };

	memcpy(t.args, b + 2, 8);
	p25_msg_parse(&t, m);
}

static void test_msg(void)
{
	p25_msg_t m;

	// the standard's channel identifier example: base 851.00625 MHz, 6.25 kHz spacing, -45 MHz, channel 9 = 851.0625 MHz
	static const uint8_t iden[12] = { 0x3D, 0x00, 0x63, 0x22, 0xD0, 0x32, 0x0A, 0x25, 0x10, 0xA2, 0xFF, 0xFF };
	p25_iden_table_t tab = { 0 };
	uint32_t rx, tx;

	msg_of(iden, &m);
	CHECK(m.kind == P25_MSG_IDEN_UPDATE && m.u.iden.id == 6, "iden kind/id");
	CHECK(m.u.iden.iden.bandwidth_hz == 12500 && m.u.iden.iden.spacing_hz == 6250 && m.u.iden.iden.base_hz == 851006250 &&
	      m.u.iden.iden.tx_offset_hz == -45000000, "iden params %u %u %u %d", m.u.iden.iden.bandwidth_hz,
	      m.u.iden.iden.spacing_hz, m.u.iden.iden.base_hz, m.u.iden.iden.tx_offset_hz);
	p25_iden_update(&tab, &m);
	CHECK(!p25_chan_freq(&tab, (p25_chan_t){ 5, 9 }, &rx, &tx), "unknown identifier");
	CHECK(p25_chan_freq(&tab, (p25_chan_t){ 6, 9 }, &rx, &tx) && rx == 851062500 && tx == 806062500, "channel freq %u %u", rx, tx);

	static const uint8_t grant[12] = { 0x00, 0xFF, 0xA3, 0xE5, 0xCC, 0x18, 0xE7, 0xF0, 0x78, 0x3C, 0, 0 };

	msg_of(grant, &m);
	CHECK(m.kind == P25_MSG_OTHER, "vendor mfid is not decoded");
	{
		uint8_t g[12];

		memcpy(g, grant, 12);
		g[1] = 0;
		msg_of(g, &m);
		CHECK(m.kind == P25_MSG_GRP_VOICE_GRANT && m.u.grp_grant.opts == 0xA3 && m.u.grp_grant.chan.id == 0xE &&
		      m.u.grp_grant.chan.number == 0x5CC && m.u.grp_grant.tg == 0x18E7 && m.u.grp_grant.src == 0xF0783C,
		      "group voice grant");
	}
	static const uint8_t upd[12] = { 0x02, 0, 0x6F, 0x55, 0xFF, 0xFF, 0x9A, 0xAA, 0x33, 0xCC, 0, 0 };

	msg_of(upd, &m);
	CHECK(m.kind == P25_MSG_GRP_VOICE_UPDATE && m.u.grp_update.chan[0].id == 6 && m.u.grp_update.chan[0].number == 0xF55 &&
	      m.u.grp_update.tg[0] == 0xFFFF && m.u.grp_update.chan[1].id == 9 && m.u.grp_update.chan[1].number == 0xAAA &&
	      m.u.grp_update.tg[1] == 0x33CC, "group voice update");
	static const uint8_t adj[12] = { 0x3C, 0, 0xCC, 0xDF, 0x3C, 0xAA, 0x55, 0x36, 0x7E, 0x51, 0, 0 };

	msg_of(adj, &m);
	CHECK(m.kind == P25_MSG_ADJACENT_SITE && m.u.site.area == 0xCC && m.u.site.opts == 0xD && m.u.site.system == 0xF3C &&
	      m.u.site.rfss == 0xAA && m.u.site.site == 0x55 && m.u.site.chan.id == 3 && m.u.site.chan.number == 0x67E &&
	      m.u.site.svc == 0x51 && (m.u.site.svc & P25_SVC_DATA) && (m.u.site.svc & P25_SVC_REGISTRATION) &&
	      (m.u.site.svc & P25_SVC_COMPOSITE) && !(m.u.site.svc & P25_SVC_VOICE), "adjacent site");
	static const uint8_t net[12] = { 0x3B, 0, 0xCA, 0xFC, 0x2B, 0xCF, 0x5B, 0xDC, 0xE7, 0x51, 0, 0 };

	msg_of(net, &m);
	CHECK(m.kind == P25_MSG_NETWORK_STATUS && m.u.site.area == 0xCA && m.u.site.wacn == 0xFC2BC && m.u.site.system == 0xF5B &&
	      m.u.site.chan.id == 0xD && m.u.site.chan.number == 0xCE7, "network status");
	static const uint8_t reg[12] = { 0x2C, 0, 0xDA, 0xE7, 0xAA, 0x55, 0x33, 0xF8, 0x3F, 0xCC, 0, 0 };

	msg_of(reg, &m);
	CHECK(m.kind == P25_MSG_UNIT_REG_RESPONSE && m.u.unit_reg.resp == P25_REG_FAIL && m.u.unit_reg.system == 0xAE7 &&
	      m.u.unit_reg.src_id == 0xAA5533 && m.u.unit_reg.src_addr == 0xF83FCC, "unit registration response");
	static const uint8_t alt[12] = { 0x39, 0, 0xE3, 0x55, 0xB6, 0xAF, 0x51, 0xEA, 0xAA, 0xAE, 0, 0xFF };

	msg_of(alt, &m);
	CHECK(m.kind == P25_MSG_ALT_CONTROL_CHANNEL && m.u.alt_cc.rfss == 0xE3 && m.u.alt_cc.site == 0x55 &&
	      m.u.alt_cc.chan[0].id == 0xB && m.u.alt_cc.chan[0].number == 0x6AF && m.u.alt_cc.svc[0] == 0x51 &&
	      m.u.alt_cc.chan[1].id == 0xE && m.u.alt_cc.chan[1].number == 0xAAA && m.u.alt_cc.svc[1] == 0xAE, "alternate control channel");
	// a transmit offset with bit 8 set is above the receive frequency
	{
		uint8_t i2[12];

		memcpy(i2, iden, 12);
		i2[3] |= 0x04;
		msg_of(i2, &m);
		p25_iden_update(&tab, &m);
		CHECK(p25_chan_freq(&tab, (p25_chan_t){ 6, 9 }, &rx, &tx) && rx == 851062500 && tx == 896062500, "positive offset %u", tx);
	}
}

int main(void)
{
	test_nid_code();
	test_rx();
	test_block();
	test_tsbk_rx();
	test_msg();
	if (failures)
		printf("# FAILED %d\n", failures);
	else
		printf("# PASSED p25Test\n");
	return failures != 0;
}
