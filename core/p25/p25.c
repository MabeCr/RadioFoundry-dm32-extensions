// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// NID encoder and decoder (p25.h). The code is the narrow-sense binary BCH(63,16,23) over GF(64) (x^6+x+1) with
// roots alpha^1..alpha^22, systematic: 16 information bits (NAC then DUID) above 47 check bits. The generator was
// derived from that definition and its minimum distance, 23, checked by enumerating all 65536 codewords
// (tests/host/p25Test.c does both again). The decoder is a maximum likelihood search over those codewords in Gray
// code order, one XOR and one popcount each: it needs no tables and corrects exactly what the code can (11 bits).
#include "p25/p25.h"

#define BCH_GEN		0xCD930BDD3B2Bull	// degree 47

static uint64_t bch_rem(uint64_t v)	// v mod g, v up to 63 bits
{
	for (int i = 62; i >= 47; i--) {
		if ((v >> i) & 1)
			v ^= BCH_GEN << (i - 47);
	}
	return v;
}

// The 63-bit codeword of 16 information bits
static uint64_t bch_word(uint16_t info)
{
	uint64_t v = (uint64_t)info << 47;

	return v | bch_rem(v);
}

static int popcount64(uint64_t v)
{
	v = v - ((v >> 1) & 0x5555555555555555ull);
	v = (v & 0x3333333333333333ull) + ((v >> 2) & 0x3333333333333333ull);
	v = (v + (v >> 4)) & 0x0F0F0F0F0F0F0F0Full;
	return (int)((v * 0x0101010101010101ull) >> 56);
}

static bool duid_valid(uint8_t d)
{
	return d == P25_DUID_HDU || d == P25_DUID_TDU || d == P25_DUID_LDU1 || d == P25_DUID_TSDU ||
	       d == P25_DUID_LDU2 || d == P25_DUID_PDU || d == P25_DUID_TDULC;
}

uint64_t p25_nid_encode(uint16_t nac, uint8_t duid)
{
	uint64_t w = bch_word((uint16_t)(((nac & 0xFFFu) << 4) | (duid & 0xFu)));
	uint64_t nid = w << 1;

	if (popcount64(nid) & 1)
		nid |= 1;
	return nid;
}

bool p25_nid_decode(uint64_t nid64, p25_nid_t *out)
{
	const uint64_t rx = nid64 >> 1;
	uint64_t rows[16];

	for (int k = 0; k < 16; k++)
		rows[k] = bch_word((uint16_t)(1u << k));

	uint64_t cw = 0;
	uint32_t g = 0;		// the Gray code word: the information bits of cw
	int best = 64;
	uint16_t best_info = 0;

	for (uint32_t i = 0; i < 65536; i++) {
		if (i) {
			int k = __builtin_ctz(i);

			cw ^= rows[k];
			g ^= 1u << k;
		}
		int d = popcount64(cw ^ rx);

		if (d < best) {
			best = d;
			best_info = (uint16_t)g;
			if (d <= 11)
				break;	// codewords are 23 apart: within 11 of one, nothing else is that near
		}
	}
	if (best > 11 || !duid_valid(best_info & 0xF))
		return false;
	out->nac = best_info >> 4;
	out->duid = best_info & 0xF;
	out->errors = (uint8_t)best;
	return true;
}
