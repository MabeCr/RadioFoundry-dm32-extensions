// SPDX-License-Identifier: MIT AND LicenseRef-p25rs-MIT
// Copyright (c) 2026 The RadioFoundry Authors
// SPDX-FileCopyrightText: 2016-2018 Mick Koch (the interleave pattern and trellis table, from p25.rs, MIT, see LICENSES/LicenseRef-p25rs-MIT.txt)
// P25 data block coding (p25_block.h)
#include <string.h>
#include "p25/p25_block.h"

// Output dibit pair (first dibit in the high two bits) of the trellis for each state and next state. The input dibit
// is the next state, so the state is the last input; 49 inputs (the 48 data dibits and a 00 flush) give 98 dibits
static const uint8_t PAIR[4][4] = {
	{ 2, 12, 1, 15 },
	{ 14, 0, 13, 3 },
	{ 9, 7, 10, 4 },
	{ 5, 11, 6, 8 },
};

#define STAGES	49u

// The transmit interleave: output dibit i is trellis dibit redirect(i). Dibit pairs go out column by column of a
// 4 pair wide array (pair q of the trellis is at column q % 4, row q / 4): 13 pairs of column 0, 12 of each other
static unsigned redirect(unsigned i)
{
	unsigned q = i / 2, c, k;

	if (q < 13) {
		c = 0;
		k = q;
	} else {
		c = 1 + (q - 13) / 12;
		k = (q - 13) % 12;
	}
	return 2 * (c + 4 * k) + (i & 1);
}

uint16_t p25_crc16(const uint8_t *d, size_t n)
{
	uint16_t crc = 0;

	for (size_t i = 0; i < n; i++) {
		crc ^= (uint16_t)d[i] << 8;
		for (unsigned b = 0; b < 8; b++)
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return (uint16_t)~crc;
}

void p25_block_encode(const uint8_t in[P25_BLOCK_BYTES], uint8_t out[P25_BLOCK_DIBITS])
{
	uint8_t t[P25_BLOCK_DIBITS];
	unsigned state = 0;

	for (unsigned i = 0; i < STAGES; i++) {
		unsigned x = i < 48 ? (in[i / 4] >> (6 - 2 * (i % 4))) & 3 : 0;
		uint8_t p = PAIR[state][x];

		t[2 * i] = p >> 2;
		t[2 * i + 1] = p & 3;
		state = x;
	}
	for (unsigned i = 0; i < P25_BLOCK_DIBITS; i++)
		out[i] = t[redirect(i)];
}

uint8_t p25_block_decode(const uint8_t in[P25_BLOCK_DIBITS], uint8_t out[P25_BLOCK_BYTES])
{
	uint8_t t[P25_BLOCK_DIBITS];
	uint8_t from[STAGES][4];
	uint16_t pm[4] = { 0, 0xFFF, 0xFFF, 0xFFF };

	for (unsigned j = 0; j < P25_BLOCK_DIBITS; j++)
		t[redirect(j)] = in[j] & 3;
	for (unsigned s = 0; s < STAGES; s++) {
		unsigned r = (unsigned)(t[2 * s] << 2 | t[2 * s + 1]);
		uint16_t np[4];

		for (unsigned n = 0; n < 4; n++) {
			unsigned bc = 0xFFFF, bm = 0xFFFF;

			for (unsigned c = 0; c < 4; c++) {
				unsigned m = pm[c] + (unsigned)__builtin_popcount(PAIR[c][n] ^ r);

				if (m < bm) {
					bm = m;
					bc = c;
				}
			}
			np[n] = (uint16_t)bm;
			from[s][n] = (uint8_t)bc;
		}
		memcpy(pm, np, sizeof pm);
	}
	// the flush ends in state 0: trace back from there
	uint8_t dib[STAGES];
	unsigned state = 0;

	for (unsigned s = STAGES; s-- > 0;) {
		dib[s] = (uint8_t)state;	// the input dibit of this stage is the state it ended in
		state = from[s][state];
	}
	memset(out, 0, P25_BLOCK_BYTES);
	for (unsigned i = 0; i < 48; i++)
		out[i / 4] |= (uint8_t)(dib[i] << (6 - 2 * (i % 4)));
	return (uint8_t)(pm[0] > 255 ? 255 : pm[0]);
}

bool p25_tsbk_decode(const uint8_t dibits[P25_BLOCK_DIBITS], p25_tsbk_t *t)
{
	uint8_t b[P25_BLOCK_BYTES];

	t->errors = p25_block_decode(dibits, b);
	t->lb = b[0] >> 7;
	t->protect = (b[0] >> 6) & 1;
	t->opcode = b[0] & 0x3F;
	t->mfid = b[1];
	memcpy(t->args, b + 2, 8);
	t->crc = (uint16_t)((b[10] << 8) | b[11]);
	t->crc_ok = p25_crc16(b, 10) == t->crc;
	return t->crc_ok;
}
