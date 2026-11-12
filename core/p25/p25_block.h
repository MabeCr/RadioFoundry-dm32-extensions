// SPDX-License-Identifier: MIT AND LicenseRef-p25rs-MIT
// Copyright (c) 2026 The RadioFoundry Authors
// SPDX-FileCopyrightText: 2016-2018 Mick Koch (the interleave pattern and trellis table, from p25.rs, MIT, see LICENSES/LicenseRef-p25rs-MIT.txt)
// P25 unconfirmed data block (docs/spec/p25.md 4): the 98-dibit block a trunking signaling data unit carries, as 96 data
// bits through the rate 1/2 trellis code and the block interleaver, and the TSBK that fills it.
#ifndef RFOUNDRY_P25_BLOCK_H
#define RFOUNDRY_P25_BLOCK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define P25_BLOCK_DIBITS	98u
#define P25_BLOCK_BYTES		12u

typedef struct {
	uint8_t lb;		// last block of the data unit
	uint8_t protect;	// the protected flag
	uint8_t opcode;		// 6 bits
	uint8_t mfid;		// manufacturer ID, 0 = standard
	uint8_t args[8];
	uint8_t crc_ok;
	uint8_t errors;		// channel bits the trellis decoder had to change
} p25_tsbk_t;

// CRC-16 of a block's first 10 bytes: x^16 + x^12 + x^5 + 1, start 0, result complemented
uint16_t p25_crc16(const uint8_t *d, size_t n);

// 12 bytes to 98 channel dibits (trellis, then interleave); the decoder takes the channel dibits back to bytes and
// returns the bits it had to change to find the nearest trellis path
void p25_block_encode(const uint8_t in[P25_BLOCK_BYTES], uint8_t out[P25_BLOCK_DIBITS]);
uint8_t p25_block_decode(const uint8_t in[P25_BLOCK_DIBITS], uint8_t out[P25_BLOCK_BYTES]);

// Decode a block as a TSBK. False when the CRC fails (the fields are still filled in)
bool p25_tsbk_decode(const uint8_t dibits[P25_BLOCK_DIBITS], p25_tsbk_t *t);

#endif
