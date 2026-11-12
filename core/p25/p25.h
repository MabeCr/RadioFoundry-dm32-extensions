// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// P25 Phase 1 (C4FM, 4800 baud) frame basics: the frame sync word, the network ID (NID) with its BCH code, and the
// dibit mapping (docs/spec/p25.md). Bit level only: no samples, no RTOS, no board. The receiver that finds frames in
// audio is p25_rx.h.
#ifndef RFOUNDRY_P25_H
#define RFOUNDRY_P25_H

#include <stdint.h>
#include <stdbool.h>

#define P25_BAUD		4800u
#define P25_SYNC_BITS		0x5575F5FF77FFull	// 48 bits: 24 dibits, all of them +3 or -3
#define P25_SYNC_DIBITS		24u
#define P25_NID_DIBITS		32u			// 64 bits: 63 of BCH(63,16,23) and a parity bit
#define P25_STATUS_EVERY	36u			// every 36th dibit from the first sync dibit is a status symbol

// Data unit IDs (the NID's low four bits)
enum {
	P25_DUID_HDU	= 0x0,	// header, before a voice call
	P25_DUID_TDU	= 0x3,	// terminator
	P25_DUID_LDU1	= 0x5,	// voice
	P25_DUID_TSDU	= 0x7,	// trunking signalling: the control channel's frames
	P25_DUID_LDU2	= 0xA,	// voice
	P25_DUID_PDU	= 0xC,	// packet data
	P25_DUID_TDULC	= 0xF,	// terminator with link control
};

typedef struct {
	uint16_t nac;		// network access code, 12 bits
	uint8_t duid;		// 4 bits
	uint8_t errors;		// bits of the 63 the decoder corrected
} p25_nid_t;

// A dibit as the four C4FM levels: 01 = +3, 00 = +1, 10 = -1, 11 = -3
static inline int8_t p25_dibit_level(uint8_t d)
{
	static const int8_t lv[4] = { 1, 3, -1, -3 };

	return lv[d & 3];
}

// The 64 NID bits (bit 63 first on air) for a NAC and DUID. The parity bit is even over all 64 (unconfirmed on air:
// the receiver ignores it)
uint64_t p25_nid_encode(uint16_t nac, uint8_t duid);
// Nearest codeword of the 63 BCH bits (bits 63..1 of nid64), exact for up to 11 errors. False when none is that
// close or the DUID isn't one of the seven defined
bool p25_nid_decode(uint64_t nid64, p25_nid_t *out);

#endif
