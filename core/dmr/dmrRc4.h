// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * RC4 (ARC4) stream cipher: the public algorithm, key schedule and keystream generator only
 */

#ifndef RFOUNDRY_DMRRC4_H
#define RFOUNDRY_DMRRC4_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
	uint8_t s[256];
	uint8_t i;
	uint8_t j;
} dmrRc4_t;

// Key schedule. keyLen is 1 to 256 octets
void dmrRc4Init(dmrRc4_t *c, const uint8_t *key, size_t keyLen);
// The next n keystream octets
void dmrRc4Keystream(dmrRc4_t *c, uint8_t *out, size_t n);
// Discard the next n keystream octets
void dmrRc4Skip(dmrRc4_t *c, size_t n);

#endif
