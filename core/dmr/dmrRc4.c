// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * RC4 (ARC4) stream cipher
 */

#include "dmr/dmrRc4.h"

void dmrRc4Init(dmrRc4_t *c, const uint8_t *key, size_t keyLen)
{
	static const uint8_t zeroKey[1] = { 0 }; // function scope: an invalid length falls back to it, which must outlive the if

	if ((keyLen == 0) || (keyLen > 256))
	{
		keyLen = 1;
		key = zeroKey;
	}

	for (int n = 0; n < 256; n++)
	{
		c->s[n] = (uint8_t)n;
	}

	uint8_t j = 0;

	for (int n = 0; n < 256; n++)
	{
		j = (uint8_t)(j + c->s[n] + key[n % keyLen]);

		uint8_t t = c->s[n];

		c->s[n] = c->s[j];
		c->s[j] = t;
	}

	c->i = 0;
	c->j = 0;
}

void dmrRc4Keystream(dmrRc4_t *c, uint8_t *out, size_t n)
{
	for (size_t k = 0; k < n; k++)
	{
		c->i = (uint8_t)(c->i + 1);
		c->j = (uint8_t)(c->j + c->s[c->i]);

		uint8_t t = c->s[c->i];

		c->s[c->i] = c->s[c->j];
		c->s[c->j] = t;
		out[k] = c->s[(uint8_t)(c->s[c->i] + c->s[c->j])];
	}
}

void dmrRc4Skip(dmrRc4_t *c, size_t n)
{
	uint8_t scratch;

	while (n--)
	{
		dmrRc4Keystream(c, &scratch, 1);
	}
}
