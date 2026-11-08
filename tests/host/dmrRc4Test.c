// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
/*
 * Host tests for the RC4 primitive, against published test vectors
 *
 *   make -C tests/host
 */

#include <stdio.h>
#include <string.h>
#include "dmr/dmrRc4.h"

static int failures = 0;

#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// Encrypt = plaintext XOR keystream; compare with the well-known ciphertexts (Wikipedia's RC4 article)
static void vector(const char *key, const char *plain, const uint8_t *want, size_t n)
{
	dmrRc4_t c;
	uint8_t ks[32], out[32];

	dmrRc4Init(&c, (const uint8_t *)key, strlen(key));
	dmrRc4Keystream(&c, ks, n);

	for (size_t i = 0; i < n; i++)
	{
		out[i] = (uint8_t)plain[i] ^ ks[i];
	}

	CHECK(memcmp(out, want, n) == 0, "key \"%s\": wrong ciphertext", key);
}

int main(void)
{
	static const uint8_t v1[] = { 0xBB, 0xF3, 0x16, 0xE8, 0xD9, 0x40, 0xAF, 0x0A, 0xD3 };
	static const uint8_t v2[] = { 0x10, 0x21, 0xBF, 0x04, 0x20 };
	static const uint8_t v3[] = { 0x45, 0xA0, 0x1F, 0x64, 0x5F, 0xC3, 0x5B, 0x38, 0x35, 0x52, 0x54, 0x4B, 0x9B, 0xF5 };

	vector("Key", "Plaintext", v1, sizeof(v1));
	vector("Wiki", "pedia", v2, sizeof(v2));
	vector("Secret", "Attack at dawn", v3, sizeof(v3));

	// RFC 6229, 40-bit key 0102030405, first 16 keystream octets, and the octets at offset 16 after a skip
	static const uint8_t key40[] = { 1, 2, 3, 4, 5 };
	static const uint8_t ks0[] = { 0xB2, 0x39, 0x63, 0x05, 0xF0, 0x3D, 0xC0, 0x27, 0xCC, 0xC3, 0x52, 0x4A, 0x0A, 0x11, 0x18, 0xA8 };
	dmrRc4_t c;
	uint8_t ks[16];

	dmrRc4Init(&c, key40, sizeof(key40));
	dmrRc4Keystream(&c, ks, sizeof(ks));
	CHECK(memcmp(ks, ks0, sizeof(ks)) == 0, "RFC 6229 40-bit key, offset 0");

	// Skipping equals generating and discarding
	dmrRc4_t a, b;
	uint8_t ka[8], kb[8], junk[100];

	dmrRc4Init(&a, key40, sizeof(key40));
	dmrRc4Init(&b, key40, sizeof(key40));
	dmrRc4Skip(&a, 100);
	dmrRc4Keystream(&b, junk, sizeof(junk));
	dmrRc4Keystream(&a, ka, sizeof(ka));
	dmrRc4Keystream(&b, kb, sizeof(kb));
	CHECK(memcmp(ka, kb, sizeof(ka)) == 0, "skip");

	// An empty key is not undefined behavior
	dmrRc4Init(&a, key40, 0);
	dmrRc4Keystream(&a, ka, sizeof(ka));

	printf(failures ? "# FAILED (%d failures)\n" : "# PASSED (0 failures)\n", failures);
	return failures ? 1 : 0;
}
