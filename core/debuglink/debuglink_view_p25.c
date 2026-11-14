// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// The p25 view section (docs/spec/debuglink.md 3.26, docs/spec/p25.md 6): the P25 listener's counters, its latest
// frame and the control channel blocks it decoded. Reading the section takes the blocks it lists
#include "config.h"
#include "debuglink/debuglink.h"

#if CONFIG_P25
#include "p25/p25_listen.h"

#define BLOCKS_PER_READ	6u

void debuglink_view_p25(dl_text_t *t)
{
	static const char *const duids[16] = { "hdu", NULL, NULL, "tdu", NULL, "ldu1", NULL, "tsdu", NULL, NULL, "ldu2",
					       NULL, "pdu", NULL, NULL, "tdulc" };
	p25_listen_stats_t s;
	p25_listen_tsbk_t b[BLOCKS_PER_READ];

	p25_listen_stats(&s);
	dl_text(t, "p25.built=1\np25.running=%u\np25.samples=%u\np25.peak=%u\np25.start_fails=%u\n", s.running ? 1u : 0u,
		(unsigned)s.samples, (unsigned)s.peak, (unsigned)s.start_fails);
	dl_text(t, "p25.syncs=%u\np25.nid_ok=%u\np25.nid_bad=%u\np25.tsbk_ok=%u\np25.tsbk_bad=%u\np25.tsbk_lost=%u\n",
		(unsigned)s.syncs, (unsigned)s.nid_ok, (unsigned)s.nid_bad, (unsigned)s.tsbk_ok, (unsigned)s.tsbk_bad,
		(unsigned)s.tsbk_overflow);
	dl_text(t, "p25.last_nac=%03X\np25.last_duid=%X\np25.last_errors=%u\np25.last_dc=%d\np25.last_amp=%u\np25.last_score=%u\n",
		s.last_nac, s.last_duid, s.last_errors, s.last_dc, s.last_amp, s.last_score_q8);
	for (unsigned i = 0; i < 16; i++) {
		if (duids[i] != NULL)
			dl_text(t, "p25.duid_%s=%u\n", duids[i], (unsigned)s.duid[i]);
	}
	uint8_t n = p25_listen_drain(b, BLOCKS_PER_READ);

	for (uint8_t i = 0; i < n; i++) {
		dl_text(t, "p25.tsbk.%u=%u:%u:%u:", i, (unsigned)b[i].ms, b[i].crc_ok, b[i].errors);
		for (unsigned k = 0; k < 12; k++)
			dl_text(t, "%02X", b[i].raw[k]);
		dl_text(t, "\n");
	}
}

#else

void debuglink_view_p25(dl_text_t *t)
{
	dl_text(t, "p25.built=0\n");
}

#endif
