// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// P25 listening (p25_listen.h)
#include "config.h"
#if CONFIG_P25
#include <string.h>
#include "p25/p25_listen.h"
#include "p25/p25_rx.h"
#include "board.h"
#include "FreeRTOS.h"

#define READ_SAMPLES	128u
#define RETRY_MS	1000u

typedef struct {
	p25_rx_t rx;
	int16_t buf[READ_SAMPLES];
} listen_t;

static listen_t *lc;		// from the RTOS heap while listening: static RAM is short on these radios
static bool enabled = true;
static p25_listen_stats_t st;	// totals of earlier listening periods plus the latest frame; the live receiver's add on
static p25_listen_tsbk_t ring[P25_LISTEN_TSBKS];
static uint8_t ring_head, ring_n;
static uint32_t retry_ms;
static bool retry_wait;
static uint32_t now;

static void on_frame(const p25_frame_t *f, void *ctx)
{
	(void)ctx;
	st.last_nac = f->nid.nac;
	st.last_duid = f->nid.duid;
	st.last_errors = f->nid.errors;
	st.last_dc = f->dc;
	st.last_amp = f->amp;
	st.last_score_q8 = f->score_q8;
}

static void on_tsbk(const p25_tsbk_t *t, uint8_t index, const p25_frame_t *f, void *ctx)
{
	(void)index;
	(void)f;
	(void)ctx;
	p25_listen_tsbk_t *e;

	if (ring_n == P25_LISTEN_TSBKS) {
		st.tsbk_overflow++;	// keep the oldest, as the capture ring does
		return;
	}
	e = &ring[(ring_head + ring_n++) % P25_LISTEN_TSBKS];
	e->ms = now;
	e->crc_ok = t->crc_ok;
	e->errors = t->errors;
	e->raw[0] = (uint8_t)((t->lb << 7) | (t->protect << 6) | t->opcode);
	e->raw[1] = t->mfid;
	memcpy(e->raw + 2, t->args, 8);
	e->raw[10] = (uint8_t)(t->crc >> 8);
	e->raw[11] = (uint8_t)t->crc;
}

void p25_listen_enable(bool on)
{
	enabled = on;
}

// Fold the live receiver's counters into the totals before it goes away
static void fold(void)
{
	const p25_rx_t *r = &lc->rx;

	st.syncs += r->syncs;
	st.nid_ok += r->nid_ok;
	st.nid_bad += r->nid_bad;
	st.tsbk_ok += r->tsbk_ok;
	st.tsbk_bad += r->tsbk_bad;
	for (unsigned i = 0; i < 16; i++)
		st.duid[i] += r->duid[i];
}

void p25_listen_tick(bool fm_rx, uint32_t now_ms)
{
	const hal_afsk_t *h = board.afsk;
	bool can = h != NULL && h->rx_start != NULL && h->rx_read != NULL;
	bool want = enabled && fm_rx && can;

	now = now_ms;
	if (want && lc == NULL && (!retry_wait || (int32_t)(now_ms - retry_ms) >= 0)) {
		lc = pvPortMalloc(sizeof(*lc));
		if (lc != NULL) {
			memset(lc, 0, sizeof(*lc));
			if (p25_rx_init(&lc->rx, h->rx_rate_hz, on_frame, NULL) && h->rx_start()) {
				p25_rx_set_tsbk(&lc->rx, on_tsbk, NULL);
				retry_wait = false;
			} else {
				vPortFree(lc);
				lc = NULL;
			}
		}
		if (lc == NULL) {
			st.start_fails++;
			retry_wait = true;
			retry_ms = now_ms + RETRY_MS;
		}
	} else if (!want && lc != NULL) {
		if (h != NULL && h->rx_stop != NULL)
			h->rx_stop();
		fold();
		vPortFree(lc);
		lc = NULL;
	}
	if (!want)
		retry_wait = false;
	if (lc == NULL)
		return;
	uint16_t k;

	while ((k = h->rx_read(lc->buf, READ_SAMPLES)) != 0) {
		for (uint16_t i = 0; i < k; i++) {
			uint32_t m = (uint32_t)(lc->buf[i] < 0 ? -(int32_t)lc->buf[i] : lc->buf[i]);

			if (m > st.peak)
				st.peak = m;
		}
		p25_rx_feed(&lc->rx, lc->buf, k);
		st.samples += k;
	}
}

bool p25_listen_running(void)
{
	return lc != NULL;
}

void p25_listen_stats(p25_listen_stats_t *out)
{
	*out = st;
	out->running = lc != NULL;
	if (lc != NULL) {
		out->syncs += lc->rx.syncs;
		out->nid_ok += lc->rx.nid_ok;
		out->nid_bad += lc->rx.nid_bad;
		out->tsbk_ok += lc->rx.tsbk_ok;
		out->tsbk_bad += lc->rx.tsbk_bad;
		for (unsigned i = 0; i < 16; i++)
			out->duid[i] += lc->rx.duid[i];
	}
}

uint8_t p25_listen_drain(p25_listen_tsbk_t *out, uint8_t max)
{
	uint8_t n = 0;

	while (n < max && ring_n != 0) {
		out[n++] = ring[ring_head];
		ring_head = (uint8_t)((ring_head + 1u) % P25_LISTEN_TSBKS);
		ring_n--;
	}
	return n;
}

#endif
