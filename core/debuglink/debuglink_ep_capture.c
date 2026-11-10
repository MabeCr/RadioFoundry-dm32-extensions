// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The RadioFoundry Authors
// The EP capture ring's debug link command (CONFIG_EP_CAPTURE; docs/spec/debuglink.md 3.25, core/dmr/epCapture.h): a
// bench tool to pull the raw octets of an encrypted call's PI headers and still-scrambled voice bursts off the
// radio, for working out a privacy scheme's recipe on a computer. Built without CONFIG_EP_CAPTURE it answers
// unavailable
#include "config.h"
#include "debuglink/debuglink.h"
#include "debuglink/debuglink_proto.h"

#if CONFIG_EP_CAPTURE

#include <string.h>
#include "dmr/epCapture.h"

static void put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
	put16(p, (uint16_t)v);
	put16(p + 2, (uint16_t)(v >> 16));
}

size_t debuglink_cmd_ep_capture(const uint8_t *req, uint16_t len, uint8_t *out, size_t cap)
{
	if (len < 1) {
		out[0] = DL_ST_BAD_ARG;
		return 1;
	}
	switch (req[0]) {
	case DL_EP_OP_INFO: {
		uint16_t count, overflow, capacity;

		epCaptureInfo(&count, &overflow, &capacity);
		out[0] = DL_ST_OK;
		out[1] = epCaptureEnabled() ? 1u : 0u;
		put16(out + 2, count);
		put16(out + 4, overflow);
		put16(out + 6, capacity);
		return 8;
	}
	case DL_EP_OP_ENABLE:
		if (len < 2 || req[1] > 1) {
			out[0] = DL_ST_BAD_ARG;
			return 1;
		}
		epCaptureEnable(req[1] != 0);
		out[0] = DL_ST_OK;
		return 1;
	case DL_EP_OP_DRAIN: {
		if (len < 2) {
			out[0] = DL_ST_BAD_ARG;
			return 1;
		}
		// each entry is DL_EP_ENTRY_HDR + up to 27 payload bytes; stay inside the reply buffer
		size_t room = (cap - 2u) / (DL_EP_ENTRY_HDR + 27u);
		size_t max = req[1];

		if (max > room) {
			max = room;
		}
		epCaptureEntry_t ent[8];	// drained in small groups: the stack copy stays cheap
		uint8_t *p = out + 2;
		uint8_t n = 0;

		while (n < max) {
			size_t want = max - n;

			if (want > 8) {
				want = 8;
			}
			size_t got = epCaptureDrain(ent, want);

			if (got == 0) {
				break;
			}
			for (size_t k = 0; k < got; k++) {
				put32(p, ent[k].slot);
				p[4] = ent[k].voiceIndex;
				p[5] = ent[k].crcOk;
				p[6] = ent[k].len;
				memcpy(p + DL_EP_ENTRY_HDR, ent[k].payload, ent[k].len);
				p += DL_EP_ENTRY_HDR + ent[k].len;
			}
			n = (uint8_t)(n + got);
		}
		out[0] = DL_ST_OK;
		out[1] = n;
		return (size_t)(p - out);
	}
	case DL_EP_OP_RESET:
		epCaptureReset();
		out[0] = DL_ST_OK;
		return 1;
	default:
		out[0] = DL_ST_BAD_ARG;
		return 1;
	}
}

#else	// !CONFIG_EP_CAPTURE

size_t debuglink_cmd_ep_capture(const uint8_t *req, uint16_t len, uint8_t *out, size_t cap)
{
	(void)req;
	(void)len;
	(void)cap;
	out[0] = DL_ST_UNAVAILABLE;
	return 1;
}

#endif
