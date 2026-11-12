// SPDX-License-Identifier: MIT AND LicenseRef-p25rs-MIT
// Copyright (c) 2026 The RadioFoundry Authors
// SPDX-FileCopyrightText: 2016-2018 Mick Koch (the field layouts and opcodes, from p25.rs, MIT, see LICENSES/LicenseRef-p25rs-MIT.txt)
// P25 trunking messages (p25_tsbk.h)
#include <string.h>
#include "p25/p25_tsbk.h"

// the block's 12 bytes are rebuilt for the field layouts, which count from byte 0 (the header)
static unsigned u16(const uint8_t *b) { return (unsigned)(b[0] << 8 | b[1]); }
static uint32_t u24(const uint8_t *b) { return (uint32_t)b[0] << 16 | (uint32_t)b[1] << 8 | b[2]; }
static uint32_t u32(const uint8_t *b) { return (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3]; }

static p25_chan_t chan(const uint8_t *b)
{
	unsigned v = u16(b);

	return (p25_chan_t){ .id = (uint8_t)(v >> 12), .number = (uint16_t)(v & 0xFFF) };
}

void p25_msg_parse(const p25_tsbk_t *t, p25_msg_t *m)
{
	// the fields in byte order, header included
	uint8_t b[12] = { 0 };
	const uint8_t *p = b + 2;	// the payload

	memset(m, 0, sizeof *m);
	m->kind = P25_MSG_OTHER;
	m->opcode = t->opcode;
	m->mfid = t->mfid;
	b[0] = (uint8_t)((t->lb << 7) | (t->protect << 6) | t->opcode);
	b[1] = t->mfid;
	memcpy(b + 2, t->args, 8);
	if (t->mfid != 0)
		return;
	switch (t->opcode) {
	case 0x00:	// group voice grant
	case 0x11:	// group data grant
		m->kind = t->opcode == 0 ? P25_MSG_GRP_VOICE_GRANT : P25_MSG_GRP_DATA_GRANT;
		m->u.grp_grant.opts = p[0];
		m->u.grp_grant.chan = chan(p + 1);
		m->u.grp_grant.tg = (uint16_t)u16(p + 3);
		m->u.grp_grant.src = u24(p + 5);
		break;
	case 0x02:	// group voice update
	case 0x12:	// group data update
		m->kind = t->opcode == 2 ? P25_MSG_GRP_VOICE_UPDATE : P25_MSG_GRP_DATA_UPDATE;
		m->u.grp_update.chan[0] = chan(p);
		m->u.grp_update.tg[0] = (uint16_t)u16(p + 2);
		m->u.grp_update.chan[1] = chan(p + 4);
		m->u.grp_update.tg[1] = (uint16_t)u16(p + 6);
		break;
	case 0x04:	// unit to unit voice grant
	case 0x06:	// unit to unit voice update
	case 0x10:	// unit data grant
		m->kind = t->opcode == 4 ? P25_MSG_UNIT_VOICE_GRANT :
			  t->opcode == 6 ? P25_MSG_UNIT_VOICE_UPDATE : P25_MSG_UNIT_DATA_GRANT;
		m->u.unit_grant.chan = chan(p);
		m->u.unit_grant.dst = u24(p + 2);
		m->u.unit_grant.src = u24(p + 5);
		break;
	case 0x1F:	// unit call alert
		m->kind = P25_MSG_UNIT_CALL_ALERT;
		m->u.call_alert.dst = u24(p + 2);
		m->u.call_alert.src = u24(p + 5);
		break;
	case 0x2B:	// location registration response
		m->kind = P25_MSG_LOC_REG_RESPONSE;
		m->u.loc_reg.resp = b[2] & 3;
		m->u.loc_reg.tg = (uint16_t)u16(b + 3);
		m->u.loc_reg.rfss = b[5];
		m->u.loc_reg.site = b[6];
		m->u.loc_reg.unit = u24(b + 7);
		break;
	case 0x2C:	// unit registration response
		m->kind = P25_MSG_UNIT_REG_RESPONSE;
		m->u.unit_reg.resp = (b[2] >> 4) & 3;
		m->u.unit_reg.system = (uint16_t)(u16(b + 2) & 0xFFF);
		m->u.unit_reg.src_id = u24(b + 4);
		m->u.unit_reg.src_addr = u24(b + 7);
		break;
	case 0x2F:	// unit deregistration acknowledge
		m->kind = P25_MSG_UNIT_DEREG_ACK;
		m->u.dereg.wacn = u24(b + 3) >> 4;
		m->u.dereg.system = (uint16_t)(u16(b + 5) & 0xFFF);
		m->u.dereg.unit = u24(b + 7);
		break;
	case 0x39:	// alternate control channel
		m->kind = P25_MSG_ALT_CONTROL_CHANNEL;
		m->u.alt_cc.rfss = p[0];
		m->u.alt_cc.site = p[1];
		m->u.alt_cc.chan[0] = chan(p + 2);
		m->u.alt_cc.svc[0] = p[4];
		m->u.alt_cc.chan[1] = chan(p + 5);
		m->u.alt_cc.svc[1] = p[7];
		break;
	case 0x3A:	// RFSS status broadcast
	case 0x3C:	// adjacent site broadcast
		m->kind = t->opcode == 0x3A ? P25_MSG_RFSS_STATUS : P25_MSG_ADJACENT_SITE;
		m->u.site.area = p[0];
		m->u.site.opts = p[1] >> 4;	// RFSS status: bit 0 of this is "networked"
		m->u.site.system = (uint16_t)(u16(p + 1) & 0xFFF);
		m->u.site.rfss = p[3];
		m->u.site.site = p[4];
		m->u.site.chan = chan(p + 5);
		m->u.site.svc = p[7];
		break;
	case 0x3B:	// network status broadcast
		m->kind = P25_MSG_NETWORK_STATUS;
		m->u.site.area = p[0];
		m->u.site.wacn = u24(p + 1) >> 4;
		m->u.site.system = (uint16_t)(u16(p + 3) & 0xFFF);
		m->u.site.chan = chan(p + 5);
		m->u.site.svc = p[7];
		break;
	case 0x3D: {	// channel identifier update
		unsigned bw = ((unsigned)(p[0] & 0xF) << 5) | (p[1] >> 3);
		unsigned off = ((unsigned)(p[1] & 7) << 6) | (p[2] >> 2);
		unsigned sp = ((unsigned)(p[2] & 3) << 8) | p[3];
		int32_t o = (int32_t)(off & 0xFF) * 250000;

		m->kind = P25_MSG_IDEN_UPDATE;
		m->u.iden.id = p[0] >> 4;
		m->u.iden.iden.base_hz = u32(p + 4) * 5;
		m->u.iden.iden.spacing_hz = sp * 125;
		m->u.iden.iden.tx_offset_hz = (off >> 8) ? o : -o;	// bit 8 set is a transmit above the receive frequency
		m->u.iden.iden.bandwidth_hz = bw * 125;
		break;
	}
	default:
		break;
	}
}

void p25_iden_update(p25_iden_table_t *t, const p25_msg_t *m)
{
	if (m->kind != P25_MSG_IDEN_UPDATE)
		return;
	t->e[m->u.iden.id & 15] = m->u.iden.iden;
	t->valid |= (uint16_t)(1u << (m->u.iden.id & 15));
}

bool p25_chan_freq(const p25_iden_table_t *t, p25_chan_t c, uint32_t *rx_hz, uint32_t *tx_hz)
{
	if (!(t->valid & (1u << (c.id & 15))))
		return false;
	const p25_iden_t *e = &t->e[c.id & 15];
	uint32_t rx = e->base_hz + e->spacing_hz * c.number;

	if (rx_hz)
		*rx_hz = rx;
	if (tx_hz)
		*tx_hz = (uint32_t)((int32_t)rx + e->tx_offset_hz);
	return true;
}
