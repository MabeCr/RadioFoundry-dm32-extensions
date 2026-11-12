// SPDX-License-Identifier: MIT AND LicenseRef-p25rs-MIT
// Copyright (c) 2026 The RadioFoundry Authors
// SPDX-FileCopyrightText: 2016-2018 Mick Koch (the field layouts and opcodes, from p25.rs, MIT, see LICENSES/LicenseRef-p25rs-MIT.txt)
// P25 trunking signaling block messages (docs/spec/p25.md 5): the standard opcodes of a control channel, split into
// fields, and the channel identifier table that turns a channel (identifier and number) into frequencies.
#ifndef RFOUNDRY_P25_TSBK_H
#define RFOUNDRY_P25_TSBK_H

#include "p25/p25_block.h"

typedef enum {
	P25_MSG_OTHER,		// an opcode or manufacturer not decoded
	P25_MSG_GRP_VOICE_GRANT,
	P25_MSG_GRP_VOICE_UPDATE,
	P25_MSG_UNIT_VOICE_GRANT,
	P25_MSG_UNIT_VOICE_UPDATE,
	P25_MSG_GRP_DATA_GRANT,
	P25_MSG_UNIT_DATA_GRANT,
	P25_MSG_GRP_DATA_UPDATE,
	P25_MSG_UNIT_CALL_ALERT,
	P25_MSG_LOC_REG_RESPONSE,
	P25_MSG_UNIT_REG_RESPONSE,
	P25_MSG_UNIT_DEREG_ACK,
	P25_MSG_ALT_CONTROL_CHANNEL,
	P25_MSG_RFSS_STATUS,
	P25_MSG_NETWORK_STATUS,
	P25_MSG_ADJACENT_SITE,
	P25_MSG_IDEN_UPDATE,
} p25_msg_kind_t;

typedef struct {
	uint8_t id;		// which channel identifier's parameters to use
	uint16_t number;	// the channel within it
} p25_chan_t;

// Services a site advertises (bit flags of the service class byte)
#define P25_SVC_COMPOSITE	0x01	// can also carry traffic when busy
#define P25_SVC_UPDATES_ONLY	0x02
#define P25_SVC_BACKUP		0x04
#define P25_SVC_DATA		0x10
#define P25_SVC_VOICE		0x20
#define P25_SVC_REGISTRATION	0x40
#define P25_SVC_AUTH		0x80

// Frequencies of one channel identifier
typedef struct {
	uint32_t base_hz;
	uint32_t spacing_hz;
	int32_t tx_offset_hz;
	uint32_t bandwidth_hz;
} p25_iden_t;

typedef struct {
	p25_msg_kind_t kind;
	uint8_t opcode, mfid;
	union {
		struct { uint8_t opts; p25_chan_t chan; uint16_t tg; uint32_t src; } grp_grant;	// group voice or data grant
		struct { p25_chan_t chan[2]; uint16_t tg[2]; } grp_update;	// group voice or data update
		struct { p25_chan_t chan; uint32_t dst, src; } unit_grant;	// unit voice grant or update, unit data grant
		struct { uint32_t dst, src; } call_alert;
		struct { uint8_t resp; uint16_t tg; uint8_t rfss, site; uint32_t unit; } loc_reg;
		struct { uint8_t resp; uint16_t system; uint32_t src_id, src_addr; } unit_reg;
		struct { uint32_t wacn; uint16_t system; uint32_t unit; } dereg;
		struct { uint8_t rfss, site; p25_chan_t chan[2]; uint8_t svc[2]; } alt_cc;
		struct {	// RFSS status, network status and adjacent site share one shape; unused fields are 0
			uint8_t area, opts, rfss, site, svc;
			uint16_t system;
			uint32_t wacn;
			p25_chan_t chan;
		} site;
		struct { uint8_t id; p25_iden_t iden; } iden;
	} u;
} p25_msg_t;

// A TSBK that passed its CRC to a message. Anything not decoded comes back as P25_MSG_OTHER with the opcode and mfid
void p25_msg_parse(const p25_tsbk_t *t, p25_msg_t *m);

// Registration response codes
#define P25_REG_ACCEPT	0
#define P25_REG_FAIL	1
#define P25_REG_DENY	2
#define P25_REG_REFUSE	3

// The sixteen channel identifiers a control channel announces
typedef struct {
	p25_iden_t e[16];
	uint16_t valid;		// bit n: e[n] is known
} p25_iden_table_t;

void p25_iden_update(p25_iden_table_t *t, const p25_msg_t *m);	// takes P25_MSG_IDEN_UPDATE, ignores the rest
// The receive and transmit frequency of a channel in Hz; false when its identifier hasn't been announced yet
bool p25_chan_freq(const p25_iden_table_t *t, p25_chan_t c, uint32_t *rx_hz, uint32_t *tx_hz);

#endif
