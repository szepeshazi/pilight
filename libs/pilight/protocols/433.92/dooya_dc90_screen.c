/*
	Copyright (C) 2016 András Szepesházi szepeshazi@gmail.com

	This file is part of pilight.

	pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with pilight. If not, see	<http://www.gnu.org/licenses/>
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../core/pilight.h"
#include "../../core/common.h"
#include "../../core/dso.h"
#include "../../core/log.h"
#include "../protocol.h"
#include "../../core/binary.h"
#include "../../core/gc.h"
#include "dooya_dc90_screen.h"

#define MIN_PULSE_LENGTH	280
#define MAX_PULSE_LENGTH	360

#define RAW_LENGTH				83		// number of raw signals forming a single message in this protocol
#define DOOYA_DC90_EPSILON		0.075	// allowed deviation from "ideal" signal length in percentage
#define SIGNAL_LENGTH			2		// the length of a high-low or low-high transition that represents a single bit
#define HEADER_LENGTH			2		// length of raw pulse train header
#define FOOTER_LENGTH			1		// length of raw pulse train footer
#define BINARY_LENGTH			(RAW_LENGTH - HEADER_LENGTH - FOOTER_LENGTH) / SIGNAL_LENGTH

// Device id bits in binary code
#define ID_START				16
#define ID_END					27
#define ID_LENGTH				ID_END - ID_START + 1

// Channel bits in binary code
#define CHANNEL_START			0
#define CHANNEL_END				7
#define CHANNEL_LENGTH			CHANNEL_END - CHANNEL_START + 1

// State bits in binary code
#define STATE_START				32
#define STATE_END				39
#define STATE_LENGTH			STATE_END - STATE_START + 1

// Sync 1 bits in binary code
#define SYNC1_START				8
#define SYNC1_END				15
#define SYNC1_LENGTH			SYNC1_END - SYNC1_START + 1

// Sync 2 bits in binary code
#define SYNC2_START				28
#define SYNC2_END				31
#define SYNC2_LENGTH			SYNC2_END - SYNC2_START + 1

#define DOOYA_DC90_SYNC1				6
#define DOOYA_DC90_SYNC2				1
#define DOOYA_DC90_STATE_DOWN			51
#define DOOYA_DC90_STATE_UP				17
#define DOOYA_INVALID_VALUE				-1

static const int header[] = {4750, 1535};	// header signal
static const int footer[] = {7775};			// footer signal
static const int low[] = {320, 740};		// signal representing the low (0) bit
static const int high[] = {680, 400};		// signal representing the high (1) bit

static int binary[BINARY_LENGTH];

static int isInRange(int value, int compareTo) {
	if ((value >= (int)(compareTo * (1.0 - DOOYA_DC90_EPSILON))) && (value <= (int)(compareTo * (1.0 + DOOYA_DC90_EPSILON)))) {
		return 1;
	}
	return 0;
}

static void createHigh(int signalIndex) {
	dooya_dc90_screen->raw[HEADER_LENGTH + (signalIndex * SIGNAL_LENGTH)] = high[0];
	dooya_dc90_screen->raw[HEADER_LENGTH + (signalIndex * SIGNAL_LENGTH) + 1] = high[1];
}

static int isLow(int signalIndex) {
	if ((isInRange(dooya_dc90_screen->raw[HEADER_LENGTH + (signalIndex * SIGNAL_LENGTH)], low[0]) &&
		(isInRange(dooya_dc90_screen->raw[HEADER_LENGTH + (signalIndex * SIGNAL_LENGTH) + 1], low[1])))) {
		return 1;
	} else {
		return 0;
	}
}

static void createLow(int signalIndex) {
	dooya_dc90_screen->raw[HEADER_LENGTH + (signalIndex * SIGNAL_LENGTH)] = low[0];
	dooya_dc90_screen->raw[HEADER_LENGTH + (signalIndex * SIGNAL_LENGTH) + 1] = low[1];
}


static int matchHeader(void) {
	if (isInRange(dooya_dc90_screen->raw[0], header[0]) &&
		isInRange(dooya_dc90_screen->raw[1], header[1])) {
		return 1;
	}
	return 0;
}

static int matchFooter(void) {
	if (isInRange(dooya_dc90_screen->raw[dooya_dc90_screen->rawlen-1], footer[0])) {
		return 1;
	}
	return 0;
}

static int validate(void) {
	if(dooya_dc90_screen->rawlen == RAW_LENGTH) {
		if(matchHeader() && matchFooter()) {
			return 0;
		}
	}
	return DOOYA_INVALID_VALUE;
}

static void createMessage(int id, int channel, int state) {

	dooya_dc90_screen->message = json_mkobject();
	json_append_member(dooya_dc90_screen->message, "id", json_mknumber(id, 0));
	json_append_member(dooya_dc90_screen->message, "channel", json_mknumber(channel, 0));
	if(state == 0) {
		json_append_member(dooya_dc90_screen->message, "state", json_mkstring("down"));
	} else if (state == 1) {
		json_append_member(dooya_dc90_screen->message, "state", json_mkstring("up"));
	}
}


static void parseCode(void) {

	int i = 0;

	for(i = 0; i < BINARY_LENGTH; i++) {
		if(isLow(i)) {
			binary[i] = 0;
		} else {
			binary[i] = 1;
		}
	}

	int channel = binToDecRev(binary, CHANNEL_START, CHANNEL_END);
	int sync1 = binToDecRev(binary, SYNC1_START, SYNC1_END);
	int id = binToDecRev(binary, ID_START, ID_END);
	int sync2 = binToDecRev(binary, SYNC2_START, SYNC2_END);
	int stateRaw = binToDecRev(binary, STATE_START, STATE_END);
	int state = DOOYA_INVALID_VALUE;

	if (stateRaw == 51) {
		state = 1;
	} else if (stateRaw == 17) {
		state = 0;
	}

	if ((sync1 == DOOYA_DC90_SYNC1) && (sync2 == DOOYA_DC90_SYNC2) && (state != DOOYA_INVALID_VALUE)) {
		createMessage(id, channel, state);
	}
}

static void clearCode(void) {

	int i = 0;

	for(i = 0; i < BINARY_LENGTH; i++) {
		createLow(i);
	}
}

static void createHeader(void) {
	dooya_dc90_screen->raw[0] = header[0];
	dooya_dc90_screen->raw[1] = header[1];
}

static void createBinarySequence(int value, int maxLength, int SequenceEndIndex) {
	int binary[maxLength];
	int length = 0, i = 0;

	length = decToBin(value, binary) + 1;
	for(i = 0; i < length; i++) {
		if(binary[i] == 1) {
			createHigh(SequenceEndIndex - length + i + 1);
		}
	}
}

static void createFooter(void) {
	dooya_dc90_screen->raw[RAW_LENGTH-1] = footer[0];
}

static void createRawCode(int id, int channel, int state) {

	int rawState;
	if (state == 0) {
		rawState = DOOYA_DC90_STATE_DOWN;
	} else {
		rawState = DOOYA_DC90_STATE_UP;
	}
	clearCode();
	createHeader();
	createBinarySequence(channel, CHANNEL_LENGTH, CHANNEL_END);
	createBinarySequence(DOOYA_DC90_SYNC1, SYNC1_LENGTH, SYNC1_END);
	createBinarySequence(id, ID_LENGTH, ID_END);
	createBinarySequence(DOOYA_DC90_SYNC2, SYNC2_LENGTH, SYNC2_END);
	createBinarySequence(rawState, STATE_LENGTH, STATE_END);
	createFooter();
}

static int createCode(struct JsonNode *code) {

	int id = DOOYA_INVALID_VALUE, channel = DOOYA_INVALID_VALUE, state = DOOYA_INVALID_VALUE;
	double itmp = DOOYA_INVALID_VALUE;

	if(json_find_number(code, "id", &itmp) == 0)
		id = (int)round(itmp);
	if(json_find_number(code, "channel", &itmp) == 0)
		channel = (int)round(itmp);
	if(json_find_number(code, "down", &itmp) == 0)
		state = 0;
	if(json_find_number(code, "up", &itmp) == 0)
		state = 1;

	if(id == DOOYA_INVALID_VALUE || channel == DOOYA_INVALID_VALUE || state == DOOYA_INVALID_VALUE) {
		logprintf(LOG_ERR, "dooya_dc90_screen: insufficient number of arguments");
		return EXIT_FAILURE;
	} else {
		createMessage(id, channel, state);
		createRawCode(id, channel, state);
		dooya_dc90_screen->rawlen = RAW_LENGTH;
	}
	return EXIT_SUCCESS;
}

static void printHelp(void) {
	printf("\t -d --down\t\t\tsend a down signal\n");
	printf("\t -u --up\t\t\tsend an up signal\n");
	printf("\t -i --id=id\t\t\tcontrol a device with this id\n");
	printf("\t -c --channel=channel\t\t\tcontrol a device on this channel\n");
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void dooyaDC90ScreenInit(void) {

	protocol_register(&dooya_dc90_screen);
	protocol_set_id(dooya_dc90_screen, "dooya_dc90_screen");
	protocol_device_add(dooya_dc90_screen, "dooya_dc90_screen", "Dooya DC90 screen");
	dooya_dc90_screen->devtype = SCREEN;
	dooya_dc90_screen->hwtype = RF433;
	dooya_dc90_screen->minrawlen = RAW_LENGTH;
	dooya_dc90_screen->maxrawlen = RAW_LENGTH;
	dooya_dc90_screen->maxgaplen = MAX_PULSE_LENGTH * PULSE_DIV;
	dooya_dc90_screen->mingaplen = MIN_PULSE_LENGTH * PULSE_DIV;

	options_add(&dooya_dc90_screen->options, 'd', "down", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	options_add(&dooya_dc90_screen->options, 'u', "up", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	options_add(&dooya_dc90_screen->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "[0-9]");
	options_add(&dooya_dc90_screen->options, 'c', "channel", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "[0-9]");

	dooya_dc90_screen->parseCode=&parseCode;
	dooya_dc90_screen->createCode=&createCode;
	dooya_dc90_screen->printHelp=&printHelp;
	dooya_dc90_screen->validate=&validate;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "Dooya DC90 screen";
	module->version = "1.0";
	module->reqversion = "6.0";
	module->reqcommit = "84";
}

void init(void) {
	dooyaDC90ScreenInit();
}
#endif
