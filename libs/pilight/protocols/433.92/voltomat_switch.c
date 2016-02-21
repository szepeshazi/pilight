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
#include "voltomat_switch.h"

#define MIN_PULSE_LENGTH	440
#define MAX_PULSE_LENGTH	500

#define RAW_LENGTH				50		// number of raw signals forming a single message in this protocol
#define VOLTOMAT_EPSILON		0.075	// allowed deviation from "ideal" signal length in percentage
#define START_SEQUENCE_LENGTH 	4		// identical leading bits for each pulse train in this protocol
#define SIGNAL_LENGTH			2		// the length of a high-low or low-high transition that represents a single bit
#define FOOTER_LENGTH			2		// length of pulse train footer
#define NUM_OF_BUTTONS			4
#define NUM_OF_STATES			2

static const int footer[] = {3005, 7175};	// footer signal
static const int low[] = {470, 1035};		// signal representing the low (0) bit
static const int high[] = {1035, 548};		// signal representing the high (1) bit

static const char *buttonStates[] = { "off", "on" };	// String representation of possible switch states
static const int codeMap[NUM_OF_BUTTONS][NUM_OF_STATES][(RAW_LENGTH-FOOTER_LENGTH) / SIGNAL_LENGTH] = {	// code map for each button's on and off states
		{	// A button states
				{0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0}, 	// A off
				{0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0} 	// A on
		},
		{	// B button states
				{0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0}, 	// B off
				{0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0} 	// B on
		},
		{	// C button states
				{0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0}, 	// C off
				{0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0} 	// C on
		},
		{	// D button states
				{0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0}, 	// D off
				{0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0} 	// D on
		}
};

typedef struct {
	int buttonId;
	int stateId;
	const char* state;
} buttonPressed;

int binary[(RAW_LENGTH - FOOTER_LENGTH ) / SIGNAL_LENGTH];

static int isInRange(int value, int compareTo) {
	if ((value >= (int)(compareTo * (1.0 - VOLTOMAT_EPSILON))) && (value <= (int)(compareTo * (1.0 + VOLTOMAT_EPSILON)))) {
		return 1;
	}
	return 0;
}

static int isHigh(int signalIndex) {
	if ((isInRange(voltomat_switch->raw[signalIndex], high[0]) && (isInRange(voltomat_switch->raw[signalIndex + 1], high[1])))) {
		return 1;
	} else {
		return 0;
	}
}

static int isLow(int signalIndex) {
	if ((isInRange(voltomat_switch->raw[signalIndex], low[0]) && (isInRange(voltomat_switch->raw[signalIndex + 1], low[1])))) {
		return 1;
	} else {
		return 0;
	}
}

static int matchFooter(void) {
	if (isInRange(voltomat_switch->raw[voltomat_switch->rawlen-2], footer[0]) &&
		isInRange(voltomat_switch->raw[voltomat_switch->rawlen-1], footer[1])) {
		return 1;
	}
	return 0;
}

static int matchStartSequence(void) {
	int i;

	for (i = 0; i < (START_SEQUENCE_LENGTH * SIGNAL_LENGTH); i+= SIGNAL_LENGTH) {
		if (isHigh(i)) {
			return 0;	// This protocol expects only low value signals at the beginning of the pulse train
		}
	}
	return 1;
}


static int validate(void) {
	if(voltomat_switch->rawlen == RAW_LENGTH) {
		if(matchFooter() && matchStartSequence()) {
			return 0;
		}
	}
	return -1;
}

static void createMessage(buttonPressed currentEvent) {
	voltomat_switch->message = json_mkobject();
	json_append_member(voltomat_switch->message, "id", json_mknumber(currentEvent.buttonId, 0));
	json_append_member(voltomat_switch->message, "state", json_mkstring(currentEvent.state));
}

static int matchBinaryPattern(const int *pattern, int patternLength) {
	int i;

	for (i = 0; i < patternLength; i++) {
		if (pattern[i] != binary[i]) {
			return 0;
		}
	}
	return 1;
}

static buttonPressed deCode(void) {
	int buttonIndex, stateIndex;
	buttonPressed currentEvent = {
			.buttonId = -1,
			.stateId = -1
	};

	for (buttonIndex = 0; buttonIndex < NUM_OF_BUTTONS; buttonIndex++) {
		for (stateIndex = 0; stateIndex < NUM_OF_STATES; stateIndex++) {
			if (matchBinaryPattern(codeMap[buttonIndex][stateIndex], (RAW_LENGTH - FOOTER_LENGTH) / SIGNAL_LENGTH)) {
				currentEvent.buttonId = buttonIndex;
				currentEvent.stateId = stateIndex;
				currentEvent.state = buttonStates[stateIndex];
				return currentEvent;
			}
		}
	}
	return currentEvent;
}

static void parseCode(void) {
	int rawIndex = 0, binaryIndex = 0;
	buttonPressed currentEvent;

	for(rawIndex = 0; rawIndex < (voltomat_switch->rawlen - FOOTER_LENGTH); rawIndex+= SIGNAL_LENGTH) {
		if(isLow(rawIndex)) {
			binary[binaryIndex++] = 0;
		} else {
			binary[binaryIndex++] = 1;
		}
	}

	currentEvent = deCode();
	if((currentEvent.buttonId != -1) && (currentEvent.stateId != -1)) {
		createMessage(currentEvent);
	}
}

static void createFooter(void) {
	voltomat_switch->raw[RAW_LENGTH-2] = footer[0];
	voltomat_switch->raw[RAW_LENGTH-1] = footer[1];
}

static void createRawCode(buttonPressed codeRequest) {
	int i;

	for (i = 0; i < (RAW_LENGTH - FOOTER_LENGTH) / SIGNAL_LENGTH; i++) {
		if (codeMap[codeRequest.buttonId][codeRequest.stateId][i] == 0) {
			voltomat_switch->raw[i * SIGNAL_LENGTH] = low[0];
			voltomat_switch->raw[(i * SIGNAL_LENGTH) + 1] = low[1];
		} else {
			voltomat_switch->raw[i * SIGNAL_LENGTH] = high[0];
			voltomat_switch->raw[(i * SIGNAL_LENGTH) + 1] = high[1];
		}
	}
	createFooter();
}

static int createCode(struct JsonNode *code) {
	buttonPressed codeRequest = {
		.buttonId = -1,
		.stateId = -1
	};
	double itmp = -1;

	if(json_find_number(code, "id", &itmp) == 0) {
		codeRequest.buttonId = (int)round(itmp);
	}

	if(json_find_number(code, "off", &itmp) == 0)
		codeRequest.stateId = 0;
	else if(json_find_number(code, "on", &itmp) == 0)
		codeRequest.stateId = 1;

	if (codeRequest.stateId == 0 || codeRequest.stateId == 1) {
		codeRequest.state = buttonStates[codeRequest.stateId];
	}

	if(codeRequest.buttonId == -1 || codeRequest.stateId == -1) {
		logprintf(LOG_ERR, "voltomat_switch: insufficient number of arguments");
		return EXIT_FAILURE;
	} else if(codeRequest.buttonId > 3 || codeRequest.buttonId < 0) {
		logprintf(LOG_ERR, "voltomat_switch: invalid id range");
		return EXIT_FAILURE;
	} else {
		createMessage(codeRequest);
		createRawCode(codeRequest);
		voltomat_switch->rawlen = RAW_LENGTH;
	}
	return EXIT_SUCCESS;
}

static void printHelp(void) {
	printf("\t -t --on\t\t\tsend an on signal\n");
	printf("\t -f --off\t\t\tsend an off signal\n");
	printf("\t -i --id=id\t\t\tcontrol a device with this id\n");
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void voltomatSwitchInit(void) {

	protocol_register(&voltomat_switch);
	protocol_set_id(voltomat_switch, "voltomat_switch");
	protocol_device_add(voltomat_switch, "voltomat_switch", "Voltomat power switch");
	voltomat_switch->devtype = SWITCH;
	voltomat_switch->hwtype = RF433;
	voltomat_switch->minrawlen = RAW_LENGTH;
	voltomat_switch->maxrawlen = RAW_LENGTH;
	voltomat_switch->maxgaplen = MAX_PULSE_LENGTH*PULSE_DIV;
	voltomat_switch->mingaplen = MIN_PULSE_LENGTH*PULSE_DIV;

	options_add(&voltomat_switch->options, 't', "on", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	options_add(&voltomat_switch->options, 'f', "off", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	options_add(&voltomat_switch->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^(3[012]?|[012][0-9]|[0-9]{1})$");


	voltomat_switch->parseCode=&parseCode;
	voltomat_switch->createCode=&createCode;
	voltomat_switch->printHelp=&printHelp;
	voltomat_switch->validate=&validate;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "voltomat_switch";
	module->version = "1.0";
	module->reqversion = "6.0";
	module->reqcommit = "84";
}

void init(void) {
	voltomatSwitchInit();
}
#endif
