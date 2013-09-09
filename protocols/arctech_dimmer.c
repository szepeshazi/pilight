/*
	Copyright (C) 2013 CurlyMo

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

#include "settings.h"
#include "log.h"
#include "protocol.h"
#include "binary.h"
#include "gc.h"
#include "arctech_dimmer.h"

void arctechDimCreateMessage(int id, int unit, int state, int all, int dimlevel) {
	arctech_dimmer->message = json_mkobject();
	json_append_member(arctech_dimmer->message, "id", json_mknumber(id));
	json_append_member(arctech_dimmer->message, "unit", json_mknumber(unit));
	if(dimlevel >= 0) {
		state = 1;
		json_append_member(arctech_dimmer->message, "dimlevel", json_mknumber(dimlevel));
	}
	if(all == 1)
		json_append_member(arctech_dimmer->message, "all", json_mknumber(all));
	if(state == 1)
		json_append_member(arctech_dimmer->message, "state", json_mkstring("on"));
	else
		json_append_member(arctech_dimmer->message, "state", json_mkstring("off"));
}

void arctechDimParseBinary(void) {
	int dimlevel = binToDecRev(arctech_dimmer->binary, 32, 35);
	int unit = binToDecRev(arctech_dimmer->binary, 28, 31);
	int state = arctech_dimmer->binary[27];
	int all = arctech_dimmer->binary[26];
	int id = binToDecRev(arctech_dimmer->binary, 0, 25);
	arctechDimCreateMessage(id, unit, state, all, dimlevel);
}

void arctechDimCreateLow(int s, int e) {
	int i;

	for(i=s;i<=e;i+=4) {
		arctech_dimmer->raw[i]=(PULSE_LENGTH);
		arctech_dimmer->raw[i+1]=(PULSE_LENGTH);
		arctech_dimmer->raw[i+2]=(PULSE_LENGTH);
		arctech_dimmer->raw[i+3]=(arctech_dimmer->pulse*PULSE_LENGTH);
	}
}

void arctechDimCreateHigh(int s, int e) {
	int i;

	for(i=s;i<=e;i+=4) {
		arctech_dimmer->raw[i]=(PULSE_LENGTH);
		arctech_dimmer->raw[i+1]=(arctech_dimmer->pulse*PULSE_LENGTH);
		arctech_dimmer->raw[i+2]=(PULSE_LENGTH);
		arctech_dimmer->raw[i+3]=(PULSE_LENGTH);
	}
}

void arctechDimClearCode(void) {
	arctechDimCreateLow(2,147);
}

void arctechDimCreateStart(void) {
	arctech_dimmer->raw[0]=PULSE_LENGTH;
	arctech_dimmer->raw[1]=(arctech_dimmer->header*PULSE_LENGTH);
}

void arctechDimCreateId(int id) {
	int binary[255];
	int length = 0;
	int i=0, x=0;

	length = decToBin(id, binary);
	for(i=0;i<=length;i++) {
		if(binary[i]==1) {
			x=((length-i)+1)*4;
			arctechDimCreateHigh(106-x, 106-(x-3));
		}
	}
}

void arctechDimCreateAll(int all) {
	if(all == 1) {
		arctechDimCreateHigh(106, 109);
	}
}

void arctechDimCreateState(int state) {
	if(state == 1) {
		arctechDimCreateHigh(110, 113);
	} else if(state == -1) {
		arctech_dimmer->raw[110]=(PULSE_LENGTH);
		arctech_dimmer->raw[111]=(PULSE_LENGTH);
		arctech_dimmer->raw[112]=(PULSE_LENGTH);
		arctech_dimmer->raw[113]=(PULSE_LENGTH);
	}
}

void arctechDimCreateUnit(int unit) {
	int binary[255];
	int length = 0;
	int i=0, x=0;

	length = decToBin(unit, binary);
	for(i=0;i<=length;i++) {
		if(binary[i]==1) {
			x=((length-i)+1)*4;
			arctechDimCreateHigh(130-x, 130-(x-3));
		}
	}
}

void arctechDimCreateDimlevel(int dimlevel) {
	int binary[255];
	int length = 0;
	int i=0, x=0;

	length = decToBin(dimlevel, binary);
	for(i=0;i<=length;i++) {
		if(binary[i]==1) {
			x=((length-i)+1)*4;
			arctechDimCreateHigh(146-x, 146-(x-3));
		}
	}
}

void arctechDimCreateFooter(void) {
	arctech_dimmer->raw[147]=(arctech_dimmer->footer*PULSE_LENGTH);
}

int arctechDimCreateCode(JsonNode *code) {
	int id = -1;
	int unit = -1;
	int state = -1;
	int all = 0;
	int dimlevel = -1;
	char *tmp;

	if(json_find_string(code, "id", &tmp) == 0)
		id=atoi(tmp);
	if(json_find_string(code, "off", &tmp) == 0)
		state=0;
	else if(json_find_string(code, "on", &tmp) == 0)
		state=1;
	if(json_find_string(code, "unit", &tmp) == 0)
		unit = atoi(tmp);
	if(json_find_string(code, "dimlevel", &tmp) == 0)
		dimlevel = atoi(tmp);
	if(json_find_string(code, "all", &tmp) == 0)
		all = 1;

	if(id == -1 || (unit == -1 && all == 0) || (dimlevel == -1 && state == -1)) {
		logprintf(LOG_ERR, "arctech_dimmer: insufficient number of arguments");
		return EXIT_FAILURE;
	} else if(id > 67108863 || id < 1) {
		logprintf(LOG_ERR, "arctech_dimmer: invalid id range");
		return EXIT_FAILURE;
	} else if((unit > 16 || unit < 0) && all == 0) {
		logprintf(LOG_ERR, "arctech_dimmer: invalid unit range");
		return EXIT_FAILURE;
	} else if(state == -1 && (dimlevel > 16 || dimlevel < 0)) {
		logprintf(LOG_ERR, "arctech_dimmer: invalid dimlevel range");
		return EXIT_FAILURE;
	} else if(dimlevel >= 0 && state == 0) {
		logprintf(LOG_ERR, "arctech_dimmer: dimlevel and state cannot be combined");
		return EXIT_FAILURE;
	} else {
		if(unit == -1 && all == 1) {
			unit = 0;
		}
		if(dimlevel >= 0) {
			state = -1;
		}
		arctechDimCreateMessage(id, unit, state, all, dimlevel);
		arctechDimCreateStart();
		arctechDimClearCode();
		arctechDimCreateId(id);
		arctechDimCreateAll(all);
		arctechDimCreateState(state);
		arctechDimCreateUnit(unit);
		if(dimlevel > -1) {
			arctechDimCreateDimlevel(dimlevel);
		}
		arctechDimCreateFooter();
	}
	return EXIT_SUCCESS;
}

void arctechDimPrintHelp(void) {
	printf("\t -u --unit=unit\t\t\tcontrol a device with this unit code\n");
	printf("\t -i --id=id\t\t\tcontrol a device with this id\n");
	printf("\t -a --all\t\t\tsend command to all devices with this id\n");
	printf("\t -d --dimlevel=dimlevel\t\tsend a specific dimlevel\n");
}

void arctechDimInit(void) {

	protocol_register(&arctech_dimmer);
	arctech_dimmer->id = malloc(18);
	strcpy(arctech_dimmer->id, "archtech_dimmers");
	protocol_add_device(arctech_dimmer, "kaku_dimmer", "KlikAanKlikUit Dimmers");
	arctech_dimmer->type = DIMMER;
	arctech_dimmer->header = 10;
	arctech_dimmer->pulse = 5;
	arctech_dimmer->footer = 38;
	arctech_dimmer->rawLength = 148;
	arctech_dimmer->lsb = 3;

	arctech_dimmer->bit = 0;
	arctech_dimmer->recording = 0;

	options_add(&arctech_dimmer->options, 'd', "dimlevel", has_value, config_value, "^([0-9]{1}|[1][0-6])$");
	options_add(&arctech_dimmer->options, 'a', "all", no_value, 0, NULL);
	options_add(&arctech_dimmer->options, 'u', "unit", has_value, config_id, "^([0-9]{1}|[1][0-5])$");
	options_add(&arctech_dimmer->options, 'i', "id", has_value, config_id, "^([0-9]{1,7}|[1-5][0-9]{7}|6([0-6][0-9]{6}|7(0[0-9]{5}|10([0-7][0-9]{3}|8([0-7][0-9]{2}|8([0-5][0-9]|6[0-3]))))))$");
	options_add(&arctech_dimmer->options, 't', "on", no_value, config_state, NULL);
	options_add(&arctech_dimmer->options, 'f', "off", no_value, config_state, NULL);

	arctech_dimmer->parseBinary=&arctechDimParseBinary;
	arctech_dimmer->createCode=&arctechDimCreateCode;
	arctech_dimmer->printHelp=&arctechDimPrintHelp;
}
