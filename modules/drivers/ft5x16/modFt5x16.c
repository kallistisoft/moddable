/*
 * Copyright (c) 2023 Pocuter, Inc.
 * Copyright (c) 2016-2021 Moddable Tech, Inc.
 *
 *   This file is part of the Moddable SDK Runtime.
 *
 *   The Moddable SDK Runtime is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   The Moddable SDK Runtime is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with the Moddable SDK Runtime.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
	FT5X16 I2C touch driver

	modeled on https://github.com/adafruit/Adafruit_FT5X16_Library
*/

#include "xsPlatform.h"
#include "xsmc.h"
#include "mc.xs.h"			// for xsID_ values
#include "mc.defines.h"
#include "modI2C.h"

#include "modPreference.h"

#ifndef MODDEF_FT5X16_HZ
	#define MODDEF_FT5X16_HZ 600000
#endif
#ifndef MODDEF_FT5X16_WIDTH
	#define MODDEF_FT5X16_WIDTH (240)
#endif
#ifndef MODDEF_FT5X16_HEIGHT
	#define MODDEF_FT5X16_HEIGHT (320)
#endif
#ifndef MODDEF_FT5X16_FLIPX
	#define MODDEF_FT5X16_FLIPX (false)
#endif
#ifndef MODDEF_FT5X16_FLIPY
	#define MODDEF_FT5X16_FLIPY (false)
#endif
#ifndef MODDEF_FT5X16_ADDR
	#define MODDEF_FT5X16_ADDR 0x38
#endif
#ifndef MODDEF_FT5X16_THRESHOLD
	#define MODDEF_FT5X16_THRESHOLD 128
#endif
#ifndef MODDEF_FT5X16_SDA
    #define MODDEF_FT5X16_SDA -1
#endif
#ifndef MODDEF_FT5X16_SCL
    #define MODDEF_FT5X16_SCL -1
#endif
#ifndef MODDEF_FT5X16_DX
    #define MODDEF_FT5X16_DX 0
#endif
#ifndef MODDEF_FT5X16_DY
    #define MODDEF_FT5X16_DY 0
#endif
#ifndef MODDEF_FT5X16_FITX
    #define MODDEF_FT5X16_FITX 1
#endif
#ifndef MODDEF_FT5X16_FITY
    #define MODDEF_FT5X16_FITY 1
#endif
#ifndef MODDEF_FT5X16_RAW
	#define MODDEF_FT5X16_RAW (false)
#endif
#if MODDEF_FT5X16_RAW || !defined(MODDEF_FT5X16_RAW_LEFT) || !defined(MODDEF_FT5X16_RAW_RIGHT) || !defined(MODDEF_FT5X16_RAW_TOP) || !defined(MODDEF_FT5X16_RAW_BOTTOM)
	#define MODDEF_FT5X16_CALIBRATE (false)
#else
	#define MODDEF_FT5X16_CALIBRATE (true)
#endif

#define FT5X16_REG_NUMTOUCHES 0x02
#define FT5X16_REG_THRESHHOLD 0x80
#define FT5X16_REG_CTRL 0x86
#define FT5X16_REG_CHIPID 0xA3
#define FT5X16_REG_VENDID 0xA8

struct ft5X16Record {
	modI2CConfigurationRecord i2c;

#if MODDEF_FT5X16_CALIBRATE
	int16_t			min_x;
	int16_t			max_x;
	int16_t			min_y;
	int16_t			max_y;
#endif
};
typedef struct ft5X16Record ft5X16Record;
typedef ft5X16Record *ft5X16;

void xs_FT5X16_destructor(void *data)
{
	if (data) {
		ft5X16 ft = data;
		modI2CUninit(&ft->i2c);
		c_free(data);
	}
}

void xs_FT5X16(xsMachine *the)
{
	ft5X16 ft;
	uint8_t data[2];
	uint8_t err;

	ft = c_calloc(1, sizeof(ft5X16Record));
	if (!ft) xsUnknownError("out of memory");
	xsmcSetHostData(xsThis, ft);

	xsmcVars(1);

	modI2CConfig(ft->i2c, MODDEF_FT5X16_HZ, MODDEF_FT5X16_SDA, MODDEF_FT5X16_SCL, MODDEF_FT5X16_ADDR, 250);
	modI2CInit(&ft->i2c);

	data[0] = FT5X16_REG_VENDID;
	err = modI2CWrite(&ft->i2c, data, 1, false);

	/* HACK HACK HACK HACK HACK HACK HACK HACK */
	if (err) {
		//xsUnknownError("write FT5X16_REG_VENDID failed");
		modLog("write FT5X16_REG_VENDID failed");
		return;
	}
	/* HACK HACK HACK HACK HACK HACK HACK HACK */


	err = modI2CRead(&ft->i2c, data, 1, true);
	if (err) xsUnknownError("read FT5X16_REG_VENDID failed");
	if ((0 != data[0]) && (1 != data[0]) && (2 != data[0]) && (17 != data[0])) xsUnknownError("bad FT5X16_REG_VENDID");

	data[0] = FT5X16_REG_CHIPID;
	err = modI2CWrite(&ft->i2c, data, 1, false);
	if (err) xsUnknownError("write FT5X16_REG_CHIPID failed");
	err = modI2CRead(&ft->i2c, data, 1, true);
	if (err) xsUnknownError("read FT5X16_REG_CHIPID failed");
	if ((6 != data[0]) && (100 != data[0])) xsUnknownError("bad FT5X16_REG_CHIPID");

	data[0] = FT5X16_REG_THRESHHOLD;
	data[1] = MODDEF_FT5X16_THRESHOLD;
	err = modI2CWrite(&ft->i2c, data, 2, true);
	if (err) xsUnknownError("write failed setting threshold");

	data[0] = FT5X16_REG_CTRL;
	data[1] = 1;		// switch to monitor mode when no touch
	err = modI2CWrite(&ft->i2c, data, 2, true);
	if (err) xsUnknownError("write failed setting ctrl");

#if MODDEF_FT5X16_CALIBRATE
	ft->min_x = MODDEF_FT5X16_RAW_LEFT;
	ft->max_x = MODDEF_FT5X16_RAW_RIGHT;
	ft->min_y = MODDEF_FT5X16_RAW_TOP;
	ft->max_y = MODDEF_FT5X16_RAW_BOTTOM;

	uint8_t prefType;
	int16_t values[4];
	uint16_t byteCountOut;
	if (modPreferenceGet("ft5X16", "calibrate", &prefType, (uint8_t *)values, sizeof(values), &byteCountOut)) {
		ft->min_x = values[0];
		ft->max_x = values[1];
		ft->min_y = values[2];
		ft->max_y = values[3];
	}
#endif
}

void xs_FT5X16_read(xsMachine *the)
{
	ft5X16 ft = xsmcGetHostData(xsThis);
	uint8_t data[32], err;
	uint8_t count, maxID = 0, i;

	xsmcVars(2);

	data[0] = FT5X16_REG_NUMTOUCHES;
	err = modI2CWrite(&ft->i2c, data, 1, false);
	if (err) {
		//xsUnknownError("write FT5X16_REG_NUMTOUCHES failed");
		modLog("write FT5X16_REG_NUMTOUCHES failed");
		return;
	}

	modI2CRead(&ft->i2c, data, 1, true);
	count = data[0] & 0x0F;

	xsmcGet(xsVar(0), xsArg(0), xsID_length);
	maxID = xsmcToInteger(xsVar(0)) - 1;

	xsmcSetInteger(xsVar(0), 0);
	for (i = 0; i <= maxID; i++) {
		xsmcGetIndex(xsVar(1), xsArg(0), i);
		xsmcSet(xsVar(1), xsID_state, xsVar(0));
	}

	if ((1 != count) && (2 != count))
		return;

	data[0] = 3;		// x registers followed by y registers, etc.
	modI2CWrite(&ft->i2c, data, 1, false);
	modI2CRead(&ft->i2c, data, count * 6, true);

	for (i = 0; i < count; i++) {
		uint8_t id = data[(i * 6) + 2] >> 4;
		uint8_t event = data[(i * 6) + 0] >> 6, state = 0;
		int16_t x = ((data[(i * 6) + 0] & 0x0F) << 8) | data[(i * 6) + 1];
		int16_t y = ((data[(i * 6) + 2] & 0x0F) << 8) | data[(i * 6) + 3];

		if (id > maxID)
			continue;

		if (0 == event)			// down
			state = 1;
		else if (2 == event)	// contact
			state = 2;
		else if (1 == event)	// lift (never happens?)
			state = 3;
		else
			continue;

		// reflect
		if (MODDEF_FT5X16_FLIPX)
			x = MODDEF_FT5X16_WIDTH - x;

		if (MODDEF_FT5X16_FLIPY)
			y = MODDEF_FT5X16_HEIGHT - y;

		// scale
		if (MODDEF_FT5X16_FITX && (240 != MODDEF_FT5X16_WIDTH))
			x = (x * MODDEF_FT5X16_WIDTH) / 240;
		if (MODDEF_FT5X16_FITY && (320 != MODDEF_FT5X16_HEIGHT))
			y = (y * MODDEF_FT5X16_HEIGHT) / 320;

#if MODDEF_FT5X16_DX
		x += MODDEF_FT5X16_DX;
		if (x > (MODDEF_FT5X16_WIDTH - 1))
			x = (MODDEF_FT5X16_WIDTH - 1);
		else if (x < 0)
			x = 0;
#endif

#if MODDEF_FT5X16_DY
		y += MODDEF_FT5X16_DY;
		if (y > (MODDEF_FT5X16_HEIGHT - 1))
			y = (MODDEF_FT5X16_HEIGHT - 1);
		else if (y < 0)
			y = 0;
#endif

#if MODDEF_FT5X16_CALIBRATE
		x = (x - ft->min_x) * ((float)(MODDEF_FT5X16_WIDTH - 1)) / (ft->max_x - ft->min_x);
		y = (y - ft->min_y) * ((float)(MODDEF_FT5X16_HEIGHT - 1)) / (ft->max_y - ft->min_y);

		if (x < 0)
			x = 0;
		else if (x > (MODDEF_FT5X16_WIDTH - 1))
			x = MODDEF_FT5X16_WIDTH - 1;

		if (y < 0)
			y = 0;
		else if (y > (MODDEF_FT5X16_HEIGHT - 1))
			y = MODDEF_FT5X16_HEIGHT - 1;
#endif

		// result
		xsmcGetIndex(xsVar(0), xsArg(0), id);
		xsmcSetInteger(xsVar(1), x);
		xsmcSet(xsVar(0), xsID_x, xsVar(1));
		xsmcSetInteger(xsVar(1), y);
		xsmcSet(xsVar(0), xsID_y, xsVar(1));
		xsmcSetInteger(xsVar(1), state);
		xsmcSet(xsVar(0), xsID_state, xsVar(1));
	}
}

