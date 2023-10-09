/*
 * Copyright (c) 2023  Pocuter Inc.
 * Copyright (c) 2016-2017  Moddable Tech, Inc.
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

#include "xsmc.h"
#include "xsHost.h"
#include "modGPIO.h"

#include "commodettoBitmap.h"
#include "commodettoPocoBlit.h"
#include "commodettoPixelsOut.h"
#include "stddef.h"		// for offsetof macro
#include "mc.xs.h"			// for xsID_ values
#include "mc.defines.h"

#include "modSPI.h"

#define modLog(foo)
#define modLogInt(foo)
#define modLogHex(foo)

// SH8601Z Commands
#define SH8601Z_CMD_SLEEPIN			0x10
#define SH8601Z_CMD_SLEEPOUT		0x11
#define SH8601Z_CMD_SETCOLUMN 		0x2A
#define SH8601Z_CMD_SETROW    		0x2B
#define SH8601Z_CMD_WRITERAM   		0x2C

#define SH8601Z_CMD_NORMALDISPLAY 	0x13
#define SH8601Z_CMD_INVERTOFF 		0x20
#define SH8601Z_CMD_INVERTON 		0x21
#define SH8601Z_CMD_DISPLAYALLOFF 	0x22
#define SH8601Z_CMD_DISPLAYALLON  	0x23

#define SH8601Z_CMD_DISPLAYOFF		0x28
#define SH8601Z_CMD_DISPLAYON		0x29

#define SH8601Z_CMD_TEARINGOFF		0x34
#define SH8601Z_CMD_TEARINGON		0x35
#define SH8601Z_CMD_IDLEON			0x35
#define SH8601Z_CMD_IDLEOFF			0x35
#define SH8601Z_CMD_PIXELFORMAT		0x3A

#define SH8601Z_CMD_DEEPSTANDBY		0x4F
#define SH8601Z_CMD_BRIGHTNESS		0x51
#define SH8601Z_CMD_CTRL1			0x53

#define SH8601Z_CMD_SPIMODE			0xC4

#define SH8601Z_CMD_NULL			0x00

#ifndef MODDEF_SH8601Z_CS_PORT
	#define MODDEF_SH8601Z_CS_PORT NULL
#endif
#ifndef MODDEF_SH8601Z_RST_PORT
	#define MODDEF_SH8601Z_RST_PORT NULL
#endif
#ifndef MODDEF_SH8601Z_OFFSET_COLUMN
	#define MODDEF_SH8601Z_OFFSET_COLUMN (0)
#endif
#ifndef MODDEF_SH8601Z_OFFSET_ROW
	#define MODDEF_SH8601Z_OFFSET_ROW (0)
#endif
#ifndef MODDEF_SH8601Z_HZ
	#define MODDEF_SH8601Z_HZ (20000000)
#endif
#ifndef MODDEF_SH8601Z_QSPI
	#define MODDEF_SH8601Z_QSPI (0)
#endif

#ifndef MODDEF_SH8601Z_INITIALIZATION
/* Default init sequence: 128x128 */
static const uint8_t gSH8601ZInitialization[] = {
	SH8601Z_CMD_SLEEPOUT, 0,
	SH8601Z_CMD_NORMALDISPLAY, 0,	
	SH8601Z_CMD_PIXELFORMAT, 1, 0x55,
	SH8601Z_CMD_CTRL1, 1, 0x20,
	SH8601Z_CMD_BRIGHTNESS, 1, 0x00,
	SH8601Z_CMD_DISPLAYON, 0,
	SH8601Z_CMD_BRIGHTNESS, 1, 0xFF,
	SH8601Z_CMD_NULL
};
#else
static const uint8_t gSH8601ZInitialization[] = MODDEF_SH8601Z_INITIALIZATION;
#endif

#ifdef MODDEF_SH8601Z_CS_PIN
	#define SCREEN_CS_ACTIVE		modGPIOWrite(&sd->cs, 0)
	#define SCREEN_CS_DEACTIVE		modGPIOWrite(&sd->cs, 1)
	#define SCREEN_CS_INIT			modGPIOInit(&sd->cs, MODDEF_SH8601Z_CS_PORT, MODDEF_SH8601Z_CS_PIN, kModGPIOOutput); \
									SCREEN_CS_DEACTIVE
#else
	#define SCREEN_CS_ACTIVE
	#define SCREEN_CS_DEACTIVE
	#define SCREEN_CS_INIT
#endif

#ifdef MODDEF_SH8601Z_RST_PIN
	#define SCREEN_RST_ACTIVE		modGPIOWrite(&sd->rst, 0)
	#define SCREEN_RST_DEACTIVE		modGPIOWrite(&sd->rst, 1)
	#define SCREEN_RST_INIT			modGPIOInit(&sd->rst, MODDEF_SH8601Z_RST_PORT, MODDEF_SH8601Z_RST_PIN, kModGPIOOutput); \
									SCREEN_RST_DEACTIVE
#else
	#define SCREEN_RST_ACTIVE
	#define SCREEN_RST_DEACTIVE
	#define SCREEN_RST_INIT
#endif

typedef struct {
	PixelsOutDispatch			dispatch;

	modSPIConfigurationRecord	spiConfig;

#ifdef MODDEF_SH8601Z_CS_PIN
	modGPIOConfigurationRecord	cs;
#endif
#ifdef MODDEF_SH8601Z_RST_PIN
	modGPIOConfigurationRecord	rst;
#endif
} spiDisplayRecord, *spiDisplay;


static bool g_IsStreaming = false;
static uint8_t g_ScreenBrightness = 0;
static uint8_t g_ScreenRotation = 0;
static bool g_IsScreenAwake = false;
static bool g_IsScreenEnabled = false;


static void sh8601zChipSelect(uint8_t active, modSPIConfiguration config);

static void sh8601zInit(spiDisplay sd);
static void sh8601zCommand(spiDisplay sd, uint8_t command, const uint8_t *data, uint16_t count, bool qspi);

static void sh8601zBegin(void *refcon, CommodettoCoordinate x, CommodettoCoordinate y, CommodettoDimension w, CommodettoDimension h);
static void sh8601zContinue(void *refcon);
static void sh8601zEnd(void *refcon);
static void sh8601zSend(void *refcon, PocoPixel *pixels, uint32_t byteLength);

#if kCommodettoBitmapFormat == kCommodettoBitmapRGB565LE
static void sh8601zSend_16LE(PocoPixel *pixels, int byteLength, void *refcon);
static const PixelsOutDispatchRecord gPixelsOutDispatch_16BE ICACHE_RODATA_ATTR = {
	sh8601zBegin,
	sh8601zContinue,
	sh8601zEnd,
	sh8601zSend_16LE,
	NULL
};
#else
	#error rgb565le pixels required
#endif

void xs_sh8601z_destructor(void *data)
{
	if (data) {
		spiDisplay sd = (spiDisplay)data;

		uint8_t byte = 0;
		sh8601zCommand( sd, SH8601Z_CMD_BRIGHTNESS, &byte, 1, false );
		sh8601zCommand( sd, SH8601Z_CMD_DISPLAYOFF, NULL, 0, false );
		sh8601zCommand( sd, SH8601Z_CMD_SLEEPIN, NULL, 0, false );

		modSPIUninit(&sd->spiConfig);
		c_free(data);
	}

	// clear initial state values
	g_IsStreaming = false;
	g_ScreenBrightness = 0;
	g_ScreenRotation = 0;
	g_IsScreenEnabled = false;	
	g_IsScreenAwake = false;
}

void xs_sh8601z(xsMachine *the)
{
	modLog("xs_sh8601z()");
	spiDisplay sd;

	sd = c_calloc(1, sizeof(spiDisplayRecord));
	if (!sd)
		xsUnknownError("no memory");

	xsmcSetHostData(xsThis, sd);

	// initialize spi driver -- CS pin is controlled manually
	modSPIConfig(sd->spiConfig, MODDEF_SH8601Z_HZ, MODDEF_SH8601Z_SPI_PORT, 0, -1, NULL );

	// enable QSPI driver mode
	if( MODDEF_SH8601Z_QSPI )
		sd->spiConfig.quad = true;

	// assign dispatch method
	sd->dispatch = (PixelsOutDispatch)&gPixelsOutDispatch_16BE;

	sh8601zInit(sd);

	// set initial state values
	g_IsStreaming = false;
	g_ScreenBrightness = 255;
	g_ScreenRotation = 0;
	g_IsScreenEnabled = true;	
	g_IsScreenAwake = true;
}

void xs_sh8601z_begin(xsMachine *the)
{
	modLog("xs_sh8601z_begin()");
	spiDisplay sd = xsmcGetHostData(xsThis);
	CommodettoCoordinate x = (CommodettoCoordinate)xsmcToInteger(xsArg(0));
	CommodettoCoordinate y = (CommodettoCoordinate)xsmcToInteger(xsArg(1));
	CommodettoDimension w = (CommodettoDimension)xsmcToInteger(xsArg(2));
	CommodettoDimension h = (CommodettoDimension)xsmcToInteger(xsArg(3));

	sh8601zBegin(sd, x, y, w, h);
}

void xs_sh8601z_send(xsMachine *the)
{
	modLog("xs_sh8601z_send()");
	spiDisplay sd = xsmcGetHostData(xsThis);
	int argc = xsmcArgc;
	const uint8_t *data;
	xsUnsignedValue count;

	xsmcGetBufferReadable(xsArg(0), (void **)&data, &count);

	if (argc > 1) {
		xsIntegerValue offset = xsmcToInteger(xsArg(1));

		if ((xsUnsignedValue)offset >= count)
			xsUnknownError("bad offset");
		data += offset;
		count -= offset;
		if (argc > 2) {
			xsIntegerValue c = xsmcToInteger(xsArg(2));
			if (c > count)
				xsUnknownError("bad count");
			count = c;
		}
	}
	(sd->dispatch->doSend)((PocoPixel *)data, count, sd);
}

void xs_sh8601z_end(xsMachine *the)
{
	spiDisplay sd = xsmcGetHostData(xsThis);
	sh8601zEnd(sd);
}

void xs_sh8601z_adaptInvalid(xsMachine *the)
{
}

void xs_sh8601z_get_pixelFormat(xsMachine *the)
{
	xsmcSetInteger(xsResult, kCommodettoBitmapFormat);
}

void xs_sh8601z_get_width(xsMachine *the)
{
	xsmcSetInteger(xsResult, MODDEF_SH8601Z_WIDTH);
}

void xs_sh8601z_get_height(xsMachine *the)
{
	xsmcSetInteger(xsResult, MODDEF_SH8601Z_HEIGHT);
}

void xs_sh8601z_get_brightness(xsMachine *the) {
	xsmcSetInteger(xsResult, g_ScreenBrightness );
}

void xs_sh8601z_set_brightness(xsMachine *the) {
	spiDisplay sd = xsmcGetHostData(xsThis);
	g_ScreenBrightness = (uint8_t)xsmcToInteger(xsArg(0));
	sh8601zCommand( sd, SH8601Z_CMD_BRIGHTNESS, &g_ScreenBrightness, 1, false );
}


void xs_sh8601z_get_enabled(xsMachine *the) {
	xsmcSetInteger(xsResult, g_IsScreenEnabled );
}

void xs_sh8601z_set_enabled(xsMachine *the) {
	spiDisplay sd = xsmcGetHostData(xsThis);
	g_IsScreenEnabled = (uint8_t)xsmcToBoolean(xsArg(0));
	sh8601zCommand( 
		sd, 
		g_IsScreenEnabled ? SH8601Z_CMD_DISPLAYON : SH8601Z_CMD_DISPLAYOFF, 
		NULL, 0, 
		false
	 );
}

void xs_sh8601z_get_sleep(xsMachine *the) {
	xsmcSetInteger(xsResult, !g_IsScreenAwake );
}

void xs_sh8601z_set_sleep(xsMachine *the) {
	spiDisplay sd = xsmcGetHostData(xsThis);
	g_IsScreenAwake = !(uint8_t)xsmcToBoolean(xsArg(0));
	sh8601zCommand( 
		sd, 
		g_IsScreenAwake ? SH8601Z_CMD_SLEEPOUT : SH8601Z_CMD_SLEEPIN, 
		NULL, 0, 
		false
	 );
}


void xs_sh8601z_get_clut(xsMachine *the)
{
	// returns undefined
}

void xs_sh8601z_set_clut(xsMachine *the)
{
	xsUnknownError("unsupported");
}

void xs_sh8601z_get_c_dispatch(xsMachine *the)
{
	xsResult = xsThis;
}

#if kCommodettoBitmapFormat == kCommodettoBitmapRGB565LE
// caller provides little-endian pixels, convert to big-endian for display
void sh8601zSend_16LE(PocoPixel *pixels, int byteLength, void *refcon)
{
	modLog("sh8601zSend_16LE(...)");
	spiDisplay sd = refcon;
	modSPITxSwap16(&sd->spiConfig, (void *)pixels, (byteLength < 0) ? -byteLength : byteLength);
}
#endif

void sh8601zCommand(spiDisplay sd, uint8_t command, const uint8_t *data, uint16_t count, bool qspi)
{
	modLog("sh8601zCommand(...)");
	/* HACK */
	//modSPIFlush();

	// 32-bit aligned command segment + 32-byte data buffer
	uint8_t cmd32_data[ 4 + 32 ];
	memset(&cmd32_data, 0x00, sizeof(cmd32_data));

	// format command header (LSB)
	cmd32_data[0] = qspi ? 0x32 : 0x02;
	cmd32_data[1] = 0x00;
	cmd32_data[2] = command;
	cmd32_data[3] = 0x00;

	// truncate command data length to 32-bytes
	// and copy data to packet buffer
	if( count > 32 ) count = 32;
	if( count ) c_memcpy(&cmd32_data[4], data, count );

	// sync send 32-byte aligned command packet and data
	SCREEN_CS_ACTIVE;
	modSPITxRx(&sd->spiConfig, (uint8_t *)&cmd32_data, 4 + count );
	if( !g_IsStreaming ) SCREEN_CS_DEACTIVE;
}

void xs_sh8601z_command(xsMachine *the)
{
	modLog("xs_sh8601z_command()");
	spiDisplay sd = xsmcGetHostData(xsThis);
	uint8_t command = (uint8_t)xsmcToInteger(xsArg(0));
	uint16_t dataSize = 0;
	uint8_t *data = NULL;
	bool qspi = false;

	if (xsmcArgc > 1) {
		dataSize = (uint16_t)xsmcGetArrayBufferLength(xsArg(1));
		data = xsmcToArrayBuffer(xsArg(1));
	}

	if (xsmcArgc > 2) {
		qspi = xsmcToInteger(xsArg(2));
	}

	sh8601zCommand(sd, command, data, dataSize, qspi);
}

void sh8601zInit(spiDisplay sd)
{
	modLog("sh8601zInit(...) - QSPI");
	modLogInt( sd->spiConfig.quad );

	uint8_t i = 0;

	SCREEN_CS_INIT;
	SCREEN_RST_INIT;

	modSPIInit(&sd->spiConfig);

	SCREEN_RST_DEACTIVE;
	modDelayMicroseconds(10000);
	SCREEN_RST_ACTIVE;
	modDelayMicroseconds(10000);
	SCREEN_RST_DEACTIVE;
	modDelayMicroseconds(10000);

	while (i < sizeof (gSH8601ZInitialization)) {
		uint8_t command, count;
		uint8_t data[16];

		command = c_read8(&gSH8601ZInitialization[i++]);
		if (SH8601Z_CMD_NULL == command)
			break;

		count = c_read8(&gSH8601ZInitialization[i++]);
		if (count > 0) {
			c_memcpy(data, gSH8601ZInitialization + i, count);
			i += count;
		}

		sh8601zCommand(sd, command, data, count, false);
		modDelayMicroseconds( 1000 );
	}

	SCREEN_CS_DEACTIVE;
}

void sh8601zChipSelect(uint8_t active, modSPIConfiguration config)
{
	modLog("sh8601zChipSelect(...)");
	return;
	spiDisplay sd = (spiDisplay)(((char *)config) - offsetof(spiDisplayRecord, spiConfig));

	if (active)
		SCREEN_CS_ACTIVE;
	else if( !g_IsStreaming )
		SCREEN_CS_DEACTIVE;
}

void sh8601zBegin(void *refcon, CommodettoCoordinate x, CommodettoCoordinate y, CommodettoDimension w, CommodettoDimension h)
{
	modLog("sh8601zBegin(...)");

	spiDisplay sd = refcon;
	uint8_t data[4];
	uint16_t xMin, xMax, yMin, yMax;

	xMin = x + MODDEF_SH8601Z_OFFSET_COLUMN;
	yMin = y + MODDEF_SH8601Z_OFFSET_ROW;
	xMax = xMin + w - 1;
	yMax = yMin + h - 1;

	data[0] = xMin >> 8;
	data[1] = xMin & 0xFF;
	data[2] = xMax >> 8;
	data[3] = xMax & 0xFF;
	sh8601zCommand(sd, SH8601Z_CMD_SETCOLUMN, data, sizeof(data), false);

	data[0] = yMin >> 8;
	data[1] = yMin & 0xFF;
	data[2] = yMax >> 8;
	data[3] = yMax & 0xFF;
	sh8601zCommand(sd, SH8601Z_CMD_SETROW, data, sizeof(data), false);
	
	g_IsStreaming = true;
	sh8601zCommand(sd, SH8601Z_CMD_WRITERAM, NULL, 0, sd->spiConfig.quad);
}

void sh8601zContinue(void *refcon)
{
	sh8601zEnd(refcon);
}

void sh8601zEnd(void *refcon)
{
	spiDisplay sd = refcon;
	g_IsStreaming = false;
	SCREEN_CS_DEACTIVE;
}
