#define DEBUG_MODULE "raspserial"

#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "console.h"
#include "crtp.h"
#include "log.h"
#include "config.h"
#include "param.h"

#include "crtp_localization_service.h"
#include "debug.h"
#include "eprintf.h"

#include "stabilizer_types.h"
#include "stabilizer.h"
#include "log.h"
#include "deck.h"

typedef enum
{
  EXT_POSITION  = 0,
  GENERIC_TYPE  = 1,
} locsrvChannels_t;

static void locSrvCrtpCB(CRTPPacket* pk);
static void extPositionHandler(CRTPPacket* pk);

static bool isInit = 0;

static void raspserialInit()
{

	if(isInit)	return;
	crtpRegisterPortCB(CRTP_PORT_LOCALIZATION, locSrvCrtpCB);
	isInit = true;
}

static void locSrvCrtpCB(CRTPPacket* pk)
{
	extPositionHandler(pk);
}

static void extPositionHandler(CRTPPacket* pk)
{
	double data = *(pk->data);
	DEBUG_PRINT("Data received: %f \n", data);
}

static bool raspserialTest()
{
	DEBUG_PRINT("I2C Test Passed. \n");
	return 1;
}

static const DeckDriver raspserialDriver = {
	.name = "myRaspserial",
	.init = raspserialInit,
	.test = raspserialTest,
};

DECK_DRIVER(raspserialDriver);

