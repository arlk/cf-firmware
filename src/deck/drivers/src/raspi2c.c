#define DEBUG_MODULE "raspi2c"

#include "FreeRTOS.h"
#include "task.h"
#include "console.h"
#include "config.h"
#include "param.h"

#include "raspi2c.h"
#include "i2cdev.h"
#include "debug.h"
#include "eprintf.h"
#include "stdint.h"
#include "string.h"
#include "log.h"
#include "deck.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit = 0;

static void raspi2cInit()
{

	if(isInit)	return;
	i2cdevInit(I2C1_DEV);
	I2Cx = I2C1_DEV;
	devAddr = 0x04;
	
	uint8_t data;
	bool status;
	status = i2cdevRead16(I2Cx, devAddr, I2CDEV_NO_MEM_ADDR, 2, &data);
	if(status)
	{
		DEBUG_PRINT("Data received: %d \n", data);
	}
	isInit = 1;
	return;

}

static bool raspi2cTest()
{
	DEBUG_PRINT("I2C Test Passed. \n");
	return 1;
}

static const DeckDriver raspi2cDriver = {
	.name = "myRaspi2c",
	.init = raspi2cInit,
	.test = raspi2cTest,
};

DECK_DRIVER(raspi2cDriver);

