#pragma once

#include "stm32f7xx_hal_uart.h"
#include "button.h"
#include "encoder.h"
#include "rgb.h"
#include "util.h"

#define MSG_RESPONSE_DEFAULT_LEN 14
#define MSG_RESPONSE_LEN 128
#define MSG_RECEIVED_MAX_LEN 128

#define SUCCESS 0
#define FAILURE -1

#define ACTIVE 1
#define INACTIVE 0

typedef struct { //todo: add "base" response char array for each debug msg, which is a constant char array
	unsigned char response[MSG_RESPONSE_LEN];
	unsigned char request[MSG_RECEIVED_MAX_LEN];
	void(* handler)(unsigned char * str);
	unsigned char active;
} DEBUG_MSG;

enum 
{
	GET_LEVEL = 0,
	GET_MUTE,
	GET_LED_VALS,
	SET_DEBUG_ON,
	SET_DEBUG_OFF,
	GET_HELP,
	NUM_DEBUG_MESSAGES
} DEBUG_MESSAGES;