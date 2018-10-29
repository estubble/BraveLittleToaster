#pragma once

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"

#define DEBOUNCE_TIME_MS 2

#define BUTTON_DEBUG_OUTPUT 

typedef struct{
	unsigned char muted; //muted
	GPIO_PinState buttonStatusOld; //encButtonStatusOld
	unsigned int lastReadTime; //lastEncReadTime	
	GPIO_TypeDef * gpioGroup; 
	uint16_t gpioPin;
}BUTTON;

void buttonInit(BUTTON * button, GPIO_TypeDef * gpioGroup, uint16_t gpioPin);
void buttonRead(BUTTON * button);