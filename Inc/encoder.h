#pragma once

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"

#define ENC_MULTIPLIER 16
#define U8_MAX 255

#define ENCODER_DEBUG_OUTPUT 

typedef struct{
	int intCurrentPosition; //intEncPos
	int intPrevPosition;    //prevEncPos
	unsigned char position; //encPos
	GPIO_PinState aStatusPrev;
	GPIO_PinState bStatusPrev;
	unsigned int lastReadTime; //lastEncReadTime
	GPIO_TypeDef * aGpioGroup; 
	uint16_t aGpioPin;
	GPIO_TypeDef * bGpioGroup; 
	uint16_t bGpioPin;
}ENCODER;

void encoderInit(ENCODER * encoder, GPIO_TypeDef * encAGpioGroup, GPIO_TypeDef * encBGpioGroup, uint16_t encAGpioPin, uint16_t encBGpioPin);
void encoderRead(ENCODER * encoder);



