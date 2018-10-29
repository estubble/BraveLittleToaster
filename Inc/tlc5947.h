#pragma once

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f7xx_hal_spi.h"

#define PWM_BUFLEN 72
#define HAL_TIMEOUT 1000
#define HAL_LATCH_DELAY 1

typedef struct 
{
	unsigned char buf[PWM_BUFLEN];
	unsigned char len;
}PWM_BUFFER;

typedef struct{
	PWM_BUFFER pwmBuffer;
	SPI_HandleTypeDef * spi; 
	GPIO_TypeDef * latchGpioGroup; 
	uint16_t latchGpioPin;
}TLC5947;

void tlc5947Init(TLC5947 * tlc5947, SPI_HandleTypeDef * spi, GPIO_TypeDef * latchGpioGroup, uint16_t latchGpioPin);
void tlc5947SetPWMOutputsFromBuffer(TLC5947 * tlc5947);