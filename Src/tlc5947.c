

#include "tlc5947.h"

void tlc5947Init(TLC5947 * tlc5947, SPI_HandleTypeDef * spi, GPIO_TypeDef * latchGpioGroup, uint16_t latchGpioPin)
{
	
	tlc5947->spi = spi;
	tlc5947->latchGpioGroup = latchGpioGroup;
	tlc5947->latchGpioPin = latchGpioPin;
	tlc5947->pwmBuffer.len = PWM_BUFLEN;	
}

void tlc5947SetPWMOutputsFromBuffer(TLC5947 * tlc5947)
{
	HAL_StatusTypeDef status;
	status = HAL_SPI_Transmit(tlc5947->spi, tlc5947->pwmBuffer.buf, tlc5947->pwmBuffer.len, HAL_TIMEOUT);  //first argument must be SPI INSTANCE, not e.g. SPI2
	if(status != HAL_OK)
	{
		//assert?
	}	
	
	HAL_GPIO_WritePin(tlc5947->latchGpioGroup, tlc5947->latchGpioPin, GPIO_PIN_SET);  //data clocked after high to low transition (see datasheet - tlc5947) 
	HAL_Delay(HAL_LATCH_DELAY);
	HAL_GPIO_WritePin(tlc5947->latchGpioGroup, tlc5947->latchGpioPin, GPIO_PIN_RESET);
	HAL_Delay(HAL_LATCH_DELAY);
	
}