
#include "encoder.h"

#include "serialDebug.h"
#include "util.h"
extern DEBUG_MSG debugOutput;

void encoderInit(ENCODER * encoder, GPIO_TypeDef * aGpioGroup, GPIO_TypeDef * bGpioGroup, uint16_t aGpioPin, uint16_t bGpioPin)
{
	encoder->aGpioGroup = aGpioGroup;
	encoder->bGpioGroup = bGpioGroup;
	encoder->aGpioPin = aGpioPin;
	encoder->bGpioPin = bGpioPin;
}	

    
void encoderRead(ENCODER * encoder)
{	
	//es this code should capture transitions properly PROVIDED the RC filter in the following link exists on A and B:
	// https://www.allaboutcircuits.com/projects/how-to-use-a-rotary-encoder-in-a-mcu-based-project/
	
	uint32_t encReadTimeNow;
	
	encReadTimeNow = HAL_GetTick();
	encoder->lastReadTime = encReadTimeNow;
	
	GPIO_PinState encAStatus, encBStatus;
	encAStatus = HAL_GPIO_ReadPin(encoder->aGpioGroup, encoder->aGpioPin);  
	encBStatus = HAL_GPIO_ReadPin(encoder->bGpioGroup, encoder->bGpioPin);   
				
	if(encAStatus == GPIO_PIN_SET) //assumes we are set to interrupt on RISING edge of A only.  A not high = noise = ignore
	{
		if(encoder->aStatusPrev == GPIO_PIN_SET)//prev A needs to be valid also
		{
			if ((encBStatus == GPIO_PIN_SET)  && (encoder->bStatusPrev == GPIO_PIN_SET)) //indicates a complete CCW transition
			{
				encoder->intCurrentPosition -= ENC_MULTIPLIER;
			}
			else if ((encBStatus == GPIO_PIN_RESET) && (encoder->bStatusPrev == GPIO_PIN_RESET)) //indicates complete CW transition
			{
				encoder->intCurrentPosition += ENC_MULTIPLIER;	
			}				
		}	
	}
		
	encoder->bStatusPrev = encBStatus;
	encoder->aStatusPrev = encAStatus;
	
	if (encoder->intCurrentPosition < 0) encoder->intCurrentPosition = 0;  //wrapped around = no!
	else if(encoder->intCurrentPosition > U8_MAX) encoder->intCurrentPosition = U8_MAX;
	
	encoder->position = (uint8_t)encoder->intCurrentPosition;	
	encoder->intPrevPosition = encoder->position;
	
	if (debugOutput.active)
	{
		unsigned char buf[64];
		sprintf(&buf[0], "zone level = %d\r\n", encoder->position);
		appendCharsToString(debugOutput.response, &buf[0], sizeof(buf), sizeof(buf));
	}		
}