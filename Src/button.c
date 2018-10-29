
#include "button.h"

#include "serialDebug.h"
#include "util.h"
extern DEBUG_MSG debugOutput;

void buttonInit(BUTTON * button, GPIO_TypeDef * gpioGroup, uint16_t gpioPin)
{
	button->gpioGroup = gpioGroup;
	button->gpioPin = gpioPin;	
	button->buttonStatusOld = GPIO_PIN_SET; 
}

//assumes low/high transition defines button press
void buttonRead(BUTTON * button)
{
	
	uint32_t timeNow = 0;
	
	timeNow = HAL_GetTick();
	
	if ((timeNow - button->lastReadTime) > DEBOUNCE_TIME_MS)
	{
		GPIO_PinState encButtonStatus;
		encButtonStatus = HAL_GPIO_ReadPin(button->gpioGroup, button->gpioPin);   //es TODO: add references to these somewhere! See TM encoder struct stuff?
		if(encButtonStatus != button->buttonStatusOld)
		{
			if ((button->buttonStatusOld == GPIO_PIN_RESET) && (encButtonStatus == GPIO_PIN_SET)) //push/release transition
			{
				button->muted ^= 0x1;
				if (debugOutput.active)
				{
					unsigned char buf[64];
					sprintf(buf, "mute status = %d\r\n", button->muted);
					appendCharsToString(debugOutput.response, &buf[0], sizeof(buf), sizeof(buf));
				}
			}
			button->buttonStatusOld = encButtonStatus;
		}	
		button->lastReadTime = timeNow;
	}
	
}