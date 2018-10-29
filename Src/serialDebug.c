

#include "serialDebug.h"

#define SERIAL_DEBUG_SIZE 128
unsigned char serialDebugRxBuf[SERIAL_DEBUG_SIZE];
unsigned int rxBufOffset;

extern void byteToAscii(unsigned char * byte);
extern int appendCharsToString(unsigned char * theString, unsigned char * charsToAdd, unsigned int len, unsigned int maxLen);

extern LED_PTR led_array[NUM_LEDS]; 
extern BUTTON button; 
extern ENCODER encoder; 

unsigned char unrecognized[] = "\r\nwhatchu talkin bout willis?\r\n> ";
unsigned char crLineFeedCursor[] = "\r\n> ";
unsigned char crLineFeed[] = "\r\n";

void getLedValsFunc(unsigned char * msg);
void getLevelFunc(unsigned char * msg);
void getMuteFunc(unsigned char * msg);
void setDebugOnFunc(unsigned char * msg);
void setDebugOffFunc(unsigned char * msg);
void getCommandsFunc(unsigned char * msg);

DEBUG_MSG debugMessages[NUM_DEBUG_MESSAGES] = {
	{ "\r\nzoneLevel = ", "getLevel", &getLevelFunc, 1},
	{ "\r\nmuteState = ", "getMute", &getMuteFunc, 1},
	{ "\r\nLedLevels = ", "getLedVals", &getLedValsFunc, 1},
	{ "\r\nDebug Out ON", "debugOn", &setDebugOnFunc, 1},
	{ "\r\nDebugOut OFF", "debugOff", &setDebugOffFunc, 1},
	{ "\r\nAvailable : ", "help", &getCommandsFunc, 1}
};

extern DEBUG_MSG debugOutput;

void serialDebugRead(UART_HandleTypeDef * uart)
{
	unsigned char c;
	c = *(uart->pRxBuffPtr - 1);
	
	if ((c != '\r') && (c != '\n'))
	{
		serialDebugRxBuf[rxBufOffset++] = c;
		rxBufOffset %= sizeof(serialDebugRxBuf);
		
		HAL_UART_Transmit_IT(uart, &c, 1);        //echo back
	}
	else
	{	
		int msg = 0;
		for (msg; msg < NUM_DEBUG_MESSAGES; msg++)
		{
			if(!strncmp(debugMessages[msg].request, serialDebugRxBuf, rxBufOffset - 1)) //TODO, fix size of comparison.
			{
				debugMessages[msg].handler(debugMessages[msg].response);
				HAL_UART_Transmit(uart, debugMessages[msg].response, sizeof(debugMessages[msg].response), HAL_MAX_DELAY);  
				for (int i = MSG_RESPONSE_DEFAULT_LEN; i < MSG_RESPONSE_LEN; i++) debugMessages[msg].response[i] = 0;
				break;
			}			
		}		
		if(msg == NUM_DEBUG_MESSAGES)
		{
			HAL_UART_Transmit(uart, unrecognized, sizeof(unrecognized), HAL_MAX_DELAY);
		}		
		//clear the buffer, reset index
		for(int i = 0; i < sizeof(serialDebugRxBuf); i++) serialDebugRxBuf[i] = 0;
		rxBufOffset = 0;		
	}	
}

void getLedValsFunc(unsigned char * msg)
{
	for (int i = 0; i < sizeof(led_array) / sizeof(LED_PTR); i++)
	{
		unsigned char response[3];	
		unsigned char msByte, lsByte;
	
		msByte = (led_array[i].level & 0xf0) >> 4;
		byteToAscii(&msByte);
				
		lsByte = led_array[i].level & 0xf;
		byteToAscii(&lsByte);
		
		response[0] = msByte;
		response[1] = lsByte;
		response[2] = ' ';
		
		appendCharsToString(msg, response, sizeof(response), MSG_RESPONSE_LEN);				
	}
	appendCharsToString(msg, crLineFeedCursor, sizeof(crLineFeedCursor), MSG_RESPONSE_LEN);
}

void getLevelFunc(unsigned char * msg)
{
	unsigned char response[3];	
	unsigned char msByte, lsByte;
				
	msByte = (encoder.position & 0xf0) >> 4;
	byteToAscii(&msByte);
	
	lsByte = encoder.position & 0xf;
	byteToAscii(&lsByte);
	
	response[0] = msByte;
	response[1] = lsByte;
	response[2] = ' ';
				
	appendCharsToString(msg, response, sizeof(response), MSG_RESPONSE_LEN);					
	appendCharsToString(msg, crLineFeedCursor, sizeof(crLineFeedCursor), MSG_RESPONSE_LEN);
}

void getMuteFunc(unsigned char * msg)
{
	unsigned char on[2] = "on";
	unsigned char off[3] = "off";
				
	if(button.muted == 1)
		appendCharsToString(msg, on, sizeof(on), MSG_RESPONSE_LEN);
	else
		appendCharsToString(msg, off, sizeof(off), MSG_RESPONSE_LEN);
	
	appendCharsToString(msg, crLineFeedCursor, sizeof(crLineFeedCursor), MSG_RESPONSE_LEN);
}

void setDebugOnFunc(unsigned char * msg)
{
	debugOutput.active = ACTIVE;
	appendCharsToString(msg, crLineFeedCursor, sizeof(crLineFeedCursor), MSG_RESPONSE_LEN);
}

void setDebugOffFunc(unsigned char * msg)
{
	debugOutput.active = INACTIVE;
	appendCharsToString(msg, crLineFeedCursor, sizeof(crLineFeedCursor), MSG_RESPONSE_LEN);
}

void getCommandsFunc(unsigned char * msg)
{
	unsigned char space = ' ';
	
	for (int i = 0; i < GET_HELP; i++)
	{
		appendCharsToString(msg, debugMessages[i].request, sizeof(debugMessages[i].request), MSG_RESPONSE_LEN);
		appendCharsToString(msg, &space, sizeof(char), MSG_RESPONSE_LEN);
	}
	appendCharsToString(msg, crLineFeedCursor, sizeof(crLineFeedCursor), MSG_RESPONSE_LEN);	
}