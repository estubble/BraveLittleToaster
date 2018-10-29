//rgb.c

#include "rgb.h"

COLOR colors[MAX_COLOR_STRUCTS];

static unsigned int rgbColorStructsPopulated = 0;

void rgbCalculateLedLevels(unsigned char zoneLevel, LED_PTR led_array[], unsigned int num_leds)
{

	int level = zoneLevel;
	int full_led_inc;
	
	full_led_inc = U8_MAX / num_leds;

	//blank current levels first
	for(int i = 0 ; i < num_leds ; i++) 
	{
		led_array[i].level = 0;
	}
	
	//now set new values
	for(int i = 0 ; i < num_leds ; i++)
	{
		if (level - full_led_inc < 0)
		{
			//this LED will not be full, calculate fill value
			int fillVal;
			fillVal = (MAX_LED_LEVEL / full_led_inc) * level;
			led_array[i].level = fillVal;
			break;
		}
		else //this LED is filled, get the level value less the LED increment and continue
		{
			level = level - full_led_inc;
			led_array[i].level = MAX_LED_LEVEL;
		}		
	}	
}

unsigned int rgbTransitionLEDColor(COLOR color, LED_PTR lednum)
{
	#define STEP 16
	
	LED_PTR led;
	
	led = lednum;//led_array[lednum];
	
	int R, targetR; //must be ints, not unsigned ints, so subtraction can go below zero and allow the comparison below to work
	int G, targetG;
	int B, targetB;
	
	int targetLevels[NUM_PRIMARY_COLORS];
	int currentLevels[NUM_PRIMARY_COLORS];
	
	if (rgbColorStructsPopulated == 0) rgbDefineCompositeColors();
	
	rgbColorToIntArray(&color, &targetLevels[0]);
	
	rgbLedToIntArray(&led, &currentLevels[0]);
	
	//if already at target, bail and return 1, indicating target composite color reached
	if((targetLevels[RED] == currentLevels[RED]) && 
		(targetLevels[GREEN] == currentLevels[GREEN]) && 
		(targetLevels[BLUE] == currentLevels[BLUE])) return 1;
	
	for (int i = 0; i < NUM_PRIMARY_COLORS; i++)
	{
		rgbStepIntToTarget(&currentLevels[i], targetLevels[i], STEP);
	}
	
	rgbIntArrayToLedBytes(&currentLevels[0], led);
	
	return 0;	
}

void rgbSetLEDColorAndLevel(COLOR color, LED_PTR lednum)
{
	//COLOR argument contains the MAX LEVEL for each of r,g,b
	//which must be scaled (keeping the ratio) according to the level arg
	
	int rSubLevel;
	int gSubLevel;
	int bSubLevel;
	
	int subLevels[NUM_PRIMARY_COLORS];
	
	LED_PTR led;
	led = lednum;
	
	//if (lednum >= NUM_LEDS) return;	
	if (led.level > MAX_LED_LEVEL) led.level = MAX_LED_LEVEL; //should never happen!  assert here?
	if (rgbColorStructsPopulated == 0) rgbDefineCompositeColors();
	
	//first, clear the led
	for(int i = 0 ; i < LED_BYTES_PER_COLOR ; i++)
	{
		*led.r[i] = *led.g[i] = *led.b[i] = 0;
	}
	
	rgbColorToIntArray(&color, &subLevels[0]);
	
	//multiply by the level argument, and divide by the max possible value 
	for(int i = 0 ; i < NUM_PRIMARY_COLORS ; i++) //3 = r,g,b
	{
		subLevels[i] *= led.level;
		subLevels[i] /= MAX_LED_LEVEL;
	}
	
	rgbIntArrayToLedBytes(&subLevels[0], led);
}


void rgbDefineCompositeColors(void)
{
	//top 4 bits no effect
	colors[BLUE].b[0] = colors[BLUE].b[1] = colors[BLUE].b[2] = 0xf;
	
	colors[RED].r[0] = colors[RED].r[1] = colors[RED].r[2] = 0xf;
	
	colors[GREEN].g[0] = colors[GREEN].g[1] = colors[GREEN].g[2] = 0xf;
	
	colors[VIOLET].g[0] = colors[VIOLET].g[1] = colors[VIOLET].g[2] = 0;
	colors[VIOLET].r[0] = colors[VIOLET].r[1] = colors[VIOLET].r[2] = 0xf;
	colors[VIOLET].b[0] = colors[VIOLET].b[1] = colors[VIOLET].b[2] = 0xf;
	
	colors[YELLOW].g[0] = colors[YELLOW].g[1] = colors[YELLOW].g[2] = 0xf;
	colors[YELLOW].r[0] = colors[YELLOW].r[1] = colors[YELLOW].r[2] = 0xf;
	colors[YELLOW].b[0] = colors[YELLOW].b[1] = colors[YELLOW].b[2] = 0;
	
	colors[LIGHT_BLUE].g[0] = colors[LIGHT_BLUE].g[1] = colors[LIGHT_BLUE].g[2] = 0xf;
	colors[LIGHT_BLUE].r[0] = colors[LIGHT_BLUE].r[1] = colors[LIGHT_BLUE].r[2] = 0;
	colors[LIGHT_BLUE].b[0] = colors[LIGHT_BLUE].b[1] = colors[LIGHT_BLUE].b[2] = 0xf;
	
	colors[WHITE].g[0] = colors[WHITE].g[1] = colors[WHITE].g[2] = 0xf;
	colors[WHITE].r[0] = colors[WHITE].r[1] = colors[WHITE].r[2] = 0xf;
	colors[WHITE].b[0] = colors[WHITE].b[1] = colors[WHITE].b[2] = 0xf;
	
	rgbColorStructsPopulated = 1;
}

//helper function.  assumes *ledBytes points to 0,1,2 bytes where 0 = msb and 2 = lsb
static void rgbIntToLedBytes(int * theInt, unsigned char * ledBytes)
{
	*ledBytes = (*theInt & 0xf00) >> 8;
	*(ledBytes + 1) = (*theInt & 0xf0) >> 4;
	*(ledBytes + 2) = (*theInt & 0xf);	
}

//helper function.  assumes *ledBytes points to 0,1,2 bytes where 0 = msb and 2 = lsb
static void rgbColorBytesToInt(unsigned char * ledBytes, int * theInt)
{
	int temp;
	
	temp = *ledBytes << 8;
	temp |= *(ledBytes + 1) << 4;
	temp |= *(ledBytes + 2);
	
	*theInt = temp;	
}

//helper function.  add or subtract step to/from theInt arg to work towards targetInt
static void rgbStepIntToTarget(int * theInt, int targetInt, int step)
{
	if (*theInt < targetInt)
	{
		*theInt += step;
		if (*theInt > targetInt) *theInt = targetInt;
	}
	else if (*theInt > targetInt)
	{
		*theInt -= step;
		if (*theInt < targetInt) *theInt = targetInt;
	}	
}

static void rgbColorToIntArray(COLOR * color, int * subLevels)
{
	rgbColorBytesToInt(&color->r[0], subLevels + RED);
	rgbColorBytesToInt(&color->g[0], subLevels + GREEN);
	rgbColorBytesToInt(&color->b[0], subLevels + BLUE);
}

static void rgbLedToIntArray(LED_PTR * led, int * subLevels)
{
	rgbColorBytesToInt(led->r[0], subLevels + RED);
	rgbColorBytesToInt(led->g[0], subLevels + GREEN);
	rgbColorBytesToInt(led->b[0], subLevels + BLUE);
}

static void rgbIntArrayToLedBytes(int * subLevels, LED_PTR led)
{
	rgbIntToLedBytes(subLevels + RED, led.r[0]);
	rgbIntToLedBytes(subLevels + GREEN, led.g[0]);
	rgbIntToLedBytes(subLevels + BLUE, led.b[0]);
}