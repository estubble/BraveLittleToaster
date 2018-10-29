//rgb.h
#pragma once

//#define NUM_LEDS 7 //should be defined in main.h
#define LED_TOTAL_BYTE_WIDTH 9 
#define LED_BYTES_PER_COLOR 3
#define MAX_LED_LEVEL 4095
#define NUM_PRIMARY_COLORS 3
#define U8_MAX 255

typedef struct //color
{
	unsigned char r[3];
	unsigned char b[3];
	unsigned char g[3];	
}COLOR;

typedef struct 
{
	unsigned char * r[3];
	unsigned char * b[3];
	unsigned char * g[3];	
	unsigned int level;
}LED_PTR;

enum
{
	RED = 0,
	GREEN,
	BLUE,
	YELLOW,
	VIOLET,
	LIGHT_BLUE,
	WHITE,
	MAX_COLOR_STRUCTS
};


unsigned int rgbTransitionLEDColor(COLOR color, LED_PTR lednum);
void rgbSetLEDColorAndLevel(COLOR color, LED_PTR lednum);
void rgbCalculateLedLevels(unsigned char zoneLevel, LED_PTR led_array[], unsigned int num_leds);
void rgbDefineCompositeColors(void);

static void rgbIntToLedBytes(int * theInt, unsigned char * ledBytes);
static void rgbColorBytesToInt(unsigned char * ledBytes, int * theInt);
static void rgbStepIntToTarget(int * theInt, int targetInt, int step);
static void rgbColorToIntArray(COLOR * color, int * subLevels);
static void rgbIntArrayToLedBytes(int * subLevels, LED_PTR led);
static void rgbLedToIntArray(LED_PTR * led, int * subLevels);
