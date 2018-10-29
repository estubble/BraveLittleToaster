
#include "util.h"

int appendCharsToString(unsigned char * theString , unsigned char * charsToAdd, unsigned int len, unsigned int maxLen)
{
	unsigned int cIndex = 0;
	int success = 0;
	unsigned char * cPtr = charsToAdd;
	int i = 0;
	
	//find first zero char in theString
	for(i = 0 ; i < maxLen ; i++, cIndex++)
	{
		if (*(theString + i) == 0)  break;
	}
	
	if (i == maxLen) success = -1;
	
	//copy len chars to the string from charsToAdd
	for(i = 0 ; i < len; i++, cIndex++)
	{
		if ( ((cIndex + theString) - theString) >= maxLen)
		{
			success = -1;
			break;
		}
		if ((*cPtr == 0)) break;
		*(theString + cIndex) = *cPtr++;
	}
	return success;	
}

void byteToAscii(unsigned char * byte)
{
	if (*byte < 10) *byte += '0';
	else
	{
		*byte -= 10;
		*byte += 'a';				
	}
	
}

int strncmp(const char *s1, const char *s2, unsigned int n)
{
	const unsigned char *c1 = (const unsigned char *)s1;
	const unsigned char *c2 = (const unsigned char *)s2;
	unsigned char ch;
	int d = 0;

	while (n--) 
	{
		d = (int)(ch = *c1++) - (int)*c2++;
		if (d || !ch)
			break;
	}

	return d;
}