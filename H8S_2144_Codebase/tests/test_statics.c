/* test_statics.c — Statics (initialized data) test
 *
 * Tests modification of the ucStr global (defined in main.c).
 */
#include "test_statics.h"
#include "../iodefine.h"
#include "../micros.h"
#include "../sci.h"

/* ucStr is defined in main.c */
extern char ucStr[];

void Statics_Test(void)
{
	volatile UINT16 uiCount;


    SendString((unsigned char*)"\r\n");
    SendString((unsigned char*)"\r\nStatics Test in progress:");
    SendString((unsigned char*)"\r\n");

	SendString((unsigned char*)"\r\nInitial str[] contents : ");
	SendString((unsigned char*)ucStr);
	SendString((unsigned char*)"\r\n");


	for (uiCount=0; uiCount<12; uiCount++)
	{
		ucStr[uiCount] = 'a' + (UINT8) uiCount;
	}
	ucStr[uiCount] = '\0';


    SendString((unsigned char*)"\r\nModified str[] contents : ");
	SendString((unsigned char*)ucStr);
    SendString((unsigned char*)"\r\n");

while(1);
}
