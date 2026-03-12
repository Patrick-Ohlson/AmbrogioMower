/* test_sci.c — SCI serial echo test */
#include "test_sci.h"
#include "../iodefine.h"
#include "../micros.h"
#include "../sci.h"

void SCI_Test(void)
{
	INT8  cSelection;
	SendString((unsigned char *)"0");
	while(1)
		{

			cSelection = GetChar();

			PutChar(cSelection);
		}
}
