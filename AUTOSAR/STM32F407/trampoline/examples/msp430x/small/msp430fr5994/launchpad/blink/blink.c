#include "tpl_os.h"
#include "msp430.h"

#define APP_Task_blink_START_SEC_CODE
#include "tpl_memmap.h"

FUNC(int, OS_APPL_CODE) main(void)
{
	// Disable the GPIO power-on default high-impedance mode
	// to activate previously configured port settings
	PM5CTL0 &= ~LOCKLPM5;
  //set GPIO P1.0 (LED1) as an output
	P1DIR = 0x01;

	StartOS(OSDEFAULTAPPMODE);
	return 0;
}

TASK(blink)
{
	P1OUT ^= 0x01; //toggle GPIO P1.0 (LED1)
	TerminateTask();
}

#define APP_Task_blink_STOP_SEC_CODE
#include "tpl_memmap.h"
