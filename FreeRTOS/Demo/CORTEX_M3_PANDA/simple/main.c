#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "partest.h"

static unsigned portBASE_TYPE params[portNUM_PROCESSORS] = { 0, 1 };

static void LedFlash(void *Parameters)
{
	volatile int cnt;
	unsigned portBASE_TYPE led = portGetCurrentCPU();

	if(Parameters != 0){
		led = *(unsigned portBASE_TYPE *)Parameters;
	}
	else{
		led = portGetCurrentCPU();
	}

	while(1){
		vParTestToggleLED(led);
		vTaskDelay(1000);
	}
}

/* Application entry point.          */
/* main0 is executed by only core 0. */
void main0(void)
{
	xTaskCreate(0, LedFlash, "LedFlash", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(1, LedFlash, "LedFlash2", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
	/* If call xTaskCreate with portNO_SPECIFIC_PROCESSOR,         */
	/* the task is not bound for cpu core.                         */
	/* (It is necessary to define configUSE_CPU_UNBOUND_TASK as 1) */
//	xTaskCreate(portNO_SPECIFIC_PROCESSOR, LedFlash, "LedFlash", 1024, &params[0], tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(portNO_SPECIFIC_PROCESSOR, LedFlash, "LedFlash2", 1024, &params[1], tskIDLE_PRIORITY + 1, NULL);

	vParTestInitialise();

	vTaskStartScheduler();
}

/* Callback function for core 1.                      */
/* This function is called after vTaskStartScheduler  */
/* and before start task code(core 0 and core 1).     */
/* main1 is not need to defined if it is unnecessary. */
void main1(void)
{
}

