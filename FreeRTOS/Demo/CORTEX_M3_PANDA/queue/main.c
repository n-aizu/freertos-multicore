#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "partest.h"

static xQueueHandle msgQue = NULL;

static void RpmsgTask(void *Parameters)
{
	int                 cnt;
	unsigned char       msg;
	portTickType        lastWake;

	msg = 0;
	lastWake = xTaskGetTickCount();

	while(1){
		xQueueSend(msgQue, &msg, portMAX_DELAY);

		lastWake = xTaskGetTickCount();
		vTaskDelayUntil(&lastWake, 500);
	}
}

static void LedFlash(void *Parameters)
{
	unsigned char msg;
	portBASE_TYPE ret;
	unsigned portBASE_TYPE cpu = portGetCurrentCPU();

	while(1){
		msg = 1;
		ret = xQueueReceive(msgQue, &msg, portMAX_DELAY);
		if(ret == pdPASS && msg == 0){
			vParTestToggleLED(cpu);
		}
	}
}

/* Application entry point.          */
/* main0 is executed by only core 0. */
void main0(void)
{
	portBASE_TYPE rTaskCpu = 0;
//	portBASE_TYPE rTaskCpu = 1;
//	portBASE_TYPE rTaskCpu = portNO_SPECIFIC_PROCESSOR;

	xTaskCreate(rTaskCpu, RpmsgTask, "rpmsg-client", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(0, LedFlash, "LedFlash", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(1, LedFlash, "LedFlash2", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);

	msgQue = xQueueCreate(5, sizeof(unsigned char));
	vParTestInitialise();

	vTaskStartScheduler();
}

