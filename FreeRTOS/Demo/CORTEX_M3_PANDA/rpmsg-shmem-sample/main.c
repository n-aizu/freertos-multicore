#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MessageQCopy.h"
#include "NameMap.h"

#include "partest.h"


#define NAME_PLUS  "rpmsg-client-plus"
#define NAME_MINUS "rpmsg-client-minus"

#define ENDPT_PLUS  (60)
#define ENDPT_MINUS (61)


static void RpmsgTask(void *Parameters)
{
	portTickType        lastWake, delay;
	MessageQCopy_Handle handle;
	Int                 ret;
	UInt32			    requestEndpoint; 
	UInt32              myEndpoint = 0;
	UInt32              remoteEndpoint;
	UInt16              len;
	int                 led;
	char                add;
	volatile char       *data;
	unsigned int        msg, idx;

	if(strcmp(NAME_PLUS, (const char *)Parameters) == 0){
		led = 0;
		delay = 1000;
		add = 1;
		data = (volatile char *)0xa1000000;
		requestEndpoint = ENDPT_PLUS;
	}
	else{
		led = 1;
		delay = 500;
		add = -1;
		data = (volatile char *)(0xa1000000 + 0x800);
		requestEndpoint = ENDPT_MINUS;
	}

	/* 1st argument must be less than MessageQCopy_MAX_RESERVED_ENDPOINT */
	/* or MessageQCopy_ASSIGN_ANY.                                       */
	handle = MessageQCopy_create(requestEndpoint, &myEndpoint);
	NameMap_register((char *)Parameters, myEndpoint);

	lastWake = xTaskGetTickCount();
	while(1){
		/* Set maximum data size that I can receive. */
		len = (UInt16)sizeof(msg);

		ret = MessageQCopy_recv(handle, (Ptr)&msg, &len, &remoteEndpoint, portMAX_DELAY);
		if(ret != MessageQCopy_S_SUCCESS || len != (UInt16)sizeof(msg)){
			vTaskDelay(500);
			continue;
		}

		for(idx = 0; idx < msg; idx++){
			if(data[idx] != ' ' && data[idx] != '.' && data[idx] != ','){
				data[idx] += add;
			}
		}

		__asm volatile( "dsb" ::: "memory" );

		MessageQCopy_send(remoteEndpoint, myEndpoint, (Ptr)&msg, (UInt16)sizeof(msg), portMAX_DELAY);
	
		vTaskDelayUntil(&lastWake, delay);
		lastWake = xTaskGetTickCount();
		vParTestToggleLED(led);
	}
}

/* Application entry point.          */
/* main0 is executed by only core 0. */
void main0(void)
{
	xTaskCreate(0, RpmsgTask, NAME_PLUS, 1024, NAME_PLUS, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(1, RpmsgTask, NAME_MINUS, 1024, NAME_MINUS, tskIDLE_PRIORITY + 1, NULL);

	vParTestInitialise();

	vTaskStartScheduler();
}

/* Callback function for core 1.                      */
/* This function is called after vTaskStartScheduler  */
/* and before start task code(core 0 and core 1).     */
/* main1 is not need to defined if it is unnecessary. */
void main1(void)
{
	/* MessageQCopy_init must be called only once */
	/* at main0 or main1.                         */
	/* If MessageQCopy_init called at main0,      */
	/* Mailbox user 2 interrupt is handled on     */
	/* core 0. Meanwhile called at main1, Mailbox */
	/* user 2 interrupt is handled on core 1.     */
	MessageQCopy_init();
}

