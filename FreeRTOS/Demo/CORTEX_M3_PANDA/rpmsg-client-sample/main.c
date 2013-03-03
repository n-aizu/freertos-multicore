#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MessageQCopy.h"
#include "NameMap.h"

#include "partest.h"

#define LINUX_MSG     ("hello world!")
#define LINUX_MSG_LEN (sizeof(LINUX_MSG) - 1)

static void RpmsgTask(void *Parameters)
{
	int                 cnt;
	portTickType        lastWake;
	Bool                ok;
	MessageQCopy_Handle handle;
	Int                 ret;
//	UInt32				requestEndpoint = 50; 
	UInt32				requestEndpoint = MessageQCopy_ASSIGN_ANY; 
	UInt32              myEndpoint = 0;
	UInt32              remoteEndpoint;
	UInt16              len;
	/* Max data length of rpmsg send/recv is RP_MSG_PAYLOAD_SIZE. */
	/* rpmsg-client-sample Linux module send only 12byte at once, */
	/* so this function reserve 16byte buffer.                    */
//	Char                buffer[RP_MSG_PAYLOAD_SIZE];
	Char                buffer[16];

	/* 1st argument must be less than MessageQCopy_MAX_RESERVED_ENDPOINT */
	/* or MessageQCopy_ASSIGN_ANY.                                       */
	handle = MessageQCopy_create(requestEndpoint, &myEndpoint);
	NameMap_register("rpmsg-client-sample", myEndpoint);

	memset(buffer, 0, sizeof(buffer));

	lastWake = xTaskGetTickCount();
	for(cnt = 0; cnt < 100; cnt++){
		/* Set maximum data size that I can receive. */
		len = (UInt16)sizeof(buffer);

		ret = MessageQCopy_recv(handle, (Ptr)buffer, &len, &remoteEndpoint, portMAX_DELAY);
		/* MessageQCopy_recv was succeeded? */
		if(ret == MessageQCopy_S_SUCCESS &&
			len == LINUX_MSG_LEN &&
			memcmp(buffer, LINUX_MSG, LINUX_MSG_LEN) == 0){

			ok = TRUE;
		}
		else{
			ok = FALSE;
		}

		if(ok){
			vParTestToggleLED(0);
		}

		vTaskDelayUntil(&lastWake, 100);
		lastWake = xTaskGetTickCount();

		if(cnt % 2 == 0){
			memcpy(buffer, "HogeHoge", 8);
		}
		else{
			memcpy(buffer, "FugaFuga", 8);
		}

		MessageQCopy_send(remoteEndpoint, myEndpoint, (Ptr)buffer, 8, portMAX_DELAY);
		/* It is recognized whether MessageQCopy_send succeeded */
		/* by output of dmesg on Linux(Cortex-A9) side.         */
	}

	while(1){
		lastWake = xTaskGetTickCount();
		vTaskDelayUntil(&lastWake, 1000);

		vParTestToggleLED(1);
	}
}

/* Application entry point.          */
/* main0 is executed by only core 0. */
void main0(void)
{
	xTaskCreate(0, RpmsgTask, "rpmsg-client", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
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

