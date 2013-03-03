/* This file is based on SYS/BIOS RPMsg code.
 *
 * Repositories:
 *  http://git.omapzoom.org/?p=repo/sysbios-rpmsg.git;a=summary
 *
 * The original license terms are as follows.
 */
/*
 * Copyright (c) 2011-2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       MessageQCopy.c
 *
 *  @brief      A simple copy-based MessageQ, to work with Linux virtio_rp_msg.
 *
 *  ============================================================================
 */

#include <string.h>

#include "MessageQCopy.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "FreeRTOS.h"
#include "list.h"
#include "queue.h"
#include "task.h"

/* =============================================================================
 * Structures & Enums
 * =============================================================================
 */

/* Various arbitrary limits: */
#define MAXMESSAGEQOBJECTS     256
#define MAXMESSAGEBUFFERS      512

#define QUE_LEN 256

/* Message Header: Must match mp_msg_hdr in virtio_rp_msg.h on Linux side. */
typedef struct MessageQCopy_MsgHeader {
    Bits32 srcAddr;                 /* source endpoint addr               */
    Bits32 dstAddr;                 /* destination endpoint addr          */
    Bits32 reserved;                /* reserved                           */
    Bits16 dataLen;                 /* data length                        */
    Bits16 flags;                   /* bitmask of different flags         */
    UInt8  payload[];               /* Data payload                       */
} __attribute__((packed)) MessageQCopy_MsgHeader;

typedef MessageQCopy_MsgHeader *MessageQCopy_Msg;

/* The MessageQCopy Object */
typedef struct MessageQCopy_Object {
	xQueueHandle qHndl;
} MessageQCopy_Object;

typedef struct {
	UInt16 token;
	Bits16 dataLen;
	Bits32 srcAddr;
	UInt8  *payload;
} Msgq_data;

static VirtQueue_Handle vQueFromHost;
static VirtQueue_Handle vQueToHost;
static MessageQCopy_Object mQueObj[MAXMESSAGEQOBJECTS];
static xList availBufList;
static portLOCK_TYPE virtQueLock;
static volatile unsigned int waitAvaileBuf = 0;


#define IS_VALID_MSGQUEOBJ(obj) ((obj)->qHndl != NULL)

static Bool MsgQueObjInit(MessageQCopy_Object *obj)
{
	obj->qHndl = xQueueCreate(QUE_LEN, sizeof(Msgq_data));
	return (obj->qHndl != NULL);
}

static Bool MessageQCopy_SendToTask(UInt16 token,
									MessageQCopy_Msg msg,
									portBASE_TYPE *switchCpu)
{
	Msgq_data     data;
	Bits32        endPnt;
	portBASE_TYPE ret;

	data.token = token;
	data.dataLen = msg->dataLen;
	data.srcAddr = msg->srcAddr;
	data.payload = msg->payload;

	if((endPnt = msg->dstAddr) >= MAXMESSAGEQOBJECTS ||
			!IS_VALID_MSGQUEOBJ(&mQueObj[endPnt])){
		return FALSE;
	}

	ret = xQueueSendToBackFromISR(mQueObj[endPnt].qHndl, &data, switchCpu);
	return (ret >= 0);
}

static Void callback_FromHost(VirtQueue_Handle vq)
{
    Int16            token;
    MessageQCopy_Msg msg;
    Bool             usedBufAdded = FALSE;
    Int              len;
	portBASE_TYPE    switchCpu;
	Bool             doSwitch[portNUM_PROCESSORS];
	int              i;

	memset(doSwitch, FALSE, sizeof(doSwitch));

	taskENTER_CRITICAL_NOT_RECURSIVE_FROM_ISR(&virtQueLock);

	/* Process all available buffers: */
	while((token = VirtQueue_getAvailBuf(vq, (Void **)&msg, &len)) >= 0){
		if(!MessageQCopy_SendToTask(token, msg, &switchCpu)){
			VirtQueue_addUsedBuf(vq, token, RP_MSG_BUF_SIZE);
			usedBufAdded = TRUE;
		}
		else if(switchCpu >= 0){
			doSwitch[switchCpu] = TRUE;
		}
	}

	if(usedBufAdded){
		/* Tell host we've processed the buffers: */
		VirtQueue_kick(vq);
	}

	taskEXIT_CRITICAL_NOT_RECURSIVE_FROM_ISR(&virtQueLock);

	for(i = 0; i < portNUM_PROCESSORS; i++){
		unsigned portBASE_TYPE curCpu = portGetCurrentCPU();

		if(doSwitch[i]){
			if(i == (Int)curCpu){
				vPortYieldFromISR();
			}
			else{
				portINTERRUPT_CORE(i);
			}
		}
	}
}

static Void callback_ToHost(VirtQueue_Handle vq)
{
	UInt16 avail;
	int    i;
	Bool   doSwitch[portNUM_PROCESSORS];
	unsigned portBASE_TYPE curCpu =  portGetCurrentCPU();
	
	taskENTER_CRITICAL_NOT_RECURSIVE_FROM_ISR(&virtQueLock);

	if(listLIST_IS_EMPTY(&availBufList) != pdFALSE){
		taskEXIT_CRITICAL_NOT_RECURSIVE_FROM_ISR(&virtQueLock);
		return;
	}

	if((avail = GET_AVAIL_COUNT(vq)) == 0){
		taskEXIT_CRITICAL_NOT_RECURSIVE_FROM_ISR(&virtQueLock);
		return;
	}

	memset(doSwitch, FALSE, sizeof(doSwitch));

	do{
		signed portBASE_TYPE ret;

		/* Because this function is not an application code,             */
		/* there is not the problem with using xTaskRemoveFromEventList. */
		ret = xTaskRemoveFromEventList(&availBufList, pdFALSE);
		if(ret >= 0){
			doSwitch[ret] = TRUE;
		}			
	}
	while(--avail > 0);

	taskEXIT_CRITICAL_NOT_RECURSIVE_FROM_ISR(&virtQueLock);

	for(i = 0; i < portNUM_PROCESSORS; i++){
		if(doSwitch[i]){
			if(i == (int)curCpu){
				vPortYieldFromISR();
			}
			else{
				portINTERRUPT_CORE(i);
			}
		}
	}
}


/* =============================================================================
 *  MessageQCopy Functions:
 * =============================================================================
 */

/*
 *  ======== MessasgeQCopy_init ========
 *
 *
 */
Int MessageQCopy_init()
{
	int i;

	portLOCK_INIT(&virtQueLock);

	for(i = 0; i < MAXMESSAGEQOBJECTS; i++){
		mQueObj[i].qHndl = NULL;
	}

	vListInitialise(&availBufList);

    /* Initialize Transport related objects: */

    /*
     * Note: order of these calls determines the virtqueue indices identifying
     * the vrings toHost and fromHost:  toHost is first!
     */
	vQueToHost = VirtQueue_create(callback_ToHost, ID_SELF_TO_A9);
	vQueFromHost = VirtQueue_create(callback_FromHost, ID_A9_TO_SELF);
	if(vQueToHost == NULL || vQueFromHost == NULL){
		return MessageQCopy_E_MEMORY;
	}

	VirtQueue_startup();
	return MessageQCopy_S_SUCCESS;
}

/*
 *  ======== MessageQCopy_create ========
 */
MessageQCopy_Handle MessageQCopy_create(UInt32 reserved, UInt32 * endpoint)
{
	MessageQCopy_Object *obj = NULL;
	Bool                found = FALSE;
	Int                 i;
	UInt16              queueIndex = 0;

	taskENTER_CRITICAL(&virtQueLock);

	if(reserved == MessageQCopy_ASSIGN_ANY){
		/* Search the array for a free slot above reserved: */
		for(i = MessageQCopy_MAX_RESERVED_ENDPOINT + 1;
			(i < MAXMESSAGEQOBJECTS) && (found == FALSE);
			i++){

			if(!IS_VALID_MSGQUEOBJ(&mQueObj[i])){
				queueIndex = i;
				found = TRUE;
				break;
			}
		}
	}
	else if((queueIndex = reserved) <= MessageQCopy_MAX_RESERVED_ENDPOINT){
		if(!IS_VALID_MSGQUEOBJ(&mQueObj[queueIndex])){
			found = TRUE;
		}
	}

	if(found){
		if(MsgQueObjInit(&mQueObj[queueIndex]) != FALSE){
			obj = &mQueObj[queueIndex];
			*endpoint = queueIndex;
		}
	}

	taskEXIT_CRITICAL(&virtQueLock);
	return obj;
}

/*
 *  ======== MessageQCopy_recv ========
 */
Int MessageQCopy_recv(MessageQCopy_Handle handle,
								Ptr data,
								UInt16 *len,
								UInt32 *rplyEndpt,
								portTickType timeout)
{
	Msgq_data     mdata;
	portBASE_TYPE qret;
	Int           ret;

	qret = (Int)xQueueReceive(handle->qHndl, &mdata, timeout);
	if(qret == (Int)pdPASS){
		if(*len > mdata.dataLen){
			*len = mdata.dataLen;
		}

		memcpy(data, mdata.payload, *len);
		*rplyEndpt = mdata.srcAddr;

		taskENTER_CRITICAL(&virtQueLock);
		VirtQueue_addUsedBuf(vQueFromHost, mdata.token, RP_MSG_BUF_SIZE);
		VirtQueue_kick(vQueFromHost);
		taskEXIT_CRITICAL(&virtQueLock);

		ret = MessageQCopy_S_SUCCESS;
	}
	else{
		ret = MessageQCopy_E_TIMEOUT;
	}

	return ret;
}

/*
 *  ======== MessageQCopy_send ========
 */
Int MessageQCopy_send(UInt32 dstEndpt,
						UInt32 srcEndpt,
						Ptr    data,
						UInt16 len,
						portTickType timeout)
{
    Int16            token;
    MessageQCopy_Msg msg;
    Int              length;
	xTimeOutType     tmchk;

	/* Send to remote processor: */
	taskENTER_CRITICAL(&virtQueLock);

	token = VirtQueue_getAvailBuf(vQueToHost, (Void **)&msg, &length);
	if(token < 0){
		if(timeout == 0){
			taskEXIT_CRITICAL(&virtQueLock);
			return MessageQCopy_E_TIMEOUT;
		}

		vTaskSetTimeOutState(&tmchk);
		waitAvaileBuf++;

		for(;;){
			portTickType waitEvent;

			/* Because this function is not an application code,          */
			/* there is not the problem with using vTaskPlaceOnEventList. */
			waitEvent = (timeout > 5) ? 5 : timeout;
			vTaskPlaceOnEventList(&availBufList, waitEvent);
			portYIELD_WITHIN_API();	
			taskEXIT_CRITICAL(&virtQueLock);

			taskENTER_CRITICAL(&virtQueLock);
			token = VirtQueue_getAvailBuf(vQueToHost, (Void **)&msg, &length);
			if(token < 0){
				if(xTaskCheckForTimeOut(&tmchk, &timeout) == pdFALSE){
					continue;
				}

				waitAvaileBuf--;
				taskEXIT_CRITICAL(&virtQueLock);
				return MessageQCopy_E_TIMEOUT;
			}

			waitAvaileBuf--;
			if(waitAvaileBuf == 0){
				/* No need to know be kicked about added buffers anymore */
				PROHIBIT_VRING_NOTIFY(vQueToHost);
			}

			break;
		}
	}

	if(len > RP_MSG_PAYLOAD_SIZE){
		len = RP_MSG_PAYLOAD_SIZE;
	}

	/* Copy the payload and set message header: */
	memcpy(msg->payload, data, len);
	msg->dataLen = len;
	msg->dstAddr = dstEndpt;
	msg->srcAddr = srcEndpt;
	msg->flags = 0;
	msg->reserved = 0;

	VirtQueue_addUsedBuf(vQueToHost, token, RP_MSG_BUF_SIZE);
	VirtQueue_kick(vQueToHost);

	taskEXIT_CRITICAL(&virtQueLock);

	return MessageQCopy_S_SUCCESS;
}

