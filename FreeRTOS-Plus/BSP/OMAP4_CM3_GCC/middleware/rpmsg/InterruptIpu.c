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
/*
 *  ======== InterruptIpu.c ========
 *  IPU Interrupt Manger
 */

#include "InterruptIpu.h"

/* Register access method. */
#define REG16(A)   (*(volatile UInt16 *) (A))
#define REG32(A)   (*(volatile UInt32 *) (A))

#define M3INT_MBX               50

/* Assigned mailboxes */
#define HOST_TO_SYSM3_MBX       0 /* Rx on SysM3 from Host */
#define M3_TO_HOST_MBX          1 /* Tx to Host from M3 */
#define DSP_TO_HOST_MBX         2 /* Tx to Host from DSP */
#define HOST_TO_DSP_MBX         3 /* Rx on DSP from Host */
#define SYSM3_TO_APPM3_MBX      4 /* Rx on AppM3 from Host/SysM3 */

#define MAILBOX_BASEADDR        (0xAA0F4000)

#define MAILBOX_MESSAGE(M)      (MAILBOX_BASEADDR + 0x040 + (0x4 * M))
#define MAILBOX_FIFOSTATUS(M)   (MAILBOX_BASEADDR + 0x080 + (0x4 * M))
#define MAILBOX_STATUS(M)       (MAILBOX_BASEADDR + 0x0C0 + (0x4 * M))
#define MAILBOX_REG_VAL(M)      (0x1 << (2 * M))

#define MAILBOX_IRQSTATUS_CLR_M3    (MAILBOX_BASEADDR + 0x124)
#define MAILBOX_IRQENABLE_SET_M3    (MAILBOX_BASEADDR + 0x128)
#define MAILBOX_IRQENABLE_CLR_M3    (MAILBOX_BASEADDR + 0x12C)


void (*userFxn)(UInt) = NULL;

Void InterruptIpu_isr();


/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*!
 *  ======== InterruptIpu_intEnable ========
 *  Enable remote processor interrupt
 */
Void InterruptIpu_intEnable()
{
	REG32(MAILBOX_IRQENABLE_SET_M3) = MAILBOX_REG_VAL(HOST_TO_SYSM3_MBX);
}

/*!
 *  ======== InterruptIpu_intDisable ========
 *  Disables remote processor interrupt
 */
Void InterruptIpu_intDisable()
{
	REG32(MAILBOX_IRQENABLE_CLR_M3) = MAILBOX_REG_VAL(HOST_TO_SYSM3_MBX);
}

/*!
 *  ======== InterruptIpu_intRegister ========
 */
Void InterruptIpu_intRegister(void (*fxn)(UInt))
{
	userFxn = fxn;
	vPortSetIrqHndl(M3INT_MBX, InterruptIpu_isr, NULL);

    /* Enable the mailbox interrupt to the M3 core */
    InterruptIpu_intEnable();
}

/*!
 *  ======== InterruptIpu_intSend ========
 *  Send interrupt to the remote processor
 */
Void InterruptIpu_intSend(UInt arg)
{
	while(REG32(MAILBOX_FIFOSTATUS(M3_TO_HOST_MBX)));
	REG32(MAILBOX_MESSAGE(M3_TO_HOST_MBX)) = arg;
}

/*!
 *  ======== InterruptIpu_intClear ========
 *  Clear interrupt and return payload
 */
UInt InterruptIpu_intClear()
{
	UInt arg = INVALIDPAYLOAD;

	/* If FIFO is empty, return INVALIDPAYLOAD */
	if (REG32(MAILBOX_STATUS(HOST_TO_SYSM3_MBX)) == 0) {
		return (arg);
	}
	else {
		/* If there is a message, return the argument to the caller */
		arg = REG32(MAILBOX_MESSAGE(HOST_TO_SYSM3_MBX));
		REG32(MAILBOX_IRQSTATUS_CLR_M3) =
				MAILBOX_REG_VAL(HOST_TO_SYSM3_MBX);
	}

	return (arg);
}

/*!
 *  ======== InterruptIpu_isr ========
 *  Calls the function supplied by the user in intRegister
 */
Void InterruptIpu_isr()
{
	UInt payload;

	payload = InterruptIpu_intClear();
	if (payload != INVALIDPAYLOAD) {
		userFxn(payload);
	}
}

