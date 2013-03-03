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
 *  @file       VirtQueue.c
 *
 *  @brief      Virtio Queue implementation for FreeRTOS
 */

#include <string.h>
#include "InterruptProxy.h"
#include "VirtQueue.h"
#include "FreeRTOSConfig.h"

#include "projdefs.h"
#include "portable.h"

#include "OMAP_CM3.h"

/* Used for defining the size of the virtqueue registry */
#define NUM_QUEUES              2

/* Predefined device addresses */
#define IPC_MEM_VRING0          0xA0000000
#define IPC_MEM_VRING1          0xA0004000
#define IPC_MEM_VRING2          0xA0008000
#define IPC_MEM_VRING3          0xA000c000

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of two)
 */
#define VQ0_SIZE                256
#define VQ1_SIZE                256
#define VQ2_SIZE                256
#define VQ3_SIZE                256

/*
 * enum - Predefined Mailbox Messages
 *
 * @RP_MSG_MBOX_READY: informs the M3's that we're up and running. will be
 * followed by another mailbox message that carries the A9's virtual address
 * of the shared buffer. This would allow the A9's drivers to send virtual
 * addresses of the buffers.
 *
 * @RP_MSG_MBOX_STATE_CHANGE: informs the receiver that there is an inbound
 * message waiting in its own receive-side vring. please note that currently
 * this message is optional: alternatively, one can explicitly send the index
 * of the triggered virtqueue itself. the preferred approach will be decided
 * as we progress and experiment with those design ideas.
 *
 * @RP_MSG_MBOX_CRASH: this message indicates that the BIOS side is unhappy
 *
 * @RP_MBOX_ECHO_REQUEST: this message requests the remote processor to reply
 * with RP_MBOX_ECHO_REPLY
 *
 * @RP_MBOX_ECHO_REPLY: this is a reply that is sent when RP_MBOX_ECHO_REQUEST
 * is received.
 *
 * @RP_MBOX_ABORT_REQUEST:  tells the M3 to crash on demand
 *
 * @RP_MBOX_BOOTINIT_DONE: this message indicates the BIOS side has reached a
 * certain state during the boot process. This message is used to inform the
 * host that the basic BIOS initialization is done, and lets the host use this
 * notification to perform certain actions.
 */
enum {
    RP_MSG_MBOX_READY           = (Int)0xFFFFFF00,
    RP_MSG_MBOX_STATE_CHANGE    = (Int)0xFFFFFF01,
    RP_MSG_MBOX_CRASH           = (Int)0xFFFFFF02,
    RP_MBOX_ECHO_REQUEST        = (Int)0xFFFFFF03,
    RP_MBOX_ECHO_REPLY          = (Int)0xFFFFFF04,
    RP_MBOX_ABORT_REQUEST       = (Int)0xFFFFFF05,
    RP_MSG_FLUSH_CACHE          = (Int)0xFFFFFF06,
    RP_MSG_BOOTINIT_DONE        = (Int)0xFFFFFF07,
    RP_MSG_HIBERNATION          = (Int)0xFFFFFF10,
    RP_MSG_HIBERNATION_FORCE    = (Int)0xFFFFFF11,
    RP_MSG_HIBERNATION_ACK      = (Int)0xFFFFFF12,
    RP_MSG_HIBERNATION_CANCEL   = (Int)0xFFFFFF13
};

#define DIV_ROUND_UP(n,d)   (((n) + (d) - 1) / (d))
#define RP_MSG_NUM_BUFS     (VQ0_SIZE) /* must be power of two */
#define RP_MSG_BUFS_SPACE   (RP_MSG_NUM_BUFS * RP_MSG_BUF_SIZE * 2)

/*
 * The alignment to use between consumer and producer parts of vring.
 * Note: this is part of the "wire" protocol. If you change this, you need
 * to update your BIOS image as well
 */
#define RP_MSG_VRING_ALIGN  (4096)

/* With 256 buffers, our vring will occupy 3 pages */
#define RP_MSG_RING_SIZE    ((DIV_ROUND_UP(vring_size(RP_MSG_NUM_BUFS, \
                            RP_MSG_VRING_ALIGN), PAGE_SIZE)) * PAGE_SIZE)

/* The total IPC space needed to communicate with a remote processor */
#define RPMSG_IPC_MEM   (RP_MSG_BUFS_SPACE + 2 * RP_MSG_RING_SIZE)

static struct VirtQueue_Object *queueRegistry[NUM_QUEUES] = {NULL};


static inline Void * mapPAtoVA(UInt pa)
{
    return (Void *)((pa & 0x000fffffU) | 0xa0000000U);
}

static inline UInt mapVAtoPA(Void * va)
{
    return ((UInt)va & 0x000fffffU) | 0x9cf00000U;
}

/*!
 * ======== VirtQueue_kick ========
 */
Void VirtQueue_kick(VirtQueue_Handle vq)
{
    /* For now, simply interrupt remote processor */
    if (vq->vring.avail->flags & VRING_AVAIL_F_NO_INTERRUPT) {
        return;
    }

    InterruptProxy_intSend(vq->id);
}

/*!
 * ======== VirtQueue_addUsedBuf ========
 */
Int VirtQueue_addUsedBuf(VirtQueue_Handle vq, Int16 head, Int len)
{
    struct vring_used_elem *used;

    if ((head > vq->vring.num) || (head < 0)) {
        return (-1);
    }

    /*
    * The virtqueue contains a ring of used buffers.  Get a pointer to the
    * next entry in that used ring.
    */
    used = &vq->vring.used->ring[vq->vring.used->idx % vq->vring.num];
    used->id = head;
    used->len = len;

    vq->vring.used->idx++;

    return (0);
}

/*!
 * ======== VirtQueue_getAvailBuf ========
 */
Int16 VirtQueue_getAvailBuf(VirtQueue_Handle vq, Void **buf, Int *len)
{
    UInt16 head;

    /* There's nothing available? */
    if (vq->last_avail_idx == vq->vring.avail->idx) {
        /* We need to know about added buffers */
        vq->vring.used->flags &= ~VRING_USED_F_NO_NOTIFY;

        return (-1);
    }
    /*
     * Grab the next descriptor number they're advertising, and increment
     * the index we've seen.
     */
    head = vq->vring.avail->ring[vq->last_avail_idx++ % vq->vring.num];

    *buf = mapPAtoVA(vq->vring.desc[head].addr);
    *len = vq->vring.desc[head].len;

    return (head);
}

/*!
 * ======== VirtQueue_isr ========
 */
Void VirtQueue_isr(UInt msg)
{
    VirtQueue_Object *vq;

    switch(msg) {
    case (UInt)RP_MSG_MBOX_READY:
    case (UInt)RP_MBOX_ABORT_REQUEST:
    case (UInt)RP_MSG_FLUSH_CACHE:
        return;

    case (UInt)RP_MBOX_ECHO_REQUEST:
        InterruptProxy_intSend((UInt)(RP_MBOX_ECHO_REPLY));
        return;

    case (UInt)RP_MSG_HIBERNATION:
    case (UInt)RP_MSG_HIBERNATION_FORCE:
        InterruptProxy_intSend((UInt)RP_MSG_HIBERNATION_CANCEL);
        return;
    default:
        /*
         *  If the message isn't one of the above, it's either part of the
         *  2-message synchronization sequence or it a virtqueue message
         */
        break;
    }

    /* Don't let unknown messages to pass as a virtqueue index */
    if (msg >= NUM_QUEUES) {
        /* Adding print here deliberately, we should never see this */
        return;
    }

    vq = queueRegistry[msg];
    if (vq) {
        vq->callback(vq);
    }
}


/*!
 * ======== VirtQueue_create ========
 */
VirtQueue_Object *VirtQueue_create(VirtQueue_callback callback, Int vqId)
{
    VirtQueue_Object *vq;
    Void *vringAddr;

    vq = pvPortMalloc(sizeof(VirtQueue_Object));
    if (!vq) {
        return (NULL);
    }

	memset(vq, 0, sizeof(vq));
    vq->callback = callback;
    vq->id = vqId;
    vq->last_avail_idx = 0;

    switch (vq->id) {
        /* IPC transport vrings */
        case ID_SELF_TO_A9:
            /* IPU -> A9 */
            vringAddr = (struct vring *) IPC_MEM_VRING0;
            break;
        case ID_A9_TO_SELF:
            /* A9 -> IPU */
            vringAddr = (struct vring *) IPC_MEM_VRING1;
            break;
        default:
            vPortFree(vq);
            return (NULL);
    }

    vring_init(&(vq->vring), RP_MSG_NUM_BUFS, vringAddr, RP_MSG_VRING_ALIGN);

    /*
     *  Don't trigger a mailbox message every time MPU makes another buffer
     *  available
     */
    if (vq->id == ID_SELF_TO_A9) {
        vq->vring.used->flags |= VRING_USED_F_NO_NOTIFY;
    }

    queueRegistry[vq->id] = vq;

    return (vq);
}

#ifndef configMBOX_INTERRUPT_PRIORITY
#define configMBOX_INTERRUPT_PRIORITY configMAX_SYSCALL_INTERRUPT_PRIORITY
#elif configMBOX_INTERRUPT_PRIORITY < configMAX_SYSCALL_INTERRUPT_PRIORITY
#undef configMBOX_INTERRUPT_PRIORITY
#define configMBOX_INTERRUPT_PRIORITY configMAX_SYSCALL_INTERRUPT_PRIORITY
#endif

/*!
 * ======== VirtQueue_startup ========
 */
Void VirtQueue_startup()
{
	InterruptIpu_intRegister(VirtQueue_isr);
	NVIC_SetPriority(MAIL_U2_M3_IRQ, (configMBOX_INTERRUPT_PRIORITY >> __NVIC_PRIO_BITS));
	NVIC_EnableIRQ(MAIL_U2_M3_IRQ);
}

