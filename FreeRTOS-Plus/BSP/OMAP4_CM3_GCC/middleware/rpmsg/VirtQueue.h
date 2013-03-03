/* This file is based on SYS/BIOS RPMsg code.
 *
 * Repositories:
 *  http://git.omapzoom.org/?p=repo/sysbios-rpmsg.git;a=summary
 *
 * The original license terms are as follows.
 */
/*
 * Copyright (c) 2011, Texas Instruments Incorporated
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
 *  @file       VirtQueue.h
 *
 *  @brief      Virtio Queue interface for FreeRTOS
 *
 *  Usage:
 *     This IPC only works between one processor designated as the Host (Linux)
 *     and one or more Slave processors (FreeRTOS).
 *
 *     For any Host/Slave pair, there are 2 VirtQueues (aka Vrings);
 *     Only the Host adds new buffers to the avail list of a vring;
 *     Available buffers can be empty or full, depending on direction;
 *     Used buffer means "processed" (emptied or filled);
 *
 *  Host:
 *    - To send buffer to the slave processor:
 *          add_avail_buf(slave_virtqueue);
 *          kick(slave_virtqueue);
 *          get_used_buf(slave_virtqueue);
 *    - To receive buffer from slave processor:
 *          add_avail_buf(host_virtqueue);
 *          kick(host_virtqueue);
 *          get_used_buf(host_virtqueue);
 *
 *  Slave:
 *    - To send buffer to the host:
 *          get_avail_buf(host_virtqueue);
 *          add_used_buf(host_virtqueue);
 *          kick(host_virtqueue);
 *    - To receive buffer from the host:
 *          get_avail_buf(slave_virtqueue);
 *          add_used_buf(slave_virtqueue);
 *          kick(slave_virtqueue);
 *
 *  All VirtQueue operations can be called in any context.
 *
 *  ============================================================================
 */

#ifndef ti_ipc_VirtQueue__include
#define ti_ipc_VirtQueue__include

#include "Std.h"
#include "virtio_ring.h"

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 *  @brief  VirtQueue Ids for the basic IPC transport rings.
 */
#define ID_SELF_TO_A9      0
#define ID_A9_TO_SELF      1

/*!
 *  @brief  Size of buffer being exchanged in the VirtQueue rings.
 */
#define RP_MSG_BUF_SIZE     (512)

#define RP_MSG_PAYLOAD_SIZE	(RP_MSG_BUF_SIZE - 16)

struct VirtQueue_Object;

/*!
 *  @brief  a queue to register buffers for sending or receiving.
 */
typedef struct VirtQueue_Object *VirtQueue_Handle;

/*!
 *  @var     VirtQueue_callback
 *  @brief   Signature of any callback function that can be registered with the
 *           VirtQueue
 *
 *  @param[in]  VirtQueue     Pointer to the VirtQueue which was signalled.
 */
typedef Void (*VirtQueue_callback)(VirtQueue_Handle);

/*!
 *  @brief      Initialize at runtime the VirtQueue
 *
 *  @param[in]  callback  the clients callback function.
 *  @param[in]  vqId      VirtQueue ID for this VirtQueue.
 *
 *  @Returns    Returns a handle to a new initialized VirtQueue.
 */
VirtQueue_Handle VirtQueue_create(VirtQueue_callback callback, Int vqId);

/*!
 *  @brief      Notify other processor of new buffers in the queue.
 *
 *  After one or more add_buf calls, invoke this to kick the other side.
 *
 *  @param[in]  vq        the VirtQueue.
 *
 *  @sa         VirtQueue_addBuf
 */
Void VirtQueue_kick(VirtQueue_Handle vq);

/*!
 *  @brief       Used at startup-time for initialization
 *
 *  Should be called before any other VirtQueue APIs
 */
Void VirtQueue_startup();


/*
 *  ============================================================================
 *  Host Only Functions:
 *  ============================================================================
 */

/*
 *  ============================================================================
 *  Slave Only Functions:
 *  ============================================================================
 */

/*!
 *  @brief      Get the next available buffer.
 *              Only used by Slave.
 *
 *  @param[in]  vq        the VirtQueue.
 *  @param[out] buf       Pointer to location of available buffer;
 *  @param[out] len       Length of the available buffer message.
 *
 *  @return     Returns a token used to identify the available buffer, to be
 *              passed back into VirtQueue_addUsedBuf();
 *              token is negative if failure to find an available buffer.
 *
 *  @sa         VirtQueue_addUsedBuf
 */
Int16 VirtQueue_getAvailBuf(VirtQueue_Handle vq, Void **buf, Int *len);

/*!
 *  @brief      Add used buffer to virtqueue's used buffer list.
 *              Only used by Slave.
 *
 *  @param[in]  vq        the VirtQueue.
 *  @param[in]  token     token of the buffer to be added to vring used list.
 *  @param[in]  len       length of the message being added.
 *
 *  @return     Remaining capacity of queue or a negative error.
 *
 *  @sa         VirtQueue_getAvailBuf
 */
Int VirtQueue_addUsedBuf(VirtQueue_Handle vq, Int16 token, Int len);

/*!
 *  @brief  a queue to register buffers for sending or receiving.
 */
typedef struct VirtQueue_Object {
    /* Id for this VirtQueue_Object */
    UInt16                  id;

    /* The function to call when buffers are consumed (can be NULL) */
    VirtQueue_callback      callback;

    /* Shared state */
    struct vring            vring;

    /* Number of free buffers */
    UInt16                  num_free;

    /* Last available index; updated by VirtQueue_getAvailBuf */
    UInt16                  last_avail_idx;

    /* Last available index; updated by VirtQueue_addUsedBuf */
    UInt16                  last_used_idx;
} VirtQueue_Object;

#define GET_AVAIL_COUNT(vq) (vq->vring.avail->idx - vq->last_avail_idx)
#define PROHIBIT_VRING_NOTIFY(vq) (vq->vring.used->flags |= VRING_USED_F_NO_NOTIFY)

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */

#endif /* ti_ipc_VirtQueue__include */
