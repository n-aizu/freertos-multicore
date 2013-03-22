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
 *  @file       MessageQCopy.h
 *
 *  @brief      A simple copy-based MessageQ, to work with Linux virtio_rp_msg.
 *
 *  The following are key features of the MessageQCopy module:
 *  - Multiple writers, single reader.
 *  - Timeouts are allowed when receiving messages.
 *  - Supports processor copy transfers only.
 *  - Sending/receiving also works between enpoints on the same processor.
 *
 *  Non-Features (as compared to MessageQ):
 *  - zero copy messaging, using registered heaps.
 *  - Dependence on a NameServer (Client furnishes the endpoint IDs)
 *  - Arbitrary reply endpoints can be embedded in message header.
 *  - Priority Queues.
 *
 *  Conceptually, the reader thread owns a message queue. The reader thread
 *  creates a message queue. The writer threads opens a created message queue
 *  to get access to it.
 *
 *  Message queues are identified by a system-wide unique 32 bit queueID,
 *  which encodes the (MultiProc) 16 bit core ID and a local 16 bit index ID.
 *
 *  The MessageQCopy header should be included in an application as follows:
 *  @code
 *  #include <ti/ipc/rpmsg/MessageQCopy.h>
 *  @endcode
 *
 *  Questions:
 *  - Workout how connections exist: how does Ducati side cleanup when Linux
 *    side goes down?
 *  - Client needs to know max size of data buffer to allocate for receive.
 *  - Do we need procID's buried in the message header addresses anymore, or
 *    isn't that now implicit in the vrings/transport.
 *  - Do we want MessageQ_Unblock in this version?  (Normally used in RCM).
 *  - Do we want the generic ISync ability to create our own synchronizers,
 *    or just hard-code semaphore usage or callbacks for simplicity?
 *
 *  ============================================================================
 */

#ifndef ti_ipc_MessageQCopy__include
#define ti_ipc_MessageQCopy__include

#include "Std.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "VirtQueue.h"

#if defined (__cplusplus)
extern "C" {
#endif


/* =============================================================================
 *  Structures & Definitions
 * =============================================================================
 */

/*!
 *  @brief      Used as the timeout value to specify wait forever
 */
#define MessageQCopy_FOREVER                portMAX_DELAY

/*!
   *  @def    MessageQCopy_S_SUCCESS
 *  @brief  Operation is successful.
 */
#define MessageQCopy_S_SUCCESS              0

/*!
 *  @def    MessageQCopy_E_FAIL
 *  @brief  Operation is not successful.
 */
#define MessageQCopy_E_FAIL                 -1

/*!
 *  @def    MessageQCopy_E_MEMORY
 *  @brief  Operation resulted in memory failure.
 */
#define MessageQCopy_E_MEMORY               -3

/*!
 *  @def    MessageQCopy_E_TIMEOUT
 *  @brief  Operation timed out.
 */
#define MessageQCopy_E_TIMEOUT              -6

/*!
 *  @def    MessageQCopy_E_NOENDPT
 *  @brief  No endpoint for a message.
 */
#define MessageQCopy_E_NOENDPT              -7

/*!
 *  @def    MessageQ_E_UNBLOCKED
 *  @brief  MessageQ was unblocked
 */
#define MessageQCopy_E_UNBLOCKED            -19

/*!
 *  @def    MessageQCopy_MAX_RESERVED_ENDPOINT
 *  @brief  Maximum Value for System Reserved Endpoints.
 */
#define MessageQCopy_MAX_RESERVED_ENDPOINT  100

/*!
 *  @def    MessageQCopy_MAX_RESERVED_ENDPOINT
 *  @brief  Maximum Value for System Reserved Endpoints.
 */
#define MessageQCopy_ASSIGN_ANY             0xFFFFFFFF

/*!
 *  @brief  MessageQCopy_Handle type
 */
typedef struct MessageQCopy_Object *MessageQCopy_Handle;

#define MSGBUFFERSIZE    512

/* =============================================================================
 *  MessageQCopy Functions:
 * =============================================================================
 */


/*!
 *  @brief      Initialize MessageQCopy Module
 *
 *  @return     Status of the call.
 *              - #MessageQCopy_S_SUCCESS denotes success.
 *              - #MessageQCopy_E_MEMORY  denotes failure.
 */
Int MessageQCopy_init();

/*!
 *  @brief      Create a MessageQ instance for receiving.
 *
 *  The returned handle is to an object containing a queue for receiving
 *  messages from the transport, and a 32 bit endpoint ID unique to this local
 *  processor.
 *
 *  @param[in]   reserved     If value is MessageQCopy_ASSIGN_ANY, then
 *                            any Endpoint can be assigned; otherwise, value is
 *                            a reserved Endpoint ID, which must be less than
 *                            or equal to MessageQCopy_MAX_RESERVED_ENDPOINT.
 *  @param[out]  endpoint     Endpoint ID for this side of the connection.
 *
 *
 *  @return     MessageQ Handle, or NULL if:
 *                            - reserved endpoint already taken;
 *                            - could not allocate object
 */
MessageQCopy_Handle MessageQCopy_create(UInt32 reserved, UInt32 * endpoint);

/*!
 *  @brief      Receives a message from a message queue
 *
 *  This function returns a status. It also copies data to the client.
 *  If no message is available, it blocks on the queue object
 *  until the data received or a timeout occurs.
 *  If a message is successfully retrieved, the message
 *  data is copied into the data pointer, and a #MessageQCopy_S_SUCCESS
 *  status is returned.
 *
 *  @param[in]     handle    MessageQ handle
 *  @param[out]    data      Pointer to the client's data buffer.
 *  @param[in/out] len       Data buffer size/Amount of data received.
 *  @param[out]    rplyEndpt Endpoint of source (for replies).
 *  @param[in]     timeout   Maximum duration to wait for a message in
 *                           tick periods.
 *
 *  @return     MessageQ status:
 *              - #MessageQCopy_S_SUCCESS: Message successfully returned
 *              - #MessageQCopy_E_TIMEOUT: MessageQCopy_recv timed out
 *
 *  @sa         MessageQCopy_send
 */
Int MessageQCopy_recv(MessageQCopy_Handle handle, Ptr data, UInt16 *len,
                      UInt32 *rplyEndpt, portTickType timeout);

/*!
 *  @brief      Sends data to a remote processor, or copies onto a local
 *              messageQ.
 *
 *  @param[in]  dstEndpt    Destination Endpoint.
 *  @param[in]  srcEndpt    Source Endpoint.
 *  @param[in]  data        Data payload to be copied and sent.
 *  @param[in]  len         Amount of data to be copied.
 *  @param[in]  timeout     Maximum duration to wait for a message sent
 *                          in tick periods.
 *  @return     Status of the call.
 *              - #MessageQCopy_S_SUCCESS: denotes success.
 *              - #MessageQCopy_E_TIMEOUT: MessageQCopy_send timed out
 */
Int MessageQCopy_send(UInt32 dstEndpt,
                      UInt32 srcEndpt,
                      Ptr    data,
                      UInt16 len,
                      portTickType timeout);

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* ti_ipc_MessageQCopy__include */
