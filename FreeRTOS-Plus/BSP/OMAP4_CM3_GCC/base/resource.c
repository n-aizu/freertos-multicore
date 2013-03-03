/* This file is based on SYS/BIOS RPMsg code(rsc_table_ipu.h)
 * and Pandaboard-FreeRTOS.(startup.c)
 *
 * Repositories:
 *  http://git.omapzoom.org/?p=repo/sysbios-rpmsg.git;a=summary
 *  https://github.com/apopple/Pandaboard-FreeRTOS
 *
 * The original license terms are as follows.
 */
/*
 * Copyright (c) 2012, Texas Instruments Incorporated
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


#include "resource.h"
/* --------------------------------------------------------- */

#define RESERVED {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

__attribute__ ((section(".resource_table")))
struct resource_table resources[] = {
	1,			/* we're the first version that implements this */
	FW_RSC_NUM,	/* number of entries in the table */
	0, 0, 		/* reserved, must be zero */
	{
		offsetof(struct resource_table, rpmsg_vdev),
		offsetof(struct resource_table, mem),
		offsetof(struct resource_table, data),
#if configSHARED_MEM_SIZE > 0
		offsetof(struct resource_table, shmem),
#endif
		offsetof(struct resource_table, devmem_vring),
		offsetof(struct resource_table, devmem0),
		offsetof(struct resource_table, devmem1),
		offsetof(struct resource_table, devmem2),
		offsetof(struct resource_table, devmem3),
		offsetof(struct resource_table, devmem4),
		offsetof(struct resource_table, devmem5),
		offsetof(struct resource_table, devmem6),
		offsetof(struct resource_table, devmem7),
	},
	/* rpmsg vdev entry */
	{
		TYPE_VDEV, VIRTIO_ID_RPMSG, 0x0, configRPMSG_FEATURES, 0, 0, 0, 2, { 0, 0 }
	},
	/* the two vrings */
	{
		IPU_MEM_RPMSG_VRING0, 4096, IPU_RPMSG_VQ0_SIZE, 1, 0
	},
	{
		IPU_MEM_RPMSG_VRING1, 4096, IPU_RPMSG_VQ1_SIZE, 2, 0
	},
	/* text */
	{
		TYPE_CARVEOUT, IPU_MEM_TEXT, 0x0, configTEXT_SIZE, 0x0, 0x0, "text"
	},
	{
		TYPE_CARVEOUT, IPU_MEM_DATA, 0x0, configDATA_SIZE, 0x0, 0x0, "data"
	},
#if configSHARED_MEM_SIZE > 0
	{
		TYPE_CARVEOUT, IPU_MEM_SHARED_DATA, 0x0, configSHARED_MEM_SIZE, 0x0, 0x0, "shared_mem"
	},
#endif
	{
		TYPE_DEVMEM, IPU_MEM_IPC_VRING, PHYS_MEM_IPC_VRING, IPU_MEM_IPC_VRING_SIZE, 0x0, 0x0, "IPU_MEM_IPC_VRING"
	},
	{
		TYPE_DEVMEM, IPU_TILER_MODE_0_1, L3_TILER_MODE_0_1, SZ_256M, 0x0, 0x0, "IPU_TILER_MODE_0_1"
	},
	{
		TYPE_DEVMEM, IPU_TILER_MODE_2, L3_TILER_MODE_2, SZ_128M, 0x0, 0x0, "IPU_TILER_MODE_2"
	},
	{
		TYPE_DEVMEM, IPU_TILER_MODE_3, L3_TILER_MODE_3, SZ_128M, 0x0, 0x0, "IPU_TILER_MODE_3"
	},
	{
		TYPE_DEVMEM, IPU_PERIPHERAL_L4CFG, L4_PERIPHERAL_L4CFG, SZ_16M, 0x0, 0x0, "IPU_PERIPHERAL_L4CFG"
	},
	{
		TYPE_DEVMEM, IPU_PERIPHERAL_L4PER, L4_PERIPHERAL_L4PER, SZ_16M, 0x0, 0x0, "IPU_PERIPHERAL_L4PER"
	},
	{
		TYPE_DEVMEM, IPU_IVAHD_CONFIG, L3_IVAHD_CONFIG, SZ_16M, 0x0, 0x0, "IPU_IVAHD_CONFIG"
	},
	{
		TYPE_DEVMEM, IPU_IVAHD_SL2, L3_IVAHD_SL2, SZ_16M, 0x0, 0x0, "IPU_IVAHD_SL2"
	},
	{
		TYPE_DEVMEM, IPU_PERIPHERAL_DMM, L3_PERIPHERAL_DMM, SZ_1M, 0x0, 0x0, "IPU_PERIPHERAL_DMM"
	},
};

__attribute__ ((section(".resource_size")))
uint32_t resourcesLen = sizeof(resources)/sizeof(*resources);

