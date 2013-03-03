/* This file is based on SYS/BIOS RPMsg code.(DucatiAmmu.cfg)
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

#include "FreeRTOSConfig.h"
#include "portmacro.h"


static const unsigned long uxLargeAddr[] = {
					0x0UL,
					0x60000000UL,
					0x80000000UL,
					0xa0000000UL
};
static const unsigned long uxLargeXlte[] =
							{ 0x1UL, 0x1UL, 0x1UL, 0x1UL };
static const unsigned long uxLargePolicy[] = {
#ifdef configTEXT_CACHE
					0x10003UL,
#else
					0x00003UL,
#endif
					0x30003UL,
#ifdef configDATA_CACHE
					0x30003UL,
#else
					0x00003UL,
#endif
					0x00003UL
};

static const unsigned long uxMedAddr[] = { 0x50000000UL };
static const unsigned long uxMedXlte[] = { 0x55040000UL };
static const unsigned long uxMedPolicy[] = { 0x30003UL };

static const unsigned long uxSmallAddr[] = {
					0xffffffffUL, /* dummy */
					0x40000000UL,
					0x20000000UL,
					0x20004000UL,					
					0x20008000UL,
					0x2000c000UL,
					0x50000000UL,
					0x50010000UL,
					0x50011000UL,
					0x50020000UL,
	
};
static const unsigned long uxSmallXlte[] = {
					0xffffffffUL,
					0x55080000UL,
					0x55020000UL,
					0x55024000UL,
					0x55028000UL,
					0x5502c000UL,
					0x55040000UL,
					0x55050000UL,
					0x55051000UL,
					0x55060000UL,
};
static const unsigned long uxSmallPolicy[] = {
					0xffffffffUL, /* dummy */
					0x00003UL,
					0xf0003UL,
					0xf0003UL,
					0xf0003UL,
					0xf0003UL,
					0x00003UL,
					0x00001UL,
					0x00001UL,
					0x00001UL
};


void vConfigAMMU( void )
{
	volatile unsigned long *uxAddrRegBase = (volatile unsigned long *)0x55080800;
	volatile unsigned long *uxXlteRegBase = (volatile unsigned long *)0x55080820;
	volatile unsigned long *uxPolicyRegBase = (volatile unsigned long *)0x55080840;
	int i;

	/* Large pages */
	for( i = 0; i < (sizeof(uxLargeAddr) / sizeof(uxLargeAddr[0])); i++ )
	{
		uxAddrRegBase[ i ] = uxLargeAddr[ i ];
		uxXlteRegBase[ i ] = uxLargeXlte[ i ];
		uxPolicyRegBase[ i ] = uxLargePolicy[ i ];
	}

	/* Medium pages */
	uxAddrRegBase = (volatile unsigned long *)0x55080860;
	uxXlteRegBase = (volatile unsigned long *)0x550808A0;
	uxPolicyRegBase = (volatile unsigned long *)0x550808E0;

	for( i = 0; i < (sizeof(uxMedAddr) / sizeof(uxMedAddr[0])); i++ )
	{
		uxAddrRegBase[ i ] = uxMedAddr[ i ];
		uxXlteRegBase[ i ] = uxMedXlte[ i ];
		uxPolicyRegBase[ i ] = uxMedPolicy[ i ];
	}

	/* Small pages. */
	uxAddrRegBase = (volatile unsigned long *)0x55080920;
	uxXlteRegBase = (volatile unsigned long *)0x550809A0;
	uxPolicyRegBase = (volatile unsigned long *)0x55080A20;

	/* Small page 0 is fixed.  */
	for( i = 1; i < (sizeof(uxSmallAddr) / sizeof(uxSmallAddr[0])); i++ )
	{
		uxAddrRegBase[ i ] = uxSmallAddr[ i ];
		uxXlteRegBase[ i ] = uxSmallXlte[ i ];
		uxPolicyRegBase[ i ] = uxSmallPolicy[ i ];
	}
}

