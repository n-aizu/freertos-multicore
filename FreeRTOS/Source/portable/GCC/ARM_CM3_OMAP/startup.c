/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!
    
    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?                                      *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    
    http://www.FreeRTOS.org - Documentation, training, latest information, 
    license and contact details.
    
    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell 
    the code with commercial support, indemnification, and middleware, under 
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under 
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

void vConfigAMMU( void ) __attribute__((weak));

void vPortStartCore1( void );
void vPortCoreIrqHandler( void );
void vPortCallConstructor( void );

void main0( void );

/*
 * Exception handlers.
 */
void vResetHandler( void ) __attribute__ (( naked ));
static void vResetHandler0( void );
static void vResetHandler1( void );
static void vNmiHandler(void);
static void vFaultHandler(void);
static void vDefaultHandler(void);
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void ) __attribute__ (( naked ));
void vPortCoreIrqHandler( void );

#ifndef configMAIN_STACK_SIZE
#define configMAIN_STACK_SIZE (2 * 1024)
#elif configMAIN_STACK_SIZE < 1024
#undef configMAIN_STACK_SIZE
#define configMAIN_STACK_SIZE 1024
#endif

__attribute__ ((section(".stack0")))
unsigned char xStack0[ configMAIN_STACK_SIZE ];

__attribute__ ((section(".stack1")))
unsigned char xStack1[ configMAIN_STACK_SIZE ];

__attribute__ ((section(".isr_vector")))
void (*pxInitVector[])(void) =
{
	(void (*)(void))(xStack0 + configMAIN_STACK_SIZE), /* Initial stack pointer */
	vResetHandler,		/* Reset */
};

__attribute__ ((section(".isr_vector0")))
void (*pxVectors0[])(void) =
{
	(void (*)( void ))(xStack0 + configMAIN_STACK_SIZE), /* Initial stack pointer */
	vResetHandler0,			/* Reset */
	vNmiHandler,			/* Nmi */
	vFaultHandler,			/* Hard fault */
	vDefaultHandler,		/* MPU fault */
	vDefaultHandler,		/* Bus fault */
	vDefaultHandler,		/* Usage fault */
	0,
	0,
	0,
	0,
	vPortSVCHandler,		/* SVCall handler */
	vDefaultHandler,		/* Debug */
	0,
	xPortPendSVHandler,		/* PendSV */
	xPortSysTickHandler,	/* SysTick */
 
	/* Vendor Handlers */
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vPortCoreIrqHandler,	/* Semaphore interrupt */
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
};

__attribute__ ((section(".isr_vector1")))
void (*pxVectors1[])(void) =
{
	(void (*)( void ))(xStack1 + configMAIN_STACK_SIZE), /* Initial stack pointer */
	vResetHandler1,			/* Reset */
	vNmiHandler,			/* Nmi */
	vFaultHandler,			/* Hard fault */
	vDefaultHandler,		/* MPU fault */
	vDefaultHandler,		/* Bus fault */
	vDefaultHandler,		/* Usage fault */
	0,
	0,
	0,
	0,
	vPortSVCHandler,		/* SVCall handler */
	vDefaultHandler,		/* Debug */
	0,
	xPortPendSVHandler,		/* PendSV */
	vDefaultHandler,		/* SysTick */
 
	/* Vendor Handlers */
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vPortCoreIrqHandler,	/* Semaphore interrupt */
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
	vDefaultHandler,
};

static void vNmiHandler( void )
{
	while(1);
}

static void vFaultHandler( void )
{
	while(1);
}

static void vDefaultHandler( void )
{
}

void vResetHandler( void )
{
	/* This is a naked function. */

	__asm volatile
	(
	"	ldr r2, =0xe00fffe0					\n"
	"	ldr r1, [r2]						\n"
	"	cmp r1, #0							\n"
	"	bne 1f								\n"
	"	bl vResetHandler0					\n"
	"										\n"
	"1:										\n"
	"	ldr r0, pxVectorsTmp				\n"
	"	ldr r2, [r0]						\n"
	"	mov sp, r2							\n"
	"	bl vResetHandler1					\n"
	"	.align 2							\n"
	"pxVectorsTmp: .word pxVectors1         \n"
	);
}

void vResetHandler0( void )
{
	extern unsigned long _sbss, _ebss;
	unsigned long *sbss, *ebss;

	/* set vector table offset for this core */
	*(volatile unsigned long **)(0xe000ed08) = 
		(unsigned long *)((unsigned long)pxVectors0 & 0x1fffffffUL);

	/* bss clear */
	for( sbss = &_sbss; sbss != &_ebss; sbss++ )
	{
		*sbss = 0;
	}

	/* configure AMMU */
	vConfigAMMU();
	/* call constructor */
	vPortCallConstructor();

	main0();
}

void vResetHandler1( void )
{
	/* set vector table offset for this core */
	*(volatile unsigned long **)(0xe000ed08) =
		(unsigned long *)((unsigned long)pxVectors1 & 0x1fffffffUL);

	/* execute task function */
	vPortStartCore1();

	/* Should not get here! */
}

void vConfigAMMU( void )
{
}
