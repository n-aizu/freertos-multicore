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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM3 port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* For backward compatibility, ensure configKERNEL_INTERRUPT_PRIORITY is
defined.  The value should also ensure backward compatibility.
FreeRTOS.org versions prior to V4.4.0 did not include this definition. */
#ifndef configKERNEL_INTERRUPT_PRIORITY
	#define configKERNEL_INTERRUPT_PRIORITY 255
#endif

/* Constants required to manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL		( ( volatile unsigned long *) 0xe000e010 )
#define portNVIC_SYSTICK_LOAD		( ( volatile unsigned long *) 0xe000e014 )
#define portNVIC_SYSPRI2			( ( volatile unsigned long *) 0xe000ed20 )
#define portNVIC_IRQPRI0			( ( volatile unsigned long *) 0xe000e400 )
#define portNVIC_IRQENABLE0			( ( volatile unsigned long *) 0xe000e100 )
#define portNVIC_SYSTICK_CLK		0x00000004
#define portNVIC_SYSTICK_INT		0x00000002
#define portNVIC_SYSTICK_ENABLE		0x00000001
#define portNVIC_IRQCORE_ENABLE		0x00000008
#define portNVIC_PENDSV_PRI			( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 16 )
#define portNVIC_SYSTICK_PRI		( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 24 )
#define portNVIC_IRQCORE_PRI		( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 24 )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR			( 0x01000000 )

/* Bit-Band and alias address. */
#define BIT_BAND_BASE  ( 0x20000000 )
#define BIT_BAND_ALIAS ( 0x22000000 )

/* Spinlock definition(Bit-Band area index). */
#define LOCK_INIT_LOCK 0	/* spinlock for lock initialisation. */
#define LOCK_START_USE 0x10	/* index 1 - 15 is reserved. */

/* The priority used by the kernel is assigned to a variable to make access
from inline assembler easier. */
const unsigned long ulKernelPriority = configKERNEL_INTERRUPT_PRIORITY;

/*
 * Setup the timer to generate the tick interrupts.
 */
static void prvSetupTimerInterrupt( void );

/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void ) __attribute__ (( naked ));
void vPortCoreIrqHandler( void );

void vPortStartCore1( void );
void vPortCallConstructor( void );

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void prvPortStartFirstTask( void ) __attribute__ (( naked ));

static void prvPrepareStartScheduler( void );

/* Lock for lock initialisation.  */
static portSPINLOCK_TYPE xLockInitLock;

static void prvInitBitBand( void );
static void prvSpinLockInitBase( unsigned int uxIndex , portSPINLOCK_TYPE *pxLock);

void main1( void ) __attribute__((weak));
static portBASE_TYPE xCore0Start = pdFALSE;
static portBASE_TYPE xCore1Start = pdFALSE;
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
	/* Simulate the stack frame as it would be created by a context switch
	interrupt. */
	pxTopOfStack--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
	*pxTopOfStack = portINITIAL_XPSR;	/* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ( portSTACK_TYPE ) pxCode;	/* PC */
	pxTopOfStack--;
	*pxTopOfStack = 0;	/* LR */
	pxTopOfStack -= 5;	/* R12, R3, R2 and R1. */
	*pxTopOfStack = ( portSTACK_TYPE ) pvParameters;	/* R0 */
	pxTopOfStack -= 8;	/* R11, R10, R9, R8, R7, R6, R5 and R4. */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
	__asm volatile (
					"	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */
					"	ldr r2, =0xe00fffe0				\n"
					"	ldr r1, [r2]					\n"
					"	add r3, r3, r1, lsl 2			\n"
					"	ldr r1, [r3]					\n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
					"	ldr r0, [r1]					\n" /* The first item in pxCurrentTCB is the task top of stack. */
					"	ldmia r0!, {r4-r11}				\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
					"	msr psp, r0						\n" /* Restore the task stack pointer. */
					"	mov r0, #0 						\n"
					"	msr	basepri, r0					\n"
					"	orr r14, #0xd					\n"
					"	bx r14							\n"
					"									\n"
					"	.align 2						\n"
					"pxCurrentTCBConst2: .word pxCurrentTCB				\n"
				);
}
/*-----------------------------------------------------------*/

static void prvPortStartFirstTask( void )
{
	__asm volatile(
					" ldr r0, =0xE000ED08 	\n" /* Use the NVIC offset register to locate the stack. */
					" ldr r0, [r0] 			\n"
					" ldr r0, [r0] 			\n"
					" msr msp, r0			\n" /* Set the msp back to the start of the stack. */
					" cpsie i				\n" /* Globally enable interrupts. */
					" svc 0					\n" /* System call to start first task. */
					" nop					\n"
				);
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portBASE_TYPE xPortStartScheduler( void )
{
	/* configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.  
	See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
	configASSERT( configMAX_SYSCALL_INTERRUPT_PRIORITY );

	prvPrepareStartScheduler();

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	prvSetupTimerInterrupt();

	xCore0Start = pdTRUE;
	__asm volatile( "dmb" ::: "memory" );

#if configCORE1_START_IN_FREERTOS == 1
	/* Start core1 */
	*(RM_MPU_M3_RSTCTRL) &= 0xfffffffd;
#endif
	while( xCore1Start == pdFALSE )
	{
		__asm volatile ( "" ::: "memory" );
	}

	/* Start the first task. */
	prvPortStartFirstTask();

	/* Should not get here! */
	return 0;
}

void prvPrepareStartScheduler( void )
{
	/* Make PendSV, CallSV and SysTick the same priroity as the kernel. */
	*(portNVIC_SYSPRI2) |= portNVIC_PENDSV_PRI;
	*(portNVIC_SYSPRI2) |= portNVIC_SYSTICK_PRI;
	/* Make core interrupt the same priority as the kernel. */
	*(portNVIC_IRQPRI0) |= portNVIC_IRQCORE_PRI;
	/* Enable core interrupt. */
	*(portNVIC_IRQENABLE0) |= portNVIC_IRQCORE_ENABLE;
}
/*-----------------------------------------------------------*/

void xPortPendSVHandler( void )
{
	/* This is a naked function. */

	__asm volatile
	(
	"	mrs r0, psp							\n"
	"										\n"
	"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
	"	ldr r2, =0xe00fffe0					\n"
	"	ldr r1, [r2]						\n"
	"	add r3, r3, r1, lsl 2				\n"
	"	ldr r2, [r3]						\n"
	"										\n"
	"	stmdb r0!, {r4-r11}					\n" /* Save the remaining registers. */
	"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
	"										\n"
	"	stmdb sp!, {r3, r14}				\n"
	"	mov r0, %0							\n"
	"	msr basepri, r0						\n"
	"	bl vTaskSwitchContext				\n"
	"	mov r0, #0							\n"
	"	msr basepri, r0						\n"
	"	ldmia sp!, {r3, r14}				\n"
	"										\n"	/* Restore the context, including the critical nesting count. */
	"	ldr r1, [r3]						\n"
	"	ldr r0, [r1]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
	"	ldmia r0!, {r4-r11}					\n" /* Pop the registers. */
	"	msr psp, r0							\n"
	"	bx r14								\n"
	"										\n"
	"	.align 2							\n"
	"pxCurrentTCBConst: .word pxCurrentTCB	\n"
	::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
unsigned long ulDummy;

	ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		vTaskIncrementTick();
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );

	/* If using preemption, also force a context switch. */
	#if configUSE_PREEMPTION == 1
		*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
		portINTERRUPT_CORE( (~portTICKEXE_CORENUM) & 0x1 );
	#endif
}
/*-----------------------------------------------------------*/

void vPortCoreIrqHandler( void )
{
	unsigned portBASE_TYPE uxCurrentCPU = portGetCurrentCPU();

	/* Clear interrupt. */
	vPortClearInterruptCore( uxCurrentCPU );
	/* Force a context switch. */
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
}
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
void prvSetupTimerInterrupt( void )
{
	/* Configure SysTick to interrupt at the requested rate. */
	*(portNVIC_SYSTICK_LOAD) = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	*(portNVIC_SYSTICK_CTRL) = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;
}
/*-----------------------------------------------------------*/

void vPortStartCore1( void )
{
	while( xCore0Start == pdFALSE )
	{
		__asm volatile ( "" ::: "memory" );
	}

	main1();

	__asm volatile( "dsb" ::: "memory" );
	xCore1Start = pdTRUE;

	portDISABLE_INTERRUPTS();

	prvPrepareStartScheduler();
	prvPortStartFirstTask();
}

void main1( void )
{
}

void vPortCallConstructor( void )
{
	prvInitBitBand();

	vTaskConstructor();
	vListConstructor();
	vQueueConstructor();
	vMemConstructor();
}

void vPortReqSwitchTask( portBASE_TYPE xSwitchRequired )
{
	if( xSwitchRequired >= 0)
	{
		if( xSwitchRequired == uxPortGetCurrentCPU() )
		{
			vPortYieldFromISR();
		}
		else
		{
			vPortInturrptCore( xSwitchRequired );
		}
	}
}

void vPortSpinLockInit( portSPINLOCK_TYPE *pxLock )
{
	static int xLockIndex = LOCK_START_USE;
	unsigned portBASE_TYPE uxCurrentCPU = uxPortGetCurrentCPU();
	unsigned long xStat;

	xStat = portSET_INTERRUPT_MASK_AND_RETURN();
	vPortSpinLock2( uxCurrentCPU, &xLockInitLock );
	{
		if( xLockIndex < portLOCK_MAX )
		{
			prvSpinLockInitBase( xLockIndex, pxLock );
			xLockIndex++;
		}
	}
	vPortSpinUnLock2( uxCurrentCPU, &xLockInitLock );
	portCLEAR_INTERRUPT_MASK_AND_SET( xStat );
}

void vPortSpinLock2( unsigned portBASE_TYPE uxProcessor, volatile portSPINLOCK_TYPE * pxLock )
{
	vPortSpinLock( uxProcessor, pxLock);
}

void vPortSpinUnLock2( unsigned portBASE_TYPE uxProcessor, volatile portSPINLOCK_TYPE * pxLock )
{
	vPortSpinUnLock( uxProcessor, pxLock);
}

void vPortSetIrqHndl( unsigned int uxIndex, void (*pxHndl)(void), void (**pxOld)(void) )
{
	/* System handlers are not allowed to be set by application code. */
	/* (except for SysTick handler on Core1) */
	if( uxIndex < 16 )
	{
		unsigned portBASE_TYPE uxCurrentCPU = portGetCurrentCPU();

		if( uxCurrentCPU == portTICKEXE_CORENUM || uxIndex != 15 )
		{
			return;
		}
	}
	/* Semaphore interrupt is not allowed to be set by application code. */
	else if( uxIndex == 19 )
	{
		return;
	}

	if( pxOld != NULL )
	{
		*pxOld = (void (*)(void))(*(volatile unsigned long **)(0xe000ed08))[ uxIndex ];
	}

	(*(volatile unsigned long **)(0xe000ed08))[ uxIndex ] = (unsigned long)pxHndl;
}

static void prvInitBitBand( void )
{
	prvSpinLockInitBase( LOCK_INIT_LOCK, &xLockInitLock );
}

static void prvSpinLockInitBase( unsigned int uxIndex, portSPINLOCK_TYPE *pxLock)
{
	int bit;

	pxLock->pxBitBand = (volatile unsigned char *)(BIT_BAND_BASE + uxIndex);
	*(pxLock->pxBitBand) = 0x0UL;

	for( bit = 0; bit < portNUM_PROCESSORS; bit++ )
	{
		pxLock->pxAlias[ bit ] =
			(volatile unsigned long *)(BIT_BAND_ALIAS + (uxIndex * 32 + bit * 4));
	}
}
