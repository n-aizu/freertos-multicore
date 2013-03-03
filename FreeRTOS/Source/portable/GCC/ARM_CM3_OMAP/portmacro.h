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


#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.  
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	unsigned portLONG
#define portBASE_TYPE	long

#if( configUSE_16_BIT_TICKS == 1 )
	typedef unsigned portSHORT portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffff
#else
	typedef unsigned portLONG portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffffffff
#endif

/*-----------------------------------------------------------*/	

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_RATE_MS			( ( portTickType ) 1000 / configTICK_RATE_HZ )		
#define portBYTE_ALIGNMENT			8
#define configARCH_SPECIFIC_LOCK	0
/*-----------------------------------------------------------*/	

/* Multicore definitions. */
#define portNUM_PROCESSORS          ( (unsigned portBASE_TYPE ) 2 )
#if ( configUSE_CPU_UNBOUND_TASK == 1 )
#define portNO_SPECIFIC_PROCESSOR   ( (unsigned portBASE_TYPE) portNUM_PROCESSORS + 1)
#endif
#define portIRQSTAT_INIT			( (unsigned portBASE_TYPE ) 0 )
#define portTICKEXE_CORENUM			( (unsigned portBASE_TYPE ) 0 )
#define portLOCK_MAX				( (unsigned portBASE_TYPE ) 16 * 1024 )

typedef struct {
	volatile unsigned long *pxAlias[portNUM_PROCESSORS];
	volatile unsigned char *pxBitBand;
} portSPINLOCK_TYPE;

static inline unsigned long uxPortGetCurrentCPU( void )
{
	return *(volatile unsigned long *)0xe00fffe0;
}

/*-----------------------------------------------------------*/

/* Scheduler utilities. */
#define portNVIC_INT_CTRL			( ( volatile unsigned long *) 0xe000ed04 )
#define portNVIC_PENDSVSET			0x10000000

static inline void vPortYieldFromISR( void )
{
	/* Set a PendSV to request a context switch. */
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
}

#define portYIELD()					vPortYieldFromISR()

void vPortReqSwitchTask( portBASE_TYPE xSwitchRequired );
#define portEND_SWITCHING_ISR( xSwitchRequired )	vPortReqSwitchTask( xSwitchRequired )

static inline void portIDLE_SLEEP( void )
{
	__asm volatile( "wfi" ::: "memory" );
}

/*-----------------------------------------------------------*/

/* Critical section management. */

static inline unsigned long ulPortSetInterruptMask( void )
{
	unsigned long stat;

	__asm volatile
	(
		"	mrs %0, basepri			\n" 
		"	mov r0, %1				\n"
		"	msr basepri, r0			\n"
		: "=r" ( stat )
		: "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY )
		: "r0", "memory"
	);

	return stat;
}

static inline void vPortClearInterruptMask( unsigned long stat )
{
	__asm volatile
	(
	 	"	mov r0, %0				\n"
		"	msr basepri, %0			\n" 
		:: "r" ( stat ) : "r0", "memory"
	);
}

static inline void vPortWMB( void )
{
	__asm volatile
	(
	 	"	dsb						\n"
	   ::: "memory"
	);
}

static inline void vPortRMB( void )
{
	__asm volatile
	(
	 	"	dsb						\n"
	   ::: "memory"
	);
}

#define portSET_INTERRUPT_MASK_FROM_ISR()		ulPortSetInterruptMask()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	vPortClearInterruptMask(x)
#define portDISABLE_INTERRUPTS()				ulPortSetInterruptMask()
#define portENABLE_INTERRUPTS()					vPortClearInterruptMask(0)
#define portSET_INTERRUPT_MASK_AND_RETURN		portSET_INTERRUPT_MASK_FROM_ISR
#define portCLEAR_INTERRUPT_MASK_AND_SET		portCLEAR_INTERRUPT_MASK_FROM_ISR

extern void vPortSpinLockInit( portSPINLOCK_TYPE *pxLock );
#define portLOCK_INIT( pxLock )		vTaskLockInit( pxLock )
#define portSPINLOCK_INIT( pxLock )	vPortSpinLockInit( pxLock ) 

static inline void vPortSpinLock( unsigned portBASE_TYPE uxProcessor, volatile portSPINLOCK_TYPE * pxLock )
{
	/* OMAP4 Cortex-M3 is not support ldrex/strex. */
	/* So, spinlock is implemented with bit-band.  */
	/* (Execute ldrex for non-local memory lead to usagefault) */
	/* cf. http://lists.infradead.org/pipermail/linux-arm-kernel/2010-October/029231.html */

	volatile unsigned long *pxAlias = pxLock->pxAlias[ uxProcessor ];
	volatile unsigned char *pxBitBand = pxLock->pxBitBand;
	unsigned char uxExpect = ( 0x1UL << uxProcessor );
	unsigned portBASE_TYPE uxVal;

retry_core1:
	while( *pxBitBand )
	{
		__asm volatile ( "wfe" ::: "memory" );
	}

	__asm volatile ( "" ::: "memory" );	

retry_core0:
	*pxAlias = 0x1UL;
	__asm volatile ( "" ::: "memory" );	
	uxVal = *pxBitBand;
	if( uxVal != uxExpect )
	{
		*pxAlias = 0x0UL;
		/* If on core1, insert delay.  */
		if( uxProcessor != portTICKEXE_CORENUM )
		{
			uxVal = *pxBitBand;
			__asm volatile ( "" ::: "memory" );	
			*pxAlias = 0x0UL;

			goto retry_core1;
		}

		while( *pxBitBand )
		{
			__asm volatile ( "" ::: "memory" );	
		}

		goto retry_core0;
	}

	__asm volatile( "dmb" ::: "memory" );
}

static inline void vPortSpinUnLock( unsigned portBASE_TYPE uxProcessor, volatile portSPINLOCK_TYPE * pxLock )
{
	volatile unsigned long *pxAlias = pxLock->pxAlias[ uxProcessor ];

	*pxAlias = 0x0UL;
	__asm volatile
	(
		"	dsb		\n"
		"	sev		\n"
		::: "memory"
	);
}

/* These are no inline function version. */
extern void vPortSpinLock2( unsigned portBASE_TYPE uxProcessor, volatile portSPINLOCK_TYPE * pxLock );
extern void vPortSpinUnLock2( unsigned portBASE_TYPE uxProcessor, volatile portSPINLOCK_TYPE * pxLock );

#define portENTER_CRITICAL( pxLock )		\
{											\
	do {									\
		vTaskAcquireLock( pxLock );			\
		__asm volatile ( "" ::: "memory" );	\
	} while (0);							\
}

#define portEXIT_CRITICAL( pxLock )			\
{											\
	do {									\
		__asm volatile ( "" ::: "memory" );	\
		vTaskReleaseLock( pxLock );			\
	} while (0);							\
}

/* vPortSpinLock is called from only vTaskAcquireLock,  */
/* so it is inline function.                            */
/* But portENTER_CRITICAL_NOT_RECURSIVE_FROM_ISR may be */
/* called from various functions, so use no inline      */
/* function vPortSpinLock2.                             */
#define portENTER_CRITICAL_NOT_RECURSIVE_FROM_ISR( pxLock )			\
{																	\
	do {															\
		vPortSpinLock2( portGetCurrentCPU(), &((pxLock)->xLock) );	\
		__asm volatile ( "" ::: "memory" );							\
	} while (0);													\
}

#define portEXIT_CRITICAL_NOT_RECURSIVE_FROM_ISR( pxLock )				\
{																		\
	do {																\
		__asm volatile ( "" ::: "memory" );								\
		vPortSpinUnLock2( portGetCurrentCPU(), &((pxLock)->xLock) );	\
	} while (0);														\
}

#define CORTEXM3_CTRL_REG ((volatile unsigned long *)0x55081000)
#define portINTERRUPT_CORE( uxProcessor ) vPortInturrptCore( uxProcessor )

#define INT_VAL_CORE0 (0x1 << 0)
#define INT_VAL_CORE1 (0x1 << 16)
#define CLR_VAL_CORE0 (~INT_VAL_CORE0)
#define CLR_VAL_CORE1 (~INT_VAL_CORE1)

static inline void vPortInturrptCore( unsigned portBASE_TYPE uxProcessor )
{
	const unsigned long intVal[] = { INT_VAL_CORE0, INT_VAL_CORE1 };

	*(CORTEXM3_CTRL_REG) |= intVal[ uxProcessor & 0x1 ];
}

static inline void vPortClearInterruptCore( unsigned portBASE_TYPE uxProcessor )
{
	const unsigned long clrVal[] = { CLR_VAL_CORE0, CLR_VAL_CORE1 };

	*(CORTEXM3_CTRL_REG) &= clrVal[ uxProcessor & 0x1 ];
}

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )
/*-----------------------------------------------------------*/

/* Architecture specific optimisations. */
#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

	/* Generic helper function. */
	__attribute__( ( always_inline ) ) static inline unsigned char ucPortCountLeadingZeros( unsigned long ulBitmap )
	{
		unsigned char ucReturn;

		__asm volatile ( "clz %0, %1" : "=r" ( ucReturn ) : "r" ( ulBitmap ) );
		return ucReturn;
	}

	/* Check the configuration. */
	#if( configMAX_PRIORITIES > 32 )
		#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
	#endif

	/* Store/clear the ready priorities in a bit map. */
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

	/*-----------------------------------------------------------*/

	#define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) uxTopPriority = ( 31 - ucPortCountLeadingZeros( ( uxReadyPriorities ) ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */
/*-----------------------------------------------------------*/

#define portNOP()

/* Setup IRQ handler. */
void vPortSetIrqHndl( unsigned int uxIndex, void (*pxHndl)(void), void (**pxOld)(void) );

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

