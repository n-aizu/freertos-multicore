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

/* these interfaces are not suppoeted on multicore target */
/*
vTaskSuspendAll
xTaskResumeAll
vTaskDelete
vTaskSuspend
vTaskResume
xTaskResumeFromISR
xTaskIsTaskSuspended
vTaskList
vTaskPlaceOnEventListRestricted
vTaskMissedYield
vTaskAllocateMPURegions
xTaskGetSchedulerState
vTaskPriorityInherit
vTaskPriorityDisinherit
vTaskEndScheduler
uxTaskPriorityGet
vTaskEnterCritical
vTaskExitCritical
xTaskGetTickCountFromISR
vTaskGetRunTimeStats
xTaskGetIdleTaskHandle
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "StackMacros.h"

/*
 * Macro to define the amount of stack available to the idle task.
 */
#define tskIDLE_STACK_SIZE	configMINIMAL_STACK_SIZE

/*
 * Task control block.  A task control block (TCB) is allocated to each task,
 * and stores the context of the task.
 */
typedef struct tskTaskControlBlock
{
	volatile portSTACK_TYPE	*pxTopOfStack;		/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE STRUCT. */
	
	xListItem				xGenericListItem;	/*< List item used to place the TCB in ready and blocked queues. */
	xListItem				xEventListItem;		/*< List item used to place the TCB in event lists. */
	unsigned portBASE_TYPE	uxPriority;			/*< The priority of the task where 0 is the lowest priority. */
	portSTACK_TYPE			*pxStack;			/*< Points to the start of the stack. */
	signed char				pcTaskName[ configMAX_TASK_NAME_LEN ];/*< Descriptive name given to the task when created.  Facilitates debugging only. */
	unsigned portBASE_TYPE  xCPUAffinity;
	#if ( configUSE_CPU_UNBOUND_TASK == 1 )
	unsigned portBASE_TYPE	xCurrentCPU;
	#endif
	#if ( portSTACK_GROWTH > 0 )
		portSTACK_TYPE *pxEndOfStack;			/*< Used for stack overflow checking on architectures where the stack grows up from low memory. */
	#endif

	#if ( configUSE_TRACE_FACILITY == 1 )
		unsigned portBASE_TYPE	uxTCBNumber;	/*< This stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
		unsigned portBASE_TYPE  uxTaskNumber;	/*< This stores a number specifically for use by third party trace code. */
	#endif

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
		pdTASK_HOOK_CODE pxTaskTag;
	#endif

	#if ( configGENERATE_RUN_TIME_STATS == 1 )
		unsigned long ulRunTimeCounter;		/*< Used for calculating how much CPU time each task is utilising. */
	#endif

	#if ( configUSE_PREEMPTION == 0 )
		portBASE_TYPE xYieldFlg;	
	#endif
	portBASE_TYPE xLastReadiedState;
} tskTCB;


/*
 * Some kernel aware debuggers require data to be viewed to be global, rather
 * than file scope.
 */
#ifdef portREMOVE_STATIC_QUALIFIER
	#define static
#endif

/*lint -e956 */
PRIVILEGED_DATA tskTCB * volatile pxCurrentTCB[ portNUM_PROCESSORS ];

PRIVILEGED_DATA static portLOCK_TYPE xReadyListLock[ portNUM_PROCESSORS ];
PRIVILEGED_DATA static portLOCK_TYPE xDelayListLock;
PRIVILEGED_DATA static portLOCK_TYPE xTaskCreateLock;
PRIVILEGED_DATA static portLOCK_TYPE xTagLock;

/* Lists for ready and blocked tasks. --------------------*/

PRIVILEGED_DATA static xList pxReadyTasksLists[ portNUM_PROCESSORS ][ configMAX_PRIORITIES ];	/*< Prioritised ready tasks. */
PRIVILEGED_DATA static xList xDelayedTaskList1;							/*< Delayed tasks. */
PRIVILEGED_DATA static xList xDelayedTaskList2;							/*< Delayed tasks (two lists are used - one for delays that have overflowed the current tick count. */
PRIVILEGED_DATA static xList * volatile pxDelayedTaskList ;				/*< Points to the delayed task list currently being used. */
PRIVILEGED_DATA static xList * volatile pxOverflowDelayedTaskList;		/*< Points to the delayed task list currently being used to hold tasks that have overflowed the current tick count. */
PRIVILEGED_DATA static xList xInfinitDelayTaskList;

/* File private variables. --------------------------------*/
PRIVILEGED_DATA static volatile unsigned portBASE_TYPE uxCurrentNumberOfTasks 	= ( unsigned portBASE_TYPE ) 0U;
PRIVILEGED_DATA static volatile portTickType xTickCount 						= ( portTickType ) 0U;
PRIVILEGED_DATA static volatile unsigned portBASE_TYPE uxTopReadyPriority[ portNUM_PROCESSORS ];
PRIVILEGED_DATA static volatile signed portBASE_TYPE xSchedulerRunning 			= pdFALSE;
PRIVILEGED_DATA static volatile portBASE_TYPE xNumOfOverflows 					= ( portBASE_TYPE ) 0;
PRIVILEGED_DATA static unsigned portBASE_TYPE uxTaskNumber 						= ( unsigned portBASE_TYPE ) 0U;
PRIVILEGED_DATA static portTickType xNextTaskUnblockTime						= ( portTickType ) portMAX_DELAY;
PRIVILEGED_DATA static volatile signed portBASE_TYPE xTaskInitialised 			= pdFALSE;

#if ( configGENERATE_RUN_TIME_STATS == 1 )

	PRIVILEGED_DATA static unsigned long ulTaskSwitchedInTime = 0UL;	/*< Holds the value of a timer/counter the last time a task was switched in. */

#endif

/* Debugging and trace facilities private variables and macros. ------------*/

/*
 * The value used to fill the stack of a task when the task is created.  This
 * is used purely for checking the high water mark for tasks.
 */
#define tskSTACK_FILL_BYTE	( 0xa5U )


/*-----------------------------------------------------------*/

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 0

	/* If configUSE_PORT_OPTIMISED_TASK_SELECTION is 0 then task selection is
	performed in a generic way that is not optimised to any particular
	microcontroller architecture. */

	/* uxTopReadyPriority holds the priority of the highest priority ready
	state task. */
	#define taskRECORD_READY_PRIORITY( uxProcessor, uxPriority )		\
	{																	\
		if( ( uxPriority ) > uxTopReadyPriority[ ( uxProcessor ) ] )	\
		{																\
			uxTopReadyPriority[ ( uxProcessor ) ] = ( uxPriority );		\
		}																\
	} /* taskRECORD_READY_PRIORITY */

	/*-----------------------------------------------------------*/

	#define taskGET_HIGHEST_PRIORITY( uxProcessor, uxPriority ) \
	{															\
		uxPriority = uxTopReadyPriority[ ( uxProcessor ) ];		\
	}

	/*-----------------------------------------------------------*/

	/* Define away taskRESET_READY_PRIORITY() and portRESET_READY_PRIORITY() as
	they are only required when a port optimised method of task selection is
	being used. */
	/* This maclo must be called in critical section.(with xReadyListLock)   */
	#define taskRESET_READY_PRIORITY( uxProcessor, uxPriority )														\
	{																												\
		while( listLIST_IS_EMPTY( &( pxReadyTasksLists[ uxProcessor ][ uxTopReadyPriority[ uxProcessor ] ] ) ) )	\
		{																											\
			--uxTopReadyPriority[ uxProcessor ];																	\
		}																											\
	}
	#define portRESET_READY_PRIORITY( uxProcessor, uxPriority )

#else /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

	/* If configUSE_PORT_OPTIMISED_TASK_SELECTION is 1 then task selection is
	performed in a way that is tailored to the particular microcontroller
	architecture being used. */

	/* A port optimised version is provided.  Call the port defined macros. */
	#define taskRECORD_READY_PRIORITY( uxProcessor, uxPriority ) \
		portRECORD_READY_PRIORITY( ( uxPriority ), uxTopReadyPriority[ ( uxProcessor ) ] )

	/*-----------------------------------------------------------*/

	/* A port optimised version is provided, call it only if the TCB being reset
	is being referenced from a ready list.  If it is referenced from a delayed
	or suspended list then it won't be in a ready list. */
	/* This maclo must be called in critical section.(with xReadyListLock)   */
	#define taskRESET_READY_PRIORITY( uxProcessor, uxPriority )										\
	{																								\
		if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ ( uxProcessor ) ][ ( uxPriority ) ] ) ) == 0 ) \
		{																							\
			portRESET_READY_PRIORITY( ( uxPriority ), ( uxTopReadyPriority[ ( uxProcessor ) ] ) );	\
		}																							\
	}

	/*-----------------------------------------------------------*/

	#define taskGET_HIGHEST_PRIORITY( uxProcessor, uxPriority ) \
		portGET_HIGHEST_PRIORITY( ( uxPriority ), ( uxTopReadyPriority[ ( uxProcessor ) ] ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */


/*-----------------------------------------------------------*/


/*
 * Several functions take an xTaskHandle parameter that can optionally be NULL,
 * where NULL is used to indicate that the handle of the currently executing
 * task should be used in place of the parameter.  This macro simply checks to
 * see if the parameter is NULL and returns a pointer to the appropriate TCB.
 */
#define prvGetTCBFromHandle( pxHandle ) ( ( ( pxHandle ) == NULL ) ? NULL : ( tskTCB * ) ( pxHandle ) )

/* Callback function prototypes. --------------------------*/
extern void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName );
extern void vApplicationTickHook( void );
		
/* File private functions. --------------------------------*/
static portBASE_TYPE prvAddTaskToReadyQueueCore( unsigned portBASE_TYPE uxProcessor, tskTCB *pxTCB, portBASE_TYPE stat ) PRIVILEGED_FUNCTION;
#if ( configUSE_CPU_UNBOUND_TASK == 1 )
static portBASE_TYPE prvUnBoundTaskToReadyQueue( tskTCB *pxTCB, portBASE_TYPE stat ) PRIVILEGED_FUNCTION;
static portBASE_TYPE prvUnBoundTaskMigration( tskTCB *pxTCB ) PRIVILEGED_FUNCTION;
#endif
static void prvSetTaskOnCurrentTCB( unsigned portBASE_TYPE uxProcessor ) PRIVILEGED_FUNCTION;
static void prvSetTaskOnCurrentTCBOnInit( unsigned portBASE_TYPE uxProcessor ) PRIVILEGED_FUNCTION;
static void prvCheckDelayedTasks( void ) PRIVILEGED_FUNCTION;

/*
 * Utility to ready a TCB for a given task.  Mainly just copies the parameters
 * into the TCB structure.
 */
static void prvInitialiseTCBVariables( tskTCB *pxTCB, const signed char * const pcName, unsigned portBASE_TYPE uxPriority, unsigned short usStackDepth ) PRIVILEGED_FUNCTION;

/*
 * Utility to ready all the lists used by the scheduler.  This is called
 * automatically upon the creation of the first task.
 */
static void prvInitialiseTaskLists( void ) PRIVILEGED_FUNCTION;

/*
 * The idle task, which as all tasks is implemented as a never ending loop.
 * The idle task is automatically created and added to the ready lists upon
 * creation of the first user task.
 *
 * The portTASK_FUNCTION_PROTO() macro is used to allow port/compiler specific
 * language extensions.  The equivalent prototype for this function is:
 *
 * void prvIdleTask( void *pvParameters );
 *
 */
static portTASK_FUNCTION_PROTO( prvIdleTask, pvParameters );

/*
 * The currently executing task is entering the Blocked state.  Add the task to
 * either the current or the overflow delayed task list.
 */
static void prvAddCurrentTaskToDelayedList( portTickType xTimeToWake ) PRIVILEGED_FUNCTION;

/*
 * Allocates memory from the heap for a TCB and associated stack.  Checks the
 * allocation was successful.
 */
static tskTCB *prvAllocateTCBAndStack( unsigned short usStackDepth, portSTACK_TYPE *puxStackBuffer ) PRIVILEGED_FUNCTION;

/*
 * When a task is created, the stack of the task is filled with a known value.
 * This function determines the 'high water mark' of the task stack by
 * determining how much of the stack remains at the original preset value.
 */
#if ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) )

	static unsigned short usTaskCheckFreeStackSpace( const unsigned char * pucStackByte ) PRIVILEGED_FUNCTION;

#endif


/*lint +e956 */


static portBASE_TYPE prvAddTaskToReadyQueueCore( unsigned portBASE_TYPE uxProcessor, tskTCB *pxTCB, portBASE_TYPE stat )
{
	unsigned portBASE_TYPE uxTopPriority;
	portBASE_TYPE xNextCPU, xDoSwitch;

	taskENTER_CRITICAL( &xReadyListLock[ uxProcessor ] );
	{
		traceMOVED_TASK_TO_READY_STATE( pxTCB );

		taskGET_HIGHEST_PRIORITY( uxProcessor, uxTopPriority );
		taskRECORD_READY_PRIORITY( uxProcessor, pxTCB->uxPriority );
		if( pxTCB->uxPriority > uxTopPriority )
		{
			xDoSwitch = pdTRUE;
		}
		else
		{
			xDoSwitch = pdFALSE;
		}

		vListInsertEnd( ( xList * ) &( pxReadyTasksLists[ uxProcessor ][ pxTCB->uxPriority ] ), &( pxTCB->xGenericListItem ) );
		#if ( configUSE_CPU_UNBOUND_TASK == 1 )
		pxTCB->xCurrentCPU = uxProcessor;
		#endif
	}
	taskEXIT_CRITICAL( &xReadyListLock[ uxProcessor ] );

	if( stat != TASK_STATE_NOTCHANGE )
	{
		pxTCB->xLastReadiedState = stat;
	}

	return xDoSwitch ? uxProcessor : -1;
}

#if ( configUSE_CPU_UNBOUND_TASK == 1 )
static portBASE_TYPE prvUnBoundTaskToReadyQueue( tskTCB *pxTCB, portBASE_TYPE stat )
{
	unsigned portBASE_TYPE uxTopPriority;
	portBASE_TYPE xNextCPU;
	int i, idx;

	/* Choose readylist to insert.                         */
	/* Execute on outside of the critical section,         */
	/* because it is not necessary to be decided strictly. */
	for( i = 0, idx = xNextCPU = pxTCB->xCurrentCPU; i < portNUM_PROCESSORS; i++ )
	{
		taskGET_HIGHEST_PRIORITY( idx, uxTopPriority );
		if( pxTCB->uxPriority > uxTopPriority )
		{
			xNextCPU = idx;
			break;
		}

		idx++;
		if( idx >= portNUM_PROCESSORS )
		{
			idx = 0;
		}
	}

	return prvAddTaskToReadyQueueCore( xNextCPU, pxTCB, stat );
}

static portBASE_TYPE prvUnBoundTaskMigration( tskTCB *pxTCB )
{
	unsigned portBASE_TYPE uxTopPriority, uxCurrentCPU;
	portBASE_TYPE xNextCPU, xSwitch;
	int i, idx;

	/* Choose readylist to insert.                         */
	/* Execute on outside of the critical section,         */
	/* because it is not necessary to be decided strictly. */
	/* (Because this is migration function, so don't       */
	/* check current cpu core readylist)                   */
	for( i = 0, idx = pxTCB->xCurrentCPU + 1; i < (portNUM_PROCESSORS - 1); i++, idx++ )
	{
		if( idx >= portNUM_PROCESSORS )
		{
			idx = 0;
		}

		taskGET_HIGHEST_PRIORITY( idx, uxTopPriority );
		if( pxTCB->uxPriority > uxTopPriority )
		{
			xNextCPU = idx;
			break;
		}
	}

	if( i < (portNUM_PROCESSORS - 1) )
	{
		uxCurrentCPU = pxTCB->xCurrentCPU;

		taskENTER_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );
		{
			vListRemove( &( pxTCB->xGenericListItem ) );
			taskRESET_READY_PRIORITY( uxCurrentCPU, pxTCB->uxPriority );
		}
		taskEXIT_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );

		xSwitch = prvAddTaskToReadyQueueCore( xNextCPU, pxTCB, TASK_STATE_NOTCHANGE );
	}
	else
	{
		xSwitch = -1;
	}

	return xSwitch;
}
#endif

/*
 * Place the task represented by pxTCB into the appropriate ready queue for
 * the task.  It is inserted at the end of the list.  One quirk of this is
 * that if the task being inserted is at the same priority as the currently
 * executing task, then it will only be rescheduled after the currently
 * executing task has been rescheduled.
 */
#if ( configUSE_CPU_UNBOUND_TASK == 1 )
#define prvAddTaskToReadyQueue( pxTCB, stat )									\
	( (pxTCB)->xCPUAffinity != portNO_SPECIFIC_PROCESSOR ) ?					\
		prvAddTaskToReadyQueueCore( (pxTCB)->xCPUAffinity, (pxTCB), stat ) :	\
		prvUnBoundTaskToReadyQueue( (pxTCB), stat )
#else
#define prvAddTaskToReadyQueue( pxTCB, stat )									\
		prvAddTaskToReadyQueueCore( (pxTCB)->xCPUAffinity, (pxTCB), stat )
#endif

static void prvSetTaskOnCurrentTCB( unsigned portBASE_TYPE uxProcessor )
{
	unsigned portBASE_TYPE uxPri;

	taskENTER_CRITICAL( &xReadyListLock[ uxProcessor ] );
	{
		taskGET_HIGHEST_PRIORITY( uxProcessor, uxPri );

		/* listGET_OWNER_OF_NEXT_ENTRY walks through the list, so the tasks of the
			same priority get an equal share of the processor time. */
		#if ( configUSE_PREEMPTION == 0 )
		{
			tskTCB *pxTCB = pxCurrentTCB[ uxProcessor ];
			unsigned portBASE_TYPE uxCurPri = pxTCB->uxPriority;
			xList *pxList = &( pxReadyTasksLists[ uxProcessor ][ uxCurPri ] );

			if( uxPri > uxCurPri ||
				listIS_CONTAINED_WITHIN( pxList, &( pxTCB->xGenericListItem ) ) == pdFALSE ||
				( uxPri == pxTCB->uxPriority &&
					pxTCB->xYieldFlg != pdFALSE ) )
			{
				listGET_OWNER_OF_NEXT_ENTRY( pxCurrentTCB[ uxProcessor ],
									&( pxReadyTasksLists[ uxProcessor ][ uxPri ] ) );
			}

			/* clear yield flag */
			pxCurrentTCB[ uxProcessor ]->xYieldFlg = pdFALSE;
		}
		#else
		{
			listGET_OWNER_OF_NEXT_ENTRY( pxCurrentTCB[ uxProcessor ],
									&( pxReadyTasksLists[ uxProcessor ][ uxPri ] ) );
		}
		#endif
	}
end:
	taskEXIT_CRITICAL( &xReadyListLock[ uxProcessor ] );
}

#define prvSetTaskOnCurrentTCBOnInit( uxProcessor )	\
{													\
	unsigned portBASE_TYPE uxPri;					\
													\
	taskGET_HIGHEST_PRIORITY( uxProcessor, uxPri );	\
	pxCurrentTCB[ uxProcessor ] = listGET_OWNER_OF_HEAD_ENTRY( &( pxReadyTasksLists[ uxProcessor ][ uxPri ] ) );\
}

/*
 * Function that looks at the list of tasks that are currently delayed to see if
 * any require waking.
 *
 * Tasks are stored in the queue in the order of their wake time - meaning
 * once one tasks has been found whose timer has not expired we need not look
 * any further down the list.
 *
 * This function must be called from vTaskIncrementTick.
 * (Context switch must not rise)
 */
static void prvCheckDelayedTasks( void )
{
	portTickType xItemValue;
	tskTCB *pxTCBS[ configMAX_TASK_NUM ];
	portBASE_TYPE xCPU[ portNUM_PROCESSORS ];
	unsigned portBASE_TYPE uxCurrentCPU;
	int i, j;

	/* Is the tick count greater than or equal to the wake time of the first
	task referenced from the delayed tasks list? */	
	taskENTER_CRITICAL( &xDelayListLock );
	{
		if( xTickCount < xNextTaskUnblockTime )
		{
			taskEXIT_CRITICAL( &xDelayListLock );
			return;
		}

		for( i = 0 ;; )
		{
			if( listLIST_IS_EMPTY( pxDelayedTaskList ) != pdFALSE )
			{
				/* The delayed list is empty.  Set xNextTaskUnblockTime to the
				maximum possible value so it is extremely unlikely that the	
				if( xTickCount >= xNextTaskUnblockTime ) test will pass next
				time through. */
				xNextTaskUnblockTime = portMAX_DELAY;
				if( i > 0 )
				{
					break;
				}

				taskEXIT_CRITICAL( &xDelayListLock );
				return;
			}
			else
			{
				/* The delayed list is not empty, get the value of the item at
				the head of the delayed list.  This is the time at which the
				task at the head of the delayed list should be removed from
				the Blocked state. */
				pxTCBS[i] = ( tskTCB * ) listGET_OWNER_OF_HEAD_ENTRY( pxDelayedTaskList );
				xItemValue = listGET_LIST_ITEM_VALUE( &( pxTCBS[ i ]->xGenericListItem ) );

				if( xTickCount < xItemValue )
				{
					/* It is not time to unblock this item yet, but the item
					value is the time at which the task at the head of the
					blocked list should be removed from the Blocked state -	
					so record the item value in xNextTaskUnblockTime. */
					xNextTaskUnblockTime = xItemValue;
					if( i > 0 )
					{
						break;
					}

					taskEXIT_CRITICAL( &xDelayListLock );
					return;
				}

				/* It is time to remove the item from the Blocked state. */	
				vListRemove( &( pxTCBS[ i ]->xGenericListItem ) );

				/* Is the task waiting on an event also? */
				if( pxTCBS[ i ]->xEventListItem.pvContainer != NULL )
				{
					vListRemove( &( pxTCBS[ i ]->xEventListItem ) );
				}

				i++;
			}
		}
	}
	taskEXIT_CRITICAL( &xDelayListLock );

	/* add tasks to ready queue */
	{
		int xFlg[ portNUM_PROCESSORS ] = { 0 };
		portBASE_TYPE xSwitchCPU;

		for( j = 0; j < i; j++ )
		{
			xSwitchCPU = prvAddTaskToReadyQueue( pxTCBS[ j ], TASK_EXPIRED );
			#if configUSE_PREEMPTION == 0
			{
				if(xSwitchCPU >= 0)
				{
					xFlg[ xSwitchCPU ] = 1;
				}
			}
			#endif
		}

		#if configUSE_PREEMPTION == 0
		{
			/* If using preemption, context switch is raised */
			/* after vTaskIncrementTick.                     */
			/* (Tick handler is must raise context switch.)  */
			uxCurrentCPU = portGetCurrentCPU();
			for( i = 0; i < portNUM_PROCESSORS; i++ )
			{
				if( xFlg[ i ] > 0 )
				{
					if( i == (int)uxCurrentCPU )
					{
						portYIELD_WITHIN_API();	
					}
					else
					{
						portINTERRUPT_CORE( i );
					}
				}
			}
		}
		#endif
	}
}
/*-----------------------------------------------------------*/

/* Constructor. */
void vTaskConstructor( void )
{
	int i;

	for( i = 0; i < portNUM_PROCESSORS; i++ )
	{
		portLOCK_INIT( &xReadyListLock[ i ] );
	}

	portLOCK_INIT( &xDelayListLock );
	portLOCK_INIT( &xTaskCreateLock );
	portLOCK_INIT( &xTagLock );
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * TASK CREATION API documented in task.h
 *----------------------------------------------------------*/
signed portBASE_TYPE xTaskGenericCreate( portBASE_TYPE xProcessor, pdTASK_CODE pxTaskCode, const signed char * const pcName, unsigned short usStackDepth, void *pvParameters, unsigned portBASE_TYPE uxPriority, xTaskHandle *pxCreatedTask, portSTACK_TYPE *puxStackBuffer )
{
signed portBASE_TYPE xReturn, xSwitchCPU;
tskTCB * pxNewTCB;

	configASSERT( pxTaskCode );
	configASSERT( ( uxPriority < configMAX_PRIORITIES ) );

	#if ( configUSE_CPU_UNBOUND_TASK == 1 )
	if( xProcessor != portNO_SPECIFIC_PROCESSOR &&
			( xProcessor < 0 || xProcessor >= portNUM_PROCESSORS ) )
	#else
	if( xProcessor < 0 || xProcessor >= portNUM_PROCESSORS )
	#endif
	{
		return errINVALID_ARGUMENT;
	}

	taskENTER_CRITICAL( &xTaskCreateLock );
	{
		if( pxTaskCode != prvIdleTask && uxCurrentNumberOfTasks >= configMAX_TASK_NUM )
		{
			traceTASK_CREATE_FAILED();
			xReturn = errLIMIT_OVER;
			goto end;
		}

		/* Allocate the memory required by the TCB and stack for the new task,
		checking that the allocation was successful. */
		pxNewTCB = prvAllocateTCBAndStack( usStackDepth, puxStackBuffer );
		if( pxNewTCB == NULL )
		{
			traceTASK_CREATE_FAILED();
			xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
			goto end;
		}

		portSTACK_TYPE *pxTopOfStack;

		#if ( configUSE_CPU_UNBOUND_TASK == 1 )
		if(xProcessor == portNO_SPECIFIC_PROCESSOR)
		{
			/* xCurrentCPU is accessed in prvAddTaskToReadyQueue. */
			pxNewTCB->xCurrentCPU = uxCurrentNumberOfTasks % portNUM_PROCESSORS;
		}
		else
		{
			pxNewTCB->xCurrentCPU = xProcessor;
		}
		#endif

		pxNewTCB->xCPUAffinity = xProcessor;

		/* Calculate the top of stack address.  This depends on whether the
		stack grows from high memory to low (as per the 80x86) or visa versa.
		portSTACK_GROWTH is used to make the result positive or negative as
		required by the port. */
		#if( portSTACK_GROWTH < 0 )
		{
			pxTopOfStack = pxNewTCB->pxStack + ( usStackDepth - ( unsigned short ) 1 );
			pxTopOfStack = ( portSTACK_TYPE * ) ( ( ( portPOINTER_SIZE_TYPE ) pxTopOfStack ) & ( ( portPOINTER_SIZE_TYPE ) ~portBYTE_ALIGNMENT_MASK  ) );

			/* Check the alignment of the calculated top of stack is correct. */
			configASSERT( ( ( ( unsigned long ) pxTopOfStack & ( unsigned long ) portBYTE_ALIGNMENT_MASK ) == 0UL ) );
		}
		#else
		{
			pxTopOfStack = pxNewTCB->pxStack;
			
			/* Check the alignment of the stack buffer is correct. */
			configASSERT( ( ( ( unsigned long ) pxNewTCB->pxStack & ( unsigned long ) portBYTE_ALIGNMENT_MASK ) == 0UL ) );

			/* If we want to use stack checking on architectures that use
			a positive stack growth direction then we also need to store the
			other extreme of the stack space. */
			pxNewTCB->pxEndOfStack = pxNewTCB->pxStack + ( usStackDepth - 1 );
		}
		#endif

		/* Setup the newly allocated TCB with the initial state of the task. */
		prvInitialiseTCBVariables( pxNewTCB, pcName, uxPriority, usStackDepth );

		/* Initialize the TCB stack to look as if the task was already running,
		but had been interrupted by the scheduler.  The return address is set
		to the start of the task function. Once the stack has been initialised
		the	top of stack variable is updated. */
		pxNewTCB->pxTopOfStack = pxPortInitialiseStack( pxTopOfStack, pxTaskCode, pvParameters );

		/* Check the alignment of the initialised stack. */
		portALIGNMENT_ASSERT_pxCurrentTCB( ( ( ( unsigned long ) pxNewTCB->pxTopOfStack & ( unsigned long ) portBYTE_ALIGNMENT_MASK ) == 0UL ) );

		if( ( void * ) pxCreatedTask != NULL )
		{
			/* Pass the TCB out - in an anonymous way.  The calling function/
			task can use this as a handle to delete the task later if
			required.*/
			*pxCreatedTask = ( xTaskHandle ) pxNewTCB;
		}
		
		/* We are going to manipulate the task queues to add this task to a
		ready list. */
		if( pxTaskCode != prvIdleTask )
		{
			uxCurrentNumberOfTasks++;
		}

		if( xTaskInitialised == pdFALSE )
		{
			/* This is the first task to be created so do the preliminary
			initialisation required.  We will not recover if this call
			fails, but we will report the failure. */
			prvInitialiseTaskLists();
			xTaskInitialised = pdTRUE;
		}

		#if ( configUSE_TRACE_FACILITY == 1 )
		{
			/* Add a counter into the TCB for tracing only. */
			pxNewTCB->uxTCBNumber = uxTaskNumber;
		}
		#endif

		uxTaskNumber++;
		portSETUP_TCB( pxNewTCB );
		traceTASK_CREATE( pxNewTCB );

		xSwitchCPU = prvAddTaskToReadyQueue( pxNewTCB, TASK_CREATED );
		if( xSchedulerRunning != pdFALSE && xSwitchCPU >= 0 )
		{
			/* If the created task is of a higher priority than the current task
			then it should run now. */
			if( xSwitchCPU == portGetCurrentCPU() )
			{
				portYIELD_WITHIN_API();
			}
			else
			{
				portINTERRUPT_CORE( xSwitchCPU );
			}	
		}

		xReturn = pdPASS;
end:
		;
	}
	taskEXIT_CRITICAL( &xTaskCreateLock );

	return xReturn;
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------
 * TASK CONTROL API documented in task.h
 *----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelayUntil == 1 )

	void vTaskDelayUntil( portTickType * const pxPreviousWakeTime, portTickType xTimeIncrement )
	{
	portTickType xTimeToWake;
	portBASE_TYPE xShouldDelay = pdFALSE;
	unsigned portBASE_TYPE uxCurrentCPU;

		configASSERT( pxPreviousWakeTime );
		configASSERT( ( xTimeIncrement > 0U ) );

		uxCurrentCPU = portGetCurrentCPU();

		taskENTER_CRITICAL( &xDelayListLock );
		{
			/* Generate the tick time at which the task wants to wake. */
			xTimeToWake = *pxPreviousWakeTime + xTimeIncrement;

			if( xTickCount < *pxPreviousWakeTime )
			{
				/* The tick count has overflowed since this function was
				lasted called.  In this case the only time we should ever
				actually delay is if the wake time has also	overflowed,
				and the wake time is greater than the tick time.  When this
				is the case it is as if neither time had overflowed. */
				if( ( xTimeToWake < *pxPreviousWakeTime ) && ( xTimeToWake > xTickCount ) )
				{
					xShouldDelay = pdTRUE;
				}
			}
			else
			{
				/* The tick time has not overflowed.  In this case we will
				delay if either the wake time has overflowed, and/or the
				tick time is less than the wake time. */
				if( ( xTimeToWake < *pxPreviousWakeTime ) || ( xTimeToWake > xTickCount ) )
				{
					xShouldDelay = pdTRUE;
				}
			}

			/* Update the wake time ready for the next call. */
			*pxPreviousWakeTime = xTimeToWake;

			if( xShouldDelay != pdFALSE )
			{
				traceTASK_DELAY_UNTIL();

				/* We must remove ourselves from the ready list before adding
				ourselves to the blocked list as the same list item is used for
				both lists. */
				taskENTER_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );
				{
					vListRemove( ( xListItem * ) &( pxCurrentTCB[ uxCurrentCPU ]->xGenericListItem ) );
					taskRESET_READY_PRIORITY( uxCurrentCPU, pxCurrentTCB[ uxCurrentCPU ]->uxPriority );
				}
				taskEXIT_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );

				prvAddCurrentTaskToDelayedList( xTimeToWake );
				portYIELD_WITHIN_API();
			}
		}
		taskEXIT_CRITICAL( &xDelayListLock );
	}
#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskDelay == 1 )

	void vTaskDelay( portTickType xTicksToDelay )
	{
	portTickType xTimeToWake;
	unsigned portBASE_TYPE uxCurrentCPU;

		uxCurrentCPU = portGetCurrentCPU();

		/* A delay time of zero just forces a reschedule. */
		if( xTicksToDelay > ( portTickType ) 0U )
		{
			taskENTER_CRITICAL( &xDelayListLock );
			{
				traceTASK_DELAY();

				/* A task that is removed from the event list while the
				scheduler is suspended will not get placed in the ready
				list or removed from the blocked list until the scheduler
				is resumed.

				This task cannot be in an event list as it is the currently
				executing task. */

				/* Calculate the time to wake - this may overflow but this is
				not a problem. */
				xTimeToWake = xTickCount + xTicksToDelay;

				/* We must remove ourselves from the ready list before adding
				ourselves to the blocked list as the same list item is used for
				both lists. */
				taskENTER_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );
				{
					vListRemove( ( xListItem * ) &( pxCurrentTCB[ uxCurrentCPU ]->xGenericListItem ) );
					taskRESET_READY_PRIORITY( uxCurrentCPU, pxCurrentTCB[ uxCurrentCPU ]->uxPriority );
				}
				taskEXIT_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );

				prvAddCurrentTaskToDelayedList( xTimeToWake );
				portYIELD_WITHIN_API();
			}
			taskEXIT_CRITICAL( &xDelayListLock );
		}
		else
		{
			#if ( configUSE_PREEMPTION == 0 )
			{
				unsigned long xStat;

				xStat = portSET_INTERRUPT_MASK_AND_RETURN();

				pxCurrentTCB[ uxCurrentCPU ]->xYieldFlg = pdTRUE;
				portYIELD_WITHIN_API();

				portCLEAR_INTERRUPT_MASK_AND_SET( xStat );
			}
			#else
			{
				portYIELD_WITHIN_API();
			}
			#endif
		}
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskPriorityGet == 1 )

	unsigned portBASE_TYPE uxTaskPriorityGetCurrent( void )
	{
	tskTCB *pxTCB;

		pxTCB = pxCurrentTCB[ portGetCurrentCPU() ];
		return pxTCB->uxPriority;
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_vTaskPrioritySet == 1 )

	unsigned portBASE_TYPE uxTaskPrioritySetCurrent( unsigned portBASE_TYPE uxNewPriority )
	{
	tskTCB *pxTCB;
	unsigned long xStat;
	unsigned portBASE_TYPE uxCurrentCPU, uxCurrentPriority;
	portBASE_TYPE xSwitch, xYieldRequired = pdFALSE;

		configASSERT( ( uxNewPriority < configMAX_PRIORITIES ) );

		/* Ensure the new priority is valid. */
		if( uxNewPriority >= configMAX_PRIORITIES )
		{
			uxNewPriority = configMAX_PRIORITIES - ( unsigned portBASE_TYPE ) 1U;
		}

		uxCurrentCPU = portGetCurrentCPU();

		xStat = portSET_INTERRUPT_MASK_AND_RETURN();
		pxTCB = pxCurrentTCB[ uxCurrentCPU ];
		traceTASK_PRIORITY_SET( pxTCB, uxNewPriority );

		uxCurrentPriority = pxTCB->uxPriority;
		if( uxCurrentPriority == uxNewPriority )
		{
			goto end;
		}

		pxTCB->uxPriority = uxNewPriority;
		/* Event lists are always in priority order. */
		listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ), configMAX_PRIORITIES - ( portTickType ) uxNewPriority );

		taskENTER_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );
		{
			/* remove current task from current priority list.  */
			vListRemove( &( pxTCB->xGenericListItem ) );
			taskRESET_READY_PRIORITY( uxCurrentCPU, uxCurrentPriority );
		}
		taskEXIT_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );

		/* add current task to new priority list. */
		xSwitch = prvAddTaskToReadyQueue( pxTCB, TASK_STATE_NOTCHANGE );
		if( xSwitch != uxCurrentCPU )
		{
			/* raise context switch on this core. */
			portYIELD_WITHIN_API();

			if( xSwitch >= 0 )
			{
				/* raise context switch on the other core. */
				portINTERRUPT_CORE( xSwitch );
			}
		}
		/* if xSwitch == uxCurrentCPU, current task is */
		/* highest priority task on this core.         */

end:
		portCLEAR_INTERRUPT_MASK_AND_SET( xStat );
		return uxCurrentPriority;
	}

#endif
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------
 * PUBLIC SCHEDULER CONTROL documented in task.h
 *----------------------------------------------------------*/


void vTaskStartScheduler( void )
{
portBASE_TYPE xReturn;
int i;

	/* Create the idle task without storing its handle. */
	for( i = 0; i < portNUM_PROCESSORS; i++ )
	{
		xReturn = xTaskCreate( i, prvIdleTask, ( signed char * ) "IDLE", tskIDLE_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY, NULL );
		if( xReturn != pdPASS)
		{
			break;
		}
	}

	if( xReturn == pdPASS )
	{
		/* Interrupts are turned off here, to ensure a tick does not occur
		before or during the call to xPortStartScheduler().  The stacks of
		the created tasks contain a status word with interrupts switched on
		so interrupts will automatically get re-enabled when the first task
		starts to run.

		STEPPING THROUGH HERE USING A DEBUGGER CAN CAUSE BIG PROBLEMS IF THE
		DEBUGGER ALLOWS INTERRUPTS TO BE PROCESSED. */
		portDISABLE_INTERRUPTS();

		for( i = 0; i < portNUM_PROCESSORS; i++ )
		{
			prvSetTaskOnCurrentTCBOnInit( i );
		}

		xSchedulerRunning = pdTRUE;
		xTickCount = ( portTickType ) 0U;

		/* If configGENERATE_RUN_TIME_STATS is defined then the following
		macro must be defined to configure the timer/counter used to generate
		the run time counter time base. */
		portCONFIGURE_TIMER_FOR_RUN_TIME_STATS();

		/* Setting up the timer tick is hardware specific and thus in the
		portable interface. */
		if( xPortStartScheduler() != pdFALSE )
		{
			/* Should not reach here as if the scheduler is running the
			function will not return. */
		}
		else
		{
			/* Should only reach here if a task calls xTaskEndScheduler(). */
		}
	}

	/* This line will only be reached if the kernel could not be started. */
	configASSERT( xReturn );
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------
 * PUBLIC TASK UTILITIES documented in task.h
 *----------------------------------------------------------*/

portTickType xTaskGetTickCount( void )
{
	return xTickCount;
}
/*-----------------------------------------------------------*/

unsigned portBASE_TYPE uxTaskGetNumberOfTasks( void )
{
	/* A critical section is not required because the variables are of type
	portBASE_TYPE. */
	return uxCurrentNumberOfTasks;
}
/*-----------------------------------------------------------*/

#if ( INCLUDE_pcTaskGetTaskName == 1 )

	signed char *pcTaskGetTaskName( xTaskHandle xTaskToQuery )
	{
	tskTCB *pxTCB;

		/* If null is passed in here then the name of the calling task is being queried. */
		if( xTaskToQuery == NULL )
		{
			xTaskToQuery = pxCurrentTCB[ portGetCurrentCPU() ];
		}

		pxTCB = prvGetTCBFromHandle( xTaskToQuery );
		configASSERT( pxTCB );
		return &( pxTCB->pcTaskName[ 0 ] );
	}

#endif
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * SCHEDULER INTERNALS AVAILABLE FOR PORTING PURPOSES
 * documented in task.h
 *----------------------------------------------------------*/

void vTaskIncrementTick( void )
{
tskTCB * pxTCB;

	if( portGetCurrentCPU() != portTICKEXE_CORENUM )
	{
		return;
	}

	/* Called by the portable layer each time a tick interrupt occurs(master core only).
	Increments the tick then checks to see if the new tick value will cause any
	tasks to be unblocked. */
	taskENTER_CRITICAL( &xDelayListLock );
	{
		/* xTickCount is only modified at this point */
		portINC_TICK_SYNC( &xTickCount );
		if( xTickCount == ( portTickType ) 0U )
		{
			xList *pxTemp;

			/* Tick count has overflowed so we need to swap the delay lists.
			If there are any items in pxDelayedTaskList here then there is
			an error! */
			configASSERT( ( listLIST_IS_EMPTY( pxDelayedTaskList ) ) );
	
			pxTemp = pxDelayedTaskList;
			pxDelayedTaskList = pxOverflowDelayedTaskList;
			pxOverflowDelayedTaskList = pxTemp;
			xNumOfOverflows++;

			if( listLIST_IS_EMPTY( pxDelayedTaskList ) != pdFALSE )
			{
				/* The new current delayed list is empty.  Set
				xNextTaskUnblockTime to the maximum possible value so it is
				extremely unlikely that the	
				if( xTickCount >= xNextTaskUnblockTime ) test will pass until
				there is an item in the delayed list. */
				xNextTaskUnblockTime = portMAX_DELAY;
			}
			else
			{
				/* The new current delayed list is not empty, get the value of
				the item at the head of the delayed list.  This is the time at
				which the task at the head of the delayed list should be removed
				from the Blocked state. */
				pxTCB = ( tskTCB * ) listGET_OWNER_OF_HEAD_ENTRY( pxDelayedTaskList );
				xNextTaskUnblockTime = listGET_LIST_ITEM_VALUE( &( pxTCB->xGenericListItem ) );
			}
		}
	}
	taskEXIT_CRITICAL( &xDelayListLock );

	/* See if this tick has made a timeout expire. */
	prvCheckDelayedTasks();

	/* The tick hook gets called at regular intervals, even if the
	scheduler is locked. */
	#if ( configUSE_TICK_HOOK == 1 )
	{
		vApplicationTickHook();
	}
	#endif

	traceTASK_INCREMENT_TICK( xTickCount );
}
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

	void vTaskSetApplicationTaskTag( xTaskHandle xTask, pdTASK_HOOK_CODE pxHookFunction )
	{
	tskTCB *xTCB;

		/* If xTask is NULL then we are setting our own task hook. */
		if( xTask == NULL )
		{
			xTCB = ( tskTCB * ) pxCurrentTCB[ portGetCurrentCPU() ];
		}
		else
		{
			xTCB = ( tskTCB * ) xTask;
		}

		/* Save the hook function in the TCB.  A critical section is required as
		the value can be accessed from an interrupt. */
		taskENTER_CRITICAL( &xTagLock );
		{
			xTCB->pxTaskTag = pxHookFunction;
		}
		taskEXIT_CRITICAL( &xTagLock );
	}

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

	pdTASK_HOOK_CODE xTaskGetApplicationTaskTag( xTaskHandle xTask )
	{
	tskTCB *xTCB;
	pdTASK_HOOK_CODE xReturn;

		/* If xTask is NULL then we are setting our own task hook. */
		if( xTask == NULL )
		{
			xTCB = ( tskTCB * ) pxCurrentTCB[ portGetCurrentCPU() ];
		}
		else
		{
			xTCB = ( tskTCB * ) xTask;
		}

		/* Save the hook function in the TCB.  A critical section is required as
		the value can be accessed from an interrupt. */
		taskENTER_CRITICAL( &xTagLock );
		{
			xReturn = xTCB->pxTaskTag;
		}
		taskEXIT_CRITICAL( &xTagLock );

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

#if ( configUSE_APPLICATION_TASK_TAG == 1 )

	portBASE_TYPE xTaskCallApplicationTaskHook( xTaskHandle xTask, void *pvParameter )
	{
	tskTCB *xTCB;
	portBASE_TYPE xReturn;
	pdTASK_HOOK_CODE pxHook;

		/* If xTask is NULL then we are calling our own task hook. */
		if( xTask == NULL )
		{
			xTCB = ( tskTCB * ) pxCurrentTCB[ portGetCurrentCPU() ];
		}
		else
		{
			xTCB = ( tskTCB * ) xTask;
		}

		taskENTER_CRITICAL( &xTagLock );
		{
			pxHook = xTCB->pxTaskTag;
		}
		taskEXIT_CRITICAL( &xTagLock );

		if( pxHook != NULL )
		{
			xReturn = pxHook( pvParameter );
		}
		else
		{
			xReturn = pdFAIL;
		}

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

void vTaskSwitchContext( void )
{
	tskTCB *pxTCB;
	portBASE_TYPE xSwitch;
	unsigned portBASE_TYPE uxCurrentCPU;

	traceTASK_SWITCHED_OUT();

	uxCurrentCPU = portGetCurrentCPU();

	#if ( configGENERATE_RUN_TIME_STATS == 1 )
	{
		unsigned long ulTempCounter;
		
			#ifdef portALT_GET_RUN_TIME_COUNTER_VALUE
				portALT_GET_RUN_TIME_COUNTER_VALUE( ulTempCounter );
			#else
				ulTempCounter = portGET_RUN_TIME_COUNTER_VALUE();
			#endif

			/* Add the amount of time the task has been running to the accumulated
			time so far.  The time the task started running was stored in
			ulTaskSwitchedInTime.  Note that there is no overflow protection here
			so count values are only valid until the timer overflows.  Generally
			this will be about 1 hour assuming a 1uS timer increment. */
			pxCurrentTCB[ uxCurrentCPU ]->ulRunTimeCounter += ( ulTempCounter - ulTaskSwitchedInTime );
			ulTaskSwitchedInTime = ulTempCounter;
	}
	#endif

	taskFIRST_CHECK_FOR_STACK_OVERFLOW( uxCurrentCPU );
	taskSECOND_CHECK_FOR_STACK_OVERFLOW( uxCurrentCPU );

	#if ( configUSE_CPU_UNBOUND_TASK == 1 )
	pxTCB = pxCurrentTCB[ uxCurrentCPU ];
	#endif

	/* Find the highest priority queue that contains ready tasks. */
	prvSetTaskOnCurrentTCB( uxCurrentCPU );

	#if ( configUSE_CPU_UNBOUND_TASK == 1 )
	if( pxCurrentTCB[ uxCurrentCPU ] != pxTCB &&			/* switched? */
		pxTCB->xCPUAffinity == portNO_SPECIFIC_PROCESSOR && /* unbound task? */
		listIS_CONTAINED_WITHIN(&( pxReadyTasksLists[ uxCurrentCPU ][ pxTCB->uxPriority ] ),
			&( pxTCB->xGenericListItem ) ) != pdFALSE )		/* still runnable? */
	{
		xSwitch = prvUnBoundTaskMigration( pxTCB );
		if( xSwitch >= 0 && xSwitch != uxCurrentCPU )
		{
			portINTERRUPT_CORE( xSwitch );
		}
	}
	#endif

	traceTASK_SWITCHED_IN();
}
/*-----------------------------------------------------------*/

void vTaskPlaceOnEventList( const xList * const pxEventList, portTickType xTicksToWait )
{
portTickType xTimeToWake;
unsigned portBASE_TYPE uxCurrentCPU;
tskTCB *pxTCB;

	configASSERT( pxEventList );

	/* THIS FUNCTION MUST BE CALLED IN CRITICAL SECTION. */

	uxCurrentCPU = portGetCurrentCPU();
	pxTCB = pxCurrentTCB[ uxCurrentCPU ];

	/* Place the event list item of the TCB in the appropriate event list.
	This is placed in the list in priority order so the highest priority task
	is the first to be woken by the event. */
	vListInsert( ( xList * ) pxEventList, ( xListItem * ) &( pxTCB->xEventListItem ) );

	/* We must remove ourselves from the ready list before adding ourselves
	to the blocked list as the same list item is used for both lists.  We have
	exclusive access to the ready lists as the scheduler is locked. */
	taskENTER_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );
	{
		vListRemove( ( xListItem * ) &( pxTCB->xGenericListItem ) );
		taskRESET_READY_PRIORITY( uxCurrentCPU, pxTCB->uxPriority );
	}
	taskEXIT_CRITICAL( &xReadyListLock[ uxCurrentCPU ] );

	/* Calculate the time at which the task should be woken if the event does
	not occur.  This may overflow but this doesn't matter. */
	if( xTicksToWait == portMAX_DELAY )
	{
		taskENTER_CRITICAL( &xDelayListLock );
		{
			vListInsertEnd( ( xList * ) &xInfinitDelayTaskList, ( xListItem * ) &( pxTCB->xGenericListItem ) );
		}
		taskEXIT_CRITICAL( &xDelayListLock );
	}
	else
	{
		taskENTER_CRITICAL( &xDelayListLock );
		{
			xTimeToWake = xTickCount + xTicksToWait;
			prvAddCurrentTaskToDelayedList( xTimeToWake );
		}
		taskEXIT_CRITICAL( &xDelayListLock );
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xTaskRemoveFromEventList( const xList * const pxEventList, portBASE_TYPE doSwitch )
{
tskTCB *pxUnblockedTCB;
portBASE_TYPE xSwitch;

	/* THIS FUNCTION MUST BE CALLED IN CRITICAL SECTION.
	   It can also be called from within an ISR. */

	/* The event list is sorted in priority order, so we can remove the
	first in the list, remove the TCB from the delayed list, and add
	it to the ready list. */

	taskENTER_CRITICAL( &xDelayListLock );
	{
		if( listLIST_IS_EMPTY( pxEventList ) == pdFALSE )
		{
			pxUnblockedTCB = ( tskTCB * ) listGET_OWNER_OF_HEAD_ENTRY( pxEventList );
			configASSERT( pxUnblockedTCB );

			vListRemove( &( pxUnblockedTCB->xEventListItem ) );
			vListRemove( &( pxUnblockedTCB->xGenericListItem ) );
		}
		else
		{
			taskEXIT_CRITICAL( &xDelayListLock );
			return -1;
		}
	}
	taskEXIT_CRITICAL( &xDelayListLock );

	xSwitch = prvAddTaskToReadyQueue( pxUnblockedTCB, TASK_EVENT_OCCURRED );
	if( doSwitch && xSwitch >= 0 )
	{
		if( xSwitch == portGetCurrentCPU() )
		{
			portYIELD_WITHIN_API();
		}
		else
		{
			portINTERRUPT_CORE( xSwitch );
		}
	}

	return xSwitch;
}
/*-----------------------------------------------------------*/

void vTaskSetTimeOutState( xTimeOutType * const pxTimeOut )
{
	configASSERT( pxTimeOut );

	taskENTER_CRITICAL( &xDelayListLock );
	{
		pxTimeOut->xOverflowCount = xNumOfOverflows;
		pxTimeOut->xTimeOnEntering = xTickCount;
	}
	taskEXIT_CRITICAL( &xDelayListLock );
}
/*-----------------------------------------------------------*/

portBASE_TYPE xTaskCheckForTimeOut( xTimeOutType * const pxTimeOut, portTickType * const pxTicksToWait )
{
portBASE_TYPE xReturn;

	configASSERT( pxTimeOut );
	configASSERT( pxTicksToWait );

	if( *pxTicksToWait == portMAX_DELAY )
	{
		xReturn = pdFALSE;
	}
	else
	{
		portTickType xTickTmp;
		portBASE_TYPE xOverflowTmp;

		taskENTER_CRITICAL( &xDelayListLock );
		{
			xTickTmp = xTickCount;
			xOverflowTmp = xNumOfOverflows;
		}
		taskEXIT_CRITICAL( &xDelayListLock );

		if( ( xOverflowTmp != pxTimeOut->xOverflowCount ) && ( xTickTmp >= ( portTickType ) pxTimeOut->xTimeOnEntering ) )
		{
			/* The tick count is greater than the time at which vTaskSetTimeout()
			was called, but has also overflowed since vTaskSetTimeOut() was called.
			It must have wrapped all the way around and gone past us again. This
			passed since vTaskSetTimeout() was called. */
			xReturn = pdTRUE;
		}
		else if( ( ( portTickType ) ( xTickTmp - ( portTickType ) pxTimeOut->xTimeOnEntering ) ) < *pxTicksToWait )
		{
			/* Not a genuine timeout. Adjust parameters for time remaining. */
			*pxTicksToWait -= ( ( portTickType ) xTickTmp - ( portTickType ) pxTimeOut->xTimeOnEntering );
			pxTimeOut->xOverflowCount = xOverflowTmp;
			pxTimeOut->xTimeOnEntering = xTickTmp;
			xReturn = pdFALSE;
		}
		else
		{
			xReturn = pdTRUE;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

#if ( configARCH_SPECIFIC_LOCK == 0 )
void vTaskLockInit( portLOCK_TYPE *pxLock )
{
	int i;

	portSPINLOCK_INIT( &pxLock->xLock );
	for( i = 0; i < portNUM_PROCESSORS; i++ )
	{
		pxLock->xNest[ i ] = 0;
		pxLock->uxStat[ i ] = portIRQSTAT_INIT;
	}
}

void vTaskAcquireLock( portLOCK_TYPE * pxLock )
{
	unsigned portBASE_TYPE uxCurrentCPU;
	portSPINLOCK_TYPE *pxLockCore; 
	portBASE_TYPE *pxNest;
	unsigned long *pxIRQStat;

	uxCurrentCPU = portGetCurrentCPU();
	pxLockCore = &pxLock->xLock;
	pxNest = &pxLock->xNest[ uxCurrentCPU ];
	pxIRQStat = &pxLock->uxStat[ uxCurrentCPU ];

	if( *pxNest == 0 )
	{
		*pxIRQStat = portSET_INTERRUPT_MASK_AND_RETURN();
		vPortSpinLock( uxCurrentCPU, pxLockCore );
	}

	(*pxNest)++;
}

void vTaskReleaseLock( portLOCK_TYPE * pxLock )
{
	unsigned portBASE_TYPE uxCurrentCPU;
	portSPINLOCK_TYPE *pxLockCore; 
	portBASE_TYPE *pxNest;
	unsigned long *pxIRQStat;

	uxCurrentCPU = portGetCurrentCPU();
	pxLockCore = &pxLock->xLock;
	pxNest = &pxLock->xNest[ uxCurrentCPU ];
	pxIRQStat = &pxLock->uxStat[ uxCurrentCPU ];

	/* interrupt is diabled at this point */
	if( *pxNest > 0 )
	{
		(*pxNest)--;

		if( *pxNest == 0 )
		{
			/* vPortSpinUnLock must guarantee to sync memory. */
			vPortSpinUnLock( uxCurrentCPU, pxLockCore );
			portCLEAR_INTERRUPT_MASK_AND_SET( *pxIRQStat );
		}
	}
}
#endif /* configARCH_SPECIFIC_LOCK */

/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )
	unsigned portBASE_TYPE uxTaskGetTaskNumber( xTaskHandle xTask )
	{
	unsigned portBASE_TYPE uxReturn;
	tskTCB *pxTCB;
	
		if( xTask != NULL )
		{
			pxTCB = ( tskTCB * ) xTask;
			uxReturn = pxTCB->uxTaskNumber;
		}
		else
		{
			uxReturn = 0U;
		}
		
		return uxReturn;
	}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )
	void vTaskSetTaskNumber( xTaskHandle xTask, unsigned portBASE_TYPE uxHandle )
	{
	tskTCB *pxTCB;
	
		if( xTask != NULL )
		{
			pxTCB = ( tskTCB * ) xTask;
			pxTCB->uxTaskNumber = uxHandle;
		}
	}
#endif


/*
 * -----------------------------------------------------------
 * The Idle task.
 * ----------------------------------------------------------
 *
 * The portTASK_FUNCTION() macro is used to allow port/compiler specific
 * language extensions.  The equivalent prototype for this function is:
 *
 * void prvIdleTask( void *pvParameters );
 *
 */
static portTASK_FUNCTION( prvIdleTask, pvParameters )
{
	/* Stop warnings. */
	( void ) pvParameters;

	for( ;; )
	{
		#if ( configUSE_IDLE_SLEEP == 1 )
		{
			portIDLE_SLEEP();
		}
		#elif ( configUSE_PREEMPTION == 0 )
		{
			/* If we are not using preemption we keep forcing a task switch to
			see if any other task has become available.  If we are using
			preemption we don't need to do this as any task becoming available
			will automatically get the processor anyway. */
			vTaskDelay( 0 );
		}
		#endif

		#if ( ( configUSE_PREEMPTION == 1 ) && ( configIDLE_SHOULD_YIELD == 1 ) )
		{
			/* When using preemption tasks of equal priority will be
			timesliced.  If a task that is sharing the idle priority is ready
			to run then the idle task should yield before the end of the
			timeslice.

			A critical region is not required here as we are just reading from
			the list, and an occasional incorrect value will not matter.  If
			the ready list at the idle priority contains more than one task
			then a task other than the idle task is ready to execute. */
			if( listCURRENT_LIST_LENGTH( &( pxReadyTasksLists[ portGetCurrentCPU() ][ tskIDLE_PRIORITY ] ) ) > ( unsigned portBASE_TYPE ) 1 )
			{
				taskYIELD();
			}
		}
		#endif

		#if ( configUSE_IDLE_HOOK == 1 )
		{
			extern void vApplicationIdleHook( void );

			/* Call the user defined function from within the idle task.  This
			allows the application designer to add background functionality
			without the overhead of a separate task.
			NOTE: vApplicationIdleHook() MUST NOT, UNDER ANY CIRCUMSTANCES,
			CALL A FUNCTION THAT MIGHT BLOCK. */
			vApplicationIdleHook();
		}
		#endif
	}
} /*lint !e715 pvParameters is not accessed but all task functions require the same prototype. */


/*-----------------------------------------------------------
 * File private functions documented at the top of the file.
 *----------------------------------------------------------*/

static void prvInitialiseTCBVariables( tskTCB *pxTCB, const signed char * const pcName, unsigned portBASE_TYPE uxPriority, unsigned short usStackDepth )
{
	/* Store the function name in the TCB. */
	#if configMAX_TASK_NAME_LEN > 1
	{
		/* Don't bring strncpy into the build unnecessarily. */
		strncpy( ( char * ) pxTCB->pcTaskName, ( const char * ) pcName, ( unsigned short ) configMAX_TASK_NAME_LEN );
	}
	#endif
	pxTCB->pcTaskName[ ( unsigned short ) configMAX_TASK_NAME_LEN - ( unsigned short ) 1 ] = ( signed char ) '\0';

	/* This is used as an array index so must ensure it's not too large.  First
	remove the privilege bit if one is present. */
	if( uxPriority >= configMAX_PRIORITIES )
	{
		uxPriority = configMAX_PRIORITIES - ( unsigned portBASE_TYPE ) 1U;
	}

	pxTCB->uxPriority = uxPriority;

	vListInitialiseItem( &( pxTCB->xGenericListItem ) );
	vListInitialiseItem( &( pxTCB->xEventListItem ) );

	/* Set the pxTCB as a link back from the xListItem.  This is so we can get
	back to	the containing TCB from a generic item in a list. */
	listSET_LIST_ITEM_OWNER( &( pxTCB->xGenericListItem ), pxTCB );

	/* Event lists are always in priority order. */
	listSET_LIST_ITEM_VALUE( &( pxTCB->xEventListItem ), configMAX_PRIORITIES - ( portTickType ) uxPriority );
	listSET_LIST_ITEM_OWNER( &( pxTCB->xEventListItem ), pxTCB );

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
	{
		pxTCB->pxTaskTag = NULL;
	}
	#endif

	#if ( configGENERATE_RUN_TIME_STATS == 1 )
	{
		pxTCB->ulRunTimeCounter = 0UL;
	}
	#endif

	#if ( configUSE_PREEMPTION == 0 )
	{
		pxTCB->xYieldFlg = pdFALSE;
	}
	#endif
}
/*-----------------------------------------------------------*/

static void prvInitialiseTaskLists( void )
{
unsigned portBASE_TYPE uxPriority, uxProcessor;

	for( uxProcessor = 0; uxProcessor < portNUM_PROCESSORS; uxProcessor++ )
	{
		for( uxPriority = ( unsigned portBASE_TYPE ) 0U; uxPriority < configMAX_PRIORITIES; uxPriority++ )
		{
			vListInitialise( ( xList * ) &( pxReadyTasksLists[ uxProcessor ][ uxPriority ] ) );
		}
#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 0
		uxTopReadyPriority[ uxProcessor ] = tskIDLE_PRIORITY;
#else
		uxTopReadyPriority[ uxProcessor ] = 0;
		taskRECORD_READY_PRIORITY( uxProcessor, tskIDLE_PRIORITY );
#endif
	}

	vListInitialise( ( xList * ) &xDelayedTaskList1 );
	vListInitialise( ( xList * ) &xDelayedTaskList2 );
	vListInitialise( ( xList * ) &xInfinitDelayTaskList );

	/* Start with pxDelayedTaskList using list1 and the pxOverflowDelayedTaskList
	using list2. */
	pxDelayedTaskList = &xDelayedTaskList1;
	pxOverflowDelayedTaskList = &xDelayedTaskList2;
}
/*-----------------------------------------------------------*/

/* This function must be called in critical section.(with xDelayListLock) */
static void prvAddCurrentTaskToDelayedList( portTickType xTimeToWake )
{
	tskTCB *pxTCB;

	pxTCB = pxCurrentTCB[ portGetCurrentCPU() ];

	/* The list item will be inserted in wake time order. */
	listSET_LIST_ITEM_VALUE( &( pxTCB->xGenericListItem ), xTimeToWake );

	if( xTimeToWake < xTickCount )
	{
		/* Wake time has overflowed.  Place this item in the overflow list. */
		vListInsert( ( xList * ) pxOverflowDelayedTaskList, ( xListItem * ) &( pxTCB->xGenericListItem ) );
	}
	else
	{
		/* The wake time has not overflowed, so we can use the current block list. */
		vListInsert( ( xList * ) pxDelayedTaskList, ( xListItem * ) &( pxTCB->xGenericListItem ) );

		/* If the task entering the blocked state was placed at the head of the
		list of blocked tasks then xNextTaskUnblockTime needs to be updated
		too. */
		if( xTimeToWake < xNextTaskUnblockTime )
		{
			xNextTaskUnblockTime = xTimeToWake;
		}
	}
}
/*-----------------------------------------------------------*/

static tskTCB *prvAllocateTCBAndStack( unsigned short usStackDepth, portSTACK_TYPE *puxStackBuffer )
{
tskTCB *pxNewTCB;

	/* Allocate space for the TCB.  Where the memory comes from depends on
	the implementation of the port malloc function. */
	pxNewTCB = ( tskTCB * ) pvPortMalloc( sizeof( tskTCB ) );

	if( pxNewTCB != NULL )
	{
		/* Allocate space for the stack used by the task being created.
		The base of the stack memory stored in the TCB so the task can
		be deleted later if required. */
		pxNewTCB->pxStack = ( portSTACK_TYPE * ) pvPortMallocAligned( ( ( ( size_t )usStackDepth ) * sizeof( portSTACK_TYPE ) ), puxStackBuffer );

		if( pxNewTCB->pxStack == NULL )
		{
			/* Could not allocate the stack.  Delete the allocated TCB. */
			vPortFree( pxNewTCB );
			pxNewTCB = NULL;
		}
		else
		{
			/* Just to help debugging. */
			memset( pxNewTCB->pxStack, ( int ) tskSTACK_FILL_BYTE, ( size_t ) usStackDepth * sizeof( portSTACK_TYPE ) );
		}
	}

	return pxNewTCB;
}
/*-----------------------------------------------------------*/

#if ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) )

	static unsigned short usTaskCheckFreeStackSpace( const unsigned char * pucStackByte )
	{
	register unsigned short usCount = 0U;

		while( *pucStackByte == tskSTACK_FILL_BYTE )
		{
			pucStackByte -= portSTACK_GROWTH;
			usCount++;
		}

		usCount /= sizeof( portSTACK_TYPE );

		return usCount;
	}

#endif
/*-----------------------------------------------------------*/

#if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )

	unsigned portBASE_TYPE uxTaskGetStackHighWaterMark( void )
	{
	tskTCB *pxTCB;
	unsigned char *pcEndOfStack;
	unsigned portBASE_TYPE uxReturn;
	unsigned long xStat;

		xStat = portSET_INTERRUPT_MASK_AND_RETURN();
		pxTCB = pxCurrentTCB[ portGetCurrentCPU() ];

		#if portSTACK_GROWTH < 0
		{
			pcEndOfStack = ( unsigned char * ) pxTCB->pxStack;
		}
		#else
		{
			pcEndOfStack = ( unsigned char * ) pxTCB->pxEndOfStack;
		}
		#endif

		uxReturn = ( unsigned portBASE_TYPE ) usTaskCheckFreeStackSpace( pcEndOfStack );
		portCLEAR_INTERRUPT_MASK_AND_SET( xStat );

		return uxReturn;
	}

#endif
/*-----------------------------------------------------------*/

#if ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) )

	xTaskHandle xTaskGetCurrentTaskHandle( void )
	{
	xTaskHandle xReturn;

		/* A critical section is not required as this is not called from
		an interrupt and the current TCB will always be the same for any
		individual execution thread. */
		xReturn = pxCurrentTCB[ portGetCurrentCPU() ];

		return xReturn;
	}

#endif
/*-----------------------------------------------------------*/

portBASE_TYPE xTaskGetLastReadiedState( void )
{
	return pxCurrentTCB[ portGetCurrentCPU() ]->xLastReadiedState;
}
/*-----------------------------------------------------------*/

