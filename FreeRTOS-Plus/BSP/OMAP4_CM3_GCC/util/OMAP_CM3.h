/* This file is based on LPC17xx.h.
 *
 * The original license terms are as follows.
 */
/**************************************************************************//**
 * @file     LPC17xx.h
 * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Header File for 
 *           NXP LPC17xx Device Series
 * @version: V1.09
 * @date:    17. March 2010

 *
 * @note
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#ifndef __OMAP_CM3_H__
#define __OMAP_CM3_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn           = -14,      /*!< 2 Non Maskable Interrupt                         */
  MemoryManagement_IRQn         = -12,      /*!< 4 Cortex-M3 Memory Management Interrupt          */
  BusFault_IRQn                 = -11,      /*!< 5 Cortex-M3 Bus Fault Interrupt                  */
  UsageFault_IRQn               = -10,      /*!< 6 Cortex-M3 Usage Fault Interrupt                */
  /*SVCall_IRQn                 = -5,*/     /*!< 11 Cortex-M3 SV Call Interrupt(reserved for Kernel) */
  DebugMonitor_IRQn             = -4,       /*!< 12 Cortex-M3 Debug Monitor Interrupt             */
  /*PendSV_IRQn                 = -2,*/     /*!< 14 Cortex-M3 Pend SV Interrupt(reserved for Kernel) */
  SysTick_IRQn                  = -1,       /*!< 15 Cortex-M3 System Tick Interrupt               */

/******  OMAP Specific Interrupt Numbers **********************************************************/
  XLATE_MMU_FAULT_IRQn          = 0,        /* L2 cache MMU interrupt                             */
  SHARED_CACHE_MMU_CPU_INT_IRQn = 1,        /* Shared cache or MMU maintenance completed          */
  CTM_TIM_EVENT_1_IRQn          = 2,        /* CTM timer event (timer #1)                         */
/*HWSEM_M3_IRQ_IRQn             = 3,*/      /* Semaphore interrupt(reserved for Kernel)           */
  IC_NEMUINTR_IRQn              = 4,        /* ICECrusher (one to each core)                      */
  IMP_FAULT_IRQn                = 5,        /* Imprecise fault (from interconnect)                */
  CTM_TIM_EVENT_2_IRQn          = 6,        /* CTM timer event (timer #2)                         */
  DSS_DISPC_IRQ_IRQn            = 7,        /* Display controller interrupt                       */
  DSS_DSI1_IRQ_IRQn             = 8,        /* Display subsystem DSI1 interrupt                   */
  DSS_DSI2_IRQ_IRQn             = 9,        /* Display subsystem DSI2 interrupt                   */
  DSS_HDMI_IRQ_IRQn             = 10,       /* Display subsystem HDMI interrupt                   */
  ISS_IRQ0_IRQn                 = 11,       /* Imaging subsystem interrupt 0                      */
  ISS_IRQ1_IRQn                 = 12,       /* Imaging subsystem interrupt 1                      */
  ISS_IRQ2_IRQn                 = 13,       /* Imaging subsystem interrupt 2                      */
  ISS_IRQ3_IRQn                 = 14,       /* Imaging subsystem interrupt 3                      */
  ISS_IRQ4_IRQn                 = 15,       /* Imaging subsystem interrupt 4                      */
  ISS_IRQ5_IRQn                 = 16,       /* Imaging subsystem interrupt 5                      */
  FDIF_IRQ_1_IRQn               = 17,       /* Face Detect interrupt 1                            */
  SDMA_IRQ_0_IRQn               = 18,       /* sDMA interrupt 0                                   */
  SDMA_IRQ_1_IRQn               = 19,       /* sDMA interrupt 1                                   */
  SDMA_IRQ_2_IRQn               = 20,       /* sDMA interrupt 2                                   */
  SDMA_IRQ_3_IRQn               = 21,       /* sDMA interrupt 3                                   */
  IVAHD_MAILBOX_IRQ_2_IRQn      = 22,       /* IVAHD mailbox interrupt 2                          */
  IVAHD_IRQ2_IRQn               = 23,       /* Sync interrupt from ICONT2 (vDMA)                  */
  IVAHD_IRQ1_IRQn               = 24,       /* Sync interrupt from ICONT1                         */
  I2C1_IRQ_IRQn                 = 25,       /* I2C1 interrupt                                     */
  I2C2_IRQ_IRQn                 = 26,       /* I2C2 interrupt                                     */
  I2C3_IRQ_IRQn                 = 27,       /* I2C3 interrupt                                     */
  I2C4_IRQ_IRQn                 = 28,       /* I2C4 interrupt                                     */
  UART3_IRQ_IRQn                = 29,       /* UART3 interrupt                                    */
  /* reserved */
  PRCM_M3_IRQ_IRQn              = 31,       /* PRCM interrupt                                     */
  /* reserved */
  /* reserved */
  MAIL_U2_M3_IRQ                = 34,       /* Mailbox user 2 interrupt                           */
  GPIO1_MPU_IRQ                 = 35,       /* GPIO1 MPU interrupt                                */
  GPIO2_MPU_IRQ                 = 36,       /* GPIO2 MPU interrupt                                */
  GPT3_IRQ                      = 37,       /* GPTIMER3 interrupt                                 */
  GPT4_IRQ                      = 38,       /* GPTIMER4 interrupt                                 */
  GPT9_IRQ                      = 39,       /* GPTIMER9 interrupt                                 */
  GPT11_IRQ                     = 40,       /* GPTIMER11 interrupt                                */
  MCSPI1_IRQ                    = 41,       /* MCSPI1 interrupt                                   */
  MCSPI2_IRQ                    = 42,       /* MCSPI2 interrupt                                   */
  /* reserved */
  /* reserved */
  /* reserved */
  /* reserved */
  /* reserved */
  DMM_IRQ_IRQn                  = 48,       /* DMM interrupt                                      */
  /* reserved */
  MMC1_IRQ_IRQn                 = 50,       /* MMC1 interrupt                                     */
  MMC2_IRQ_IRQn                 = 51,       /* MMC2 interrupt                                     */
  MMC3_IRQ_IRQn                 = 52,       /* MMC3 interrupt                                     */
  MMC4_IRQ_IRQn                 = 53,       /* MMC4 interrupt                                     */
  MMC5_IRQ_IRQn                 = 54,       /* MMC5 interrupt                                     */
  /* reserved */
  FSUSB_SMI_IRQ_IRQn            = 56,       /* FS-USB - host controller SMI interrupt             */
  HSUSB_EHCI_IRQ_IRQn           = 57,       /* HSUSB MP host interrupt EHCI controller            */
  HSUSB_TLL_IRQ_IRQn            = 58,       /* HSUSB MP TLL interrupt                             */
  FSUSB_IRQ_IRQn                = 59,       /* FS-USB - host controller Interrupt                 */
  HSUSB_OTG_IRQ_IRQn            = 60,       /* HSUSB OTG controller interrupt                     */
  HSUSB_OTG_DMA_IRQ_IRQn        = 61,       /* HSUSBOTG HSUSB OTG DMA interrupt                   */
  HSI_P1_MPU_IRQ_IRQn           = 62,       /* HSI Port 1 interrupt                               */
  HSI_P2_MPU_IRQ_IRQn           = 63,       /* HSI Port 2 interrupt                               */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          4         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */


#include "core_cm3.h"                       /* Cortex-M3 processor and core peripherals           */

#endif  // __OMAP_CM3_H__
