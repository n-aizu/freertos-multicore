#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/* MPU_M3 sub-system reset register. */
/* (Because RM_MPU_M3_RSTCTRL is a virtual address, so it is defined in bsp_config.h) */
#define RM_MPU_M3_RSTCTRL (volatile unsigned long *)( 0xaa306910 )

#endif /* BSP_CONFIG_H */

