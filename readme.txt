-----------
DESCRIPTION
-----------

This software is port of FreeRTOS Multicore for the Cortex-M3 processors on the Pandaboard ES.
(Maybe run on OMAP4430 Pandaboard, but tested only on OMAP4460 Pandaboard ES)

This software is based on:

+ FreeRTOS V7.2.0
 - and backport the optimised task selection mechanism from FreeRTOS V7.3.0
 - http://www.freertos.org/

+ Pandaboard-FreeRTOS
 - https://github.com/apopple/Pandaboard-FreeRTOS

+ FreeRTOS Multicore
 - http://freertosxcore.svn.sourceforge.net/

+ sysbios-rpmsg
 - http://git.omapzoom.org/?p=repo/sysbios-rpmsg.git;a=summary


This software is run on Linux remoteproc framework only.

This software is tested with:
+ Linux kernel
 - Linaro TI Landing Team kernel
  - http://git.linaro.org/gitweb?p=landing-teams/working/ti/kernel.git;a=summary
  - branch:tilt-3.4/commit:edb2dff2ab14d66f123d43a18b2b27cecf598cdf
 
+ Linux userland
 - Buildroot 2012.11.1
  - http://buildroot.uclibc.org/downloads/buildroot-2012.11.1.tar.bz2

+ Cross toolchain(to build FreeRTOS) 
 - Code Sourcery GCC ver4.5
 - Launchpad GCC ver4.6 and 4.7
  - https://launchpad.net/gcc-arm-embedded/4.6/4.6-2012-q4-update/+download/gcc-arm-none-eabi-4_6-2012q4-20121016.tar.bz2
  - https://launchpad.net/gcc-arm-embedded/4.7/4.7-2012-q4-major/+download/gcc-arm-none-eabi-4_7-2012q4-20121208-linux.tar.bz2

This software is port of FreeRTOS, but it has limited API subsets of original FreeRTOS.
See below for implemented APIs.

--------
FEATURES
--------

Implemented APIs(in comparison with original FreeRTOS V7.2.0):

[Task Creation]
+ xTaskCreate
 - cpu core argument is added.

[Task Control]
+ vTaskDelay
+ vTaskDelayUntil

[Task Utilities]
+ xTaskGetCurrentTaskHandle
+ uxTaskGetStackHighWaterMark
+ pcTaskGetTaskName
+ xTaskGetTickCount
+ uxTaskGetNumberOfTasks
+ vTaskSetApplicationTaskTag
+ xTaskGetApplicationTaskTag
+ xTaskCallApplicationTaskHook

[Kernel Control]
+ taskYIELD
+ taskENTER_CRITICAL
 - extended with using spinlock
+ taskEXIT_CRITICAL
+ taskDISABLE_INTERRUPTS
+ taskENABLE_INTERRUPTS
+ vTaskStartScheduler

[Queue Management]
+ uxQueueMessagesWaiting
+ xQueueCreate
+ xQueueReset
+ xQueueSend
+ xQueueSendToBack
+ xQueueSendToBack
+ xQueueSendToFront
+ xQueueReceive
+ xQueuePeek
+ xQueueSendFromISR
+ xQueueSendToBackFromISR
+ xQueueSendToFrontFromISR
+ xQueueReceiveFromISR

[Semaphores]
+ vSemaphoreCreateBinary
+ xSemaphoreCreateCounting
+ xSemaphoreTake
+ xSemaphoreTakeFromISR
+ xSemaphoreGive
+ xSemaphoreGiveFromISR 

[Others]
+ portSET_INTERRUPT_MASK_FROM_ISR
+ portCLEAR_INTERRUPT_MASK_FROM_ISR
+ pvPortMalloc
+ vPortFree


Newly implemented APIs(included port APIs):

[Task Control]
+ uxTaskPriorityGetCurrent
+ uxTaskPrioritySetCurrent
 - get/set priority of current task
 - designed to implement priority ceiling mutexes(with binary semaphore)

[Kernel Control]
+ taskENTER_CRITICAL_NOT_RECURSIVE_FROM_ISR
+ taskEXIT_CRITICAL_NOT_RECURSIVE_FROM_ISR
 - the start of a critical code region without disabling interrupts(for optimisation)
 - not allowed to be called recursively
 - allowed to be called within only ISR
+ portINTERRUPT_CORE
 - interrupt other cpu core to switch context

[Others]
+ portSET_INTERRUPT_MASK_AND_RETURN
+ portCLEAR_INTERRUPT_MASK_AND_SET
 - same as portSET_INTERRUPT_MASK_FROM_ISR/portCLEAR_INTERRUPT_MASK_FROM_ISR
 - allowed to be called from out of ISR
+ portLOCK_INIT
 - initialization of lock object for taskENTER_CRITICAL
+ portGetCurrentCPU
 - get current cpu core number
 - cpu core number starts at 0
+ vPortSetIrqHndl
 - set ISR to the vector table
 - vector table is independent every cpu core

Please refer to multicore_features.txt for FreeRTOS Multicore specific features,
and refer to rpmsg.txt for rpmsg APIs.
(Sorry, multicore_features.txt and rpmsg.txt are not written yet)

-----------
DIRECTORIES
-----------

Directories:

+ FreeRTOS/Source contains the FreeRTOS Multicore real time kernel source code.

+ FreeRTOS/Demo contains a pre-configured demo project.
  
+ FreeRTOS-Plus contains additional FreeRTOS components.
  (mainly, files that ported from sysbios-rpmsg)
  THESE ARE LICENSED SEPARATELY FROM FreeRTOS although all contain open source options. 
  See the license files in each respective directory for information.

-------------
SETUP EXAMPLE
-------------

[Preparation]
1. Checkout FreeRTOS Multicore for Pandaboard sources
ex)
$ mkdir ~/work/repo
$ cd ~/work/repo
$ git clone git://github.com/n-aizu/freertos-multicore.git

[SD card]
1. Insert SD card to Linux PC

2. Make file system
ex)
$ cd ~/work/repo/freertos-multicore/FreeRTOS/Demo/CORTEX_M3_PANDA/misc
$ sudo LANG=C sh mksdcard_panda_linux.sh /dev/sdc
("/dev/sdc" is SD card location. Obviously, make sure you're using the
right value for your machine)

3. Mount SD card

[Boot loader]
1. Checkout source
ex)
$ git clone git://git.linaro.org/boot/u-boot-linaro-stable.git u-boot-linaro
$ git checkout -b 2013.02.2 2013.02.2

2. Build
ex)
$ make omap4_panda_config
(set $PATH for arm cross toolchain)
$ make CROSS_COMPILE=arm-none-linux-gnueabi- -j10

3. Copy files to SD card
ex)
$ cp MLO /media/boot/
(MLO must be copied first)
$ cp u-boot.bin /media/boot/
$ cp ~/work/repo/freertos-multicore/FreeRTOS/Demo/CORTEX_M3_PANDA/misc/preEnv.txt /media/boot/
("/media" is SD card mount path)

[Root file system]
1. Download tar package
ex)
$ wget http://buildroot.uclibc.org/downloads/buildroot-2012.11.1.tar.bz2
$ tar jxf buildroot-2012.11.1.tar.bz2
$ cd buildroot-2012.11.1/

2. Configure
ex)
$ cp ~/work/repo/freertos-multicore/FreeRTOS/Demo/CORTEX_M3_PANDA/misc/buildroot-2012.11.1.config .config
$ make oldconfig

3. Build
ex)
$ make

4. Copy files to SD card
ex)
$ sudo tar jxf output/images/rootfs.tar.bz2 --directory=/media/rootfs/
$ sudo chown -R root. /media/rootfs/
$ sudo chmod 777 -R /media/rootfs/

[Linux]
1. Checkout source
ex)
$ git clone git://git.linaro.org/landing-teams/working/ti/kernel.git kernel-tilt
$ cd kernel-tilt
$ git checkout -b tilt-3.4 remotes/origin/tilt-3.4
(commit:edb2dff2ab14d66f123d43a18b2b27cecf598cdf)

2. Build
ex)
$ cp ~/work/repo/freertos-multicore/FreeRTOS/Demo/CORTEX_M3_PANDA/misc/tilt-3.4.config .config
$ make ARCH=arm oldconfig
(set $PATH for arm cross toolchain)
$ make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- uImage modules -j10

3. Copy files to SD card
ex)
$ cp arch/arm/boot/uImage /media/boot/
$ sudo make ARCH=arm modules_install INSTALL_MOD_PATH=/media/rootfs/
$ sudo chmod 777 -R /media/rootfs/lib/modules

[FreeRTOS]
1. Move to demo source directory
ex)
$ cd ~/work/repo/freertos-multicore/FreeRTOS/Demo/CORTEX_M3_PANDA/simple

2. Build

* If you use OMAP4430 Pandaboard, add "-DOMAP4430_PANDA" to CFLAGS.
* (This option affects only ParTest.c, not affects kernel and rpmsg library)

ex)
(set $PATH for arm cross toolchain)
$ make CROSS_COMPILE=arm-none-eabi-

3. Copy files to SD card
ex)
$ sudo mkdir /media/rootfs/lib/firmware/
$ sudo cp ducati-m3-core0.xem3 /media/rootfs/lib/firmware/

-------
RUNNING
-------

After bootup, you should load kernel modules on Pandaboard.

ex)
# modprobe omap_remoteproc
# modprobe rpmsg_client_sample
(rpmsg_client_sample is necessary only when execute rpmsg-client-sample demo)

-------
WARNING
-------

This software is provided WITHOUT ANY WARRANTY.

