FREERTOS_SOURCE_DIR = ../../../../FreeRTOS
FREERTOS_SOURCE_PLUS_DIR = ../../../../FreeRTOS-Plus
APPLICATION_SOURCE_DIR = ./
DEMO_COMMON_DIR = ../common
BUILD_DIR = ./Build
CROSS_COMPILE ?= arm-none-eabi-

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
IMAGE_LOADADDR=0x83000000

CFLAGS = -g -mcpu=cortex-m3 -mthumb \
		 -I$(APPLICATION_SOURCE_DIR) \
		 -I$(DEMO_COMMON_DIR) \
		 -I$(FREERTOS_SOURCE_DIR)/Source/include -I$(FREERTOS_SOURCE_DIR)/Source/portable/GCC/ARM_CM3_OMAP \
		 -I$(FREERTOS_SOURCE_PLUS_DIR)/BSP/OMAP4_CM3_GCC/base -I$(FREERTOS_SOURCE_PLUS_DIR)/BSP/OMAP4_CM3_GCC/util \
		 -I$(FREERTOS_SOURCE_PLUS_DIR)/BSP/OMAP4_CM3_GCC/middleware/rpmsg \
		 -DGCC_ARMCM_OMAP -O2
LDFLAGS = -L$(FREERTOS_SOURCE_PLUS_DIR)/BSP/OMAP4_CM3_GCC/base -Tomap4_cm3.ld
LIBGCC=$(shell $(CC) -mthumb -mcpu=cortex-m3 -print-libgcc-file-name)
LIBC=$(shell $(CC) -mthumb -mcpu=cortex-m3 -print-file-name=libc.a)

FREERTOS_SOURCE_DIRS = $(FREERTOS_SOURCE_DIR)/Source $(FREERTOS_SOURCE_DIR)/Source/portable/MemMang $(FREERTOS_SOURCE_DIR)/Source/portable/GCC/ARM_CM3_OMAP $(FREERTOS_SOURCE_PLUS_DIR)/BSP/OMAP4_CM3_GCC/base $(FREERTOS_SOURCE_PLUS_DIR)/BSP/OMAP4_CM3_GCC/middleware/rpmsg

VPATH = $(FREERTOS_SOURCE_DIRS):$(APPLICATION_SOURCE_DIR):$(DEMO_COMMON_DIR)
APPLICATION_OBJS = $(BUILD_DIR)/main.o $(BUILD_DIR)/ParTest.o

OBJS =	$(BUILD_DIR)/list.o \
	$(BUILD_DIR)/queue.o \
	$(BUILD_DIR)/tasks.o \
	$(BUILD_DIR)/port.o \
	$(BUILD_DIR)/startup.o \
	$(BUILD_DIR)/resource.o \
	$(BUILD_DIR)/heap_4.o \
	$(BUILD_DIR)/ammu.o \
	$(APPLICATION_OBJS)

all: ducati-m3-core0.xem3

-include $(OBJS:.o=.d)

ducati-m3-core0.xem3: $(OBJS) $(LIBGCC) $(LIBC) omap4_cm3.ld
	$(LD) $(LDFLAGS) $(OBJS) $(LIBGCC) $(LIBC) -o ducati-m3-core0.xem3

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)
	$(CC) -c -MD $(CFLAGS) -o $@ -Wp,-MD,$(BUILD_DIR)/$*.d $<
	sed -i -e 's|.*:|$(BUILD_DIR)/$*.o:|' $(BUILD_DIR)/$*.d

clean:
	rm -f $(BUILD_DIR)/*.d
	rm -f $(BUILD_DIR)/*.o
	rm -f ducati-m3-core0.xem3

.PHONY: clean
