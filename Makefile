# Makefile - build script */
APPNAME=usbtest

INCDIR=./include
SRCDIR=./source
LDDIR=./ldscripts
 
CUBEDIR=../STM32Cube_FW_F4_V1.1.0

# build environment
PREFIX ?= 
ARMGNU ?= $(PREFIX)arm-none-eabi

FLOAT_TYPE=hard
 
# source files
SOURCES_ASM += startup_stm32f401xc.s
SOURCES_C   += $(CUBEDIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
SOURCES_C   += $(CUBEDIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
SOURCES_C   += $(CUBEDIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
SOURCES_C   += $(wildcard $(SRCDIR)/*.c)

SOURCES_LD  := $(wildcard $(LDDIR)/*.ld)
 
# object files
LDS	+= $(patsubst %.ld,-T%.ld,$(SOURCES_LD))
OBJS	+= $(patsubst %.c,%.o,$(SOURCES_C))
OBJS	+= $(patsubst %.s,%.o,$(SOURCES_ASM))
    	     
# Build flags
DEPENDFLAGS := -MD -MP

INCLUDES    += -I$(INCDIR)
INCLUDES    += -I$(CUBEDIR)/Drivers/STM32F4xx_HAL_Driver/Inc
INCLUDES    += -I$(CUBEDIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include
INCLUDES    += -I$(CUBEDIR)/Drivers/CMSIS/Include

BASEFLAGS = -mcpu=cortex-m4 -mthumb -mthumb-interwork -O0 -Wall  -g3 
COMPFLAGS = -DSTM32F401xC -DHSE_VALUE=8000000 -DDEBUG
LDFLAGS   += $(LDS) -Xlinker --gc-sections -L"$(LDDIR)"
#-nostartfiles -nostdlib 
ifeq ($(FLOAT_TYPE), hard)
BASEFLAGS += -fsingle-precision-constant -Wdouble-promotion
BASEFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
else
BASEFLAGS += -mfloat-abi=soft
endif

WARNFLAGS   := -Wall -O0 -Wall -g3

ASFLAGS     := $(INCLUDES) $(DEPENDFLAGS) -D__ASSEMBLY__
CFLAGS      := $(COMPFLAGS) $(INCLUDES) $(DEPENDFLAGS) $(BASEFLAGS) $(WARNFLAGS)
CFLAGS      += -std=c11

# build rules
all: $(APPNAME).bin post-build
 
include $(wildcard *.d)
 
$(APPNAME).elf: $(OBJS) $(SOURCES_LD)
	$(ARMGNU)-gcc $(BASEFLAGS) $(LDFLAGS) $(OBJS) -o $@

$(APPNAME).bin: $(APPNAME).elf
	$(ARMGNU)-objcopy -O binary $(APPNAME).elf $(APPNAME).bin

post-build: $(APPNAME).bin
	$(ARMGNU)-size --format=berkeley $(APPNAME).elf

clean:
	$(RM) -f $(OBJS) $(APPNAME).elf $(APPNAME).bin
	$(RM) -f $(patsubst %.c,%.d,$(SOURCES_C))

# C.
%.o: %.c
	$(ARMGNU)-gcc $(CFLAGS) -c $< -o $@
 
# AS.
%.o: %.s
	$(ARMGNU)-gcc $(ASFLAGS) -c $< -o $@

