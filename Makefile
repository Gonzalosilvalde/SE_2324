#TOOLCHAIN=~/toolchain/gcc-arm-none-eabi-4_9-2014q4/bin
#PREFIX=$(TOOLCHAIN)/arm-none-eabi-
PREFIX=arm-none-eabi-
ARCHFLAGS=-mthumb -mcpu=cortex-m0plus

COMMONFLAGS=-g3 -Og -Wall -Werror $(ARCHFLAGS)


CFLAGS_LED= -D CPU_MKL46Z128VLH4 -I./drivers/led -I./includes/ $(COMMONFLAGS)
CFLAGS_HELLO = -D CPU_MKL46Z128VLH4 -I./drivers/hello -I./includes/ $(COMMONFLAGS)

LDFLAGS_LED=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET_LED).map,-Tlink.ld
LDFLAGS_HELLO=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET_HEL).map,-Tlink.ld


CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size
RM=rm -f

TARGET_HEL=hello_world
TARGET_LED=led_blinky
 

SRC_HEL:=$(wildcard drivers/hello/*.c $(TARGET_HEL).c startup.c)
OBJ_HEL=$(patsubst %.c, %.o, $(SRC_HEL))

SRC_LED=$(wildcard drivers/led/*.c $(TARGET_LED).c startup.c)
OBJ_LED=$(patsubst %.c, %.o, $(SRC_LED))

all: build_hel build_led size

build_hel: elf_hel srec_hel bin_hel
elf_hel: $(TARGET_HEL).elf
srec_hel: $(TARGET_HEL).srec
bin_hel: $(TARGET_HEL).bin

build_led: elf_led srec_led bin_led
elf_led: $(TARGET_LED).elf
srec_led: $(TARGET_LED).srec
bin_led: $(TARGET_LED).bin

clean_bin:
	$(RM) $(TARGET_HEL).srec $(TARGET_HEL).elf $(TARGET_HEL).bin $(TARGET_HEL).map $(TARGET_LED).srec $(TARGET_LED).elf $(TARGET_LED).bin $(TARGET_LED).map

clean_obj:
	$(RM) $(OBJ_HEL) $(OBJ_LED)

clean: clean_bin clean_obj


%.o: %.c
	$(CC) -c $(ARCHFLAGS) $(if $(filter $(SRC_LED),$<),$(CFLAGS_LED),$(CFLAGS_HELLO)) -o $@ $<


$(TARGET_HEL).elf: $(OBJ_HEL)
	$(LD) $(LDFLAGS_HELLO) -o $@ $(OBJ_HEL)

$(TARGET_LED).elf: $(OBJ_LED)
	$(LD) $(LDFLAGS_LED) -o $@ $(OBJ_LED)

%.srec: %.elf
	$(OBJCOPY) -O srec $< $@

%.bin: %.elf
	    $(OBJCOPY) -O binary $< $@

size:
	$(SIZE) $(TARGET_HEL).elf

flash_hello: clean build_hel 
	openocd -f openocd.cfg -c "program $(TARGET_HEL).elf verify reset exit"

flash_led: clean build_led
	openocd -f openocd.cfg -c "program $(TARGET_LED).elf verify reset exit"