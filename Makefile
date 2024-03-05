# Definición de variables
PREFIX=arm-none-eabi-
ARCHFLAGS=-mthumb -mcpu=cortex-m0plus

COMMONFLAGS=-g3 -Og -Wall -Werror $(ARCHFLAGS)

CFLAGS= -D CPU_MKL46Z128VLH4 -I./drivers/$(TARGET) -I./includes/ $(COMMONFLAGS)

LDFLAGS=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET).map,-Tlink.ld

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size
RM=rm -f

# Definición de las fuentes y los objetos
SRC_COMMON = startup.c
SRC_HEL = $(wildcard drivers/hello_world/*.c) hello_world.c $(SRC_COMMON)
SRC_LED = $(wildcard drivers/led_blinky/*.c) led_blinky.c $(SRC_COMMON)
OBJ_HEL = $(patsubst %.c, %.o, $(SRC_HEL))
OBJ_LED = $(patsubst %.c, %.o, $(SRC_LED))

# Reglas de construcción
all: build_hel build_led size

build_hel: $(OBJ_HEL)
	$(LD) $(LDFLAGS) -o $(TARGET).elf $(OBJ_HEL)
	$(OBJCOPY) -O srec $(TARGET).elf $(TARGET).srec
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

build_led: $(OBJ_LED)
	$(LD) $(LDFLAGS) -o $(TARGET).elf $(OBJ_LED)
	$(OBJCOPY) -O srec $(TARGET).elf $(TARGET).srec
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

size:
	$(SIZE) *.elf

clean:
	$(RM) *.srec *.elf *.bin *.map $(OBJ_HEL) $(OBJ_LED)

%.o: %.c
	$(CC) -c $(ARCHFLAGS) $(CFLAGS) -o $@ $<

flash_hello: TARGET = hello_world
flash_hello: clean build_hel
	openocd -f openocd.cfg -c "program $(TARGET).elf verify reset exit"

flash_led: TARGET = led_blinky
flash_led: clean build_led
	openocd -f openocd.cfg -c "program $(TARGET).elf verify reset exit"
