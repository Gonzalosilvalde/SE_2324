# Definición de variables
PREFIX=arm-none-eabi-
ARCHFLAGS=-mthumb -mcpu=cortex-m0plus

COMMONFLAGS=-g3 -Og -Wall -Werror $(ARCHFLAGS)

CFLAGS= -D CPU_MKL46Z128VLH4 -I./drivers/$(TARGET) -I./includes/ $(COMMONFLAGS)

LDFLAGS=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET).map,-Tlink.ld

TARGET = 

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size
RM=rm -f

# Definición de los fuentes y los objetos
SRC_COMMON = startup.c
SRC_HEL = $(wildcard drivers/hello_world/*.c) hello_world.c $(SRC_COMMON)
SRC_LED = $(wildcard drivers/led_blinky/*.c) led_blinky.c $(SRC_COMMON)
OBJ_hello_world = $(patsubst %.c, %.o, $(SRC_HEL))
OBJ_led_blinky = $(patsubst %.c, %.o, $(SRC_LED))

# Reglas de construcción
all: build_hel build_led size

build_hel: $(OBJ_hello_world)
	$(LD) $(LDFLAGS) -o $(TARGET).elf $(OBJ_hello_world)
	$(OBJCOPY) -O srec $(TARGET).elf $(TARGET).srec
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

build_led: $(OBJ_led_blinky)
	$(LD) $(LDFLAGS) -o $(TARGET).elf $(OBJ_led_blinky)
	$(OBJCOPY) -O srec $(TARGET).elf $(TARGET).srec
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

size:
	$(SIZE) *.elf

clean:
	$(RM) *.srec *.elf *.bin *.map $(OBJ_hello_world) $(OBJ_led_blinky)

%.o: %.c
	$(CC) -c $(ARCHFLAGS) $(CFLAGS) -o $@ $<

flash_hello: TARGET = hello_world
flash_hello: clean build_hel
	openocd -f openocd.cfg -c "program $(TARGET).elf verify reset exit"

flash_led: TARGET = led_blinky
flash_led: clean build_led
	openocd -f openocd.cfg -c "program $(TARGET).elf verify reset exit"
