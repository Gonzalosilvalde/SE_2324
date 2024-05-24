PREFIX=arm-none-eabi-
ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
COMMONFLAGS=-g3 -Og -Wall -Werror $(ARCHFLAGS)

# Rutas de inclusión de las cabeceras
INCLUDE_DIRS := include drivers BOARD utilities
INC_FLAGS := $(addprefix -I, $(INCLUDE_DIRS))

# Opciones de compilación
CFLAGS=-D CPU_MKL46Z128VLH4 $(INC_FLAGS) $(COMMONFLAGS)

# Opciones de enlazado
LDFLAGS=$(COMMONFLAGS) --specs=nosys.specs -Wl,--gc-sections,-Map,$(TARGET).map,-TMKL46Z256xxx4_flash.ld 
LDLIBS=

# Herramientas
CC=$(PREFIX)gcc
AS=$(PREFIX)gcc
LD=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size
RM=rm -f

TARGET=main

# Buscar archivos fuente en todos los directorios especificados
SRC=$(wildcard *.c $(foreach dir,$(INCLUDE_DIRS),$(dir)/*.c) startup_MKL46Z4.S)
OBJ=$(patsubst %.c,%.o,$(patsubst %.S,%.o,$(SRC)))

# Reglas
all: build size

build: elf srec bin

elf: $(TARGET).elf

srec: $(TARGET).srec

bin: $(TARGET).bin

clean:
	$(RM) $(TARGET).srec $(TARGET).elf $(TARGET).bin $(TARGET).map $(OBJ)

$(TARGET).elf: $(OBJ)
	$(LD) $(LDFLAGS) $(OBJ) $(LDLIBS) -o $@

%.srec: %.elf
	$(OBJCOPY) -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(AS) $(ARCHFLAGS) -c $< -o $@

size:
	$(SIZE) $(TARGET).elf

flash: all
	openocd -f openocd.cfg -c "init; reset init; program $(TARGET).elf verify reset exit"
