# Tailored to STMF103C8
# ------------------------------

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

LDSCRIPT = ../libopencm3/lib/stm32/f1/stm32f103x8.ld
OPENCM3_LIB = ../libopencm3/lib/libopencm3_stm32f1.a

CFLAGS += -Os -std=gnu99 -ggdb3
CFLAGS += -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd
CFLAGS += -Wextra -Wshadow -Wimplicit-function-declaration -Wredundant-decls
CFLAGS += -Wmissing-prototypes -Wstrict-prototypes -Wall -Wundef 
CFLAGS += -fno-common -ffunction-sections -fdata-sections
CFLAGS += -I ../libopencm3/include -DSTM32F1 -MD

CLDFLAGS += --static -nostartfiles -T $(LDSCRIPT) -ggdb3
CLDFLAGS += -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd
CLDFLAGS += -Wl,-Map=usbdfu.map -Wl,--cref -Wl,--gc-sections
CLDFLAGS += -L ../libopencm3/lib

LDFLAGS += -l $(patsubst lib%.a,%,$(notdir $(OPENCM3_LIB)))
LDFLAGS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

# ------------------------------

usbdfu: usbdfu.bin
	@

%.bin: %.elf Makefile
	$(OBJCOPY) -Obinary $< $@

%.o: %.c $(OPENCM3_LIB) Makefile
	$(CC) -c $(CFLAGS) -o $@ $(filter %.c,$^)

%.elf: %.o $(OPENCM3_LIB) Makefile
	$(CC) $(CLDFLAGS) -o $@ $(filter %.o,$^) $(LDFLAGS)

flash: usbdfu.bin Makefile
	st-flash write $< 0x8000000

clean:
	rm -f usbdfu.bin usbdfu.d usbdfu.map
