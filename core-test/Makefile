# Tailored to STMF103C8

# ------------------------------

ACM_DEV=/dev/ttyACM0

STM32CORE_DIR = ../
LINKER_DIR = $(STM32CORE_DIR)/linker
OPENCM3_DIR = $(STM32CORE_DIR)/libopencm3
CORE_DIR = $(STM32CORE_DIR)/core

# ------------------------------

CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy

LDSCRIPT = $(LINKER_DIR)/stm32f103x8-dfu.ld
OPENCM3_LIB = $(OPENCM3_DIR)/lib/libopencm3_stm32f1.a

CXXFLAGS += -std=gnu++17 -g0 -Os
CXXFLAGS += -Wall -Wno-register -Wno-pointer-arith
CXXFLAGS += -fconcepts -fno-exceptions -fno-rtti
CXXFLAGS += -fno-common -ffunction-sections -fdata-sections
CXXFLAGS += -mcpu=cortex-m3 -mthumb -msoft-float
CXXFLAGS += -I $(OPENCM3_DIR)/include -L $(OPENCM3_DIR)/lib

LDFLAGS += --static -nostartfiles -specs=nosys.specs -Wl,--gc-sections
LDFLAGS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
LDFLAGS += -l $(patsubst lib%.a,%,$(notdir $(OPENCM3_LIB)))
LDFLAGS += -T $(LDSCRIPT)

CPPFLAGS = -DSTM32F1 -DRCC_LED1=RCC_GPIOC -DPORT_LED1=GPIOC -DPIN_LED1=GPIO13

# ------------------------------

SOURCES = $(shell find $(CORE_DIR) -name "*.cpp")
HEADERS = $(shell find $(CORE_DIR) -name "*.h")

CXXFLAGS += -I $(CORE_DIR)/..

# ------------------------------

core-test: core-test.bin
	@

%.bin: %.elf Makefile
	$(OBJCOPY) -Obinary $< $@

%.elf: %.cpp $(SOURCES) $(HEADERS) $(OPENCM3_LIB) Makefile
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ $(filter %.cpp,$^) $(LDFLAGS)

flash: core-test.bin ENTER_DFU Makefile
	dfu-util -a 0 -d 0483:df11 -s 0x08002000:leave -D $<
	
clean:
	rm -f *.elf *.bin

.PHONY: ENTER_DFU
.ONESHELL:
ENTER_DFU:
	@echo "ENTER_DFU -> $(ACM_DEV)"
	@s=`perl -e 'use POSIX; tcsendbreak(3, 0)' 2>&1 3>$(ACM_DEV)`
	
	@if grep -q "Device or resource busy" <<< "$$s"; then
		@echo "ENTER_DFU: $(ACM_DEV): Device or resource busy"
		@exit 1
	@fi
	
	@sleep 1
