# Reference: https://makefiletutorial.com

CC = ../../toolchain/mac/gcc-arm-none-eabi-10-2020-q4-major/bin/./arm-none-eabi-gcc
OBJCOPY = ../../toolchain/mac/gcc-arm-none-eabi-10-2020-q4-major/bin/./arm-none-eabi-objcopy
OBJDUMP = ../../toolchain/mac/gcc-arm-none-eabi-10-2020-q4-major/bin/./arm-none-eabi-objdump
SIZE = ../../toolchain/mac/gcc-arm-none-eabi-10-2020-q4-major/bin/./arm-none-eabi-size
BUILD_DIR := ./build
FINAL_BINARY := final_binary.bin
FINAL_ELF := final_elf.elf
MARCH = cortex-m4
LINKER_FILE := l0_lowlevel/linker/stm32f411ceux_linker.ld
OPTIMIZATION = 2
CFLAGS = -c -mcpu=$(MARCH) -mthumb -mfloat-abi=softfp -std=gnu11 -Wall -O$(OPTIMIZATION)
LDFLAGS = -mcpu=$(MARCH) -mthumb -mfloat-abi=softfp --specs=nano.specs -T $(LINKER_FILE) -Wl,-Map=${BUILD_DIR}/final.map

SRC_DIRS := ./
INC_DIRS := $(shell find $(SRC_DIRS) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))
# Preprocessor Control Flags. Generate intermediate Makefiles
# Reference: https://gcc.gnu.org/onlinedocs/gcc/Preprocessor-Options.html#Preprocessor-Options
PREP_CON_FLAGS := $(INC_FLAGS) -MMD -MP

# Find names of C source files
SRCS := $(shell find $(SRC_DIRS) -name *.c)
# Geneate .o names for the C source files found, e.g., main.c => ./build/main.o
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
# Generate dependencies file names (suffixed with .d)
# Reference as to why: https://stackoverflow.com/questions/19114410/what-is-d-file-after-building-with-make
DEPS := $(OBJS:.o=.d)

# Build the final .bin file that is flashable to the mcu
$(BUILD_DIR)/$(FINAL_BINARY): ${BUILD_DIR}/$(FINAL_ELF)
	$(OBJCOPY) -O binary ${BUILD_DIR}/$(FINAL_ELF) ${BUILD_DIR}/$(FINAL_BINARY)

# Build the .elf file
$(BUILD_DIR)/$(FINAL_ELF): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# Build C source files
$(BUILD_DIR)/%.c.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(PREP_CON_FLAGS) $(CFLAGS) -c $< -o $@

dump:
	$(OBJDUMP) -h ${BUILD_DIR}/$(FINAL_ELF)

flash: ${BUILD_DIR}/$(FINAL_BINARY)
	../../st-programmer/stm32flash/./stm32flash -w ${BUILD_DIR}/$(FINAL_BINARY) -b 115200 -v -g 0x0 /dev/tty.SLAB_USBtoUART

.PHONY: size
size:
	$(SIZE) --format=berkeley ${BUILD_DIR}/$(FINAL_ELF)

.PHONY: clean
clean:
	rm -r $(BUILD_DIR)

-include $(DEPS)