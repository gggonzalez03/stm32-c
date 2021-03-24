CC = ../../toolchain/mac/gcc-arm-none-eabi-10-2020-q4-major/bin/./arm-none-eabi-gcc
MARCH = cortex-m4
BUILD = ./build
CFLAGS = -c -mcpu=$(MARCH) -mthumb -std=gnu11 -Wall -O0
INC = -I./l0_lowlevel/arm-software/cmsis/core/ -I./
LDFLAGS = -mcpu=$(MARCH) -mthumb -mfloat-abi=softfp --specs=nano.specs -T l0_lowlevel/linker/stm32f411ceux_linker.ld -Wl,-Map=${BUILD}/final.map

${BUILD}/all: ${BUILD}/main.o ${BUILD}/stm32f411ceux_startup.o ${BUILD}/system_calls.o ${BUILD}/final.elf ${BUILD}/final.bin

${BUILD}/main.o: l5_application/main.c
	$(CC) $(INC) $(CFLAGS) -o $@ $^

${BUILD}/stm32f411ceux_startup.o: l0_lowlevel/startup/stm32f411ceux_startup.c
	$(CC) $(INC) $(CFLAGS) -o $@ $^

${BUILD}/system_calls.o:	l4_io/newlib-nano/system_calls.c
	$(CC) $(INC) $(CFLAGS) -o $@ $^

${BUILD}/final.elf: ${BUILD}/main.o ${BUILD}/stm32f411ceux_startup.o ${BUILD}/system_calls.o
	$(CC) $(LDFLAGS) -o $@ $^

${BUILD}/final.bin: ${BUILD}/final.elf
	../../toolchain/mac/gcc-arm-none-eabi-10-2020-q4-major/bin/./arm-none-eabi-objcopy -O binary ${BUILD}/final.elf ${BUILD}/final.bin

clean:
	rm -rf ${BUILD}/*.o ${BUILD}/*.elf ${BUILD}/*.bin ${BUILD}/*.map

flash: ${BUILD}/final.bin
	../../st-programmer/stm32flash/./stm32flash -w ${BUILD}/final.bin -v -g 0x0 /dev/tty.SLAB_USBtoUART

$(shell   mkdir -p $(BUILD))