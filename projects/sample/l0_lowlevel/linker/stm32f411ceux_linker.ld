ENTRY(Reset_Handler)

MEMORY
{
  flash_512k (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  sram_128k (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

__base_flash = ORIGIN(flash_512k);
__top_flash = __base_flash + LENGTH(flash_512k);

__base_sram_128k = ORIGIN(sram_128k);
__top_sram_128k = __base_sram_128k + LENGTH(sram_128k);

__base_stack = __top_sram_128k;

__heap_size = 32k;

SECTIONS
{
  .text : ALIGN(4)
  {
    *(.interrupt_vector_table)
    *(.text*)
    *(.init)
    *(.fini)
    *(.rodata*)
    . = ALIGN(4);
  } > flash_512k

  __text_end__ = LOADADDR(.data);
  .data : ALIGN(4)
  {
    __data_start__ = .;
    *(.data*)
    . = ALIGN(4);
    __data_end__ = .;
  } > sram_128k AT> flash_512k

  .bss  : ALIGN(4)
  {
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    . = ALIGN(4);
    __end__ = .;
    __heap_start__ = __end__;
    __heap_end__ = __end__ + __heap_size;
  } > sram_128k
}