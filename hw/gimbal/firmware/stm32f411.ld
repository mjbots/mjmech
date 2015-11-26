/*
    GNU linker script for STM32F411
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 0x080000 /* entire flash, 512 KiB */
    FLASH_ISR (rx)  : ORIGIN = 0x08000000, LENGTH = 0x004000 /* sector 0, 16 KiB */
    FLASH_TEXT (rx) : ORIGIN = 0x08020000, LENGTH = 0x060000 /* sectors 5,6,7 4*128KiB = 384 KiB */
    RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 0x020000 /* 128 KiB */
}

/* produce a link error if there is not this amount of RAM for these sections */
_minimum_stack_size = 2K;
_minimum_heap_size = 16K;

/* top end of the stack */

/*_stack_end = ORIGIN(RAM) + LENGTH(RAM);*/
_estack = ORIGIN(RAM) + LENGTH(RAM) - 1;

/* RAM extents for the garbage collector */
_ram_end = ORIGIN(RAM) + LENGTH(RAM);
_heap_end = 0x2001c000; /* tunable */

/* define output sections */
SECTIONS
{
    /* The startup code goes first into FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */

        . = ALIGN(4);
    } >FLASH_ISR

    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        *(.glue_7)   */    /* glue arm to thumb code */
        *(.glue_7t)  */    /* glue thumb to arm code */

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;        /* define a global symbol at end of code */
    } >FLASH_TEXT

    /*
     * Stack unwinding and exception handling sections.
     *
     * ARM compilers emit object files with .ARM.extab and .ARM.exidx sections
     * when using C++ exceptions. Also, at least GCC emits those sections when
     * dividing large numbers (64-bit) in C. So we have to handle them.
     *
     * (ARM uses .ARM.extab and .ARM.exidx instead of the .eh_frame section
     * used on x86.)
     */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH_TEXT

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH_TEXT

    /*
     * Newlib and Eglibc (at least) need these for C++ support.
     *
     * (Copied from Sourcery CodeBench Lite: arm-none-eabi-gcc -V)
     */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH_TEXT
    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH_TEXT
    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH_TEXT

    /* used by the startup to initialize data */
    _sidata = LOADADDR(.data);

    /* This is the initialized data section
    The program executes knowing that the data is in the RAM
    but the loader puts the initial values in the FLASH (inidata).
    It is one task of the startup to copy the initial values from FLASH to RAM. */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;        /* create a global symbol at data start; used by startup code in order to initialise the .data section in RAM */
        _ram_start = .;    /* create a global symbol at ram start for garbage collector */
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */

        . = ALIGN(4);
        _edata = .;        /* define a global symbol at data end; used by startup code in order to initialise the .data section in RAM */
    } >RAM AT> FLASH_TEXT

    /* Uninitialized data section */
    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        _sbss = .;         /* define a global symbol at bss start; used by startup code */
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end; used by startup code and GC */
        __bss_end__ = .;
    } >RAM

    /* this is to define the start of the heap, and make sure we have a minimum size */
    .heap :
    {
        . = ALIGN(4);
	PROVIDE ( end = . );
	PROVIDE ( _end = . );
        _heap_start = .;    /* define a global symbol at heap start */
        . = . + _minimum_heap_size;
    } >RAM

    /* this just checks there is enough RAM for the stack */
    .stack :
    {
        . = ALIGN(4);
        . = . + _minimum_stack_size;
        . = ALIGN(4);
    } >RAM

    /* Remove information from the standard libraries */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
