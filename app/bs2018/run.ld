__main_stack_size__     = 0x0800;
__process_stack_size__  = 0x2000;

MEMORY
{
    flash		: org = 0x08020000, len = 384k
    flash_itc	: org = 0x00220000, len = 384k
    itcram		: org = 0x00000000, len = 16k
    dtcram		: org = 0x20000000, len = 127k
	BOOT		: org = 0x2001FC00, len = 1k
    sram1		: org = 0x20020000, len = 368k
    sram2		: org = 0x2007C000, len = 16k
}

__ram_start__           = ORIGIN(sram1);
__ram_size__            = LENGTH(sram1);
__ram_end__             = __ram_start__ + __ram_size__;

ENTRY(Reset_Handler)

SECTIONS
{
    . = 0;
    _text = .;

	/* vectors MUST go first */	
	startup : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.vectors))
    } > flash_itc AT > flash


	/* then comes all the babbling stuff */
    constructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > flash_itc AT > flash

    destructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end = .);
    } > flash_itc AT > flash
    
	/* And now comes the main program code */
    .text : ALIGN(4)
    {
        *(.text.startup.*)
        *(.text)
        *(.text.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > flash_itc AT > flash
    
    /* Uninitialized variables in Data TCM */
    .dtc_bss :
    {
        . = ALIGN(8);
        PROVIDE(_dtc_bss_start = .);
        
        /* STACKS */
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
        
        /* OS variables and thread stacks */
        . = ALIGN(4);
        *(.bss.ch)
        *(.bss.endmem)
        *(.bss.nextmem)
        *(.bss.default_heap)
        . = ALIGN(4);
        
		*(.bss.__dtc_*)
        . = ALIGN(4);
        
        PROVIDE(_dtc_bss_end = .);
    } > dtcram
    
    /* Initialized variables for data TCM */
    .dtc_data :
    {
    	. = ALIGN(4);
    	PROVIDE(_dtc_data_start = .);
    	*(.dtc*)
    	*(.dtc.*)
    	*(.data.__dtc_*)
    	. = ALIGN(4);
    	PROVIDE(_dtc_data_end = .);
    } > dtcram AT > flash
    
    .itc_text :
    {
   		. = ALIGN(4);
    	PROVIDE(_itc_text_start = .);
    	*(.itc*)
    	*(.itc.*)
    	. = ALIGN(4);
    	PROVIDE(_itc_text_end = .);
    } > itcram AT > flash
    
    
    /* Initialized variables go to SRAM1, values stored in flash */
    .data :
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        *(.data.*)
        *(.ramtext*)
        *(.ramtext.*)
        *(.rodata.__ram_*)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > sram1 AT > flash

    .rodata : ALIGN(4)
    {
		. = ALIGN(4);
		PROVIDE(__rodata_base__ = .);
		*(.rodata)
		*(.rodata.*)
		KEEP(*(.rodata.version))
		. = ALIGN(4);
		PROVIDE(__rodata_end__ = .);
    } > flash
    
	/* Unitialized variables that will be zeroed */
    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
		PROVIDE(end = .);
		_end = .;
    } > sram1    
    
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash_itc AT > flash

    .ARM.exidx :
    {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash_itc AT > flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash_itc AT > flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash_itc AT > flash
    
    _dtc_data_load = LOADADDR(.dtc_data);
    _itc_text_load = LOADADDR(.itc_text);
	_textdata = LOADADDR(.data);
	
	.sram2 (NOLOAD) :
	{
		*(.sram2)
		*(.sram2*)
	} > sram2

	.boot_bss (NOLOAD) :
	{
		*(.boot)
		*(.boot*)
	} > BOOT
}

__heap_base__   = _end;
__heap_end__    = __ram_end__;
