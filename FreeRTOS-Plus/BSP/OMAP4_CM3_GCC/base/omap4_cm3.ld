ENTRY(vResetHandler)

MEMORY
{
    TEXT (rwx) : ORIGIN = 0x00000000, LENGTH = 256M
	DATA (rwx) : ORIGIN = 0x80000000, LENGTH = 256M
}

SECTIONS
{
	.text : 
	{ 
		KEEP(*(.isr_vector));
		. = ALIGN(1024);
		KEEP(*(.isr_vector0));
		. = ALIGN(1024);
		KEEP(*(.isr_vector1));
		* (.text*);
		* (.rodata*);
		* (.init*);
		* (.fini*);
	} > TEXT

	.data :
	{
		* (.data);
	} > DATA

	.bss :
	{
		. = ALIGN(4);
		_sbss = .;
		* (.bss);
		. = ALIGN(4);
		_ebss = .;
	} > DATA

	.stack :
	{
		. = ALIGN(8);
		* (.stack0);
		. = ALIGN(8);
		* (.stack1);
	}

	.resource_size :
	{
		KEEP(*(.resource_size));
	} > DATA

	.resource_table :
	{
		KEEP(*(.resource_table));
	} > DATA
}
