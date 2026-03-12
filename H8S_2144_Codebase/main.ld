OUTPUT_ARCH(h8300s)

MEMORY
{
    rom (rx) : o = 0x00000000, l = 128k
    ram (rw) : o = 0x00FFE080, l = 128k
}

SECTIONS
{
	.vects 0x00000000 :
	{
	} > rom
	.text  :
	{
		*(.text)
		etext = .;
	} > rom
	.init :
	{
		*(.init)
	} > rom
	.fini :
	{
		*(.fini)
	}
	.got :
	{
		*(.got)
		*(.got.plt)
	} > rom
	.rodata :
	{
		*(.rodata)
		*(.rodata.*)
		_erodata = .;
	} > rom
	.eh_frame_hdr :
	{
		*(.eh_frame_hdr)
	} > rom
	.eh_frame :
	{
		*(.eh_frame)
	} > rom
	.jcr :
	{
		*(.jcr)
	} > rom
	.tors :
	{
		__CTOR_LIST__ = .;
		___ctors = .;
		*(.ctors)
		___ctors_end = .;
		__CTOR_END__ = .;
		__DTOR_LIST__ = .;
		___dtors = .;
		*(.dtors)
		___dtors_end = .;
		__DTOR_END__ = .;
	} > rom

	/* .data initializers stored here in ROM, right after .tors.
	 * start.S copies from _mdata (ROM) to _data.._edata (RAM).
	 * MUST come BEFORE .fill so the fill pattern doesn't clobber it.
	 * Removed (NOLOAD) — that was preventing the linker from
	 * emitting the initialized data into the ROM image. */
	_mdata = .;

	.data 0x00FFE080 : AT (_mdata)
	{
		_data = .;
		*(.data)
		*(.data.*)
		_edata = .;
	} > ram

	/* Fill unused ROM between end of .data initializers and the
	 * asm_code section. _mdata + sizeof(.data) = end of init data in ROM. */
	.fill (_mdata + SIZEOF(.data)) :
	{
		FILL(0xDEADBEEF);
		. = ORIGIN(rom) + LENGTH(rom) - 4;
		BYTE(0xAA)
		___ROM_AT = .;
	} > rom
	.asm_code 0x1fffd :
    {
        _asm_start = .;
        *(.asm_code)
        *(.my_asm)  /* Alternative names */
        _asm_end = .;
    } > rom
	.gcc_exc :
	{
		*(.gcc_exc)
	} > ram
	.bss :
	{
		_bss = .;
		*(.bss)
		*(COMMON)
		_ebss = .;
		_end = .;
	} > ram
	/* .noinit: RAM that is NOT zeroed by start.S.
	 * Used for mon_magic (Trace32 monitor soft-reset re-entry flag).
	 * On cold boot, contents are random — code must use a magic value. */
	.noinit :
	{
		_noinit = .;
		*(.noinit)
		*(.noinit.*)
		_enoinit = .;
	} > ram
	.stack 0x00FFEFFC :
	{
		_stack = .;
	} > ram
	
    _end_of_rom = .;
}
