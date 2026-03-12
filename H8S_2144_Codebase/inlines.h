/****************************************************************
KPIT Cummins Infosystems Ltd, Pune, India. - 10-Mar-2003.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************/
extern __inline__ void set_interrupt_mask (unsigned char mask) __attribute__ ((always_inline));
extern __inline__ void sleep (void) __attribute__ ((always_inline));
extern __inline__ void nop (void) __attribute__ ((always_inline));
extern __inline__ void set_ccr (unsigned char mask) __attribute__ ((always_inline));
extern __inline__ unsigned char get_ccr (void) __attribute__ ((always_inline));
extern __inline__ void *get_sp (void) __attribute__ ((always_inline));


extern __inline__ void set_interrupt_mask (unsigned char mask)
{
  asm ("mov %0l, r0l"::"r"(mask));
  asm ("and.b #01,r0l");
  asm ("rotr.b r0l");
  asm ("ldc r0l,ccr");
}

extern __inline__ void sleep (void)
{
  asm ("sleep");
}

extern __inline__ void nop (void)
{
  asm ("nop");
}

extern __inline__ void set_ccr (unsigned char mask)
{
  asm ("ldc %0l,ccr"::"r"(mask));
}

extern __inline__ unsigned char get_ccr (void)
{
  unsigned char value;
  asm ("stc ccr,%0l":"=g"(value):);
  return(value);
}

extern __inline__ void *get_sp (void)
{
  void *sp;
  asm("mov.l r7,%0":"=g"(sp):);
  return(sp);
}
