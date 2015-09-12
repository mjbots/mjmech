#ifndef __MLIB2_MCOMMON_H__
#define __MLIB2_MCOMMON_H__

/*
#
# Copyright (C) 2014 Mikhail Afanasyev
#
# This file is part of AVR-mlib2.
#
# AVR-mlib is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# AVR-mlib is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with AVR-mlib.  If not, see <http://www.gnu.org/licenses/>.
#
*/

// mlib2 misc definitions file
#define BSET(port, mask, bits)  port = ((port) & ~(mask)) | (bits)

#define reboot()             asm("rjmp __ctors_end\n ");

// special 'bit' accessor
struct bit_rec_8b {
  uint8_t b0:1; uint8_t b1:1; uint8_t b2:1; uint8_t b3:1;
  uint8_t b4:1; uint8_t b5:1; uint8_t b6:1; uint8_t b7:1;
}  __attribute__ ((packed));

// BITVAR
// allows usage of individual bits as variables (for static bits only!)
#define BITVAR(var, bit)     ( ((volatile struct bit_rec_8b *)(&var))->b##bit )

// Bit-definition macros
// Specify up to 8 bit number as 8 binary digits
#define B8(d) ((unsigned char)B8__(HEX__(d)))
// Same, but as two groups of 4 digits
#define B8a(n1,n2) ((B8(n1) << 4) | B8(n2))

#define HEX__(n) 0x##n##LU
#define B8__(x) ((x&0x0000000FLU)?1:0) \
+((x&0x000000F0LU)?2:0) \
+((x&0x00000F00LU)?4:0) \
+((x&0x0000F000LU)?8:0) \
+((x&0x000F0000LU)?16:0) \
+((x&0x00F00000LU)?32:0) \
+((x&0x0F000000LU)?64:0) \
+((x&0xF0000000LU)?128:0)

// A macro to switch system clock to a highest rate.
#if defined(CLKPR)
#  define INIT_SYSTEM_CLOCK()   {  CLKPR = (1<<CLKPCE); CLKPR = 0; }
#  define MAYBE_INIT_SYSTEM_CLOCK()    INIT_SYSTEM_CLOCK()
#else
#  define MAYBE_INIT_SYSTEM_CLOCK()   { ; }
#endif

// A macro to enter bootloader
#ifdef __AVR_ATmega8__
#define ENTER_BOOTLOADER() (((void(*)(void))0x1C00)());
#elif defined(__AVR_ATmega328__)
#define ENTER_BOOTLOADER() { cli(); asm ( "jmp 0x3800" ); };
#elif defined(__AVR_ATmega168__)
#define ENTER_BOOTLOADER() { cli(); asm ( "jmp 0x3800" ); };
#endif

// String helpers
#define printf_PSTR(msg, args...)   printf_P(PSTR(msg) , ## args)

#endif
