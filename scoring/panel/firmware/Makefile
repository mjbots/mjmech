PROG=panel

OBJS=

FOOT_DBG_FLAG = #-dbg
UPDATE_VCS := $(shell ./revision_update.py vcs_version.h)

#GCC_PART = __AVR_ATtiny85__
#AVR_PART = attiny85

GCC_PART = __AVR_ATtiny85__
AVR_PART = attiny85

CFLAGS= -mmcu=avr2 -D$(GCC_PART) -g -I../avr_include -Os -mcall-prologues -Winline -Wall -DF_CPU=8000000 -DFOOT_DBG_FLAG=\"$(FOOT_DBG_FLAG)\" -std=gnu99

all : $(PROG).srec

clean :
	rm -f *.o *.out *.srec *.hex


$(PROG).srec : $(PROG).out
	avr-objcopy -j .text -O srec $(PROG).out $(PROG).srec 
	avr-objcopy -j .text -O ihex $(PROG).out $(PROG).hex

$(PROG).out : $(PROG).o $(OBJS)
	avr-gcc $(CFLAGS) $(PROG).o $(OBJS) -o $(PROG).out

%.o : %.c
	avr-gcc -c $(CFLAGS) $< -o $@

load : $(PROG).srec
	avrdude -p $(AVR_PART) -c avrispmkII -U flash:w:$(PROG).srec

depend:
	makedepend -I../avr_include $(PROG).c $(OBJS:%.o=%.c)
# DO NOT DELETE

foot.o: /usr/include/inttypes.h /usr/include/features.h
foot.o: /usr/include/stdc-predef.h /usr/include/sys/cdefs.h
foot.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
foot.o: /usr/include/stdint.h /usr/include/bits/wchar.h /usr/include/string.h
foot.o: /usr/include/xlocale.h /usr/include/stdlib.h
foot.o: /usr/include/bits/waitflags.h /usr/include/bits/waitstatus.h
foot.o: /usr/include/endian.h /usr/include/bits/endian.h
foot.o: /usr/include/bits/byteswap.h /usr/include/bits/types.h
foot.o: /usr/include/bits/typesizes.h /usr/include/bits/byteswap-16.h
foot.o: /usr/include/sys/types.h /usr/include/time.h
foot.o: /usr/include/sys/select.h /usr/include/bits/select.h
foot.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
foot.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
foot.o: /usr/include/alloca.h /usr/include/bits/stdlib-float.h
foot.o: /usr/include/ctype.h /usr/include/stdio.h /usr/include/libio.h
foot.o: /usr/include/_G_config.h /usr/include/wchar.h
foot.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h i2c.h
foot.o: util.h vcs_version.h compact_software_uart.h usart_rate_calc.h
util.o: util.h
i2c.o: /usr/include/string.h /usr/include/features.h
i2c.o: /usr/include/stdc-predef.h /usr/include/sys/cdefs.h
i2c.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
i2c.o: /usr/include/xlocale.h i2c.h util.h
