PROG=panel

OBJS=

#UPDATE_VCS := $(shell ./revision_update.py vcs_version.h)

AVR_PART = attiny85

CFLAGS= -mmcu=$(AVR_PART) -g -I../avr_include -Os -mcall-prologues -Winline -Wall -DF_CPU=8000000 -std=gnu99
CFLAGS+= -Werror

all : $(PROG).srec

clean :
	rm -f *.o *.out *.srec *.hex


$(PROG).srec : $(PROG).out
	avr-size --mcu=$(AVR_PART) --format=avr $(PROG).out | paste - - -
	avr-objcopy -j .text -j .data -O ihex $(PROG).out $(PROG).hex

$(PROG).out : $(PROG).o $(OBJS)
	avr-gcc $(CFLAGS) $(PROG).o $(OBJS) -o $(PROG).out

%.o : %.c *.h
	avr-gcc -c $(CFLAGS) $< -o $@

load : $(PROG).srec
	avrdude -p $(AVR_PART) -c avrispmkII -U flash:w:$(PROG).hex

depend:
	makedepend -I../avr_include $(PROG).c $(OBJS:%.o=%.c)
# DO NOT DELETE
