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
