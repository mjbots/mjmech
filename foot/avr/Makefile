PROG=foot

CFLAGS= -mmcu=avr2 -D__AVR_ATtiny861__ -g -I../avr_include -Os -mcall-prologues -Winline -Wall -DF_CPU=8000000

all : $(PROG).srec

clean :
	rm -f *.o *.out *.srec *.hex


$(PROG).srec : $(PROG).out
	avr-objcopy -j .text -O srec $(PROG).out $(PROG).srec 
	avr-objcopy -j .text -O ihex $(PROG).out $(PROG).hex

$(PROG).out : $(PROG).o
	avr-gcc $(CFLAGS) $(PROG).o -o $(PROG).out

$(PROG).o : $(PROG).c
	avr-gcc $(CFLAGS) $(PROG).c -c

load : $(PROG).srec
	avrdude -p attiny861 -c avrispmkII -U flash:w:$(PROG).srec

depend:
	makedepend -I../avr_include $(PROG).c
