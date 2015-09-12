#
# Copyright (C) 2010-2014 Mikhail Afanasyev
#
# This file is part of AVR-mlib2.
#
# AVR-mlib2 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# AVR-mlib2 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with AVR-mlib2.  If not, see <http://www.gnu.org/licenses/>.
#

# The master makefile should have declared MLIB macro, which is a list
# of files to include. All files will be copied into mlib2 files and used
# as dependecies; additionally .c files will have corresponding .h file
# audo-included, and linked into project.
# Prepend directory name, and split it into .c and .h files
MLIB2_FULL=$(MLIB2:%=mlib2/%)
MLIB2_FULL_C=$(filter %.c,$(MLIB2_FULL))
MLIB2_FULL_DEPS=$(MLIB2_FULL) $(MLIB2_FULL_C:%.c=%.h) \
	 mlib2/mcommon.h mlib2/common.mk mlib2/prog.mk

# dependecies on each C file
COMDEPS=$(wildcard *.h) $(MLIB2_FULL_DEPS)
# list of files to compile. May be overriden by user.
CCFILES=$(wildcard *.c)
# where intermediate files are placed, auto-created
OBJDIR=obj-$(MCUTYPE)
# name of output (with _hex prefix as needed)
OUT=firmware
# output hex for flash
OUT_FLASH_HEX=$(OBJDIR)/$(OUT).hex
# output hex for eeprom - created only if eeprom section is not empty
OUT_EEPROM_HEX=$(OBJDIR)/$(OUT)_eeprom.hex
# map files, elf, elf dump, etc...
OUTINFO=$(OBJDIR)/$(OUT)

include mlib2/prog.mk

OPTF=-Os
WERR=-Werror
CCFLAGS=$(OPTF) -mmcu=$(MCUTYPE) -g3 -Imlib -I. -Wall $(WERR) -Wno-main $(CFLAGS)
CCFLAGS+=-std=gnu99 -Winline
# -nostdinc -nostdlib -I/usr/avr/include -I/usr/lib/gcc-lib/avr/3.3/include/
# -nostartfiles -nodefaultlib
OBJFILES_LIB=$(MLIB2_FULL_C:mlib2/%.c=$(OBJDIR)/mlib2.%.o)
OBJFILES_C=$(CCFILES:%.c=$(OBJDIR)/%.o)
OBJFILES+=$(OBJFILES_LIB) $(OBJFILES_C)
LDFLAGS = -Wl,-Map,$(OUTINFO).map -Wl,--cref

# add dead code elimination
CFLAGS += -fdata-sections -ffunction-sections -Wl,--gc-sections
LDFLAGS += -fdata-sections -ffunction-sections -Wl,--gc-sections

CFLAGS += $(if $(F_CPU), -DF_CPU=$(F_CPU))


.DEFAULT_GOAL := default

default: $(OUT_FLASH_HEX) size
	@echo "Run 'make flash eeprom fuses-write' to program all areas of the chip"

all: $(OUT_FLASH_HEX)

help:
	@echo MLIB C files:
	@echo " " $(MLIB2_FULL_C)
	@echo MLIB dependencies:
	@echo " " $(MLIB2_FULL_DEPS)
	@echo
	@echo Programming targets: flash eeprom fuses-write uisp-check
	@echo Other targets: view clean backup

.SUFFIXES:             # delete all default suffixes
.SUFFIXES: .o .c

.PRECIOUS: mlib2/%     # do not delete files in mlib2

.PHONY: flash eeprom fuses-write uisp-check view help run size install

view: $(OUT_FLASH_HEX)
	less $(OUTINFO).lst $(OUTINFO).map

$(OUT_FLASH_HEX): $(OUTINFO).elf
	avr-objcopy -j .text -j .data -O ihex $(OUTINFO).elf $(OUT_FLASH_HEX)
	@if avr-size -A $(OUTINFO).elf | grep -q eeprom; \
		then echo Extracting EEPROM; avr-objcopy -j .eeprom --change-section-lma .eeprom=0 -O ihex $(OUTINFO).elf $(OUT_EEPROM_HEX); \
		else echo > $(OUT_EEPROM_HEX); fi
	@# remove eemprom file if it is empty (1 line - checksum only)
	@if [ `wc -l < $(OUT_EEPROM_HEX)` = 1 ]; then rm $(OUT_EEPROM_HEX); fi;

$(OUTINFO).elf: $(OBJDIR) $(OBJFILES) $(OBJEXTRA)
	avr-gcc -mmcu=$(MCUTYPE) -g -o $(OUTINFO).elf  $(LDFLAGS) $(OBJFILES_LIB) $(OBJFILES_C) ${OBJEXTRA}
	@avr-objdump -h -S $(OUTINFO).elf > $(OUTINFO).lst

size: $(OUTINFO).elf
	@(printf "\n\n"; avr-size -C --mcu=$(MCUTYPE) $(OUTINFO).elf) | paste - - -

#avr-ld -o obj/firmware.hex --nostdlib --oformat ihex $(OBJFILES)

$(OBJDIR)/%.o : %.c $(COMDEPS)
	avr-gcc -c $(CCFLAGS) -Wa,-ahlmsdn=$@.a -o $@ $<

$(OBJDIR)/mlib2.%.o : mlib2/%.c mlib2/%.h $(COMDEPS)
	avr-gcc -c $(CCFLAGS) -Wa,-ahlmsdn=$@.a -o $@ $<

$(OBJDIR)/%.o: %.S $(COMDEPS)
	avr-gcc -c $(CCFLAGS) -x assembler-with-cpp -o $@ $<


$(OBJDIR):
	mkdir $(OBJDIR)

clean:
	rm -rf *~ $(OBJDIR) obj-*

veryclean: clean
	rm -rf mlib
