#
# Copyright (C) 2010-2014 Mikhail Afanasyev
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

# programming mode string
PROG ?= isp
# Avrdude port to use -- define this value to override all value for all PROG values
AD_PORT ?= $(if $(AD_PORT_$(PROG)), $(AD_PORT_$(PROG)), $(AD_PORT_DEFAULT))

# overrideable avrdude binary
AVRDUDE_BIN ?= avrdude
# overridable MCUTYPE for programming (if gcc and avrdude name differs)
AD_MCUTYPE ?= $(MCUTYPE)

# avrdude programmer ID (defaults to PROG value)
AD_PROGID ?= $(if $(AD_PROGID_$(PROG)), $(AD_PROGID_$(PROG)), $(PROG))

# overrideable avrdude programmer options
AD_OPTS ?= $(AD_OPTS_$(PROG))

# overrideable baudrate
AD_BAUD ?= $(AD_BAUD_$(PROG))

# overridable commands to enter/exit bootloader
# may also be overriden on per-port basis
BL_ENTER ?= $(BL_ENTER_$(PROG))
BL_LEAVE ?= $(BL_LEAVE_$(PROG))


#
# Per-PROG default port values/overrides
#
AD_PORT_DEFAULT ?= usb

# Overrides for specific programmer names
AD_PROGID_stk500 = stk500v1

AD_OPTS_dragon_isp = -B 10

AD_OPTS_dragon_dw = -B 10 -u

AD_BAUD_arduino = 57600
AD_PORT_arduino ?= /dev/serial/by-id/usb-Arduino*
BL_ENTER_arduino = enter-stk500.py -b $(AD_BAUD) $(AD_PORT)

# pololu PGM-02B programmer
AD_OPTS_pololu = -B 5
AD_BAUD_pololu = 115200
AD_PROGID_pololu = avrispmkii
AD_PORT_pololu ?= /dev/serial/by-id/usb-Silicon_Labs_Pololu_Orangutan_Programmer_*

AD_OPTS_bl = -u
AD_BAUD_bl = 19200
AD_PROGID_bl = avr109

AVRDUDE=$(AVRDUDE_BIN) -c $(AD_PROGID) -P $(AD_PORT) -p $(AD_MCUTYPE) \
  $(if $(AD_BAUD), -B $(AD_BAUD)) $(AD_OPTS)


ifeq (${MCUTYPE}, atmega163)
BL_ENTER_bl = ../doc/fdl-launch-bootloader.py $(AD_PORT) 38400
else
BL_ENTER_bl = ../doc/fdl-launch-bootloader.py $(AD_PORT) 19200
endif
BL_LEAVE_bl = sleep 0.5 && echo -n X > $(AD_PORT)

prog-mode-enter:
ifneq ($(BL_ENTER),)
	$(RSH_COMMAND) $(BL_ENTER)
endif

prog-mode-leave:
ifneq ($(BL_LEAVE),)
	$(RSH_COMMAND) $(BL_LEAVE)
endif


run: prog-mode-leave

install: flash
	$(MAKE) prog-mode-leave

flash: $(OUT_FLASH_HEX) prog-mode-enter
ifeq ($(RSH_COMMAND),)
	$(AVRDUDE) -U flash:w:$(OUT_FLASH_HEX)
else
	$(RSH_COMMAND) "cat >| /tmp/avr-hex" < $(OUT_FLASH_HEX)
	$(RSH_COMMAND) $(AVRDUDE) -U flash:w:/tmp/avr-hex:i
endif

eeprom: $(OUT_FLASH_HEX) prog-mode-enter
	$(AVRDUDE) -U eeprom:w:$(OUT_EEPROM_HEX)

fuses-write: prog-mode-enter
	$(RSH_COMMAND) $(AVRDUDE) -U lfuse:w:${FUSES_LOW_${MCUTYPE}}:m -U hfuse:w:${FUSES_HIGH_${MCUTYPE}}:m

prog-check: prog-mode-enter
	$(RSH_COMMAND) $(AVRDUDE)
