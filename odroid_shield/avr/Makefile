#
#   MUTABLE STUFF
#

# MCU TYPE
MCUTYPE=atmega644p
F_CPU=16000000

PROG=arduino
AD_PORT=/dev/ttyACM99
AD_BAUD=115200
RSH_COMMAND=ssh odroid@odroid -t

# library files used
MLIB2=hwuart.c enter-stk500.py

# Pull from mlib2 master if it exists
MLIB2_BASE=~/proj/atmel/mlib2
-include ${MLIB2_BASE}/mlib2_update.mk

include mlib2/common.mk
