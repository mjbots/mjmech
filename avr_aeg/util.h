// Copyright 2012 Josh Pieper.  All rights reserved.

#ifndef UTIL_H
#define UTIL_H

#include <avr/io.h>

void uint8_to_hex(char** buf, uint8_t value);
void uint16_to_hex(char** buf, uint16_t value);

uint8_t parse_hex2(char** buf, uint8_t* value);

#endif
