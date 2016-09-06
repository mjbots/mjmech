// Copyright 2012-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HW_H
#define HW_H

/****
  Teensy 2.0++ AT90USB1286

  Port A:
   A0 I - estop 1
   A1 I - estop 2
   A2 I - estop 3
   A3 I -
   A4 I -
   A5 I -
   A6 I -
   A7 I -

  Port B:
   B0 I - servo 0 in
   B1 I - servo 1 in
   B2 I - servo 2 in
   B3 I - servo 3 in
   B4 I - servo 4 in
   B5 I - servo 5 in
   B6 I - servo 6 in
   B7 I - servo 7 in

  Port C:
   C0 O - servo 0 out
   C1 O - servo 1 out
   C2 O - servo 2 out
   C3 O - servo 3 out
   C4 O - servo 4 out
   C5 O - servo 5 out
   C6 O - servo 6 out
   C7 O - servo 7 out

  Port D:
   D0 X - I2C SCL
   D1 X - I2C SDA
   D2 I - Motor encoder 1
   D3 I - Motor encoder 2
   D4 O - green LED
   D5 O - blue LED
   D6 O - red LED

  Port F:
   F0 I - Vsense 0
   F1 I - Vsense 1
   F2 I - Vsense 2
   F3 I - Vsense 3
   F4 I - Vsense 4
   F5 I - Vsense 5
   F6 I - Vsense 6
   F7 I - Vsense 7

****/

#define SERVO_IN B
#define SERVO_OUT C
#define ENCODER_IN D
#define ENCODER_IN_BIT 2
#define LASER_OUT_PORT D
#define LASER_OUT_BIT 4
#define LED_OUT_PORT D
#define LED_OUT_BIT 5

#define HW_JOIN(x, y) x ## y
#define PORT(x) HW_JOIN(PORT, x)
#define PIN(x) HW_JOIN(PIN, x)
#define DDR(x) HW_JOIN(DDR, x)

#endif
