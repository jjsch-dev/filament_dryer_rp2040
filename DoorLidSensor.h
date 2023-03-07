/**
 * It detects the opening of the door with a mechanical switch and includes an 
 * anti-bounce function to avoid reading errors.
 * 
 * MIT License
 * 
 * Copyright (c) 2023 Juan Schiavoni
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */ 
#pragma once

#include "Arduino.h"

#define DOOR_LID_PIN            22  // GPIO22
#define DOOR_LID_DEBOUNCE       50

class DoorLidSensor  
{
public:
  DoorLidSensor(int pin, unsigned long debounce);
  ~DoorLidSensor() {};

  bool begin(void);
  bool update(void);
  bool state(void);
  
private:
  
 int lid_pin;                           // Mechanical switch pin
 int last_state;                        // Last switch state
 int current_state;                     // Current state of the switch
 unsigned long debounce_time;           // Debounce time in milliseconds
 unsigned long last_debounce_time;      // Last debounce time
 bool lid_state;                        // True = lid open;
};
