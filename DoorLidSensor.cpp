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
#include "DoorLidSensor.h"
 
DoorLidSensor::DoorLidSensor(int pin, unsigned long debounce) {

  lid_pin = pin;

  debounce_time = debounce;
}

bool DoorLidSensor::begin(void) {
  pinMode(lid_pin, INPUT_PULLUP);

  current_state = HIGH;
  last_state = LOW; 
  
  last_debounce_time = 0;

  lid_state = false;
  
  return true;
}

// Returns True when the door is open.
bool DoorLidSensor::update(void) {
 
  // Read the current state of the switch with debounce
  current_state = digitalRead(lid_pin);
  if (current_state != last_state) {
    last_debounce_time = millis();
  }

  // When the debounce time elapses, it ensures that the current state of 
  // the switch is stable before updating the state.
  if ((millis() - last_debounce_time) > debounce_time) {
    if (current_state == digitalRead(lid_pin)) {
      lid_state = (current_state == HIGH);
    }
  }

  last_state = current_state;
  
  return lid_state;
}

bool DoorLidSensor::state(void) {
  return lid_state;  
}
