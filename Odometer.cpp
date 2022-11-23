/**
 * Turns counter of the roller where the filament spool rests, depending on 
 * the configuration it can be used to turn the heater on and off.
 * 
 * MIT License
 * 
 * Copyright (c) 2022 Juan Schiavoni
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
#include "Odometer.h"

static Odometer* p_instance;

static void isr_data() {
  p_instance->handle_isr();
} 
  
Odometer::Odometer(int pin, ParamStorage& storage) :
          pstorage(storage) {
  counter = 0;
  do_pin = pin;
  p_instance = this;
}

bool Odometer::begin() {
  attachInterrupt(do_pin, isr_data, FALLING);

  return true;
}

bool Odometer::update() {
  return true;
}

int Odometer::ge_mode() {
  return pstorage.odom_mode();  
}

void Odometer::set_mode(int value) {
  int new_val = pstorage.odom_mode() + value;
  
  if ((new_val >= ODOM_MODE_DISABLED) && (new_val <= ODOM_MODE_BOTH)) {
    pstorage.write_odom_mode(new_val);
  }   
}

int Odometer::get_counter() {
  return counter;
} 

void Odometer::handle_isr() {
  counter++;
} 

 
