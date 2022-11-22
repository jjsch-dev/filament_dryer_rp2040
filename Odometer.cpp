/**
 * Encapsulates the temperature and humidity sensors.  
 * One read at a time is performed to optimize the time the CPU 
 * hangs while performing the read.
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
  
Odometer::Odometer(int pin) {
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

int Odometer::get_counter() {
  return counter;
} 

void Odometer::handle_isr() {
  counter++;
} 

 
