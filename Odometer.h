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
#pragma once

#include "Arduino.h"
#include "ParamStorage.h"

#define TCRT5000_D0_PIN         14  // GPIO14

#define ODOM_MODE_DISABLED      0
#define ODOM_MODE_START         1
#define ODOM_MODE_STOP          2
#define ODOM_MODE_BOTH          3
#define ODOM_MODE_DEFAULT       ODOM_MODE_DISABLED

class Odometer  
{
public:
  Odometer(int pin, ParamStorage& storage);
  ~Odometer() {};

  bool begin();
  bool update();
  int get_counter();  
  int ge_mode();
  void set_mode(int value);

public:  
  void handle_isr();

private:
  ParamStorage& pstorage;
  volatile int counter;
  
  int do_pin;
};
