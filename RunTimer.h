/**
 * Controls the operation timer.  
 * Use the time.h library to parse the elapsed seconds.
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

#include <time.h>
#include "Arduino.h"

#include "ParamStorage.h"

#define TIME_DEFAULT 6

class RunTimer 
{
public:
  RunTimer(unsigned long max_time, ParamStorage& storage);
  ~RunTimer() {};

  bool update();
  void inc_time(int count);
  int get_time(void);
  void start();
  void reset();
  int get_hours(void);
  int get_minutes(void);
  int get_seconds(void);
  char* ascii_time(void);
    
private:
  ParamStorage&   pstorage;
  unsigned long   start_time; 
  unsigned long   max_inc;
  struct tm*      tm;
};
