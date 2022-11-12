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
#include "RunTimer.h"


RunTimer::RunTimer(unsigned long max_time) {
  max_inc = max_time;

  reset();
}

/*
 * If the timer is running, calculate the elapsed time in seconds and use the 
 * locate_time function to get the hour, minute, and second.
 * 
 * Return true when the time expires.
 */
bool RunTimer::update() {
unsigned long now = millis(); 
time_t s = 0;
bool ret_val = false;

  if (start_time > 0) {
    long time_left_ms = ((set_time * 3600000) - (now - start_time));
    
    ret_val = (time_left_ms <= 0);
    
    if (!ret_val) { 
      s = time_left_ms / 1000;
    }  
  }
  
  tm = localtime(&s); 
  
  return ret_val;
}

void RunTimer::inc_time(int count) {
  int new_val = set_time + count;
  
  if ((new_val >= 0) && (new_val <= max_inc)) {
    set_time = new_val;
  } 
}
    
int RunTimer::get_time(void) {
  return set_time;
}

void RunTimer::start() {
  start_time = millis();
}

void RunTimer::reset() {
  start_time = 0;
  set_time = 0;
  time_t s = 0;
  tm = localtime(&s);
}
    
int RunTimer::get_hours(void) {
  return tm->tm_hour; 
}

int RunTimer::get_minutes(void) {
  return tm->tm_min; 
}

int RunTimer::get_seconds(void) {
  return tm->tm_sec;
}
