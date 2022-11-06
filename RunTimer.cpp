/**
 * Controls the operation timer.  
 * (C) Juan Schiavoni 2022
 *
 * Use the time.h library to parse the elapsed seconds.
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
