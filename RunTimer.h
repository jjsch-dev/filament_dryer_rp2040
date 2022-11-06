/**
 * Controls the operation timer.  
 * (C) Juan Schiavoni 2022
 *
 * Use the time.h library to parse the elapsed seconds.
 */ 
#pragma once

#include <time.h>
#include "Arduino.h"

class RunTimer 
{
public:
  RunTimer(unsigned long max_time);
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
  int             set_time;  
  unsigned long   start_time; 
  unsigned long   max_inc;
  struct tm*      tm;
};
