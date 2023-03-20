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

#define TCRT5000_D0_PIN           14  // GPIO14

#define ODOM_MODE_DISABLED        0
#define ODOM_MODE_START           1
#define ODOM_MODE_STOP            2
#define ODOM_MODE_BOTH            3
#define ODOM_MODE_DEFAULT         ODOM_MODE_DISABLED

#define ODOM_MINUTES_MIN          1
#define ODOM_MINUTES_MAX          30
#define ODOM_MINUTES_DEFAULT      2

#define ODOM_TURNS_DEFAULT        0

#define ODOM_DIAMETER_MIN         0
#define ODOM_DIAMETER_MAX         220   // Max 220 mm
#define ODOM_DIAMETER_DEFAULT     200   // Min 200 mm

#define ODOM_ENCODER_DIAMETER     13.7  // The theoretical diameter is 13.9, the print may differ by a few tenths.
#define ODOM_PULSES_BY_TURNS      12    // The encoder has 6 black marks but generates 12 pulses per turn, 
                                        // because the IRQ triggers on both edges.

#define ODOM_DETECTION_TIME_MIN   5
#define ODOM_DETECTION_TIME_MAX   500
#define ODOM_DETECTION_COUNT      10    // 12 pulses are equivalent to one turn on the encoder.
#define ODOM_60_MINUTES_TO_MILLIS 60000
        
typedef void (*callback_odom_start_t)(void);
typedef void (*callback_odom_stop_t)(void);

class Odometer  
{
public:
  Odometer(int pin, ParamStorage& storage);
  ~Odometer() {};

  bool begin(callback_odom_start_t c_start, callback_odom_stop_t c_stop);
  bool update(bool heater_on, bool lid_open);
  int get_counter();  
  int get_mode();
  int get_minutes();
  float get_turns();
  int get_diameter();
  void set_mode(int value);
  void set_minutes(int value);
  void set_turns(int value);
  void set_diameter(int value);
  void reset_timer(void);
  
public:  
  void handle_isr();

private:
  ParamStorage& pstorage;
  volatile int counter;
  int last_turns; 
  int start_turns;
  
  int do_pin;
  bool heater_on;
  unsigned long start_time;
  unsigned long time_turns_update;
  
  void update_turns(unsigned long now);
  
  callback_odom_stop_t  odom_stop;
  callback_odom_start_t odom_start;

private:
  unsigned long last_pulse_time;
  bool last_pid_state;
  bool pid_start;
};
