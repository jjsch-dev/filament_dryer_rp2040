/**
 * Controls the beds that are responsible for heating the filament box.  
 *
 * Encapsulates the PID and Tunner library.
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

#include <QuickPID.h>
#include <sTune.h>

#include "ParamStorage.h"

#define MODE_STOP               0
#define MODE_RUN_PID            1
#define MODE_RUN_TUNE           2

#define KP_DEFAULT              29.576  
#define KI_DEFAULT              0.056    
#define KD_DEFAULT              0.222   

#define ST_DISABLED             0
#define ST_INITIALICE           1
#define ST_RUN_PID              2
#define ST_RUN_TUNE             3
#define ST_WAIT_BED_TEMP_DROP   4

#define HOT_BED_LEFT_PIN        10      // Hot left bed controller pin by PWM (490 Hz)
#define HOT_BED_RIGHT_PIN       11      // Hot right bed controller pin by PWM (490 Hz).
#define FAN_PIN                 12      // Fan controller pin by PWM (980 Hz).

#define OUT_ON                  255     // MAX PWM duty cicle  
#define OUT_OFF                 0       // MIN PWM duty cicle

#define MIN_SETPOINT            40      // Minimum Box temperature. 
#define MAX_SETPOINT            60      // Maximun Box temperature.

class HeaterController 
{
public:
  HeaterController(int pwm_freq, int pwm_res, float max_bed_temp, ParamStorage& storage);
  ~HeaterController() {};

  bool begin(void);
  float update(float box_temp, float bed_temp);
  void inc_setpoint(int count);
  int get_setpoint(void);
  int get_mode(void);
  void set_mode(int mode);
  int tuning_percentage(void );
    
private:
  QuickPID  pid;
  sTune tuner; 
  ParamStorage& pstorage;
  
  int   _mode;
  int   pid_status;
  int   tune_status;
  int   pwm_frequency;
  int   pwm_resolution;
  float max_bed_temp;
  
  float pid_input;
  float pid_output;
  float pid_setpoint;

private:
  uint32_t  tune_settle_time_sec;
  uint32_t  tune_test_time_sec;     // runPid interval = testTimeSec / samples
  uint16_t  tune_samples;
  float     tune_input_span;
  float     tune_output_span;
  float     tune_output_start;
  float     tune_output_step;
  float     tune_temp_limit;
  uint8_t   tune_debounce;
  uint16_t  tune_samples_count;

  
  float     tune_input;
  float     tune_output;
  float     tune_setpoint;

private:
  float pid_controller(float box_temp, float bed_temp);
  float tune_controller(float input);
  void  pwm(int output);
  void  fan_cooler(int output);
};
