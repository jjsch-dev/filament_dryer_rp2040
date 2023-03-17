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
  last_turns = 0;
  heater_on = false;
  do_pin = pin;
  p_instance = this;
  time_turns_update = 0;
  start_turns = 0;

  last_pulse_time = 0;
  last_pid_state = false;
  pid_start = false;
}

bool Odometer::begin(callback_odom_start_t c_start, callback_odom_stop_t c_stop) {
  pinMode(do_pin, INPUT_PULLUP);
  attachInterrupt(do_pin, isr_data, FALLING); //RISING); //CHANGE); //FALLING);

  odom_start = c_start;
  odom_stop = c_stop;
  last_turns = pstorage.odom_turns();
  time_turns_update = millis();

  return true;
}

void Odometer::reset_timer(void) {
  start_time = millis();
}

bool Odometer::update(bool pid_on, bool lid_open) {
unsigned long now = millis(); 
unsigned long elapsed_time = (now - start_time);

  if ((last_pid_state != pid_on) || lid_open) {
    start_turns = 0;
    pid_start = false;  
  }

  last_pid_state = pid_on;
  
  if (!pid_on) {
    if ((pstorage.odom_mode() == ODOM_MODE_START) || (pstorage.odom_mode() == ODOM_MODE_BOTH)) {
      /* 
       * Waits for a single turn to elapse and that it is within the time window (50-500 mS) 
       * to be considered valid. If it detects two or more turns, it discards them. Prevents 
       * bumps in the box from generating a false start. Waits 5 pulses to turn on the equipment.
       */
      if (pid_start && !lid_open) {
        odom_start();
      }

      reset_timer();
    }
  } else {  
    if ((pstorage.odom_mode() == ODOM_MODE_STOP) || (pstorage.odom_mode() == ODOM_MODE_BOTH)) {
      if (last_turns == pstorage.odom_turns()) {
        if ((pstorage.odom_minutes() * ODOM_60_MINUTES_TO_MILLIS) <= elapsed_time) {
          odom_stop();
        }
      } else {
        reset_timer();
      }
    }
  }
  
  update_turns(now);

  return true;
}

void Odometer::update_turns(unsigned long now) {
  last_turns = pstorage.odom_turns();

  /*
   * Updates the number of laps every 10 minutes to avoid exceeding 
   * the allowed recording cycles of the flash (approx. 10000).
   */
  if ((time_turns_update + 600000) < now) {
    pstorage.save();
    time_turns_update = now;
  }
}
  
int Odometer::get_mode() {
  return pstorage.odom_mode();  
}

void Odometer::set_mode(int value) {
  int new_val = pstorage.odom_mode() + value;
  
  if ((new_val >= ODOM_MODE_DISABLED) && (new_val <= ODOM_MODE_BOTH)) {
    pstorage.write_odom_mode(new_val);
  }   
}

int Odometer::get_minutes() {
  return pstorage.odom_minutes();
}

void Odometer::set_minutes(int value) {
  int new_val = pstorage.odom_minutes() + value;
  
  if ((new_val >= ODOM_MINUTES_MIN) && (new_val <= ODOM_MINUTES_MAX)) {
    pstorage.write_odom_minutes(new_val);
  }   
}

void Odometer::set_diameter(int value) {
  int new_val = pstorage.odom_diameter() + value;
  
  if ((new_val >= ODOM_DIAMETER_MIN) && (new_val <= ODOM_DIAMETER_MAX)) {
    pstorage.write_odom_diameter(new_val);
  }   
}

float Odometer::get_turns() {
  float diam_ratio = ODOM_ENCODER_DIAMETER;
  diam_ratio /= pstorage.odom_diameter();
  
  float encoder_turns = pstorage.odom_turns();
  encoder_turns /= ODOM_PULSES_BY_TURNS;
  
  return (encoder_turns * diam_ratio);
}

int Odometer::get_diameter() {
  return pstorage.odom_diameter();
}

/*
 * The lap counter can be reset from the menu by turning the encoder to the left.
 */
void Odometer::set_turns(int value) {
  if (value < 0) {
    pstorage.write_odom_turns(ODOM_TURNS_DEFAULT);
  }   
}

void Odometer::handle_isr() {
unsigned long now;

  if (digitalRead(do_pin) == LOW) {
    now = millis();
 
    pstorage.write_odom_turns(pstorage.odom_turns() + 1);
  
    unsigned long elapsed_time = now - last_pulse_time;
  
    if ((elapsed_time >= ODOM_DETECTION_TIME_MIN) && 
        (elapsed_time <= ODOM_DETECTION_TIME_MAX)) {
      start_turns++;
  
      if (++start_turns >= ODOM_DETECTION_COUNT) {
        pid_start = true;  
        start_turns = 0;
      }
    } else {
      start_turns = 0;  
    }
    
    last_pulse_time = now;
  }
} 

 
