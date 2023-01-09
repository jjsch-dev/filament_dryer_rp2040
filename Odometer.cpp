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
}

bool Odometer::begin(callback_odom_start_t c_start, callback_odom_stop_t c_stop) {
  attachInterrupt(do_pin, isr_data, FALLING);

  odom_start = c_start;
  odom_stop = c_stop;
  last_turns = pstorage.odom_turns();
  time_turns_update = millis();
  return true;
}

void Odometer::reset_timer(void) {
  start_time = millis();
}

bool Odometer::update(bool pid_on) {
unsigned long now = millis(); 

  if ((pstorage.odom_mode() == ODOM_MODE_START) || (pstorage.odom_mode() == ODOM_MODE_BOTH)) {
    if (!pid_on && (last_turns < pstorage.odom_turns())) {
      odom_start();    
    }
  } 
  
  if ((pstorage.odom_mode() == ODOM_MODE_STOP) || (pstorage.odom_mode() == ODOM_MODE_BOTH)) {
    if (pid_on) {
      if (last_turns == pstorage.odom_turns()) {
        if ((pstorage.odom_minutes() * 60000) <= (now - start_time)) {
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
  pstorage.write_odom_turns(pstorage.odom_turns() + 1);
} 

 
