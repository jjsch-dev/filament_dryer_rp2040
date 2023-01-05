/**
 * Controls the beds that are responsible for heating the filament box.  
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
#include "HeaterController.h"

HeaterController::HeaterController(int pwm_freq, int pwm_res, float max_bed, ParamStorage& storage) :
                  pstorage(storage),
                  pid(&pid_input, &pid_output, &pid_setpoint),
                  tuner(&tune_input, &tune_output, tuner.ZN_PID, tuner.directIP, tuner.printOFF) {    
  _mode = MODE_STOP;
  pid_status = ST_DISABLED;
  tune_status = ST_DISABLED;    
  pwm_frequency = pwm_freq;
  pwm_resolution = pwm_res;
  max_bed_temp = max_bed;
  
  tune_settle_time_sec = 10;
  tune_test_time_sec = 250;     // runPid interval = testTimeSec / samples (500 mS)
  tune_samples = 500;
  tune_input_span = 60;
  tune_output_span = pwm_res; 
  tune_output_start = 0;
  tune_output_step = (pwm_res * 20) / 100;  // Set initial step in % depending on pwm resolution;
  tune_temp_limit = 60;
  tune_debounce = 1;
  tune_samples_count = 0;
  
  pid_input = 0; 
  pid_output = 0;
  tune_input = 0;
  tune_output = 0;
  tune_setpoint = 50;

  // The sample time of the PID has to match that of the tuner, and uses uS as the unit.
  pid_sample_timeout = (tune_test_time_sec * 1000000) / tune_samples;
}

bool HeaterController::begin(void) {
  tuner.Configure(tune_input_span, tune_output_span, tune_output_start, 
                  tune_output_step, tune_test_time_sec, tune_settle_time_sec, tune_samples);
  tuner.SetEmergencyStop(tune_temp_limit);
  
  pid.SetSampleTimeUs(pid_sample_timeout); // 500 mS en uS
  pid.SetOutputLimits(0, pwm_resolution);

  pid.SetProportionalMode(pid.pMode::pOnMeas);
  pid.SetAntiWindupMode(pid.iAwMode::iAwClamp);

  pid_setpoint = pstorage.setpoint(); 
  
  set_tunings();  
  
  analogWriteFreq(pwm_frequency);
  analogWriteRange(pwm_resolution);
  
  pinMode(FAN_PIN, OUTPUT);
  
  fan_cooler(OUT_OFF);
  pwm(OUT_OFF);
 
  moisture_servo.attach(MOISTURE_SERVO_PIN);  // attaches the servo on GI28

  return moisture_door(false);
}

float HeaterController::update(float box_temp, float bed_temp) {
float output = 0;

  if (_mode == MODE_RUN_PID) {
    output = pid_controller(box_temp, bed_temp);
  } else if (_mode == MODE_RUN_TUNE){
    output = tune_controller(box_temp, bed_temp);
  }
  
  pwm(output);
   
  return output;
}
 
void HeaterController::inc_setpoint(int count) {
  int new_val = pstorage.setpoint() + count;

  // The temperature can be regulated between 40 and 60 degrees. 0 = disabled
  if ((pstorage.setpoint() == 0) && (new_val < MIN_SETPOINT) && (count > 0)) {
    pstorage.write_setpoint(MIN_SETPOINT);    
  } else if(new_val < MIN_SETPOINT) {
    pstorage.write_setpoint(0); 
  } else if (new_val <= MAX_SETPOINT) {
    pstorage.write_setpoint(new_val);
  } 

  // It is necessary to keep a local copy of the setpoint because the PID library uses it as a reference.
  pid_setpoint = pstorage.setpoint(); 
}

int HeaterController::get_setpoint(void) {
  return pstorage.setpoint();
}

int HeaterController::get_mode(void) {
  return _mode;
}

void HeaterController::set_mode(int new_mode) {
  _mode = new_mode;
}

void HeaterController::start(void) {
  if (_mode == MODE_RUN_TUNE) {
    tune_status = ST_INITIALICE; 
    pid_status = ST_DISABLED; 
  } else if (_mode == MODE_RUN_PID) {
    pid_status = ST_INITIALICE;
    tune_status = ST_DISABLED;
  }
}

void HeaterController::stop(void) {
  /* 
   *  If the mode is to stop the heater, it turns off the fan and the PWM output. 
   *  Initializes the PID or Tuner when the previous mode is different from the current one.
   */
  fan_cooler(OUT_OFF);
  moisture_door(false);
  pwm(OUT_OFF);
  pid_status = ST_DISABLED;
  tune_status = ST_DISABLED;
  _mode = MODE_STOP;
}

void HeaterController::pwm(int output) {
  analogWrite(HOT_BED_LEFT_PIN, output);
  analogWrite(HOT_BED_RIGHT_PIN, output);
}  

/**
 * Use the Arduino PWM function to control the cooler fan in ON/OFF mode.
 */  
void HeaterController::fan_cooler(int output) {
  digitalWrite(FAN_PIN, (output == OUT_ON) ? HIGH : LOW );
}

float HeaterController::pid_controller(float box_temp, float bed_temp) {
  pid_input = box_temp;
  
  switch (pid_status) {
    case ST_DISABLED:
      pid.SetMode(pid.Control::manual);
      fan_cooler(OUT_OFF);
      moisture_door(false); 
    break;
    case ST_INITIALICE:
      pid.SetMode(pid.Control::manual);
      pid_output = 0;
      pid.SetMode(pid.Control::automatic);
      fan_cooler(OUT_ON);
      moisture_door(true); 
      pid_status = ST_RUN_PID; 
    break;
    case ST_RUN_PID:
      if (bed_temp > max_bed_temp) {
        pid.SetMode(pid.Control::manual);
        pid_status = ST_WAIT_BED_TEMP_DROP;
      }
    break;
    case ST_WAIT_BED_TEMP_DROP: 
      if (bed_temp < (max_bed_temp - 5)) {
        pid_status = ST_RUN_PID;
        pid.SetMode(pid.Control::automatic);
      }
    break;
    default:
      pid_status = ST_DISABLED;
    break;
  }

  pid.Compute();

  return (pid_status == ST_RUN_PID) ? pid_output : 0;
}

float HeaterController::tune_controller(float input, float bed_temp) {
  switch (tune_status) {
    case ST_INITIALICE:
      pid_status =  ST_DISABLED;
      pid.SetMode(pid.Control::manual);
      pid_output = 0;   
      fan_cooler(OUT_ON); 
      moisture_door(true); 
      tuner.Reset();
      tune_samples_count = 0;
      tune_status = ST_RUN_PID;
    break;
    case ST_RUN_PID: {
      if (bed_temp > max_bed_temp) {
        tune_status = ST_WAIT_BED_TEMP_DROP;
      } else {
        switch (tuner.Run()) {
          case tuner.sample: 
            tune_input = input;
            tune_samples_count++;
          break;
          // update PID with the new tunings
          case tuner.tunings: 
            float kp, ki, kd;
            tuner.GetAutoTunings(&kp, &ki, &kd); 
            pstorage.write_pid_const(kp, ki, kd);
            pstorage.save();
            set_tunings(); 
            stop(); 
          break;
        } 
      }
    }
    break;
    case ST_WAIT_BED_TEMP_DROP: 
      if (bed_temp < (max_bed_temp - 5)) {
        tune_status = ST_RUN_PID;
      }
    break;
    default:
      tune_status = ST_DISABLED;
    break;
  }  

  return (tune_status == ST_RUN_PID) ? tune_output : 0;
}

void HeaterController::set_tunings(void) {
  pid.SetTunings(pstorage.kp(), pstorage.ki(), pstorage.kd());
}

bool HeaterController::setpoint_reached(void) {
  if (pid_status == ST_RUN_PID) {
    float diff = get_setpoint() - pid_input;

    if ((diff >= -0.5) && (diff <= 0.5)) {
      return true; 
    }
  }

  return false;
}

int HeaterController::tuning_percentage(void) {
  /*
   * The tuner takes a sample every 1000 mS, both in the setup time and in the 
   * computation time. In order for the percentage to match the number of samples, 
   * the waiting seconds must be added to the total.
   */
  float total = tune_samples_count;
  total /= (tune_samples + tune_settle_time_sec);
  total *= 100;
  
  return total;
}

bool HeaterController::moisture_door(bool state) {
  
  int angle = state ? pstorage.moisture_open_angle() : pstorage.moisture_close_angle();
  
  // Control del position of the moisture vent door.
  if (angle <= MOISTURE_DOOR_MAX) {
    if (moisture_servo.read() != angle) {
      moisture_servo.write(angle);   
    }
  
    return true;
  }

  return false;
}

void HeaterController::set_moisture_angle(bool state, int value) {

  int angle = state ? pstorage.moisture_open_angle() : pstorage.moisture_close_angle();
  
  int new_val = angle + value;
  
  if ((new_val >= MOISTURE_DOOR_MIN) && (new_val <= MOISTURE_DOOR_MAX)) {
    moisture_servo.write(new_val);
    if (moisture_servo.read() == new_val) {
      if (state) {
        pstorage.write_moisture_open_angle(new_val);  
      } else {
        pstorage.write_moisture_close_angle(new_val); 
      }
    }  
  } else {
    if (state) {
      pstorage.write_moisture_open_angle(moisture_servo.read());  
    } else {
      pstorage.write_moisture_close_angle(moisture_servo.read()); 
    }
  }
}
