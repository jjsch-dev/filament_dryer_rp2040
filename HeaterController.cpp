/**
 * Controls the beds that are responsible for heating the filament box.  
 * (C) Juan Schiavoni 2022
 *
 * Encapsulates the PID and Tunner library.
 */ 
#include "HeaterController.h"

HeaterController::HeaterController(int pwm_freq, int pwm_res, float max_bed) :
                  pid(&pid_input, &pid_output, &pid_setpoint), 
                  pid_const(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT),
                  tuner(&tune_input, &tune_output, tuner.ZN_PID, tuner.directIP, tuner.printOFF) {    
  _mode = MODE_STOP;
  pid_status = ST_DISABLED;
  tune_status = ST_DISABLED;    
  pwm_frequency = pwm_freq;
  pwm_resolution = pwm_res;
  max_bed_temp = max_bed;
  
  tune_settle_time_sec = 10;
  tune_test_time_sec = 500;     // runPid interval = testTimeSec / samples
  tune_samples = 500;
  tune_input_span = 60;
  tune_output_span = pwm_res; 
  tune_output_start = 0;
  tune_output_step = (pwm_res * 25) / 100;  // Set initial step in % depending on pwm resolution;
  tune_temp_limit = 60;
  tune_debounce = 1;
  tune_samples_count = 0;
  
  pid_setpoint = 0; 
  pid_input = 0; 
  pid_output = 0;
  tune_input = 0;
  tune_output = 0;
  tune_setpoint = 50;
}

bool HeaterController::begin() {
bool ret_val;

  tuner.Configure(tune_input_span, tune_output_span, tune_output_start, 
                  tune_output_step, tune_test_time_sec, tune_settle_time_sec, tune_samples);
  tuner.SetEmergencyStop(tune_temp_limit);
  
  pid.SetSampleTimeUs(200000); // 200 mS en uS
  pid.SetOutputLimits(0, pwm_resolution);

  pid.SetProportionalMode(pid.pMode::pOnMeas);
  pid.SetAntiWindupMode(pid.iAwMode::iAwClamp);
  
  ret_val = pid_const.begin(); 
  
  pid.SetTunings(pid_const.kp(), pid_const.ki(), pid_const.kd());  
  
  analogWriteFreq(pwm_frequency);
  analogWriteRange(pwm_resolution);
  
  pinMode(FAN_PIN, OUTPUT);
  
  fan_cooler(OUT_OFF);
  pwm(OUT_OFF);
  
  return ret_val;
}

float HeaterController::update(float box_temp, float bed_temp) {
float output = 0;
  
  if (_mode == MODE_RUN_PID) {
    output = pid_controller(box_temp, bed_temp);
  } else if (_mode == MODE_RUN_TUNE){
    output = tune_controller(box_temp);
  }
  
  pwm(output);
   
  return output;
}
 
void HeaterController::inc_setpoint(int count) {
  int new_val = (int) pid_setpoint + count;

  // The temperature can be regulated between 40 and 60 degrees. 0 = disabled
  if ((pid_setpoint == 0) && (new_val < MIN_SETPOINT) && (count > 0)) {
    pid_setpoint = MIN_SETPOINT;    
  } else if(new_val < MIN_SETPOINT) {
    pid_setpoint = 0; 
  }else if (new_val <= MAX_SETPOINT) {
    pid_setpoint = new_val;
  }    
}

int HeaterController::get_setpoint(void) {
  return ((_mode == MODE_RUN_TUNE) ? tune_setpoint : pid_setpoint);
}

int HeaterController::get_mode(void) {
  return _mode;
}

void HeaterController::set_mode(int new_mode) {

  /* 
   *  If the mode is to stop the heater, it turns off the fan and the PWM output. 
   *  Initializes the PID or Tuner when the previous mode is different from the current one.
   */
  if (new_mode == MODE_STOP) {
    pid_setpoint = 0;
    fan_cooler(OUT_OFF);
    pwm(OUT_OFF);
    pid_status = ST_DISABLED;
    tune_status = ST_DISABLED;
  }
  
  if ((new_mode == MODE_RUN_TUNE) && (_mode != MODE_RUN_TUNE)) {
    tune_status = ST_INITIALICE;  
  } else if ((new_mode == MODE_RUN_PID) && (_mode != MODE_RUN_PID)) {
    pid_status = ST_INITIALICE;
  }

  _mode = new_mode;
}
    
void HeaterController::pwm(int output) {
  analogWrite(HOT_BED_LEFT_PIN, output);
  analogWrite(HOT_BED_RIGHT_PIN, output);
}  

/**
 * Use the Arduino PWM function to control the cooler fan in ON/OFF mode.
 */  
void  HeaterController::fan_cooler(int output) {
  digitalWrite(FAN_PIN, (output == OUT_ON) ? HIGH : LOW );
}

float HeaterController::pid_controller(float box_temp, float bed_temp) {
  pid_input = box_temp;

  switch (pid_status) {
    case ST_DISABLED:
      pid.SetMode(pid.Control::manual);
      fan_cooler(OUT_OFF);
    break;
    case ST_INITIALICE:
      pid.SetMode(pid.Control::manual);
      pid_output = 0;
      pid.SetMode(pid.Control::automatic);
      fan_cooler(OUT_ON);
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

float HeaterController::tune_controller(float input) {
  
  if (tune_status == ST_INITIALICE) {
    pid_status =  ST_DISABLED;
    pid.SetMode(pid.Control::manual);
    pid_output = 0;   
    fan_cooler(OUT_ON); 
    pid_setpoint = 0;

    tuner.Reset();
    tune_samples_count = 0;
    tune_status = ST_RUN_PID;
  } else if(tune_status == ST_RUN_PID) {
    uint8_t state = tuner.Run();
    
    switch (state) {
      case tuner.sample: // active once per sample during test
        tune_input = input;
        tune_samples_count++;
      break;
  
      case tuner.tunings: // active just once when sTune is done
        float kp, ki, kd;
        tuner.GetAutoTunings(&kp, &ki, &kd); // sketch variables updated by sTune

        pid_const.store(kp, ki, kd);
        
        pid.SetTunings(kp, ki, kd); // update PID with the new tunings
        
        set_mode(MODE_STOP); 
      break;
    } 
  }

  return tune_output;
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
