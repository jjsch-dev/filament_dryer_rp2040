#include "HeaterController.h"

HeaterController::HeaterController() :
                  pid(&pid_input, &pid_output, &pid_setpoint, KP_DEFAULT, KI_DEFAULT, KD_DEFAULT, DIRECT),
                  pid_const(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT),
                  tuner(&tune_input, &tune_output, tuner.ZN_PID, tuner.directIP, tuner.printOFF) {    
    _mode = MODE_STOP;
    pid_status = ST_DISABLED;
    tune_status = ST_DISABLED;    
    
    tune_settle_time_sec = 10;
    tune_test_time_sec = 500;     // runPid interval = testTimeSec / samples
    tune_samples = 500;
    tune_input_span = 70;
    tune_output_span = 255;
    tune_output_start = 0;
    tune_output_step = 100;
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
    
    pid.SetSampleTime(100);

    if ((ret_val = pid_const.begin())) {
        pid.SetTunings(pid_const.kp(), pid_const.ki(), pid_const.kd());  
    }
    
    fan_cooler(OUT_OFF);
    pwm(OUT_OFF);

    return ret_val;
}

float HeaterController::update(float box_temp, float bed_left_temp, float bed_right_temp) {
float output = 0;
  
  if (_mode == MODE_RUN_PID) {
    output = pid_controller(box_temp, bed_left_temp, bed_right_temp);
  } else if (_mode == MODE_RUN_TUNE){
    output = tune_controller(box_temp);
  }

  pwm(output);
   
  return output;
}
 
void HeaterController::inc_setpoint(int count) {
    
    int new_val = (int) pid_setpoint + count;
    
    if ((new_val >= 0) && (new_val <= 60)) {
        pid_setpoint = new_val;
    }    
}

int HeaterController::get_setpoint(void) {
    return pid_setpoint;
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
    analogWrite(FAN_PIN, output);
}

float HeaterController::pid_controller(float input, float bed_left_temp, float bed_right_temp) {
  pid_input = input;

  switch (pid_status) {
    case ST_DISABLED:
      pid.SetMode(MANUAL);
      fan_cooler(OUT_OFF);
    break;
    case ST_INITIALICE:
      pid.SetMode(MANUAL);
      pid_output = 0;
    
      pid_status = ST_RUN_PID; 
      
      pid.SetMode(AUTOMATIC); 
      fan_cooler(OUT_ON);
    break;
    case ST_RUN_PID:
      if ( max(bed_left_temp, bed_right_temp) > BED_MAX_TEMP ) {
        pid.SetMode(MANUAL);  
        pid_output = 0;
        pid_status = ST_WAIT_BED_TEMP_DROP;
      }
    break;
    case ST_WAIT_BED_TEMP_DROP: 
      if ( max(bed_left_temp, bed_right_temp) < (BED_MAX_TEMP - 5) ) {
        pid_status = ST_RUN_PID;
        pid.SetMode(AUTOMATIC);  
      }
    break;
    default:
      pid_status = ST_DISABLED;
    break;
  }

  pid.Compute();

  return (pid_status != ST_DISABLED) ? pid_output : 0;
}

float HeaterController::tune_controller(float input) {
  
  if (tune_status == ST_INITIALICE) {
    pid_status =  ST_DISABLED;
    pid.SetMode(MANUAL);
    pid_output = 0;   
    fan_cooler(OUT_ON); 
    pid_setpoint = 0;
    //set_time = 0;
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
        tuner.printTunings();
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
