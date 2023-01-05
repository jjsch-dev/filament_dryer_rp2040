/**
 * 3D Filament dryer using a Raspberry Pi Pico (RP2040). 
 *
 * Uses a bed heating to remove moisture from the air in order to 
 * prevent filament degradation and improve 3D printing quality.
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
#include "TempSensors.h"
#include "HeaterController.h"
#include "RunTimer.h"
#include "menu_config.h"
#include "ParamStorage.h"
#include "Odometer.h"

#define FIRMWARE_VERSION      "1.0.7"   // Version actual del firmware.

#define SAMPLE_TIMEOUT_100MS  100       // Refresh time for the sensor

/*
 * NOTE: with arduinopico version 2.6.2 the resolution in 8 bits does not work 
 * when two pins are used. See issue:
 * https://github.com/earlephilhower/arduino-pico/issues/955
 * 
 * It is fixed with push #926 and can be used from the master branch using git.
 * https://github.com/earlephilhower/arduino-pico/pull/962
 */
#define PWM_FREQUENCY         500       // Set a similar frequency of the Arduino Nano PWM (490Hz).
#define PWM_RESOLUTION        255       // Set 8 bits for PWM resolution (0-255).

#define MAX_HOURS             48
#define BED_MAX_TEMP          80.00

#define SPLASH_VERSION_TIME   5000      // Version screen display time in mS.

uint8_t           refresh_display = 10;
unsigned long     splash_timer;
bool              factory_reset = false;

ParamStorage      param_storage(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT, THERMS_DEFAULT,
                                SETPOINT_DEFAULT, TIME_DEFAULT, ODOM_MODE_DEFAULT, 
                                ODOM_MINUTES_DEFAULT, ODOM_TURNS_DEFAULT, 
                                ODOM_DIAMETER_DEFAULT, MOISTURE_CLOSE_DEFAULT, 
                                MOISTURE_OPEN_DEFAULT);
TempSensors       sensors(SAMPLE_TIMEOUT_100MS, param_storage);
HeaterController  heater(PWM_FREQUENCY, PWM_RESOLUTION, BED_MAX_TEMP, param_storage);
RunTimer          timer(MAX_HOURS, param_storage);
UserInterface     ui(menu_list, sizeof(menu_list));
Odometer          odometer(TCRT5000_D0_PIN, param_storage);

void bool_selection(char* str_buff, bool value) {
  if (value) {
    sprintf(str_buff, "ON");
  } else {
    sprintf(str_buff, "OFF");
  } 
}

void odom_mode_sel(char* str_buff, int mode) {
  switch (mode) {
    case ODOM_MODE_DISABLED:
      sprintf(str_buff, "OFF");
    break;
    case ODOM_MODE_START:
      sprintf(str_buff, "STAR");
    break;
    case ODOM_MODE_STOP:
      sprintf(str_buff, "STOP");
    break;
    case ODOM_MODE_BOTH:
      sprintf(str_buff, "BOTH");
    break;
    default:
      sprintf(str_buff, "-");
  }
}
  
/*
 * Callback function that invokes the UI when it needs to update the 
 * parameters of each configuration menu item.
 */
char* callback_menu_get(char* str_buff, int item_id) {
  switch (item_id) {
    case MNU_TURN_ENABLE_ID:
      bool_selection(str_buff, heater.get_mode() == MODE_RUN_PID);
    break;
    case MNU_BOX_TEMP_ID:
      sprintf(str_buff, "%d", heater.get_setpoint());
    break;
    case MNU_REMAINING_TIME_ID:
      sprintf(str_buff, "%d", timer.get_time());
    break;
    case MNU_TUNE_ENABLE_ID:
      bool_selection(str_buff, heater.get_mode() == MODE_RUN_TUNE);
    break;
    case MNU_THERMISTORS_ID:
      sprintf(str_buff, "%d", sensors.get_therms());
    break;
    case MNU_HEATER_TEMP_ID:
      sprintf(str_buff, "%02.0f", max(sensors.bed_left_celcius(), sensors.bed_right_celcius()));
    break;
    case MNU_ODOM_MODE_ID:
      odom_mode_sel(str_buff, odometer.get_mode());
    break;
    case MNU_ODOM_MINUTES_ID:
      sprintf(str_buff, "%d", odometer.get_minutes());
    break;
    case MNU_ODOM_TURNS_ID:
      sprintf(str_buff, "%2.1f", odometer.get_turns());
    break;
    case MNU_ODOM_DIAMETER_ID:
      sprintf(str_buff, "%d", odometer.get_diameter());
    break;
    case MNU_MOISTURE_CLOSE_ID:
      sprintf(str_buff, "%d", param_storage.moisture_close_angle());
    break;
    case MNU_MOISTURE_OPEN_ID:
      sprintf(str_buff, "%d", param_storage.moisture_open_angle());
    break;
    case MNU_KP_ID:
      sprintf(str_buff, "%2.2f", param_storage.kp());
    break;
    case MNU_KI_ID:
      sprintf(str_buff, "%2.2f", param_storage.ki());
    break;
    case MNU_KD_ID:
      sprintf(str_buff, "%2.2f", param_storage.kd());
    break;
    case MNU_FACTORY_RESET_ID:
      bool_selection(str_buff, factory_reset);
    break;
    case MNU_FIRMWARE_VERSION_ID:
      sprintf(str_buff, "%s", FIRMWARE_VERSION);
    break;
    default:
      sprintf(str_buff, "--");
    break;
  }

  return str_buff;
}

/*
 * Callback function that invokes the UI when editing is complete.
 */
void callback_menu_end_edit(int item_id) {
  switch (item_id) {
    case MNU_TURN_ENABLE_ID:
      if (heater.get_mode() == MODE_RUN_PID) {
        timer.start();
        heater.start();
        odometer.reset_timer();
      } else if (heater.get_mode() == MODE_STOP) {
        timer.reset();
        heater.stop();
      }
    break;
    case MNU_TUNE_ENABLE_ID: 
      if ((heater.get_mode() == MODE_RUN_TUNE)) {
        timer.reset();
        heater.start();
      } else if (heater.get_mode() == MODE_STOP) {
        timer.reset();
        heater.stop();
      }
    break;
    case MNU_THERMISTORS_ID:
    case MNU_BOX_TEMP_ID:
    case MNU_REMAINING_TIME_ID:
    case MNU_ODOM_MODE_ID:
    case MNU_ODOM_MINUTES_ID:
    case MNU_ODOM_TURNS_ID:
    case MNU_ODOM_DIAMETER_ID:
    case MNU_MOISTURE_CLOSE_ID:
    case MNU_MOISTURE_OPEN_ID:
      param_storage.save();
    break;
    case MNU_FACTORY_RESET_ID:
      if (factory_reset) {
        factory_reset = false;

        heater.set_mode(MODE_STOP);
        param_storage.write_pid_const(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT);
        param_storage.write_therms(THERMS_DEFAULT);
        param_storage.write_setpoint(SETPOINT_DEFAULT);
        param_storage.write_time(TIME_DEFAULT);
        param_storage.write_odom_mode(ODOM_MODE_DEFAULT);
        param_storage.write_odom_minutes(ODOM_MINUTES_DEFAULT);
        param_storage.write_odom_diameter(ODOM_DIAMETER_DEFAULT);
        param_storage.write_moisture_close_angle(MOISTURE_CLOSE_DEFAULT);
        param_storage.write_moisture_open_angle(MOISTURE_OPEN_DEFAULT);
        param_storage.save();  
        heater.set_tunings();
      }
    break;
  } 
}

/*
 * Callback function that invokes the UI on exit.
 */
void callback_menu_exit(int item_id) {
  refresh_display = 10;
}

/*
 * Callback function that invokes the UI when it needs to change the 
 * parameters of one configuration menu item.
 */
bool callback_menu_set(int value, int item_id) {
  switch (item_id) {
    case MNU_TURN_ENABLE_ID:
      if ((heater.get_setpoint() > 0) && (timer.get_time() > 0)) {
        heater.set_mode((value > 0) ? MODE_RUN_PID : MODE_STOP);
      } 
    break;
    case MNU_BOX_TEMP_ID: 
      heater.inc_setpoint(value);
    break;
    case MNU_REMAINING_TIME_ID:
      timer.inc_time(value);
    break;
    case MNU_TUNE_ENABLE_ID: 
      heater.set_mode((value > 0) ? MODE_RUN_TUNE : MODE_STOP);
    break;
    case MNU_THERMISTORS_ID:
      sensors.set_therms(value);
    break;
    case MNU_FACTORY_RESET_ID:
      factory_reset = (value > 0) ? true : false;
    break;
    case MNU_ODOM_MODE_ID: 
      odometer.set_mode(value);
    break;
    case MNU_ODOM_MINUTES_ID: 
      odometer.set_minutes(value);
    break;
    case MNU_ODOM_DIAMETER_ID: 
      odometer.set_diameter(value);
    break;
    case MNU_ODOM_TURNS_ID: 
      odometer.set_turns(value);
    break;
    case MNU_MOISTURE_CLOSE_ID:
      heater.set_moisture_angle(false, value);  
    break;
    case MNU_MOISTURE_OPEN_ID:
      heater.set_moisture_angle(true, value);  
    break;
    default:
      return false;
  }

  return true;
}

/*
 * turns on the PID, when it detects that the filament spool is moving 
 * and is not in tuning mode.
 */
void callback_odom_start(void) {
  if (heater.get_mode() == MODE_STOP) {
    heater.set_mode(MODE_RUN_PID);
    timer.start();
    heater.start();
    odometer.reset_timer();
  }
}

/*
 * Turn off the PID when detects that the filament spool has not 
 * moved for the programmed time. 
 * As long as the PID has reached the programmed temperature.
 */
void callback_odom_stop(void) {
  if (heater.setpoint_reached()) {
    timer.reset();
    heater.stop();
  } else {
    /*
     * Reset the shutdown timer to prevent the stop callback from 
     * firing continuously while the desired temperature is reached
     */
    odometer.reset_timer();  
  }
}

/*
 * Displays the firmware version screen for 3 seconds or until the encoder is pressed.
 */
bool display_version() {
bool ret_val = (ui.clicks() == 0);

  if (ret_val) {
    ret_val = (splash_timer + SPLASH_VERSION_TIME > millis());
    
    if (ret_val) { 
      ui.display.clearDisplay();             
      ui.display.setTextSize(2);                
      ui.display.setTextColor(WHITE);             
    
      ui.display.setCursor(15, 0);
      ui.display.println("Filament");
      ui.display.setCursor(35, 20);
      ui.display.println("Dryer");
      ui.display.setCursor(35, 40);   
      ui.display.print(FIRMWARE_VERSION);
      ui.display.display();
    } 
  }

  return ret_val;
}

/*
 * Shows the runining information.
 */
void update_display_info() {
char buff[100];
  
  ui.display.clearDisplay();             
  ui.display.setTextSize(2);                
  ui.display.setTextColor(WHITE);             
  
  ui.display.setCursor(22, 0);   
  int setpoint = (heater.get_mode() == MODE_STOP) ? 0 : heater.get_setpoint();
  sprintf(buff, "%02.0f/%02d C", sensors.box_celcius(), setpoint);
  ui.display.print(buff);

  if (heater.get_mode() == MODE_RUN_TUNE) {
    ui.display.setCursor(10, 22);
    sprintf(buff, "Bed %02.0f-%02.0f", sensors.bed_left_celcius(), sensors.bed_right_celcius());
  } else {
    ui.display.setCursor(15, 22);
    sprintf(buff, "%02d:%02d:%02d", timer.get_hours(), timer.get_minutes(), timer.get_seconds());
  }
  ui.display.print(buff); 
  
  if (heater.get_mode() == MODE_RUN_TUNE) {
    ui.display.setCursor(10, 42);
    sprintf(buff, "Tune %02d %%", heater.tuning_percentage());
  } else {
    ui.display.setCursor(27, 42);
    sprintf(buff, "%02.1f %%", sensors.box_humidity());
  }
  ui.display.print(buff); 
  
  ui.display.display();                 
}

/*
 * It sends the controller parameters through the serial port to graph the behavior 
 * curve of the system, both in PID and Tune modes.
 * Note: Since the system takes more than 20 minutes to stabilize, it sends a sample 
 * every 6 seconds so that it can be displayed in 500 samples.
 */
void plot_pid(float pwm_val, float bed_temp) {
static uint8_t plot_delay = 0;
  
  if (Serial) {
    if (heater.get_mode() == MODE_STOP) { 
      plot_delay = 0; 
    } else if ((plot_delay++ == 0) || (heater.get_mode() == MODE_RUN_TUNE)) {
      pwm_val /= PWM_RESOLUTION;
      pwm_val *= 100;
          
      Serial.print(F("Setpoint:"));     Serial.print(heater.get_setpoint());  Serial.print(F(", "));
      Serial.print(F("BoxTemp:"));      Serial.print(sensors.box_celcius());  Serial.print(F(", "));
      Serial.print(F("BoxHumidity:"));  Serial.print(sensors.box_humidity()); Serial.print(F(", "));
      Serial.print(F("PWM:"));          Serial.print(pwm_val);                Serial.print(F(", "));
      if (sensors.get_therms() > 0) {
        Serial.print(F("BedTemp:"));    Serial.print(bed_temp);               Serial.print(F(", "));
        Serial.print(F("BedMax:"));     Serial.print(BED_MAX_TEMP);
      }           
      Serial.println();
    } else if (plot_delay >= 6) {
      plot_delay = 0;
    }
  }
}

/*
 * Show one time the introduction message when USB serial port is connected.
 */
void show_intro_msg(void) {
static bool one_time = true;
  
  if (Serial && one_time) {
    Serial.print("Filament dryer controller - version: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.print("Kp: "); Serial.print(param_storage.kp());
    Serial.print(" Ki: "); Serial.print(param_storage.ki());
    Serial.print(" Kd: "); Serial.println(param_storage.kd());
    Serial.print("Spool turns: "); Serial.println(odometer.get_turns(), 2);
    one_time = false;
  } 
}
  
/**
 * 
 */
void setup() {
  Serial.begin(115200);

  param_storage.begin();
  ui.begin(callback_menu_get, callback_menu_set, 
           callback_menu_end_edit, callback_menu_exit);
  sensors.begin();
  heater.begin();
  odometer.begin(callback_odom_start, callback_odom_stop);
  
  splash_timer = millis();
}

void loop() {
  show_intro_msg();

  int menu_mode = ui.update();
  
  // when the timer in hours expires, it stops the heater.
  if (timer.update()) {
    heater.stop();
    timer.reset();
  }
  
  // Update sensor values every 100 mS.
  if (sensors.update()) {
    refresh_display++;
  }

  // The odometer functions only run in PID mode.
  odometer.update(heater.get_mode() == MODE_RUN_PID);
  
  // Displays the firmware version screen for a seconds or until the encoder is pressed.
  if (!display_version()) {
    float  bed_temp = max(sensors.bed_left_celcius(), sensors.bed_right_celcius());
    float pwm_val = heater.update(sensors.box_celcius(), bed_temp);
  
    // Once per second.
    if (refresh_display >= 10) {
      refresh_display = 0;
  
     /*
      * Shows status information, when not in setup mode.
      */
      if (menu_mode == MENU_MODE_INFO) {  
        update_display_info();
      }
      
      /*
       * Send the input output and setpoint values of the PID and tuner. 
       * Use the Arduino Plotter to plot the system response.
       * Note: The output is converted to percentage.
       */
      plot_pid(pwm_val, bed_temp);
    }
  }
}
