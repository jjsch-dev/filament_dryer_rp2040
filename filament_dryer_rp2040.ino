/**
 * 3D Filament dryer using a Raspberry Pi Pico (RP2040). 
 * (C) Juan Schiavoni 2022
 *
 * Uses a bed heating to remove moisture from the air in order to 
 * prevent filament degradation and improve 3D printing quality.
 */ 
#include "TempSensors.h"
#include "HeaterController.h"
#include "RunTimer.h"
#include "UserInterface.h"

#define FIRMWARE_VERSION      "0.1.0"   // Version actual del firmware.

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

menu_item_t menu_list[] = {
  {"Exit", MENU_MODE_EXIT, 80}, 
  {"Temp:", MENU_MODE_EDIT, 80}, 
  {"Time:", MENU_MODE_EDIT, 80}, 
  {"Tune:", MENU_MODE_EDIT, 80},
  {"Bed:", MENU_MODE_INFO, 80},
  {"Kp:", MENU_MODE_INFO, 60},
  {"Ki:", MENU_MODE_INFO, 60},
  {"Kd:", MENU_MODE_INFO, 60},
  {"V:", MENU_MODE_INFO, 50}
};

uint8_t           refresh_display = 10;
unsigned long     splash_timer;

TempSensors       sensors(SAMPLE_TIMEOUT_100MS);
HeaterController  heater(PWM_FREQUENCY, PWM_RESOLUTION, BED_MAX_TEMP);
RunTimer          timer(MAX_HOURS);

/*
 * Callback function that invokes the UI when it needs to update the 
 * parameters of each configuration menu item.
 */
char* callback_menu_get(char* value, int item_id) {
  switch(item_id) {
      case 1:
        sprintf(value, "%d", heater.get_setpoint());
      break;
      case 2:
        sprintf(value, "%d", timer.get_time());
      break;
      case 3:
        if (heater.get_mode() == MODE_RUN_TUNE) {
          sprintf(value, "ON");
        } else {
          sprintf(value, "OFF");
        }
      break;
      case 4:
        sprintf(value, "%02.0f", max(sensors.bed_left_celcius(), sensors.bed_right_celcius()));
      break;
      case 5:
        sprintf(value, "%2.2f", heater.pid_const.kp());
      break;
      case 6:
        sprintf(value, "%2.2f", heater.pid_const.ki());
      break;
      case 7:
        sprintf(value, "%2.2f", heater.pid_const.kd());
      break;
      case 8:
        sprintf(value, "%s", FIRMWARE_VERSION);
      break;
      default:
        sprintf(value, "--");
      break;
    }

    return value;
}

/*
 * Check if it is necessary to go to PID run mode
 */
void check_if_start_pid(void) {
  if ((heater.get_setpoint() > 0) && (timer.get_time() > 0)) {
    if (heater.get_mode() == MODE_RUN_TUNE) {
      // In tuner mode, the set point is fixed at 50 degrees. And when it goes into 
      // stop mode, it resets to zero, so you have to save it for later use.
      int setpoint = heater.get_setpoint();
      heater.set_mode(MODE_STOP);  
      heater.inc_setpoint(setpoint);
    } 
    
    heater.set_mode(MODE_RUN_PID);
    timer.start();
  }   
}

/*
 * Callback function that invokes the UI when it needs to change the 
 * parameters of one configuration menu item.
 */
bool callback_menu_set(int value, int item_id, int item_type) {
  if(item_type == MENU_MODE_EXIT) {
    refresh_display = 10;
    if (heater.get_mode() == MODE_RUN_TUNE) { 
      if((heater.get_setpoint() == 0) || (timer.get_time() == 0)) {
        heater.set_mode(MODE_STOP);
        timer.reset();
      }
    }
  } else {
    switch (item_id) {
      case 1: 
        heater.inc_setpoint(value);
        check_if_start_pid();
      break;
      case 2:
        timer.inc_time(value);
        check_if_start_pid();
      break;
      case 3: 
        heater.set_mode((value > 0) ? MODE_RUN_TUNE : MODE_STOP);
        timer.reset();
      break;
      default:
        return false;
    }
  }

  return true;
}

UserInterface ui(menu_list, sizeof(menu_list), callback_menu_get, callback_menu_set);

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
  sprintf(buff, "%02.0f/%02d C", sensors.box_celcius(), heater.get_setpoint());
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
      Serial.print(F("BedTemp:"));      Serial.print(bed_temp);               Serial.print(F(", "));
      Serial.print(F("BedMax:"));       Serial.print(BED_MAX_TEMP);
                 
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
    Serial.print("Kp: "); Serial.print(heater.pid_const.kp());
    Serial.print(" Ki: "); Serial.print(heater.pid_const.ki());
    Serial.print(" Kd: "); Serial.println(heater.pid_const.kd());
    one_time = false;
  } 
}
  
/**
 * 
 */
void setup() {
  Serial.begin(115200);

  ui.begin();

  sensors.begin();
  
  heater.begin();

  splash_timer = millis();
}

void loop() {
  show_intro_msg();

  int menu_mode = ui.update();
  
  // when the timer in hours expires, it stops the heater.
  if (timer.update()) {
    heater.set_mode(MODE_STOP);
    timer.reset();
  }
  
  // Update sensor values every 100 mS.
  if (sensors.update()) {
    refresh_display++;
  }

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
