/**
 * 3D Filament dryer using a Raspberry Pi Pico (RP2040). 
 * (C) Juan Schiavoni 2022
 *
 * Uses a bed heating to remove moisture from the air in order to 
 * prevent filament degradation and improve 3D printing quality.
 */ 
#include <Wire.h>
#include <EncoderButton.h>

#include <Adafruit_SSD1306.h>
#include "TempSensors.h"
#include "HeaterController.h"
#include "RunTimer.h"

#define SCREEN_WIDTH          128     // OLED display width, in pixels
#define SCREEN_HEIGHT         64      // OLED display height, in pixels
#define OLED_RESET            -1      // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_ADDR             0x3C

#define OLED_WIRE             Wire1
#define OLED_SDA_PIN          2
#define OLED_SCL_PIN          3

#define ENCONDER_CLK_PIN      6
#define ENCONDER_DT_PIN       7
#define ENCONDER_BUTTON_PIN   8

#define SAMPLE_TIMEOUT_100MS  100     // Refresh time for the sensor

/*
 * NOTE: for now the resolution in 8 bits does not work with frequencies lower 
 * than 1Khz with the porting for Arduino of Raspberry PI Pico. See issue:
 * https://github.com/earlephilhower/arduino-pico/issues/955
 */
#define PWM_FREQUENCY         1000    // Set a similar frequency of the Arduino Nano PWM (490Hz).
#define PWM_RESOLUTION        1024    // Set 8 bits for PWM resolution (0-255).

#define MAX_HOURS             48
#define BED_MAX_TEMP          80.00

uint8_t           menu_sel = 0;
uint8_t           refresh_display = 10;

TempSensors       sensors(SAMPLE_TIMEOUT_100MS);
HeaterController  heater(PWM_FREQUENCY, PWM_RESOLUTION, BED_MAX_TEMP);
Adafruit_SSD1306  display(SCREEN_WIDTH, SCREEN_HEIGHT, &OLED_WIRE, OLED_RESET); 
EncoderButton     *eb; 
RunTimer          timer(MAX_HOURS);

/*
 * Shows the configuration menu.
 */
void display_menu() {
  display.clearDisplay();            // Clear the display everytime
  display.setTextColor(WHITE);
  display.setTextSize(2);

  display.setCursor(15, 0);
  display.println("Temp:");
  display.setCursor(80, 0);
  display.println(heater.get_setpoint());

  display.setCursor(15, 20);
  display.println("Time:");
  display.setCursor(80, 20);
  display.println(timer.get_time());

  display.setCursor(15, 40);
  display.println("Tune:");
  display.setCursor(80, 40);
  if (heater.get_mode() == MODE_RUN_TUNE) {
    display.println("ON");
  } else {
    display.println("OFF");
  }
  display.setCursor(2, (menu_sel - 1) * 20);
  display.println(">");

  display.display();
}

/*
 * Shows the runining information.
 */
void update_display_info() {
char buff[100];
  
  display.clearDisplay();             
  display.setTextSize(2);                
  display.setTextColor(WHITE);             
  
  display.setCursor(22, 0);   
  sprintf(buff, "%02.0f/%02d C", sensors.box_celcius(), heater.get_setpoint());
  display.print(buff);

  if (heater.get_mode() == MODE_RUN_TUNE) {
    display.setCursor(10, 22);
    sprintf(buff, "Bed %02.0f-%02.0f", sensors.bed_left_celcius(), sensors.bed_right_celcius());
  } else {
    display.setCursor(15, 22);
    sprintf(buff, "%02d:%02d:%02d", timer.get_hours(), timer.get_minutes(), timer.get_seconds());
  }
  display.print(buff); 
  
  if (heater.get_mode() == MODE_RUN_TUNE) {
    display.setCursor(10, 42);
    sprintf(buff, "Tune %02d %%", heater.tuning_percentage());
  } else {
    display.setCursor(27, 42);
    sprintf(buff, "%02.1f %%", sensors.box_humidity());
  }
  display.print(buff); 
  
  display.display();                 
}

/**
 * Process the click event.
 */
void on_click(EncoderButton& eb) {

  if (menu_sel++ < 3) {
    display_menu();
  } else {
    menu_sel = 0;
    
    refresh_display = 10;
    if (heater.get_mode() == MODE_RUN_TUNE) {
      timer.reset();  
    } else {
      if ((heater.get_setpoint() > 0) && (timer.get_time() > 0)) {
        heater.set_mode(MODE_RUN_PID);
        timer.start();
      } else {
        heater.set_mode(MODE_STOP);
        timer.reset();
      }
   } 
  }
}

/**
 * Procces the 'encoder' event
 */
void on_encoder(EncoderButton& eb) {
int new_val;

  if (menu_sel > 0) {
    if (menu_sel == 1) {
      heater.inc_setpoint(eb.increment());
    } else if (menu_sel == 2) {
      timer.inc_time(eb.increment());
    } else if (menu_sel == 3) {
      heater.set_mode((eb.increment() > 0) ? MODE_RUN_TUNE : MODE_STOP);
    }
  
    display_menu();
  }
}

void plot_pid(float input, float output, float setpoint, float bed_temp, float bed_max) {
  if (Serial) {  
    Serial.print(F("Setpoint:"));  Serial.print(setpoint);  Serial.print(F(", "));
    Serial.print(F("Input:"));     Serial.print(input);     Serial.print(F(", "));
    Serial.print(F("Output:"));    Serial.print(output);    Serial.print(F(", "));
    Serial.print(F("Bed:"));       Serial.print(bed_temp);  Serial.print(F(", "));
    Serial.print(F("BedMax:"));    Serial.print(bed_max);   
    Serial.println();
  }
}

/*
 * Show one time the introduction message when USB serial port is connected.
 */
void show_intro_msg(void) {
static bool one_time = true;
  
  if (Serial && one_time) {
    Serial.println("Filament dryer controller.");
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
  
  /*
   * With a global instance of the object, the constructor is called before the Arduino 
   * core is initialized, and the attachInterrupt() function does not work. 
   * That's why many object libraries have a '.begin()' member function. 
   * You can also use new to create the object in the config function.
   */
  eb = new EncoderButton(ENCONDER_DT_PIN, ENCONDER_CLK_PIN, ENCONDER_BUTTON_PIN);
  eb->setPressedHandler(on_click);
  eb->setEncoderHandler(on_encoder);
  eb->setDebounceInterval(10);

  OLED_WIRE.setSDA(OLED_SDA_PIN);
  OLED_WIRE.setSCL(OLED_SCL_PIN);
   
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR); //Start the OLED display
  display.clearDisplay();
  display.display();

  sensors.begin();
  
  heater.begin();
}

void loop() {
  show_intro_msg();
  
  eb->update();

  // when the timer in hours expires, it stops the heater.
  if (timer.update()) {
    heater.set_mode(MODE_STOP);
    timer.reset();
  }
  
  // Update sensor values every 100 mS.
  if (sensors.update()) {
    refresh_display++;
  }
  
  float pwm_val = heater.update(sensors.box_celcius(), 
                                sensors.bed_left_celcius(),
                                sensors.bed_right_celcius());

  // Once per second.
  if (refresh_display >= 10) {
    refresh_display = 0;

   /*
    * Shows status information, when not in setup mode.
    */
    if (menu_sel == 0) {  
      update_display_info();
    }
    
    /*
     * Send the input output and setpoint values of the PID and tuner. 
     * Use the Arduino Plotter to plot the system response.
     * Note: The output is converted to percentage.
     */
    if (heater.get_mode() != MODE_STOP) { 
      pwm_val /= PWM_RESOLUTION;
      pwm_val *= 100;
      plot_pid(sensors.box_celcius(), pwm_val, heater.get_setpoint(),
               max(sensors.bed_left_celcius(), sensors.bed_right_celcius()),
               BED_MAX_TEMP); 
    }
  }
}
