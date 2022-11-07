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

#define FIRMWARE_VERSION      "0.0.9" // Version actual del firmware.

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
 * NOTE: with arduinopico version 2.6.2 the resolution in 8 bits does not work 
 * when two pins are used. See issue:
 * https://github.com/earlephilhower/arduino-pico/issues/955
 * 
 * It is fixed with push #926 and can be used from the ppp branch using git.
 * https://github.com/earlephilhower/arduino-pico/pull/962
 */
#define PWM_FREQUENCY         500     // Set a similar frequency of the Arduino Nano PWM (490Hz).
#define PWM_RESOLUTION        255     // Set 8 bits for PWM resolution (0-255).

#define MAX_HOURS             48
#define BED_MAX_TEMP          80.00

#define SPLASH_VERSION_TIME   5000    // Version screen display time in mS.

uint8_t           menu_sel = 0;
uint8_t           refresh_display = 10;

unsigned long     splash_timer;
bool              encoder_clicked = false;

TempSensors       sensors(SAMPLE_TIMEOUT_100MS);
HeaterController  heater(PWM_FREQUENCY, PWM_RESOLUTION, BED_MAX_TEMP);
Adafruit_SSD1306  display(SCREEN_WIDTH, SCREEN_HEIGHT, &OLED_WIRE, OLED_RESET); 
EncoderButton     *eb; 
RunTimer          timer(MAX_HOURS);

/*
 * Displays the firmware version screen for 3 seconds or until the encoder is pressed.
 */
bool display_version() {
bool ret_val = !encoder_clicked;

  if (ret_val) {
    ret_val = (splash_timer + SPLASH_VERSION_TIME > millis());
    
    if (ret_val) { 
      display.clearDisplay();             
      display.setTextSize(2);                
      display.setTextColor(WHITE);             
    
      display.setCursor(15, 0);
      display.println("Filament");
      display.setCursor(35, 20);
      display.println("Dryer");
      display.setCursor(35, 40);   
      display.print(FIRMWARE_VERSION);
      display.display();
    }
  }

  return ret_val;
}

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

  encoder_clicked = true;
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

void plot_pid(float pwm_val) {
  if (Serial) {
    float  bed_temp = max(sensors.bed_left_celcius(), sensors.bed_right_celcius());
    
    pwm_val /= PWM_RESOLUTION;
    pwm_val *= 100;
        
    Serial.print(F("Setpoint:"));     Serial.print(heater.get_setpoint());  Serial.print(F(", "));
    Serial.print(F("BoxTemp:"));      Serial.print(sensors.box_celcius());  Serial.print(F(", "));
    Serial.print(F("BoxHumidity:"));  Serial.print(sensors.box_humidity()); Serial.print(F(", "));
    Serial.print(F("PWM:"));          Serial.print(pwm_val);                Serial.print(F(", "));
    Serial.print(F("BedTemp:"));      Serial.print(bed_temp);               Serial.print(F(", "));
    Serial.print(F("BedMax:"));       Serial.print(BED_MAX_TEMP);
               
    Serial.println();
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

  splash_timer = millis();
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

  // Displays the firmware version screen for a seconds or until the encoder is pressed.
  if (!display_version()) {
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
        plot_pid(pwm_val); 
      }
    }
  }
}
