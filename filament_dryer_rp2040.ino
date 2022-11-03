/* 
 *  Controller of the filament dryer for Raspberry Pi Pico based in RP2040.
 */
 
#include <Wire.h>
#include <EncoderButton.h>

#include <Adafruit_SSD1306.h>
#include "TempSensors.h"
#include "HeaterController.h"

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

int               set_time;
uint8_t           menu_sel = 0;
uint8_t           refresh_display = 10;

TempSensors       sensors(SAMPLE_TIMEOUT_100MS);
HeaterController  heater;
Adafruit_SSD1306  display(SCREEN_WIDTH, SCREEN_HEIGHT, &OLED_WIRE, OLED_RESET); 
EncoderButton     *eb; 

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
  display.println(set_time);

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
  display.clearDisplay();             // Clear the display everytime
  display.setTextSize(2);             // Setting the text size and color         
  display.setTextColor(WHITE);             
  display.setCursor(0,0);             // Setting the cursor position

  display.print(sensors.box_celcius(), 1);        
  display.print("/");
  display.print(heater.get_setpoint());
  display.print(" ");
  display.print((char)247);            
  display.print("C");

  display.setCursor(0,22);
  display.print("Bed "); 
  display.print(sensors.bed_left_celcius(), 0);
  display.print("-");
  display.print(sensors.bed_right_celcius(), 0);
  display.print((char)247);

  display.setCursor(0,42); 
  if (heater.get_mode() == MODE_RUN_TUNE) {
    display.print("Tune: ");

    display.print(heater.tuning_percentage(), 1);
  } else {
    display.print("H: ");
    display.print(sensors.box_humidity(), 1);  //                                          
  }
  display.print(" %");
  
  display.display();                 // The display takes effect
}

/**
 * A function to handle the 'clicked' event
 */
void on_click(EncoderButton& eb) {

  if (menu_sel++ < 3) {
    display_menu();
  } else {
    menu_sel = 0;
    
    refresh_display = 10;
    if (heater.get_mode() != MODE_RUN_TUNE) {
      if (heater.get_setpoint() > 0) {
        heater.set_mode(MODE_RUN_PID);
      } else {
        heater.set_mode(MODE_STOP);
      }
    }
  }
}

/**
 * A function to handle the 'encoder' event
 */
void on_encoder(EncoderButton& eb) {
int new_val;

  if (menu_sel > 0) {
    if (menu_sel == 1) {
      heater.inc_setpoint(eb.increment());
    } else if( menu_sel == 2) {
      new_val = set_time + eb.increment();
      if ((new_val >= 0) && (new_val <= 48)) {
        set_time = new_val;
      } 
    } else if( menu_sel == 3) {
      heater.set_mode((eb.increment() > 0) ? MODE_RUN_TUNE : MODE_STOP);
    }
  
    display_menu();
  }
}

void plot_pid(float input, float output, float setpoint) {
  if (Serial) {  
    Serial.print(F("Setpoint:"));  Serial.print(setpoint);  Serial.print(F(", "));
    Serial.print(F("Input:"));     Serial.print(input);     Serial.print(F(", "));
    Serial.print(F("Output:"));    Serial.print(output);    Serial.print(F(","));
    Serial.println();
  }
}


/*
 * Show introduction message when USB serial port is connected.
 */
void show_intro_msg(void) {
static bool do_it = false;
  
  if (Serial) {
    if (!do_it) {
      Serial.println("Filament dryer controller.");
      Serial.print("Kp: "); Serial.print(heater.pid_const.kp());
      Serial.print(" Ki: "); Serial.print(heater.pid_const.ki());
      Serial.print(" Kd: "); Serial.println(heater.pid_const.kd());
      do_it = true;
    }
  } else {
    do_it = false;
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

  // Update sensor values every 100 mS.
  if (sensors.update()) {
    refresh_display++;
  }
    
  /*
   * Shows status information, when not in setup mode
   */
  if ((menu_sel == 0) && (refresh_display >= 5)) {
    refresh_display = 0;
    update_display_info();
  }
  
  float pwm_val = heater.update(sensors.box_celcius(), 
                                sensors.bed_left_celcius(),
                                sensors.bed_right_celcius());

  /*
   * Record the input, output, and setpoint values of the PID and tuner, 
   * to use the Arduino Plotter to plot the system response.
   * Note: The output is converted to a percentage.
   */
  if (refresh_display == 0) { 
    plot_pid(sensors.box_celcius(), (pwm_val/255.0f)*100, heater.get_setpoint()); 
  }
}
