/* 
 *  Controller of the filament dryer for Raspberry Pi Pico based in RP2040.
 */
 
#include <Wire.h>
#include <EncoderButton.h>

#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <sTune.h>
#include "PIDConst.h"
#include "TempSensors.h"

#define SCREEN_WIDTH          128     // OLED display width, in pixels
#define SCREEN_HEIGHT         64      // OLED display height, in pixels
#define OLED_RESET            -1      // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_ADDR             0x3C

#define OLED_WIRE             Wire1
#define OLED_SDA_PIN          2
#define OLED_SCL_PIN          3

#define HOT_BED_LEFT_PIN      10      // Hot left bed controller pin by PWM (490 Hz)
#define HOT_BED_RIGHT_PIN     11      // Hot right bed controller pin by PWM (490 Hz).
#define FAN_PIN               12      // Fan controller pin by PWM (980 Hz).

#define ENCONDER_CLK_PIN      6
#define ENCONDER_DT_PIN       7
#define ENCONDER_BUTTON_PIN   8

#define BED_MAX_TEMP          80.00
#define SAMPLE_TIMEOUT_100MS  100     // Refresh time for the sensor

#define ST_DISABLED           0
#define ST_INITIALICE         1
#define ST_RUN_PID            2
#define ST_WAIT_BED_TEMP_DROP 3

#define BOX_KP  29.576  //4.08
#define BOX_KI  0.056   //0.02 
#define BOX_KD  0.222   // 0.00

uint8_t           pid_status = ST_DISABLED;
float             box_temp;                     // Here we store the temperature and humidity values
float             box_humidity;
double            bed_left_temp;                // Left bed temperature
double            bed_right_temp;               // Right bed temperature
int               set_temp;
int               set_time;
uint8_t           tune_status = ST_DISABLED;    // True = autotune PID.
uint8_t           menu_sel = 0;

uint32_t          tune_settle_time_sec = 10;
uint32_t          tune_test_time_sec = 500;     // runPid interval = testTimeSec / samples
const uint16_t    tune_samples = 500;
const float       tune_input_span = 70;
const float       tune_output_span = 255;
float             tune_output_start = 0;
float             tune_output_step = 100;
float             tune_temp_limit = 60;
uint8_t           tune_debounce = 1;
int               tune_samples_count = 0;

//Define Variables we'll be connecting to
double pid_setpoint, pid_input, pid_output;
float tune_input, tune_output, tune_setpoint = 50;

PIDConst          pid_const(BOX_KP, BOX_KI, BOX_KD);
TempSensors       sensors(SAMPLE_TIMEOUT_100MS);

Adafruit_SSD1306  display(SCREEN_WIDTH, SCREEN_HEIGHT, &OLED_WIRE, OLED_RESET); 
EncoderButton     *eb; 
PID               pid_temp(&pid_input, &pid_output, &pid_setpoint, BOX_KP, BOX_KI, BOX_KD, DIRECT); 
sTune tuner =     sTune(&tune_input, &tune_output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);

uint8_t           refresh_display = 10;
bool              limit_max_bed = false;

/*
 * Shows the static configuration menu.
 */
void static_menu() {
  display.clearDisplay();            // Clear the display everytime
  display.setTextColor(WHITE);
  display.setTextSize(2);

  display.setCursor(15, 0);
  display.println("Temp:");
  display.setCursor(80, 0);
  display.println(set_temp);

  display.setCursor(15, 20);
  display.println("Time:");
  display.setCursor(80, 20);
  display.println(set_time);

  display.setCursor(15, 40);
  display.println("Tune:");
  display.setCursor(80, 40);
  if (tune_status == ST_DISABLED) {
    display.println("OFF");
  } else {
    display.println("ON");
  }
  display.setCursor(2, (menu_sel - 1) * 20);
  display.println(">");

  display.display();
}

/**
 * A function to handle the 'clicked' event
 */
void on_click(EncoderButton& eb) {

  if (menu_sel++ < 3) {
    static_menu();
  } else {
    menu_sel = 0;
    
    refresh_display = 10;
    
    pid_status = (set_temp > 0) ? ST_INITIALICE : ST_DISABLED;
  }
}

/**
 * A function to handle the 'encoder' event
 */
void on_encoder(EncoderButton& eb) {
int new_val;

  if (menu_sel > 0) {
    if (menu_sel == 1) {
      new_val = set_temp + eb.increment();
      if ((new_val >= 0) && (new_val <= 60)) {
        set_temp = new_val;
      } 
    } else if( menu_sel == 2) {
      new_val = set_time + eb.increment();
      if ((new_val >= 0) && (new_val <= 48)) {
        set_time = new_val;
      } 
    } else if( menu_sel == 3) {
      tune_status = (eb.increment() > 0) ? ST_INITIALICE : ST_DISABLED;
    }
  
    static_menu();
  }
}

uint8_t plotCount = 0;

void plotter(float input, float output, float setpoint) {
 /*Serial.print(F("Setpoint:"));  Serial.print(setpoint);  Serial.print(F(", "));
 Serial.print(F("Input:"));     Serial.print(input);     Serial.print(F(", "));
 Serial.print(F("Output:"));    Serial.print(output);    Serial.print(F(","));
 Serial.println();
*/}

/**
 * 
 */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Filament dryer controller.");

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

  pid_temp.SetSampleTime(100);

  sensors.begin();
  
  tuner.Configure(tune_input_span, tune_output_span, tune_output_start, 
                  tune_output_step, tune_test_time_sec, tune_settle_time_sec, tune_samples);
  tuner.SetEmergencyStop(tune_temp_limit);

  if (pid_const.begin()) {
    pid_temp.SetTunings(pid_const.kp(), pid_const.ki(), pid_const.kd());  
  }
}

/**
 * Use the Arduino PWM function to control the cooler fan in ON/OFF mode.
 */
void cooler_control(boolean on){
  analogWrite(FAN_PIN, (on ? 255 :0));
}

float tune_controller(float input) {
float output = 0;

  Serial.println(tune_status);
  
  if (tune_status == ST_INITIALICE) {
    pid_status =  ST_DISABLED;
    pid_temp.SetMode(MANUAL);
    pid_output = 0;   
    cooler_control(true); 
    set_temp = 0;
    set_time = 0;
    //tuner.Reset();
    tune_samples_count = 0;
    tune_status = ST_RUN_PID;
  } else if(tune_status == ST_RUN_PID) {
    uint8_t state = tuner.Run();
    Serial.print("state: "); Serial.println(state);
    switch (state) {
      case tuner.sample: // active once per sample during test
        tune_input = input;
        output = tune_output;
        tune_samples_count++;
      break;
  
      case tuner.tunings: // active just once when sTune is done
        float kp, ki, kd;
        tuner.GetAutoTunings(&kp, &ki, &kd); // sketch variables updated by sTune

        pid_const.store(kp, ki, kd);
        
        pid_temp.SetTunings(kp, ki, kd); // update PID with the new tunings
        tuner.printTunings();
        pid_status = ST_DISABLED;
        cooler_control(false);
      break;
    } 
  }
  
  return output;
}

float pid_controller(float input) {
  pid_input = input;

  switch (pid_status) {
    case ST_DISABLED:
      pid_temp.SetMode(MANUAL);
      cooler_control(false);
    break;
    case ST_INITIALICE:
      pid_temp.SetMode(MANUAL);
      pid_output = 0;
    
      pid_status = ST_RUN_PID; 
      pid_setpoint = set_temp;
      
      pid_temp.SetMode(AUTOMATIC); 
      cooler_control(true);
    break;
    case ST_RUN_PID:
      if ( max(bed_left_temp, bed_right_temp) > BED_MAX_TEMP ) {
        pid_temp.SetMode(MANUAL);  
        pid_output = 0;
        pid_status = ST_WAIT_BED_TEMP_DROP;
      }
    break;
    case ST_WAIT_BED_TEMP_DROP: 
      if ( max(bed_left_temp, bed_right_temp) < (BED_MAX_TEMP - 5) ) {
        pid_status = ST_RUN_PID;
        pid_temp.SetMode(AUTOMATIC);  
      }
    break;
    default:
      set_temp = 0;
      pid_status = ST_DISABLED;
    break;
  }

  pid_temp.Compute();

  return (pid_status != ST_DISABLED) ? pid_output : 0;
}

void loop() {
int pwm_val;
  
  eb->update();

  if (sensors.update()) {
    box_temp = sensors.box_celcius();
    box_humidity = sensors.box_humidity();
    bed_left_temp = sensors.bed_left_celcius(); 
    bed_right_temp = sensors.bed_right_celcius(); 
    refresh_display++;
  }
     
  /*
   * Shows status information, when not in setup mode
   */
  if ((menu_sel == 0) && (refresh_display >= 5)) {
    refresh_display = 0;
    
    display.clearDisplay();             // Clear the display everytime
    display.setTextSize(2);             // Setting the text size and color         
    display.setTextColor(WHITE);             
    display.setCursor(0,0);             // Setting the cursor position
    display.print("T: ");               // Display the temperature and humidity as "T: 24.6/60 
    display.print(box_temp, 1);         //                                          H: 59.1 %
    display.print("/");
    display.print(set_temp);
    //display.print(" C");              //                                          23.5 23.4"
    display.setCursor(0,22); 
    //display.print("B");
    display.print(bed_left_temp, 1);
    display.print(" ");
    display.print(bed_right_temp, 1);
    //display.print(" C");

    
    display.setCursor(0,42); 
    if (tune_status == ST_DISABLED){
      display.print("H: ");
      display.print(box_humidity, 1);
      display.print(" %");
    } else {
      display.print("Tune: ");
     
      float total = tune_samples_count/tune_samples;
      total *= 100;
      display.print(total, 1);
    }
    
    display.display();                 // The display takes effect
  }

  if (tune_status == ST_DISABLED) {
    pwm_val = pid_controller(box_temp);
  } else {
    pwm_val = tune_controller(box_temp);
  }
  
  analogWrite(HOT_BED_LEFT_PIN, pwm_val);
  analogWrite(HOT_BED_RIGHT_PIN, pwm_val);

  if (refresh_display == 0) { 
    plotter(box_temp, (pwm_val/255.0f)*100, pid_setpoint); 
  }
}
