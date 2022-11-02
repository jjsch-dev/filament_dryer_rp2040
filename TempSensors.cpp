#include "TempSensors.h"

TempSensors::TempSensors(unsigned long timeout_ms) : 
             sht(HTU21D_ADDR, SHT_WIRE),
             bed_left_therm(BED_LEFT_THERM_PIN, 0),
             bed_right_therm(BED_RIGHT_THERM_PIN, 0) {
    
    sample_timeout = timeout_ms;
    last_sample = millis() + sample_timeout + 1;
    
    box_temp = 0;  
    humidity = 0;
    bed_left_temp = 0;
    bed_right_temp = 0; 
    sensor_id = 0;
}
 
bool TempSensors::begin() {

    SHT_WIRE.setSDA(SHT_SDA_PIN);
    SHT_WIRE.setSCL(SHT_SCL_PIN);
  
    sht.begin(); // include "Wire.begin();" that start the i²c communication

    /* 
     * The resolution of the pico nano processor ADC is fixed at 12 bits (4096), 
     * but the tables for calculating temperatures are made for 10-bit steps (1024), 
     * so you need to set the resolution to 10 bits.
     */
    analogReadResolution(10);

    /* 
     * Raspberry Pi Pico 
     * Driving high the SMPS mode pin (GPIO23), to force the power supply into PWM mode, 
     * can greatly reduce the inherent ripple of the SMPS at light load, and therefore the 
     * ripple on the ADC supply. 
     */
    pinMode(SMPS_MODE_PIN, OUTPUT);
    digitalWrite(SMPS_MODE_PIN, HIGH);
  
    return true;
}

bool TempSensors::update() {
unsigned long now = millis();    
    /*
     * To prevent the reading of the humidity and temperature sensor (60 mS delay) 
     * from interfering with the debounce timer of the encoder button (10 mS), 
     * one is performed every 100 mS.
     */
    if (now > (last_sample + sample_timeout) ) {
        last_sample = now;

        /*
         * Alternate the bed temperature reading to improve the stability.
         */
        switch(sensor_id++) {
            case 0:
                if (sht.measure()) {
                  box_temp = sht.getTemperature();
                  humidity = sht.getHumidity();
                }
            break;
            
            case 1:
                bed_left_temp = bed_left_therm.analog2temp(); 
            break;
            
            case 2:
                bed_right_temp = bed_right_therm.analog2temp(); 
                sensor_id = 0;
            break;
            
            default:
                sensor_id = 0;
        }

        return true;
    }
    
    return false;   
}

float TempSensors::box_celcius(void) {
    return box_temp;  
}

float TempSensors::box_humidity(void) {
    return humidity;
}

float TempSensors::bed_left_celcius(void) {
    return bed_left_temp;
}

float TempSensors::bed_right_celcius(void) {
    return bed_right_temp; 
}
