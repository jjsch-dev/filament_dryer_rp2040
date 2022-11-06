/**
 * Encapsulates the temperature and humidity sensors.  
 * (C) Juan Schiavoni 2022
 *
 * One read at a time is performed to optimize the time the CPU 
 * hangs while performing the read.
 */ 
#pragma once

#include "Arduino.h"
#include <HTU21D.h>
#include <thermistor.h>

#define SHT_WIRE              Wire
#define SHT_SDA_PIN           0
#define SHT_SCL_PIN           1
#define HTU21D_ADDR           0x40

#define BED_LEFT_THERM_PIN    A0
#define BED_RIGHT_THERM_PIN   A1
#define SMPS_MODE_PIN         23  // GPIO23

class TempSensors  
{
public:
  TempSensors(unsigned long timeout_ms);
  ~TempSensors() {};

  bool begin();
  bool update();
  float box_celcius(void);
  float box_humidity(void);
  float bed_left_celcius(void);
  float bed_right_celcius(void);

private:
  HTU21D          sht;
  thermistor      bed_left_therm;   // Connect A0 to the 3950 temperature sensor. 
  thermistor      bed_right_therm;  // Connect A1 to the 3950 temperature sensor. 

  float           box_temp;  
  float           humidity;
  float           bed_left_temp;
  float           bed_right_temp;
   
  unsigned long   last_sample;
  unsigned long   sample_timeout;
  int             sensor_id;
};
