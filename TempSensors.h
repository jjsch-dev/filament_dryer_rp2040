/**
 * Encapsulates the temperature and humidity sensors.  
 * One read at a time is performed to optimize the time the CPU 
 * hangs while performing the read.
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
#pragma once

#include "Arduino.h"
#include <HTU21D.h>
#include <thermistor.h>

#include "ParamStorage.h"

#define SHT_WIRE              Wire
#define SHT_SDA_PIN           0
#define SHT_SCL_PIN           1
#define HTU21D_ADDR           0x40

#define BED_LEFT_THERM_PIN    A0
#define BED_RIGHT_THERM_PIN   A1
#define SMPS_MODE_PIN         23  // GPIO23

#define THERMS_DEFAULT        2   // By default, the left and right thermistors are enabled.

class TempSensors  
{
public:
  TempSensors(unsigned long timeout_ms, ParamStorage& storage);
  ~TempSensors() {};

  bool begin();
  bool update();
  float box_celcius(void);
  float box_humidity(void);
  float bed_left_celcius(void);
  float bed_right_celcius(void);

  void set_therms(int count);
  int get_therms(void);
  
private:
  HTU21D          sht;
  thermistor      bed_left_therm;   // Connect A0 to the 3950 temperature sensor. 
  thermistor      bed_right_therm;  // Connect A1 to the 3950 temperature sensor. 
  ParamStorage&   pstorage;
  
  float           box_temp;  
  float           humidity;
  float           bed_left_temp;
  float           bed_right_temp;
   
  unsigned long   last_sample;
  unsigned long   sample_timeout;
  int             sensor_id;
};
