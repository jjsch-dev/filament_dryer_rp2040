/**
 * Stores PID constants and another params in EEPROM.  
 * As the RP2040 does not have EEPROM, picoarduino simulates it with a flash page.
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

#define CONST_INITED          0x434f4e13 //53        // ASCII = CONS

#define ADDRESS_MAGIC_NUM     0
#define ADDRESS_KP            ADDRESS_MAGIC_NUM + sizeof(int)
#define ADDRESS_KI            ADDRESS_KP + sizeof(float)
#define ADDRESS_KD            ADDRESS_KI + sizeof(float) 
#define ADDRESS_THERMS        ADDRESS_KD + sizeof(float) 
#define ADDRESS_SETPOINT      ADDRESS_THERMS + sizeof(int)
#define ADDRESS_TIME          ADDRESS_SETPOINT + sizeof(int)
#define ADDRESS_ODOM_MODE     ADDRESS_TIME + sizeof(int)
#define ADDRESS_ODOM_MINUTES  ADDRESS_ODOM_MODE + sizeof(int)
#define ADDRESS_ODOM_TURNS    ADDRESS_ODOM_MINUTES + sizeof(int)

class ParamStorage  
{
public:
  ParamStorage(float kp, float ki, float kd, int therms, 
               int setpoint, int hours, int odom_mode, 
               int odom_minutes, int odom_turns);
  ~ParamStorage() {};
  
  bool begin();
  float kp(void);
  float ki(void);
  float kd(void);
  int   therms(void);
  int   setpoint(void);
  int   get_time(void);
  int   odom_mode(void);
  int   odom_minutes(void);
  int   odom_turns(void);
  
  float read_kp(void);
  float read_ki(void);
  float read_kd(void);
  int   read_therms(void);
  int   read_setpoint(void);
  int   read_time(void);
  int   read_odom_mode(void);
  int   read_odom_minutes(void);
  int   read_odom_turns(void);
  
  void write_pid_const(float kp, float ki, float kd);
  void write_therms(int therms);
  void write_setpoint(int temp);
  void write_time(int hours);
  void write_odom_mode(int mode);
  void write_odom_minutes(int minutes);
  void write_odom_turns(int turns);
  void save(void);
  
private:
  int read_magic(void);
  void write_magic(int val);
  
  float _kp;  
  float _ki;
  float _kd; 
  int _therms;
  int _setpoint;
  int _time;
  int _odom_mode;
  int _odom_minutes;
  int _odom_turns;
  int magic_number;
};
