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
#include "Arduino.h"
#include <EEPROM.h>
#include "ParamStorage.h"

ParamStorage::ParamStorage(float kp, float ki, float kd, int therms, 
                           int setpoint, int hours, int odom_mode, 
                           int odom_minutes, int odom_turns,
                           int odom_diameter) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _therms = therms;
  _setpoint = setpoint;
  _time = hours;
  _odom_mode = odom_mode;
  _odom_minutes = odom_minutes;
  _odom_turns = odom_turns;
  _odom_diameter = odom_diameter;
}

bool ParamStorage::begin(void) {
  EEPROM.begin(256);
  
  if (read_magic() == CONST_INITED){
    read_kp();
    read_ki();
    read_kd();
    read_therms();
    read_setpoint();
    read_time();
    read_odom_mode();
    read_odom_minutes();
    read_odom_turns();
    read_odom_diameter();
    return true;
  } 
  
  return false;
}

int ParamStorage::read_magic(void) {
  EEPROM.get(ADDRESS_MAGIC_NUM, magic_number);
  return magic_number;
}

void ParamStorage::write_magic(int val){
  magic_number = val;
  EEPROM.put(ADDRESS_MAGIC_NUM, val);
}

float ParamStorage::kp(void) {
  return _kp;
}

float ParamStorage::ki(void) {
  return _ki;
}

float ParamStorage::kd(void) {
  return _kd;
}

int ParamStorage::therms(void) {
  return _therms;
}

int ParamStorage::setpoint(void) {
  return _setpoint;
}

int ParamStorage::get_time(void) {
  return _time;
}

int ParamStorage::odom_mode(void) {
  return _odom_mode;
}

int ParamStorage::odom_minutes(void) {
  return _odom_minutes;
}

int ParamStorage::odom_turns(void) {
  return _odom_turns;
}

int ParamStorage::odom_diameter(void) {
  return _odom_diameter;
}

float ParamStorage::read_kp(void) {
  EEPROM.get(ADDRESS_KP, _kp);
  return _kp;
}

float ParamStorage::read_ki(void) {
  EEPROM.get(ADDRESS_KI, _ki);
  return _ki;
}

float ParamStorage::read_kd(void) {
  EEPROM.get(ADDRESS_KD, _kd);
  return _kd;
}

int ParamStorage::read_therms(void) {
  EEPROM.get(ADDRESS_THERMS, _therms);
  return _therms;
}

int ParamStorage::read_setpoint(void) {
  EEPROM.get(ADDRESS_SETPOINT, _setpoint);
  return _setpoint;
}

int ParamStorage::read_time(void) {
  EEPROM.get(ADDRESS_TIME, _time);
  return _time;
}

int ParamStorage::read_odom_mode(void) {
  EEPROM.get(ADDRESS_ODOM_MODE, _odom_mode);
  return _odom_mode;
}

int ParamStorage::read_odom_minutes(void) {
  EEPROM.get(ADDRESS_ODOM_MINUTES, _odom_minutes);
  return _odom_minutes;
}

int ParamStorage::read_odom_turns(void) {
  EEPROM.get(ADDRESS_ODOM_TURNS, _odom_turns);
  return _odom_turns;
}

int ParamStorage::read_odom_diameter(void) {
  EEPROM.get(ADDRESS_ODOM_DIAMETER, _odom_diameter);
  return _odom_diameter;
}
  
void ParamStorage::write_pid_const(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  EEPROM.put(ADDRESS_KP, _kp);
  EEPROM.put(ADDRESS_KI, _ki);
  EEPROM.put(ADDRESS_KD, _kd);
}

void ParamStorage::write_therms(int therms) {
  _therms = therms;
  EEPROM.put(ADDRESS_THERMS, _therms);
}

void ParamStorage::write_setpoint(int temp) {
  _setpoint = temp;
  EEPROM.put(ADDRESS_SETPOINT, _setpoint);
}

void ParamStorage::write_time(int hours) {
  _time = hours;
  EEPROM.put(ADDRESS_TIME, _time);
}

void ParamStorage::write_odom_mode(int mode) {
  _odom_mode = mode;
  EEPROM.put(ADDRESS_ODOM_MODE, _odom_mode);
}

void ParamStorage::write_odom_minutes(int minutes) {
  _odom_minutes = minutes;
  EEPROM.put(ADDRESS_ODOM_MINUTES, _odom_minutes);
}

void ParamStorage::write_odom_turns(int turns) {
  _odom_turns = turns;
  EEPROM.put(ADDRESS_ODOM_TURNS, _odom_turns);
}

void ParamStorage::write_odom_diameter(int diameter) {
  _odom_diameter = diameter;
  EEPROM.put(ADDRESS_ODOM_DIAMETER, _odom_diameter);
}

/*
 * Since the eeprom is a simulated module in the RP2040 on Arduino, 
 * the library's put only write the RAM buffer allocated with begin. 
 * And when commit is called, the flash is updated by stopping the IRQs.
 */
void ParamStorage::save(void){
  if (magic_number != CONST_INITED) {
    write_magic(CONST_INITED);
  }
  
  EEPROM.commit();
}
