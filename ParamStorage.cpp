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

ParamStorage::ParamStorage(float kp, float ki, float kd, int therms){
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _therms = therms;
}

bool ParamStorage::begin(void){
  EEPROM.begin(256);
  
  if (read_magic() == CONST_INITED){
      read_kp();
      read_ki();
      read_kd();
      read_therms();
      
      return true;
  } 
  
  return false;
}

int ParamStorage::read_magic(void) {
  EEPROM.get(ADDRESS_MAGIC_NUM, magic_number);
  return magic_number;
}

void ParamStorage::write_magic(int val){
  EEPROM.put(ADDRESS_MAGIC_NUM, val);
}

float ParamStorage::kp(void){
  return _kp;
}

float ParamStorage::ki(void){
  return _ki;
}

float ParamStorage::kd(void){
  return _kd;
}

int ParamStorage::therms(void) {
  return _therms;
}

float ParamStorage::read_kp(void){
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

void ParamStorage::write_pid_const(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void ParamStorage::write_therms(int therms) {
  _therms = therms;

}
void ParamStorage::save(void){
  EEPROM.put(ADDRESS_KP, _kp);
  EEPROM.put(ADDRESS_KI, _ki);
  EEPROM.put(ADDRESS_KD, _kd);
  EEPROM.put(ADDRESS_THERMS, _therms);
  
  if (magic_number != CONST_INITED) {
    write_magic(CONST_INITED);
  }
  
  EEPROM.commit();
}
