/**
 * Stores PID constants in EEPROM.  
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
#include "PIDConst.h"

PIDConst::PIDConst(float default_kp, float default_ki, float default_kd){
  _kp = default_kp;
  _ki = default_ki;
  _kd = default_kd;
}

bool PIDConst::begin(void){
  EEPROM.begin(256);
  
  if (read_magic() == CONST_INITED){
      read_kp();
      read_ki();
      read_kd();
  
      return true;
  } 
  
  return false;
}

int PIDConst::read_magic(void) {
  EEPROM.get(ADDRESS_MAGIC_NUM, magic_number);
  return magic_number;
}

void PIDConst::write_magic(int val){
  EEPROM.put(ADDRESS_MAGIC_NUM, val);
}

float PIDConst::kp(void){
  return _kp;
}

float PIDConst::ki(void){
  return _ki;
}

float PIDConst::kd(void){
  return _kd;
}

float PIDConst::read_kp(void){
  EEPROM.get(ADDRESS_KP, _kp);
  return _kp;
}

float PIDConst::read_ki(void){
  EEPROM.get(ADDRESS_KI, _ki);
  return _ki;
}

float PIDConst::read_kd(void){
  EEPROM.get(ADDRESS_KD, _kd);
  return _kd;
}

void PIDConst::store(float kp, float ki, float kd){
  EEPROM.put(ADDRESS_KP, kp);
  EEPROM.put(ADDRESS_KI, ki);
  EEPROM.put(ADDRESS_KD, kd);
  
  if (magic_number != CONST_INITED) {
      write_magic(CONST_INITED);
  }

  _kp = kp;
  _ki = ki;
  _kd = kd;
  
  EEPROM.commit();
}
