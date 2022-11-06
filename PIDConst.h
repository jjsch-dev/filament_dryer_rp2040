/**
 * Stores PID constants in EEPROM.  
 * (C) Juan Schiavoni 2022
 *
 * As the RP2040 does not have EEPROM, picoarduino simulates it with a flash page.
 */ 
#pragma once

#define CONST_INITED        0x434f4e13 //53        // ASCII = CONS

#define ADDRESS_MAGIC_NUM   0
#define ADDRESS_KP          ADDRESS_MAGIC_NUM + sizeof(int)
#define ADDRESS_KI          ADDRESS_KP + sizeof(float)
#define ADDRESS_KD          ADDRESS_KI + sizeof(float) 

class PIDConst  
{
public:
  PIDConst(float default_kp, float default_ki, float default_kd);
  ~PIDConst() {};
  
  bool begin();
  float kp(void);
  float ki(void);
  float kd(void);
  
  float read_kp(void);
  float read_ki(void);
  float read_kd(void);
  
  void store(float kp, float ki, float kd);
private:
  int read_magic(void);
  void write_magic(int val);
  
  float _kp;  
  float _ki;
  float _kd; 
  int magic_number;
};
