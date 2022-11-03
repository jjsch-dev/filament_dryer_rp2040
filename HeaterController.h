#pragma once
#ifndef HEATER_CONTROLLER_H_
#define HEATER_CONTROLLER_H_

#include "Arduino.h"

#include <PID_v1.h>
#include <sTune.h>

#include "PIDConst.h"

#define MODE_STOP               0
#define MODE_RUN_PID            1
#define MODE_RUN_TUNE           2

#define KP_DEFAULT              29.576  
#define KI_DEFAULT              0.056    
#define KD_DEFAULT              0.222   

#define ST_DISABLED             0
#define ST_INITIALICE           1
#define ST_RUN_PID              2
#define ST_RUN_TUNE             3
#define ST_WAIT_BED_TEMP_DROP   4

#define HOT_BED_LEFT_PIN        10      // Hot left bed controller pin by PWM (490 Hz)
#define HOT_BED_RIGHT_PIN       11      // Hot right bed controller pin by PWM (490 Hz).
#define FAN_PIN                 12      // Fan controller pin by PWM (980 Hz).

#define OUT_ON                  255     // MAX PWM duty cicle  
#define OUT_OFF                 0       // MIN PWM duty cicle

#define BED_MAX_TEMP            80.00

class HeaterController 
{
public:
    HeaterController();
    ~HeaterController() {};

    bool begin();
    float update(float box_temp, float bed_left_temp, float bed_right_temp);
    void inc_setpoint(int count);
    int get_setpoint(void);
    int get_mode(void);
    void set_mode(int mode);
    int tuning_percentage(void );

    PIDConst    pid_const;
    
private:
    PID         pid;  
    sTune       tuner; 
    
    int   _mode;
    int   pid_status;
    int   tune_status;
    
    double pid_input;
    double pid_output;
    double pid_setpoint;

private:

    uint32_t  tune_settle_time_sec;
    uint32_t  tune_test_time_sec;     // runPid interval = testTimeSec / samples
    uint16_t  tune_samples;
    float     tune_input_span;
    float     tune_output_span;
    float     tune_output_start;
    float     tune_output_step;
    float     tune_temp_limit;
    uint8_t   tune_debounce;
    uint16_t  tune_samples_count;

    
    float     tune_input;
    float     tune_output;
    float     tune_setpoint;

private:
    float pid_controller(float input, float bed_left_temp, float bed_right_temp);
    float tune_controller(float input);
    void  pwm(int output);
    void  fan_cooler(int output);
};

#endif
