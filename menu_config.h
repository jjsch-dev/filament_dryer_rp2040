/**
 * Setup menu definition.
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

#include "UserInterface.h"

#define MNU_EXIT_ID                 0
#define MNU_TURN_ENABLE_ID          1
#define MNU_BOX_TEMP_ID             2
#define MNU_REMAINING_TIME_ID       3
#define MNU_TUNE_ENABLE_ID          4
#define MNU_THERMISTORS_ID          5
#define MNU_CALIB_THERMISTORS_ID    6
#define MNU_HEATER_TEMP_ID          7
#define MNU_ODOM_MODE_ID            8
#define MNU_ODOM_MINUTES_ID         9
#define MNU_ODOM_DIAMETER_ID        10
#define MNU_ODOM_TURNS_ID           11
#define MNU_MOISTURE_OPEN_ID        12
#define MNU_MOISTURE_CLOSE_ID       13
#define MNU_KP_ID                   14
#define MNU_KI_ID                   15
#define MNU_KD_ID                   16
#define MNU_FACTORY_RESET_ID        17
#define MNU_FIRMWARE_VERSION_ID     18

menu_item_t menu_list[] = {
  {"Exit",    MENU_MODE_EXIT, 80},
  {"Turn:",   MENU_MODE_EDIT, 80},  
  {"Temp:",   MENU_MODE_EDIT, 80}, 
  {"Time:",   MENU_MODE_EDIT, 80}, 
  {"Tune:",   MENU_MODE_EDIT, 80},
  {"Therm:",  MENU_MODE_EDIT, 90},
  {" cali:",  MENU_MODE_EDIT, 90},
  {"Heat:",   MENU_MODE_INFO, 80},
  {"Odom:",   MENU_MODE_EDIT, 80},
  {" off:",   MENU_MODE_EDIT, 90},
  {" dia:",   MENU_MODE_EDIT, 80},
  {" t:",     MENU_MODE_EDIT, 50},
  {"Open:",   MENU_MODE_EDIT, 80},
  {"Close:",  MENU_MODE_EDIT, 91},
  {"Kp:",     MENU_MODE_INFO, 50},
  {"Ki:",     MENU_MODE_INFO, 50},
  {"Kd:",     MENU_MODE_INFO, 50},
  {"Frst:",   MENU_MODE_EDIT, 80},
  {"V:",      MENU_MODE_INFO, 50}
};
