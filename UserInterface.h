/**
 * User interface for OLED and Rotary enconder.  
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
#include <Wire.h>

#include <EncoderButton.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH          128       // OLED display width, in pixels
#define SCREEN_HEIGHT         64        // OLED display height, in pixels
#define OLED_RESET            -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_ADDR             0x3C

#define OLED_WIRE             Wire1
#define OLED_SDA_PIN          2
#define OLED_SCL_PIN          3

#define ENCONDER_CLK_PIN      6
#define ENCONDER_DT_PIN       7
#define ENCONDER_BUTTON_PIN   8

#define MENU_MODE_INFO        0
#define MENU_MODE_SEL         1
#define MENU_MODE_EDIT        2
#define MENU_MODE_EXIT        3

#define ITEM_CAPTION_LEN      16
#define DISPLAY_LINES         3

#define UI_EXIT_TIMEOUT       60000     // User idle time to exit the menu in mS.
#define UI_UPDATE_TIMEOUT     1000      // Update the menu info when expire.

typedef char* (*callback_get_t)(char* value, int item_id);
typedef bool (*callback_set_t)(int value, int id);
typedef void (*callback_end_edit_t)(int id);
typedef void (*callback_exit_t)(int id);

const typedef struct menu_item_t {
	char text[ITEM_CAPTION_LEN];
	int type;
  int info_x;
} menu_item;

class UserInterface 
{
public:
  UserInterface(menu_item_t* m_list, size_t m_size);
  ~UserInterface() {};

  int update();
  bool begin(callback_get_t c_get, callback_set_t c_set, 
             callback_end_edit_t c_end_edit, callback_exit_t c_exit);
  void on_click(void);
  void on_encoder(int increment);
  int clicks(void);
  
  Adafruit_SSD1306  display;   

private:
  EncoderButton     *eb; 
  menu_item_t       *menu_list;
  int               item_count; 
  int               menu_sel;
  int               menu_mode;
  int               menu_end; 
  int               click_count = 0;
  unsigned long     exit_timeout;
  unsigned long     update_timeout;
  
  void reset_timeout(void); 
  
private:  
  callback_get_t        item_get;
  callback_set_t        item_set;
  callback_end_edit_t   end_edit;
  callback_exit_t       menu_exit;
  
  void display_menu();
};
