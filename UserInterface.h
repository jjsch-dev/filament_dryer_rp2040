/**
 * User interface for OLED and Rotary enconder.  
 * (C) Juan Schiavoni 2022
 *
 * Use the time.h library to parse the elapsed seconds.
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

typedef char* (*callback_get_t)(char* value, int item_id);
typedef bool (*callback_set_t)(int value, int id, int item_type);

typedef void (*callback_enconder_t)(EncoderButton &);

const typedef struct menu_item_t {
	char text[ITEM_CAPTION_LEN];
	int type;
  int info_x;
} menu_item;

class UserInterface 
{
public:
  UserInterface(menu_item_t* m_list, int m_size, callback_get_t c_get, callback_set_t c_set);
  ~UserInterface() {};

  int update();
  bool begin();
  void on_click(void);
  void on_encoder(int increment);
  int clicks(void);
  
  Adafruit_SSD1306  display;   

private:
  EncoderButton     *eb; 
  menu_item_t*      menu_list;
  int               item_count; 
  int               menu_sel;
  int               menu_mode;
  int               menu_end; 
  int               click_count = 0;
  
  callback_get_t    item_get;
  callback_set_t    item_set;
  
  void display_menu();
};
