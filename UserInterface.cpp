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
#include "UserInterface.h"

static UserInterface* p_ui;

static void wrapper_on_click(EncoderButton& eb) {
  p_ui->on_click();
}

static void wrapper_on_encoder(EncoderButton& eb) {
  p_ui->on_encoder(eb.increment()); 
}

UserInterface::UserInterface(menu_item_t* m_list, int m_size) :
               display(SCREEN_WIDTH, SCREEN_HEIGHT, &OLED_WIRE, OLED_RESET) {
  menu_list = m_list; 
  item_count= m_size / sizeof(menu_item_t);
  menu_end  = item_count - 1;
  menu_mode = 0;
  menu_sel  = 0;
  
  p_ui = this;
}

int UserInterface::update() {
  eb->update();

  return menu_mode;
}

bool UserInterface::begin(callback_get_t c_get, callback_set_t c_set,
                          callback_end_edit_t c_end_edit, callback_exit_t c_exit) {
  item_get  = c_get;
  item_set  = c_set;
  end_edit  = c_end_edit;
  menu_exit = c_exit;
  
  /*
   * With a global instance of the object, the constructor is called before the Arduino 
   * core is initialized, and the attachInterrupt() function does not work. 
   * That's why many object libraries have a '.begin()' member function. 
   * You can also use new to create the object in the config function.
   */
  eb = new EncoderButton(ENCONDER_DT_PIN, ENCONDER_CLK_PIN, ENCONDER_BUTTON_PIN);
  eb->setPressedHandler(wrapper_on_click);
  eb->setEncoderHandler(wrapper_on_encoder);
  eb->setDebounceInterval(10);
  
  OLED_WIRE.setSDA(OLED_SDA_PIN);
  OLED_WIRE.setSCL(OLED_SCL_PIN);
   
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR); //Start the OLED display
  display.clearDisplay();
  display.display();

  return true;
}

/*
 * Shows the configuration menu.
 */
void UserInterface::display_menu() {
int y_pos = 0;
int menu_start = menu_sel - (DISPLAY_LINES - 1); 
char buff[ITEM_CAPTION_LEN];

  if (menu_start < 0) {
    menu_start = 0;   
  }
  
  display.clearDisplay();            
  display.setTextSize(2);
  
  for (int x=0; x<DISPLAY_LINES; x++) {
    if ((menu_mode == MENU_MODE_EDIT) && ((x + menu_start) == menu_sel)) {
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(15, y_pos);
    display.println(menu_list[x + menu_start].text);

    int type = menu_list[x + menu_start].type;
    if ((type == MENU_MODE_EDIT) || (type == MENU_MODE_INFO)) {
      display.setCursor(menu_list[x + menu_start].info_x, y_pos);
      display.setTextColor(WHITE);
    
      display.println(item_get(buff, x + menu_start));
    }
    
    y_pos += 20;
  }
  
  display.setTextColor(WHITE);
  display.setCursor(2, (menu_sel - menu_start) * 20);
  display.println(">");

  display.display();
}

/**
 * Processes the click event.
 * To finish the configuration, with the encoder select the exit menu and press click.
 * To modify a parameter, select it with the encoder, click to edit, then use the 
 * encoder to change the value.
 */
void UserInterface::on_click(void) {

  if (menu_mode == MENU_MODE_INFO) {
    menu_mode = MENU_MODE_SEL;
    menu_sel = 0;
  } else if (menu_mode == MENU_MODE_SEL) {
    if (menu_list[menu_sel].type == MENU_MODE_EXIT) {
      menu_exit(menu_sel);
      menu_mode = MENU_MODE_INFO;   
    } else if (menu_list[menu_sel].type == MENU_MODE_EDIT) {
      menu_mode = MENU_MODE_EDIT;
    }
  } else if (menu_mode == MENU_MODE_EDIT) {
    end_edit(menu_sel);
    menu_mode = MENU_MODE_SEL;
  } 
  
  display_menu();

  click_count++;
}

/**
 * Procces the 'encoder' event
 */
void UserInterface::on_encoder(int increment) {
int new_val;

  if (menu_mode != MENU_MODE_INFO) {
    if (menu_mode == MENU_MODE_EDIT) {
      item_set(increment, menu_sel);
    } else if (menu_mode == MENU_MODE_SEL) {
      uint8_t new_val = menu_sel + increment;

      if ((new_val >= 0) && (new_val <= menu_end) ) {
        menu_sel = new_val;
      }
    }
      
    display_menu();
  }
}

int UserInterface::clicks(void) {
  return click_count; 
}
