# 3D Filament Dryer
Heats and dries wet filaments to improve print quality and longevity.
 
The purpose of this project is to build a dehumidifier with Arduino modules such as humidity and temperature sensors, mosfets and rep rap printer heatbeds.

Due to limitations in the RAM memory of the Arduino Nano, the Raspberry Pi Pico was chosen, which also increases the flash to 2 MBytes.

![alt text](images/prototype-front.png)

![alt text](images/prototype-up.png)

Temperature control
-------------------
The temperature control module is based on the [QuickPID](https://github.com/Dlloydev/QuickPID) library which has an advanced anti-windup mode which prevents deep saturation and reduces overshoot.

The task of calculating the PID constants is based on the same author's [sTune](https://github.com/Dlloydev/sTune) library which uses an open-loop PID auto-tuner using a novel s-curve kneepoint test method.

Main Sensor
-----------
The main sensor of the equipment that is used to obtain the humidity and temperature of the box, is the [SHT21](https://sensirion.com/products/catalog/SHT21/) from Sensirion that can be obtained as a mounted module, for example, from [Adafruit](https://www.adafruit.com/product/1899). 

The [HTU21D](https://github.com/devxplained/HTU21D-Sensor-Library) library is used to obtain temperature compensated humidity through one of the I2C ports.

Protection Sensor
-----------------
As the box is printed in Petg, the heater should not exceed 80 degrees, that's why a [100 KHOM at 25ºC and a B of 3950K ±1% thermistor](https://a.aliexpress.com/_m0eyg30) is installed in each heated bed, and to read the temperature the [thermistor](https://github.com/miguel5612/ThermistorLibrary) library is used, which is a port of the firmware reprap.

Since there is no Arduino module for the thermistors, the divider resistor and filter capacitor are installed on the berry pico board as shown in the picture.

![alt text](images/thermistor_berry.png)

User Interfase
--------------
The user interface is based on a 0.96-inch screen for presentation and a rotary encoder for input.
To control the display via I2C the [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306) library is used, and for the rotary encoder the [EncoderButton]() library, which depends on the [Encoder library](https://www.pjrc.com/teensy/td_libs_Encoder.html) and [Bounce2 library](https://github.com/thomasfredericks/Bounce2) libraries.

The User Interface class uses these libraries to iterate a list of items that make up the configuration menu, and with callback functions (get/set/edit/end edit/exit) it communicates with the application.

Heater
------
To heat the box, two 3D printer beds are used as heating elements with two 15A mofsets that regulate the power with PWM channels.
To homogenize the environment of the box, a 40x40 mm fan cooler is used that for now turns on at 100%.

Moisture Ventilation
--------------------
So that the moisture from the filament that evaporates when heating the box goes outside, a ventilation valve is incorporated that is activated by an [Servo SG92](https://www.adafruit.com/product/169).

To control it, the internal [Servo](https://github.com/earlephilhower/arduino-pico/tree/master/libraries/Servo) library of the arduinopico porting is used, which is based on the PIO of the Rasperry Pi Pico RP2040.

Odometer
--------
One of the axes where the filament spool rests has a 6-position encoder so that an optical [Sensor TCRT5000 Infrared Reflection](https://www.aliexpress.com/i/1005004306354385.html) detects its movement.

One of the functions is to turn on the equipment when it detects that the reel has started to rotate and also to turn off the equipment when it no longer rotates in a certain time.

It also counts the number of turns that the spool has given since the last reset, transforming the pulses into turns by setting the diameter of the spool.

Menu
----
The configuration menu maintains a simplified operation from the time when the Arduino Nano was used, which has little RAM and FLASH memory, this is not the case with the Berry.

When the equipment is in information mode it uses three data lines. 

![alt text](images/display_info.png)

- The first on the left shows the current temperature of the box and on the right the set one (0 = device off).
- The second line shows the remaining operating time.
- The third is the relative humidity of the box.

To enter the configuration menu you have to press the encoder.

![alt text](images/display_enter_cfg.png)

Using the rotary encoder you can explore all the menu items, and when you need to modify a value, you have to press the encoder button, if it is editable the item is marked by inverting the colors.

![alt text](images/display_item_edit.png)

Use the rotary encoder to modify the value of the element, which can be numeric or a list of options. When the rotary encoder button is pressed, in addition to saving the value in the eeprom memory, editing ends.

## Menu List
- **Exit** press the button to exit the menu.
- **Turn** turns the temperature control on/off. 
- **Temp** select the temperature of the box, the range is from 40 to 60 ° Celcius. 0 = disabled.
- **Time** select temperature control run time in hours, range is from 1 to 72 hours. 0 = disabled.
- **Tune** On to allow the device to calculate the PID constants and save them to the eeprom. It is advisable to do it without a spool and starting from room temperature.
- **Therm**  select the number of thermistors that sense the temperature of the heater to prevent it from melting the bed supports (80°Celsius). At least one is recommended.
- **Heat** displays the temperature of the heating element. Use the thermistor that is hotter.
- **Odom** activates the functions of turning on (Start) the heating element when the reel rotates, or turning off (Stop) when it has stopped for a while. (Both) activates both functions.
  - **off** time in minutes that it waits to turn off the heater when the spool stops spinning. 
  - **dia** diameter in millimeters of the spool to convert pulses to turns.
  - **t** number of turns that the reel has given since the last reset of the counter. Move the encoder to the left to reset it.
- **Open** position in degrees of the servo when the vent door is open. Typically 90 degrees.
- **Close** position in degrees of the servo when the vent door is closed. Typically 43 degrees.
- **Kp** proportional constant of the PID controller.
- **Ki** integrative constant of the PID controller.
- **Kd** derivative constant of the PID controller.
- **Frst** On to restore all parameters to factory.
- **V** firmware version. 

Schematic
---------
![alt text](images/filament_dryer_bb.png)

![alt text](images/filament_dryer_schem.png)

![alt text](images/prototype-electronic.png)

Power Supply
------------
The main power supply is 12V 20A, which is responsible for powering the heaters, the fan, and through a swicthing step-down supplying the 5V to the Berry Pico, which in turn uses the internal 3.3V regulator to power the CPU and the rest of the electronics.

The 5V step-down is connected to the berry pico with a shotcky diode so that the board can be simultaneously connected to a PC's USB, for example, to get system response via a serial port.

![alt text](images/power_supply_5v.png)

> Before installing the 5V supply, set the output voltage to 5.5V to compensate for diode drop.

The thermistors are connected to the ADC_REF so that the end of the positive resistive divider matches the reference of the analog-to-digital converter.

To connect the power supply to the equipment, it is recommended to use a 1.5 mm (16AWG) section cable to avoid falls, in this case we use a 70-thread [speaker cable](https://stingerelectronics.com/products/16ga-speaker-wire-white-500-roll). The length should not exceed 2 meters.

If the output voltage can be adjusted, regulate it to 12.5V to compensate for the losses in the connection cable and in the power switch.

![alt text](images/power_supply.png)

Power Switch and Connector
--------------------------
In order for the wiring to support the power consumed by the equipment, a [KCD4 ROCKER SWITCH ON OFF DPST 4 PIN](https://a.aliexpress.com/_mMvt3Wa) and a [4-pin GX Aviation Connector](https://renhotecpro.com/product/gx16-butt-joint-straight-connector-metal-shell) were chosen, to divide into two 10A maximum circuits (one for each heater).

![alt text](images/power_switch.png)

Download the firmware in Raspberry Pi Pico
------------------------------------------
Hold down the BOOTSEL button while plugging the board into USB. The uf2 file [filament_dryer_rp2040.ino.uf2](https://github.com/jjsch-dev/filament_dryer_rp2040/tree/master/bin/filament_dryer_rp2040.ino.uf2) should then be copied to the USB mass storage device that appears. Once programming of the new firmware is complete the device will automatically reset and be ready for use.

![alt text](images/boot_sel.png)

System Response
---------------
The device can plot the system response over USB using the Arduino serial plotter.

The curves below show how humidity drops as time goes by and the temperature rises.

## Reference
- **SetPoint** selected temperature for the box (40 to 60° Celsius).
- **BoxTemp** box temperature.
- **BoxHumidity** relative humidity of the box.
- **PWM** power of the heaters (0 to 100%).
- **BedMax** maximum temperature that heater can reach (80° Celsius).
- **BedTemp** heaters temperature.

60 degree graph
![alt text](images/plot_60.png)

55 degree graph
![alt text](images/plot_55.png)

50 degree graph
![alt text](images/plot_50.png)

45 degree graph
![alt text](images/plot_45.png)

40 degree graph
![alt text](images/plot_40.png)

3D Printed Parts
----------------
Most of the pieces that make up the equipment are manufactured by the FDM method of 3D printing. Because the equipment can reach 60° Celcius, the box is printed in petg that resists between 5 and 10 degrees more than PLA; and the rollers, having to support the weight of the filament spool, use ABS.

![alt text](images/explode_view.png)

The lid that contains the vent valve and the bottle for the silica gel are printed in PLA and the fixing magnets are glued with cyanoacrylate to resist deformation due to the plastic's temperature.

Click on the link to have a [3D view](https://myhub.autodesk360.com/ue2d9d88e/g/shares/SH35dfcQT936092f0e437288384b4c88dbbf) of the equipment model developed by Manuel Vela.
