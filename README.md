# 3D Filament Dryer
Heats and dries wet filaments to improve print quality and longevity.
 
The purpose of this project is to build a dehumidifier with Arduino modules such as humidity and temperature sensors, mosfets and rep rap printer heatbeds.

Due to limitations in the RAM memory of the Arduino Nano, the Raspberry Pi Pico was chosen, which also increases the flash to 2 MBytes.

![alt text](images/prototype-front.png)

![alt text](images/prototype-up.png)

Assembly Video
--------------
As the equipment has many parts, an assembly manual in this instance of the project becomes slow to produce, so we decided from Fusion 360 to generate a video where, in addition to seeing the dehumidifier explode, the assembly steps, screws and magnets are shown.

> All the magnets are placed under pressure (the holes have almost no play) and they must also be glued with cyanoacrylate so that the temperature does not loosen them.

> The cap handle and optical sensor bracket screws are parkers, the rest are M2x5 or M3x10/12.

https://user-images.githubusercontent.com/55675185/213236978-26854887-6648-46aa-a964-80c38ac183d2.mp4

Temperature control
-------------------
The temperature control module is based on the [QuickPID](https://github.com/Dlloydev/QuickPID) library which has an advanced anti-windup mode which prevents deep saturation and reduces overshoot.

The task of calculating the PID constants is based on the same author's [sTune](https://github.com/Dlloydev/sTune) library which uses an open-loop PID auto-tuner using a novel s-curve kneepoint test method.

Main Sensor
-----------
The main sensor of the equipment that is used to obtain the humidity and temperature of the box, is the [SHT21](https://sensirion.com/products/catalog/SHT21/) from Sensirion that can be obtained as a mounted module, for example, from [Adafruit](https://www.adafruit.com/product/1899). 

The [HTU21D](https://github.com/devxplained/HTU21D-Sensor-Library) library is used to obtain temperature compensated humidity through one of the I2C ports.

![alt text](images/temp_humidity_sensor.png)

Protection Sensor
-----------------
As the box is printed in Petg, the heater should not exceed 80 degrees, that's why a [100 KHOM at 25ºC and a B of 3950K ±1% thermistor](https://a.aliexpress.com/_m0eyg30) is installed in each heated bed, and to read the temperature the [thermistor](https://github.com/miguel5612/ThermistorLibrary) library is used, which is a port of the firmware reprap.

![alt text](images/thermistor_mounted.png)

Since there is no Arduino module for the thermistors, the divider resistor and filter capacitor are installed on the berry pico board as shown in the picture.

![alt text](images/thermistor_berry.png)

> Note: For now the thermistor library does not include the pull request that adds compatibility with arduinopico, so use this [link](https://github.com/jjsch-dev/ThermistorLibrary) to download the modified one.

User Interfase
--------------
The user interface is based on a 0.96-inch OLED screen that displays text and uses the [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306) library through an I2C interface.

![alt text](images/oled_display.png)

For user input, we use a rotary encoder and the [EncoderButton](https://github.com/Stutchbury/EncoderButton) library, which depends on the [Encoder Library](https://www.pjrc.com/teensy/td_libs_Encoder.html) and the [EncoderLibrary Bounce2](https://github.com/thomasfredericks/Bounce2) libraries.

![alt text](images/rotary_encoder.png)

> Note: For now the Encoder library does not include the pull request that adds compatibility with arduinopico, so use this [link](https://github.com/jjsch-dev/Encoder) to download the modified one.

Heater
------
To heat the box, two 3D printer beds are used as heating elements.

![alt text](images/heat_bed.png)

And two [15A mofsets](https://a.aliexpress.com/_mNwI6G6) that regulate the power with PWM channels.

![alt text](images/mosfet.png)

To homogenize the environment of the box, a [4010 fan cooler](https://a.aliexpress.com/_ms9WyQe) is used that for now turns on at 100%.

![alt text](images/fan_cooler.png)

Moisture Ventilation
--------------------
So that the moisture from the filament that evaporates when heating the box goes outside, a ventilation valve is incorporated that is activated by an [Servo SG92](https://www.adafruit.com/product/169).

![alt text](images/servo.png)

To control it, the internal [Servo](https://github.com/earlephilhower/arduino-pico/tree/master/libraries/Servo) library of the arduinopico porting is used, which is based on the PIO of the Rasperry Pi Pico RP2040.

- The valve is made up of three parts: a rectangular hole in the box cover with a 45-degree chamfer to increase the contact surface, the valve that has a 0.2 mm offset, and a post that, in addition to being the stop, supports the magnet that brings the door to the closed position. To fasten it, two M2 * 5 screws are used.

![alt text](images/valve_parts.png)

![alt text](images/valve_cap.png)

- When the servo arm moves down and stops pushing on the valve, the magnets attract each other and create a force of approximately 300 grams to close the vent.

![alt text](images/valve_close.png)

- To open the valve, the servo moves the arm approximately 45 degrees, it should be noted that the servo makes the most effort at the beginning, and the plastic gear version can make a torque of 1.5 kg per centimeter.

![alt text](images/valve_open.png)

- The servo arm is a combined part between 3D printing and injection that is sold with the kit. This is because in addition to the fact that the shaft rack is difficult to print, heat easily warps common printed plastics.

![alt text](images/valve_servo.png)

Install the arm on the servo
----------------------------

![alt text](images/arm_servo_parts.png)

- Once the electronics of the equipment have been assembled and the firmware downloaded, connect it to the USB and select turn off the equipment in the menu so that the servo turns to the closed position.

![alt text](images/arm_servo_off.png)

- Insert the lever that comes with the servo kit more or less at 95° with respect to the vertical of the drawer as shown in the figure. 

![alt text](images/arm_servo_abs.png)

- Insert the PETG printed lever and use the long screw that comes with the kit to attach the parts to the servo shaft.

![alt text](images/arm_servo_close.png)

- Go to the menu and turn on the equipment.

![alt text](images/arm_servo_on.png)

- The arm should go up as the image shows.

![alt text](images/arm_servo_open.png)

- To adjust the positions use the menu (close/open) to modify the opening and closing angles, pay special attention that in the **open** position the valve has play and does not touch the magnet post, and in the **close** position it does not touch the valve so the servo doesn't jam hard and get hot. 

![alt text](images/arm_servo_adjust.png)

Odometer
--------
One of the axes where the filament spool rests has a 6-position encoder so that an optical [Sensor TCRT5000 Infrared Reflection](https://www.aliexpress.com/i/1005004306354385.html) detects its movement.

![alt text](images/optical_sensor_back.png)

![alt text](images/optical_sensor_front.png)


One of the functions is to turn on the equipment when it detects that the reel has started to rotate and also to turn off the equipment when it no longer rotates in a certain time.

It also counts the number of turns that the spool has given since the last reset, transforming the pulses into turns by setting the diameter of the spool.

Spool rollers
-------------
The rollers where the spool rests are made up of two parts: a threaded bar M8 * 95mm that gives resistance to deformation by temperature and an ABS cover that positions them. In order for the spool to rotate as smoothly as possible, the axis is supported by two 608 ZZ bearings.

![alt text](images/spool_rollers_parts.png)

The two ABS covers are different: the used for the optical encoder has three parts, two of a light color (silver or white) and one in black so that the contrast is as high as possible. The second cover is formed by two equal pieces and the color is indistinct.

![alt text](images/spool_rollers_assembly.png)

>The ABS covers are threaded so that they are taut when adjusted.

Odometer sensitivity adjustment
-------------------------------
In order for the TTL output (D0) of the odometer sensor to change when the roller is moved (from white to black and vice versa), the comparator sensitivity must be adjusted with the potentiometer.

- Use an allen key to unscrew the cap on the back of the box and insert a screwdriver until it contacts the potentiometer.

![alt text](images/unscrew_odometer_cap.png)

- Turn the potentiometer until the sensor status indicator turns off.

![alt text](images/odometer_sensor_off.png)

- Rotate the roller by hand until it turns back on, repeat this operation until you are sure the sensor detects transitions from white to black.

![alt text](images/odometer_sensor_on.png)

Go to the setup menu to verify that the turns counter (t) increases as the reel rotates.

![alt text](images/odometer_turns.png)

Menu
----
The configuration menu maintains a simplified operation from the time when the Arduino Nano was used, which has little RAM and FLASH memory, this is not the case with the Berry.

When the device is in information mode, it displays three lines of data. 

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

Assembly of electronics version one
![alt text](images/prototype-electronic.png)

Assembly of electronics version two
![alt text](images/prototype-electronic_v2.png)

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

The male pneumatic connector can be rotated 180 degrees so that the outlet is horizontal or vertical.

![alt text](images/pneumatic_connector.png)

Click on the link to have a [3D view](https://myhub.autodesk360.com/ue2d9d88e/g/shares/SH35dfcQT936092f0e437288384b4c88dbbf) of the equipment model developed by Manuel Vela.

Bill of Materials
-----------------
The following is a complete list of materials including 3D printed parts, electronic components, fasteners, and miscellaneous.

<div class="ritz grid-container" dir="ltr"><table class="waffle" cellspacing="0" cellpadding="0"><thead><tr><th class="row-header freezebar-origin-ltr"></th><th id="0C0" style="width:30px;" class="column-headers-background">A</th><th id="0C1" style="width:220px;" class="column-headers-background">B</th><th id="0C2" style="width:43px;" class="column-headers-background">C</th><th id="0C3" style="width:51px;" class="column-headers-background">D</th><th id="0C4" style="width:270px;" class="column-headers-background">E</th></tr></thead><tbody><tr style="height: 20px"><th id="0R0" style="height: 20px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 20px">1</div></th><td class="s0" dir="ltr">Ítem</td><td class="s0" dir="ltr">Description</td><td class="s0" dir="ltr">U/M</td><td class="s0" dir="ltr">QTY</td><td class="s0" dir="ltr">Reference img</td></tr><tr style="height: 99px"><th id="0R1" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">2</div></th><td class="s1" dir="ltr">1</td><td class="s2" dir="ltr">Raspberry Pi Pico Rp2040</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/8TmmMiYkgfd8EWHlEeb-BA3BY8eFT15meI2zd19_7CN-Rbii0NZlwT1Ptn-kU58tumHTVaokofeGljJGc4WVsJ3eAXrhY_IrpshNvn9Eb86-cUcogoE0pwlz3j-di-Xa0YTWPhw6PoZpWL001KtO0my8CfUcF5bAgzVmFfBqKrly9KqgeZBDibovRfjr9g=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R2" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">3</div></th><td class="s1" dir="ltr">2</td><td class="s2" dir="ltr">Si7021 I2c humidity and temperature sensor</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/26egQA14dQzAI3gAcaHfTGphs1BnINHA8hl6v41Up4m79PnlENhPl7e2T7ihCbFiOPGdnUmFAvRj67ZlnVKsxNQVhPJt9sUP4AJeYDS8lfY7cvpoUQoWC7lt6XZ3C_zfuiVkYwa3Lceo9Kk7tLr0_lXEOAEVzSdBIZqyl1PvyI2GqwBGjfBEebkIVip8Iw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R3" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">4</div></th><td class="s1" dir="ltr">3</td><td class="s2" dir="ltr">Oled display 0.96 blue 128x64 I2c Sh1106 Gm009605v4</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/_rw8tOkJOXXNRShEtWJGKpGDT7c94sC36zpaain8qMd1xfjsBhTlshDIZmQWgCf7lxTzFtQWjraOpG304AsJXATn-64oX7xvsCERSmgyT9bVdtnMFAkvKQxnfyZ-w9Gq2VzZOtHrM6BCkvixZ3MmQ9hJNfpjl_qlo4oFysmTCLOIC6AqamB-r7HCFIDsZw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R4" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">5</div></th><td class="s1" dir="ltr">4</td><td class="s2" dir="ltr">Power supply step-down DC-DC 1.5 A 26v 3a Mp1584 regulator</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/YqKlPFha9a_vy7mO856sHYgWrJDgFemt7QaI5ndGzKDMg-78gIwwOQmtsaYWMY6e22TRTu5U26uvfqf8KxhEsIqh880U7PoxxnBm0mTT9uhdgLPJEpQkOGiK7b8B2p6mmzgpx7GpjjKsnHu5zhDJKuQEsiuu58nXn7VmUlEmQp_66hr7cZ_rQak2NaGeAg=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R5" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">6</div></th><td class="s1" dir="ltr">5</td><td class="s2" dir="ltr">Axial fan cooler 40 x 40 mm 12V</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/GpMK-6TUnzcsrhL_EVr1dWkMhNTwsrR9L87eWCNnL_R7l8KUvo0ngt726AiwgoPFAevFw1r2zx6A9-dgRAcVz6qcfHy-Adk-mvEV4btVwi5q1Y3x93gFJTvTmqV4bEWL80uS7vCpWXILX9a_v5Ef28R_WftVx0F1NhyR7ue4IesNqqJDxztSpKljD4D1qw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R6" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">7</div></th><td class="s1" dir="ltr">6</td><td class="s2" dir="ltr">Rotative encoder KY-040</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/K5f6SxlMSbvjCM846lJWUM5b62b-hhN0UXPdQN2vL_79KOyQL9bBfHuxag82WppSAu5KWyR1KkeqpCOdueOOgjKeWur5fXZ02cV1eJEFz37vYE7I9Vep7cHT3lNO9ZJihO2nLzxFP0iExyt3Oh0rbcaMh0EPra6P_5bolYBQ54OpAd3MOSk636h0pMeHlw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R7" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">8</div></th><td class="s1" dir="ltr">7</td><td class="s2" dir="ltr">NTC 100K B3950 Thermistors</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh3.googleusercontent.com/S7bDOSs2eqfTgc0V8RNaLU5sXhL_5GHpEqcL38tKzqu_ovBu7sLur0HHBb_6dcCYs_kHjQi98MidkiHh_r6OXOlit5T1HcUUEBMxfbsddwcdntA2HNApOUYkED2Qo-eqMMsxNmRawFtIHzReeWu94tUbylHCAz4wQP3tS0uuOhBNMaeRIFniVVnnx--D5w=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R8" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">9</div></th><td class="s1" dir="ltr">8</td><td class="s2" dir="ltr">Heat bed Mk2b 12v/24v for 3d printer</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/YAhZC1WV42KL2Vt4Mxet06ET4bdwH2AjP9MbPHZDwlDsFIllVgdtJcPArDMwFEak2IjUscXRbsbJcS2lYzEhAq-IihINvj-416zgB08GGzZihmOedgUtBZMJjnK-l3PkEfa-_m9t2RLFkxJyNf2JxY8ms6q56NSpJJvFjZ_Vid8okiNIPBaVCic_X6pdiQ=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R9" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">10</div></th><td class="s1" dir="ltr">9</td><td class="s2" dir="ltr">Quick connect term female FDD1.25-250 0.250&quot; (6.35mm) 16-22 AWG with cap</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">4</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/bc_w114_hWeMtIDPX7IOU0B7rlkYRhigt31JpR7i83YP_7hMsDWhZwt9Pw9i71gLF8VPaRrVlzQ0kBC4q4LwvqTaj8BN9_FOHdCVl3mwrGEj14JRO1za5bynx4Ik6ameDpNrgMnDWR1QhvNHivCngTa1lbrFkjD6psSiH5GsEqif8N4FeEuV1HEJCGUW-A=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R10" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">11</div></th><td class="s1" dir="ltr">10</td><td class="s2" dir="ltr">PWM Mosfet Modul 5-36v 15a 400W</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">3</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/l7z84tBa4ODsX4xnMz8_dD5nu1zoE_8Z_1NVpaj6KLIQKAw3fJ0C3ltpEhmHd0letMGjWHnwNjdMT3BL7NWxADV2HBJUDxq1UKfIQ1nalo0fD9lAEFT_8Xdr7vlYszAwhUoHbNvWRERijKlo6MrYR3d-QYI40gvPSoeRez-gKun8UDYdOV7XTwbOM1D8Rw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R11" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">12</div></th><td class="s1" dir="ltr">11</td><td class="s2" dir="ltr">Infrared sensor Tcrt5000</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/ocxsFJJ83RePvWHlYOwKqHtBXkVOMRiryq7RgMK3aY1ezUHFBBHfEbSJeKZoVMluf1CJGJ7U5ptjLsKnXqdROHAofPd7Mg-poIwcDRukGcDUxPe7HNemkp16MBDW1_YqK4I-DknL6pQ5pMK2_2vV5wHU8T_1KaId5tbaRVwC95NhRlm9ItQNWsUCUZ4Yhw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R12" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">13</div></th><td class="s1" dir="ltr">12</td><td class="s2" dir="ltr">Mini Servo Tower Pro Sg90 9g</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/_FDJRkMUFtBUqkCFlGC-87nmGFqg71xkZJwu6QjsGl_06zrrfibxAH9qtKyyzJ9Ba92tfaY5Qe-jdGWHbfCdNTwsaPH2H7vMoGeG-AGchDqbWGnlAoAkYKrKljMkRYGSwz4guo_N8-XCjx4nFEciF494tkShG50TXtKmP2zIcaIzLnJO17Hi7IgXaDEY2Q=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R13" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">14</div></th><td class="s1" dir="ltr">13</td><td class="s2" dir="ltr">KCD4 heavy duty switch 15a 250v</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/gfdauyjwgOpx3gIJAatY-lCDGTOFzcnAVd6LSS-1PEzvdmN6o0n1Qv6Y1PFUKliXg1mBnBOZAO0pI_loUJfWgu164gYbywxTI2R3pZYNvkYHRKtX6sodeHY4_bhuOXoMsTjuwgq0Y-O9Zdr98VfcSHATdRXBVZze9FWnSiZTyLU-Wf6GPyG0R_OwbrHqyQ=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R14" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">15</div></th><td class="s1" dir="ltr">14</td><td class="s2" dir="ltr">GX16 Aviation Butt-Joint Straight Connector Metal Shell</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/57VEJs4g5rZmRCwsLeBTP9DWtuXE9c4GXlVKA6BxNohP1RlmFQ4d45Gf6BC8vcPOpeXV0tbSzA8E2R84HjxQ02QiSB3rD7-2qHH9AP2QgzWWk_qaM-sL8zJSIi5dfA236cCJVJMXsnyUiBPh3GkCyLLqZKNIT-4AaTOcz2A5a1depxwU1nHH35spBcB4fQ=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R15" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">16</div></th><td class="s1" dir="ltr">15</td><td class="s2" dir="ltr">Interlock switch PP4003 with fuse</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh3.googleusercontent.com/OAn5FsnT1s_mSFh_QFTcKjAkPXD0zbnaLdYFnZ5h6lPG62GRUszdYdb6Wnt56gxxMnUdfLW4LU5WMJTlNIdjgVtRRG5E6Fs2SJqyaijHlprZEd94ENA6Qf0WnBY4QsfakGePONnQFayg5GH-nMqMA6OVM-15ViRNK5aUrSgjU7phU50dTUhOLzk5ef5DwA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R16" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">17</div></th><td class="s1" dir="ltr">16</td><td class="s2" dir="ltr">Dupont 2.54 mm female metalic connector</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">8</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh3.googleusercontent.com/TeuJwu_4hLs8MWMsgotpIUTprw_mL5JlCz1ZAwrtXKW_HWE-UA2dBkeFlEhlEs1kb0Yq3AEA14uyUJ-DkL9q7sXIqx9in_aOnFqOTtNzBRJPWFkZ9CBGQy8dfvWuHCrK_RPV-vVCo4Lfr6vap1GjWOnOk58Ic8XDQY_vT4LYaA7RImBSkF3kuxVFNAlPKw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R17" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">18</div></th><td class="s1" dir="ltr">17</td><td class="s2" dir="ltr">Dupont 2.54 mm female plastic connector 4 ways</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/NH9iqJQZB5fuxWiLnJjZnUa0_NrvWVk2mZApHSkNkaqeJBaObTiQw3zoRRyxfxvAif_qcX6SQfMeKx9UKWHgJ4t_l2t_AIISofNFsgNgsCW7YP8kjHO6339bl4Sd66Mia8A003hiCNpcCK3uxn93qUDtYE94sH99tGEgcFc7sXy3247ICeiZjVGtsY7H_Q=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R18" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">19</div></th><td class="s1" dir="ltr">18</td><td class="s2" dir="ltr">Flat cable 4 Conductors Awg</td><td class="s3" dir="ltr">MTS</td><td class="s4" dir="ltr">0,6</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/oC1F0DdvqvNZd9yNAklsCv__Md-a5RUhBWXg-PkWi1H4ax55jg3UAIfk4nXITSOqI55IrLR8QPvwksF-73d8Wr6K_7JIcbT4mfx7_zl1gGiOGhNLbnP1DY70Lp7clmaajHxlYKz4GWsCEoZZrgGKj479PNU8-SqcXHZoWRys7kyyKbhOrweKRfTtISal_A=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R19" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">20</div></th><td class="s1" dir="ltr">19</td><td class="s2" dir="ltr">16GA speaker wire</td><td class="s3" dir="ltr">MTS</td><td class="s4" dir="ltr">2</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/CzhOEqWsHT89y4FoVsae9gNbnfFQRomlvw6f0Z00dqgTBE8Zc4PwOEid-pKCXHBHN4M69Oa8fPYQFw4GCusG9AknKQyMcKXhDuWelK8qWuje1GoliTgXsxDPmjde_bNKjK33nZD1CpS0W00Gcd690fgzhOWFKbrjrFu21w363SCgb6B3XDQoZVU70_ZiZQ=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R20" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">21</div></th><td class="s1" dir="ltr">20</td><td class="s2" dir="ltr">Bearing 608zz</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">4</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/dLRFc_CMng5iayig85V0_GQz6x8oqjaqij4f__AJdbVFVrynfIBzD9RpwIAti66pXcvPNdVFR0STV81W_jIM2a4svRlIEX5fzr43BOGlbCPFAkH5LjU-zyUdtaLeT5b6uYQT_bwaHC5qysUhzxkGUM9TNOR29RR8JyEmU57zg9r7et9_fhh52r7or0o0nQ=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R21" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">22</div></th><td class="s1" dir="ltr">21</td><td class="s2" dir="ltr">Neumatic joint for hotend M10 (reference: Creality Ender 3)</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh3.googleusercontent.com/_7KyzkiKFJvJWQ3pcQ00mkrNTDfJ6CCPQnoUAvBPLn4qwMlawuMFst58oboTkv7QtqoH9l8OyUNwZ4Oe8Q8PVA9SSlBflt7Mco5BKhGk-DsO_08YB7t7gjinWc51rvmKm0Sgo2ChfYnT8TLGeEO4nXM_kpY0MwkDj-8kG9LXIOL0yrVgRUyC6-qIPA3JKg=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R22" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">23</div></th><td class="s1" dir="ltr">22</td><td class="s2" dir="ltr">Neodymium magnet D5xH3 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">28</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/5WpzifXr-WOXjhegr7-6rqjW_D50KPYOMxaY7SVldT_OnFxmp_hUjzgTRvQTSccoUacw42uVoZ2q5cz5nVXX3kONWMAZphH45hXlTkN_3tT7jscr2xc7zeY8v6LFGbz8rMJ5cqr4cXQaLc4iDEmVd_DLWGMvHk7zYvVcBUPtiAXtlOKuzcp6yq9twUgEfA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R23" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">24</div></th><td class="s1" dir="ltr">23</td><td class="s2" dir="ltr">DIN 912 Allen screw M2 x 0,4 x 5 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">16</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/tL6HJHB0KNW11SUgRdTEfnnudx2RTfzI6AH-MXfmaq9CSNcR1x6n-X_gofMnUr9r5QE0bsDkkt7HNWPCQGPcWleIwKRz4VD6vYYF-TEimpeXW8EcSWkQnBDHP1qq3R7rl-w1P-TKbbcjULnsiaaZYGe3FcLx5bnCQ3HfKlST9eRbsMce8O4PoTaTMbPENA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R24" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">25</div></th><td class="s1" dir="ltr">24</td><td class="s2" dir="ltr">DIN 912 Allen screw M3 x 0,5 x 10 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh3.googleusercontent.com/pe1z9kYJoTRpYXbrztHo0aJEmdJnys006AwkWWJeHE4Nyw0ShrGvHdIMyE3Xq_Y8Y3TxNFia02jfoILYmR1ExGqX13rLTqFMAQsGixf1d0Ag2FRrEPPQKfo9SHb6GXXhObQDSZY1FeERVOdgfVvw8UUkP7wC8HqpRPTNTOyay5wSFiw2Hm8Nc6PiIr1vug=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R25" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">26</div></th><td class="s1" dir="ltr">25</td><td class="s2" dir="ltr">DIN 912 Allen screw M3 x 0,5 x 12 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">5</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh3.googleusercontent.com/6_ITOBIm2ArVT-fEXim_JmoFJ8pvAO3RzcCpZhSsZgEL2v1VSwQe2FlqcTD-QSErFNxztrMzMpxLUZVs6u0ItkQN9DDYI75SRa7dXjdhd0UTV0jUcHvIu45PDjUHT2opE3ZqS3DIrwUgwuEpZCKhbuNDSk9Hz5Bf3lQGhlkU9Mkn6GPJcP-BI4Lp5_Z5Dg=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R26" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">27</div></th><td class="s1" dir="ltr">26</td><td class="s2" dir="ltr">DIN 912 Allen screw M3 x 0,5 x 20 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">5</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/m7vejGnvCMbO8DTzFO3sn6vXk7W8qNz5OmVdPzvI18G0vLZ4s8cOfFKqyoDXzjDulthSarWagT0S9d18HiWOi6Siumq14rzPOUHnmFnaI6iQDjdlIxrrc8-smOpIZdO1lVaihRRV6HK9v3A5pcEixqvRfkGkLrTBX1fgTdprtj-2TEixoUfWaMzthRjUfg=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R27" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">28</div></th><td class="s1" dir="ltr">27</td><td class="s2" dir="ltr">M3 x 0,5 mm Square nut</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">7</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/AXpMNVcXh4EvTR3807fUWAP-rxuZDLikwxgr85WbDxMsA0RulsAn9wgae9OOlcdJ1WMDyBGRdxxCcbzv1hw1kRH74Ro5KWZqusE0ZZwrNfnmr-YtMiEBfMyYtsLtICoF2FbZdyL0AoGFRYYQRmhR-s-C4J5O6lr52jgzf32JJYfr9AfC7Cm8Jsdz8GbP0g=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R28" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">29</div></th><td class="s1" dir="ltr">28</td><td class="s2" dir="ltr">Parker self tapping screw #4 D2,9 mm x L3/8&quot;</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">12</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/bVcwK6MJEXNx94EOpJoA-hN4oDWPjOM4O30LZbn3ByeETKRfZflSENSMSBlwwFYb07AiluM4V5bVn3m5TpajfU39SQsx2E9z95WtSOs05IzvNjHrK7eOA3q6aj3hK-FzYEnFzBFFWXXnVGORK2XXfyQ2c9UuVbN5K1T0iPVKCNnuLhRB4rTSzmBKFDxegA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R29" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">30</div></th><td class="s1" dir="ltr">29</td><td class="s2" dir="ltr">Threaded rod M8 x 1,25 mm, lenght 95 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/jPhSElFNKsO3zhR6rVIBn_wh-02RJV9FIITR9H6UldUljDuNER2_lT2IAVucLrf6_kCmdbj-kE6lm2wK-c024V2zKBhNc-SbThboKgLOHVZd6dJx0Eb4Cddl34U1-EL_vFrP6F0AeGGiKP6oF9bP7bRX21ObEiD4yRjFbIGgBXLxdreb0e-Q1doQOI2MxA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R30" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">31</div></th><td class="s1" dir="ltr">30</td><td class="s2" dir="ltr">dry_box_baseBottom_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/P7i-CsRb3Jzu4oquPRs4Z05mwjOsWNbjx8q97m7DGu8dtzyGS4JppNOnMQSG6WJRH_LxOhAt-I8pbY92xptwy8dYRMaVpx7EAGu99ws1hqZVtHMHbQIy9fIwjkY391wZ7SWL23zzXKRDF_rw3BBm_hoHyNqnJkK-Roh2lnoIe4ZOkwFmsh-t0UnLSOYubA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R31" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">32</div></th><td class="s1" dir="ltr">31</td><td class="s2" dir="ltr">bearing_top_cover_v1.0</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">4</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/4tV19dNmEsOz8kHQNxUa_raQveOX9YinspvigIT6P0d3o-LimB51vRsb4o5sNe2OnedmYYlN0Wvp02AxOjFDITOvkUkfjtr4QZwzcHgLvC4bFcKR69vNxjRBdJN2Y2CdFqHxeuNqhdALd902n8b3t4tr25t7PRXh65votQXaeNFJ1le0vlXkUNk0HqPnmQ=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R32" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">33</div></th><td class="s1" dir="ltr">32</td><td class="s2" dir="ltr">optic_support_v1.0</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/kbukSPHfMAFTJHes6MHVyEZte2Vej0VonGr2gf6bLANmhaj7jjUE5lfBJEOEHpgxeXWhhNO3gyzbt8FLt6mAMY7sP_A3S55P_N9BSF98FtvymqIn7MilDq8nSE__Uyc418IiYply6O-I953ECOkci8kyXaUkXRNkn3wljhyLMXIirrEFgDqtM77dosgN5g=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R33" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">34</div></th><td class="s1" dir="ltr">33</td><td class="s2" dir="ltr">racord_base_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/4ruambzmTiDpV5UC33RSfJzrzDX3H-PyDHK8pE8-_oEcQ8qspmapS7ZAMgypzXUokqkp8MclqGzZ_qi0Pfx65q6Nb1w70cKtORBql7rSZHGFpXafkvX2TP9RsPqqI376cXw_8MDyO-oUWrlrLOZ7-xP_qYXg38R7nVTIXXuuhriFb7UzcJ5ZBuAeIz8GzA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R34" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">35</div></th><td class="s1" dir="ltr">34</td><td class="s2" dir="ltr">special_screw_v1</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/61BGx3YgsP_Sp8KiFqriONd3Yo_MDEC-4sEh1f__bdT43LEoOVCExt3sk4Oaah8Ukm_nJHOcHlu8_wwplWGQfEnB3JlhpCevZvalJNKFidhGaNufwi4EcTTnyZjhBjQqqwr1dtIb-x86nmZMdNrYDgfMu4a0TZ_b951L-8fa5dAic6MW-P4QeUlBWveG4A=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R35" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">36</div></th><td class="s1" dir="ltr">35</td><td class="s2" dir="ltr">handle_base_v1.1</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/dPp3isqRDVgDV7rI4xOcVjFKC5X7t1h32KunT75HgC8nePFWbkeO1ifHUgdEj2FIDpnCr7Y8b5GtfT60h7aUBx--2invE-MwY73dmEiIraRG4PjKoaFWKMK4vU91LFepJQkkJXWPKe9GD1oYz6pM4BbZ69xlQYVjlkhXQ2MNTBVqYc4z5YBZtLCxZ9zxgA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R36" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">37</div></th><td class="s1" dir="ltr">36</td><td class="s2" dir="ltr">handle_v1.1</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/HT9DRsdodF_nNUdv3h22h_k5NR3K1AYa7GqPAfEq5f2n83_ohka-mzRt25vI0scG02Qw1qwR2PdKWnHw4EOi29F4JV1QinjrM7FUJP5ogpotlLOxMCcyC5r6zkaYk2janbsv-DIHnn9t9NS7m2NJxB6l5p1Zi8Kp50Zuqitz6cpCRV9nrjJ81h5gbCkbTg=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R37" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">38</div></th><td class="s1" dir="ltr">37</td><td class="s2" dir="ltr">dry_box_baseTop_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/jj0DwTULIgshxAIlrIauBbAE97GZay2HK_LX7Vhigue1slWMy_D3tbsJbwUMpqfgbOZ7pe2_0pUWsPjBh_LajMYYx6hXl9YkwBPJRszNB7Wh6tqlTsH4vwk-hmZ-BPIBpc0biTEZ4wEPyJvV4pAzPd_Ea3F-Vs5LV-3xBZrolbMqwLSZpqfk1iM7PzHhAw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R38" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">39</div></th><td class="s1" dir="ltr">38</td><td class="s2" dir="ltr">dry_box_top_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh3.googleusercontent.com/t6rULMZcm_pGgbld7nxh4-_V-6WpnQUUnSYM7zQSVnxmvHnFf2pS41CbUrXi6OwmJQAFv6ay8D9knVRD8EFeMAVFIMaKgCp_9zvBD7wWMhk5skud22Db1ZlRnb4tsL0Ydctky9TmCuRlTUGF0Kv_94vj3QIrEFLARgoBhKyeCAajRJk7tFfqNJesosMZJA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R39" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">40</div></th><td class="s1" dir="ltr">39</td><td class="s2" dir="ltr">magnet_gate_base_v1.0</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/sJlt5CiCjD5igBk79RfzoqYB372p6Dn2XQ3Zt0PgS1KAyYhCN4NhFqJSzAfhej8ogKleTe0A4srK4AFIqx9jEZIIQ3zPfNF6ZLEsiT69hrCyvAfYLuDUadmaG7nTmIzU1Pr3cX6Uiqa8mNyMvP4ipgiEFp9LvlYLX_aCO6O0yKxhj7uLQsPLsofd7U2ufQ=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R40" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">41</div></th><td class="s1" dir="ltr">40</td><td class="s2" dir="ltr">arm_servo</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/nv8YSE3hAZc_999_1vKoVkV8wnbsv-PwXrsHZmuuL8W6Dzv_PwBtlKtDGMiLEm5D2rBJCgno-LG4hcukM5HqvzqLgmOlou7LKdZekgsqrxi4yPWxcKnnGpXP1Te3WlTAIHrL8X6eB7J0aVrgTwm9ldbKmOd99IgYQUpCC8w7Qi1RLAyWm1NZoXV3KEOKdA=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R41" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">42</div></th><td class="s1" dir="ltr">41</td><td class="s2" dir="ltr">dry_box_gate_v2.0</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/l7UvceH9fR882d_BE4aUoQpg3NYIQHthdDf64KuC0OXYSuQ0CPP5I6k7cwgquZRJHa6P3VGGLyqpAzoZ-GxdpqKET4pijl3dfO0Qe6Uz1hmWZWBi9YKBBANiPQJoy7Kto_5eFJr7qI-PJAMrdwxJBGooJxf7x4dmoggWR3XU8tXwQdLui-YTAUj3oKVUsg=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R42" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">43</div></th><td class="s1" dir="ltr">42</td><td class="s2" dir="ltr">electronics_case_mb_cover_v1.5</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/X3OBhqADn0GSYD7Osg1VfEEAlO9OahN_o2qjvOhPtV_qqbX-orC8vsnG5QIBYvVzu_trPEKA5ytAPAUyPujqJRxwPqvKXK73RoGRGqI4nLxOs7Et847voc-db-CY7A_kMnRiwfrCpI1a27RdClHCoC0qh74p23vlAJ_zOijn1W3BJn5vkDNUxFLyzsFeAw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R43" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">44</div></th><td class="s1" dir="ltr">43</td><td class="s2" dir="ltr">electronics_case_mb_bottom_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh3.googleusercontent.com/Tfe6rpS-jPlaGP9lziePE_YO-mFo1bLGg-OPJhQUpKTAK63VggmT8U0PoqWHo9vKLYNFrGOYxr4rvvya4rSddLnT1VK266eXakVHt3NuQ02xGaxeomn0gHOrvQ9oSqTn7pII6FX0edcwgqeLjmhSoioTwmQ1GSPrw6KZluG9D0ZPW5vtf1Tw9sNAP-2jpg=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R44" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">45</div></th><td class="s1" dir="ltr">44</td><td class="s2" dir="ltr">stepdown_support_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/Rfytp42lfYYjS-DPZTU0yAk7T32QFXL7r3xpJjsrOlJnsN1CLb4HpJjELSSO2cWjzsjVhKqSpdPIWtRM2cztIQnxoiC80iC2oGWf9g7xtBMHt6QDWL5WyZFocXBBAg8fLtQuvXArgmQM8eoFuFMASmOFRgtUB1YszMuGjx6aktWcZavN0kwLHxIbwu2YbQ=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R45" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">46</div></th><td class="s1" dir="ltr">45</td><td class="s2" dir="ltr">LCD-knob_v2 v2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/CQKJ6F7pQ4iQnjPCYpa2aGtICO0wPUlcr8pYeuja2aMM9y0edMTmG7WSj6YICNpK7BC-j629Q5TI5kHrgdd5DmRSVt81ZG_ZkcAE10rv6osL45E2GumUscUhvHNpZq73b7hCFFvnAuWG-JQoQi9LLtNMVBIDazPOBC7nWenjbqLcWHqEc08IGRaSH6aN4w=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R46" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">47</div></th><td class="s1" dir="ltr">46</td><td class="s2" dir="ltr">axis_cover_short</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/sTXnIywtm76u0BF7R5GSwzXJ-cozxNtiODWHuOHzZPmMydB35X-x_N6-rwQ5pmjmu0ul9eyBJrNOdy9Va_NXz6fnU7G1HW0iBYUGKChWo7B4ztQXhh-qb-7jeF2BOWS89jlXKQ_awsHjdt2JkxJ4EwBWQbJ_kTKFRVUx79PgqxWZTxGzJoM0PdG7UveTsg=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R47" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">48</div></th><td class="s1" dir="ltr">47</td><td class="s2" dir="ltr">axis_cover_joint</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh5.googleusercontent.com/dsD503Io-DwyNLkOG9khSWFTqXgTxHEyj6tMRbh4SdAZVSy7a0vMRjKqpfbCTUUefOJvClpblJeyeamPI1ADFdg68yUEVkQgsP4NCw5yFz6YmOWmi0I0Mjt0SLalk1u_BRuIL0LLkoh9Hr4XezjVNl-ltahhH21tV2BGBOlC7zjDUSCnJ85AM6dJQXMyDw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R48" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">49</div></th><td class="s1" dir="ltr">48</td><td class="s2" dir="ltr">axis_cover_long</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh6.googleusercontent.com/6GX0_ClzSxoMl07BkOSlD_3WrYzwFPHAoJ_6vcNE488cllT2Cwo2J6EptnMMWl5z2lyj2v2qGjyedTk7Zw8wQG2GFUIvwsqLc1bh4d9yr43gW7NeBvoziFBNHs5lXTl3oRRxNftcbKqAggJ72ggbXL-A9o6GxopagFbDQ-43WPF87Kw1jbbr1j3dvkSe0w=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr><tr style="height: 99px"><th id="0R49" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">50</div></th><td class="s1" dir="ltr">49</td><td class="s2" dir="ltr">axis_cover_nut</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td><td class="s5"><div style="width:270px;height:99px;"><img src="https://lh4.googleusercontent.com/uuTD1wX2cSlpurkD4LdPISIk9i-aHDgxYBPOCyglfvyRJpjvygHw_a_Yn8pusg5CK4nE681QwDv40y3vrEWFMcWogf_vsrCP5aGwKCGPF4Czy7w2SMiVSPi2-4vu-e19iT1Xrs-sEoFUo_JV-1x_Pvnri9pxT8owU8E537PLA0pulnRvNpa7nCnoC0_rEw=w270-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td></tr></tbody></table></div>
