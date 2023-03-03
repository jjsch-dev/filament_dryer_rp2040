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

Thermistors for Over Temperature Protection
-------------------------------------------
The heaters can reach very high temperatures that can damage the system or cause a fire. As the box is printed in Petg a protection system was implemented that turns off the heaters when a temperature above a 80 degrees is detected. The system uses two thermistors of [100 KHOM at 25ºC and a B of 3950K ±1% thermistor](https://a.aliexpress.com/_m0eyg30) to measure the temperature of each heater. 
Thermistors are resistive sensors that vary their value according to temperature. However, due to the deviation of the associated hardware (4k7 resistance and reference voltage of the analog-digital converter), it is necessary to calibrate the thermistors to improve their accuracy. The calibration is done using as reference the temperature of the SHT21 sensor that measures the temperature of the thermal box and has an accuracy of 0.5%. The calibration consists of calculating a factor that is applied to the [thermistor](https://github.com/miguel5612/ThermistorLibrary) library readings to correct their value. This factor is stored in the Arduino’s EEPROM memory for use in future measurements. The calibration can be done from the Calib menu on the LCD when temperatures are stabilized.

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
  - **cali** calibrate the thermistors to improve their accuracy. It is important to calibrate them when temperatures are stabilized (12 hs OFF) to get better results. 
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

<div class="ritz grid-container" dir="ltr"><table class="waffle no-grid" cellspacing="0" cellpadding="0">
<thead>
</thead>
<tbody><tr style="height: 20px"><th id="0R0" style="height: 20px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 20px">1</div></th><td class="s0" dir="ltr">Image</td><td class="s0" dir="ltr">Description</td><td class="s0" dir="ltr">U/M</td><td class="s0" dir="ltr">QTY</td></tr><tr style="height: 99px"><th id="0R1" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">2</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/aJBvWFpAYIPUcupg_vlcCDMWfxYHxMmgwQYzMHhnn4hTZSQLyCumj5t4_bxedT4lihSjKqDjVQnfYaZNxWbjOW-SCGSxnUNzavPSIOAAp6pUJdgkfwJxHHn-BwVeba3Sfx04X1FD7YM_KalVuJEFoYImOjPpC6dzoGkDNTLaOYXoDcoAsFdL6ir9TMG6XQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Raspberry Pi Pico Rp2040</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R2" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">3</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/26egQA14dQzAI3gAcaHfTGphs1BnINHA8hl6v41Up4m79PnlENhPl7e2T7ihCbFiOPGdnUmFAvRj67ZlnVKsxNQVhPJt9sUP4AJeYDS8lfY7cvpoUQoWC7lt6XZ3C_zfuiVkYwa3Lceo9Kk7tLr0_lXEOAEVzSdBIZqyl1PvyI2GqwBGjfBEebkIVip8Iw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Si7021 I2c humidity and temperature sensor</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R3" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">4</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/PXq9YypkHZoFe4zWh7mnJcxNa9wr86WK5Fy5sLB8Knq4W_SPKHWJxdJ5wjXvVmcZsZRE3xtFwYHDfkTogvXNZmO-UCMZri_VSRaJX0LjdRpYHqO8txDaRm-1fwmuJ0-9mgNc2oFKvr1cFVfI37dFiJYVxrziZrSsUXYDAPv5wtnEJ8uDu4nZAfYvnoxoXg=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Oled display 0.96 blue 128x64 I2c Sh1106 Gm009605v4</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R4" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">5</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/YqKlPFha9a_vy7mO856sHYgWrJDgFemt7QaI5ndGzKDMg-78gIwwOQmtsaYWMY6e22TRTu5U26uvfqf8KxhEsIqh880U7PoxxnBm0mTT9uhdgLPJEpQkOGiK7b8B2p6mmzgpx7GpjjKsnHu5zhDJKuQEsiuu58nXn7VmUlEmQp_66hr7cZ_rQak2NaGeAg=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Power supply step-down DC-DC 1.5 A 26v 3a Mp1584 regulator</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R5" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">6</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/YTnPrC74wzGC_a7ThJrKcYGan_yHVkq67JXDGS2Xm2eOziRaD6tiOg12wRnCGh5lwlUQLBHAv0_pkXNBzKkC_-fu8QsY4r1OcZ373KPAjg1ldIXum7p6vo-D-ZtB1oyzqKzMNbTTmVQOTXOUo73lMLG7X0P1CIo8b-kGGdiL9HIP_27qqyXN3If0-0O2GA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Axial fan cooler 40 x 40 mm 12V</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R6" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">7</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/K5f6SxlMSbvjCM846lJWUM5b62b-hhN0UXPdQN2vL_79KOyQL9bBfHuxag82WppSAu5KWyR1KkeqpCOdueOOgjKeWur5fXZ02cV1eJEFz37vYE7I9Vep7cHT3lNO9ZJihO2nLzxFP0iExyt3Oh0rbcaMh0EPra6P_5bolYBQ54OpAd3MOSk636h0pMeHlw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Rotative encoder KY-040</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R7" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">8</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/E9ygdrSBNmXIXfptD1Ptu7Z_mg7SVSii1HJIxLfejOYdb5UCmtGFhXgJGr5u4LIGvDUzVZo1Tpq3vqRGgaJQDkXKn0eFv6_7ZeZMM8qXuTykwvaWS6ZfH7haZmMCJ3sSWJAI7gfO4Nxp9h620yRrtkvkuRUPDcSi568F0z6dyZ51mbIzaGaH7ROtKsl5EQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">NTC 100K B3950 Thermistors</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td></tr><tr style="height: 99px"><th id="0R8" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">9</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/YAhZC1WV42KL2Vt4Mxet06ET4bdwH2AjP9MbPHZDwlDsFIllVgdtJcPArDMwFEak2IjUscXRbsbJcS2lYzEhAq-IihINvj-416zgB08GGzZihmOedgUtBZMJjnK-l3PkEfa-_m9t2RLFkxJyNf2JxY8ms6q56NSpJJvFjZ_Vid8okiNIPBaVCic_X6pdiQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Heat bed Mk2b 12v/24v for 3d printer</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td></tr><tr style="height: 99px"><th id="0R9" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">10</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/5og4UhpKdK84bzN38SySCcriq4lbLHwOHB8GorTaEDYhAGATzovRBSihZScg_52NVpSujorUR3DcClHG9-0T8hz2GI7xjtjgOE3xQ6j9gcls_EQvHzpZAGWmq5z4-cs4IT4fa6f9HRp18RrObWSDECnyNR4BOey1TYRYjmDPbDjjQ3xfECN2klX8X1-abw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Quick connect term female FDD1.25-250 0.250&quot; (6.35mm) 16-22 AWG with cap</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">4</td></tr><tr style="height: 99px"><th id="0R10" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">11</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/t4zN4QXC_fFzNduPv7cSput88pG-v9sEocLrFEJmA1j0wl-xhVndho7XB3nWER5y7ifLhcG2REpMkz_5giDbbD6CMtfDpD8cGF2278uSpRksYM9U2jSbPHvP0qG8cpHjxnsTk7gX4K-du7IegpyKmGATPMMtqqUR1b4sbrxfVKIAhleLCMXNaU8n_Zn-Lg=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">PWM Mosfet Modul 5-36v 15a 400W</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">3</td></tr><tr style="height: 99px"><th id="0R11" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">12</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/H-ampSzS4d5AINp7aro8NH0WR24_nAvtaUs_hZEPasZ1Vc0G9a5M2YeHDquiCJPBWVSJOFzBnnOoRtcsV1ox545Q-tKLt_QtEpP_81quS2pGsbfdb25Y6ZDVPf2qS1YOT9L6uLkyrigp0u6QGYYZlBSatgeyNrJCUrHELdWp3_z4k8kPSscuXTINMMDP7A=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Infrared sensor Tcrt5000</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R12" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">13</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/_FDJRkMUFtBUqkCFlGC-87nmGFqg71xkZJwu6QjsGl_06zrrfibxAH9qtKyyzJ9Ba92tfaY5Qe-jdGWHbfCdNTwsaPH2H7vMoGeG-AGchDqbWGnlAoAkYKrKljMkRYGSwz4guo_N8-XCjx4nFEciF494tkShG50TXtKmP2zIcaIzLnJO17Hi7IgXaDEY2Q=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Mini Servo Tower Pro Sg90 9g</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R13" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">14</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/gfdauyjwgOpx3gIJAatY-lCDGTOFzcnAVd6LSS-1PEzvdmN6o0n1Qv6Y1PFUKliXg1mBnBOZAO0pI_loUJfWgu164gYbywxTI2R3pZYNvkYHRKtX6sodeHY4_bhuOXoMsTjuwgq0Y-O9Zdr98VfcSHATdRXBVZze9FWnSiZTyLU-Wf6GPyG0R_OwbrHqyQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">KCD4 heavy duty switch 15a 250v</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R14" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">15</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/dWvAv09UR5TNm3JbybUbdNhowMfjatfU3NPiemv9O_R09BspIpZKvNjbzdOVlfxucXJTcmApLEHY2nlbuY-Iu0odrh-w5fm6es_pHIgCFGoPlKj0_2SX2QpPn39PjQTMVzkHv9-pl5HcDZzgIAO_lsmj1EBQcu9kTsGeVkdsnMPMmQKtiPDtedBDVKp24g=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">GX16 Aviation Butt-Joint Straight Connector Metal Shell</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R15" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">16</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/_lxx0BqyqVMKTKmSj6-Q1q86DaIWf43zVZ-zZ5tscBr7Y9ylge78mMYxDZLxPuj4UmbkvSwVbBUJlzv9zLFrG_0hCXl98h5oPAIo6qFAnUMXBIhOTUmWU1lNJbY7ia0zb2lX64-GHbXXAtvzZUAqOG1yqiWOkLxWsNY496t0TBoKuOeEcVmwXt1W7zp9jw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Interlock switch PP4003 with fuse</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R16" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">17</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/LZF-1IDONa_MkovUwR8DdSJCXPJx19R1wuJAZAlLRNOwX9SH2F9CP1GfiJAivwfsbAJLVNp-H7tplDlBRX9qeT-TSdIEaMixAopYUPXP4EkKSD8wxC0teas7PXL7aXgbP2_OaWOH94HgpLH6Ej86tmm3yyb56cLKN-bilqV-6B2FBTCMB4di7-5Sd6YNlA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Dupont 2.54 mm female metalic connector</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">8</td></tr><tr style="height: 99px"><th id="0R17" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">18</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/VoX7Dk2UwLPt4KoFV1_jsYj9IxBs1OiS2BqrNs1fAvLKHgb6QvDCUQrdH8MmnxMzhBL_mEfl073hvSRmWnZN2JfMc9WnAiYI16k0XShEW6mAlzUaOUxYyLDbgQO_G1KcfK746-c8B-fJx5dgM1-rg4sWYcXyOZmxux_xGtNO9CzxWs-4WlGLeCPsasCBNA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Dupont 2.54 mm female plastic connector 4 ways</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td></tr><tr style="height: 99px"><th id="0R18" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">19</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/KAOvKU6xtp2Ou8EM__miwM2MrH990R2GFLyfcnPvBxc6mgbWjE8NKmxYipgubUvvwKEcQYnKFaHAwYKJEHqL7D3TM6kgl6nKP3pJ7XitG7iVp_ZTJ_oziaxlOrpaU1UnP2NYyXycklMuGyIZS5K139TeVzMvW4Gfk5PR7rDqqF-hPEyaOXvS6qp1Qc2jzA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Flat cable 4 Conductors Awg</td><td class="s3" dir="ltr">MTS</td><td class="s4" dir="ltr">0,6</td></tr><tr style="height: 99px"><th id="0R19" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">20</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/CzhOEqWsHT89y4FoVsae9gNbnfFQRomlvw6f0Z00dqgTBE8Zc4PwOEid-pKCXHBHN4M69Oa8fPYQFw4GCusG9AknKQyMcKXhDuWelK8qWuje1GoliTgXsxDPmjde_bNKjK33nZD1CpS0W00Gcd690fgzhOWFKbrjrFu21w363SCgb6B3XDQoZVU70_ZiZQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">16GA speaker wire</td><td class="s3" dir="ltr">MTS</td><td class="s4" dir="ltr">2</td></tr><tr style="height: 99px"><th id="0R20" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">21</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/71_eNFhIEPsvuIcJfD9AwchB3YQRupzexbEic4e7smscuYL64TvPV-27oYugi6Ic9UN5dgqMqfDPq8UuU5lMYoVJ3tQOxDsdsxAF1u78nr9H7I29tcsbT73qdJLU6TosE3XNonm_XXuehgCqP-jB_emlo2dhQ8piEgiNNMBVjNcty1wUUNzQP8tYfBfX8w=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Bearing 608zz</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">4</td></tr><tr style="height: 99px"><th id="0R21" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">22</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/GmDCIROP8KwgERw07hLBhXWfq8gRNhgs4fSJGqq4kmGTxlxlieTs3tXuFdnxteOII0N6RSyl8oRYw4p3WgE3ZBXgX0-cNtB_tB0KmA55is6eyW2KciRsdx35gwKpje80r0dK6qiTwSejenmxvstyNZri52LO8mGcq8EDM_lHdFNWJi1fG3ZlBztphhVo0g=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Neumatic joint for hotend M10 (reference: Creality Ender 3)</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R22" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">23</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/PnE-AOLtzBp3IURCpC0sdxFMWm88rTcoW0dFFhQ84Kn85Js4U24t6ekSffHwgh_F_tu6b0biGmzqfBZNthfRDaQ9Athp29F9QJyKDpZmRSqiZdEU_gmtciWIMmHnc5QENuIbTc1nfNkLSsFd3M0T9d81tcwQgqlGQCXhDdqcd256_Iw6JytkzMoOKvfnwg=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Neodymium magnet D5xH3 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">28</td></tr><tr style="height: 99px"><th id="0R23" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">24</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/tKTUo7GwzkEACo2JdbzaK3ssLVJNmSGRNQ0h3fuEfKPc33yy-wvyBOSSSevHpvCLI9K_-cLiSv2Y1KWXU1KfviZ6s0UMXJ8T4m-xAo8aec0Ofm7IqPqqj6TMVAbkOOI76yU1185ulVFbNlE923lc4Wj3JXrOTbqm2HtOovfC7Du379QJUStwZjSbCc_MVQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">DIN 912 Allen screw M2 x 0,4 x 5 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">16</td></tr><tr style="height: 99px"><th id="0R24" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">25</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/Td-UXArGJokVvEQtrCoHaE70pmmCAiq3F8YQ84ETHr3wLfEcxWA7Ja-PMFcywgkkHVFWlXvJ7_hXjIiqQxgFiRaV8EKAYGMHXKm7urfNKHCoq2-EmEUCfegYmgaZdUa_bIBhhDpzLjXkp7kEmA73Rzrki2utQfFvfCmB4k7J5PBlTuPAU3UlQE33oopZqA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">DIN 912 Allen screw M3 x 0,5 x 10 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td></tr><tr style="height: 99px"><th id="0R25" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">26</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/6_ITOBIm2ArVT-fEXim_JmoFJ8pvAO3RzcCpZhSsZgEL2v1VSwQe2FlqcTD-QSErFNxztrMzMpxLUZVs6u0ItkQN9DDYI75SRa7dXjdhd0UTV0jUcHvIu45PDjUHT2opE3ZqS3DIrwUgwuEpZCKhbuNDSk9Hz5Bf3lQGhlkU9Mkn6GPJcP-BI4Lp5_Z5Dg=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">DIN 912 Allen screw M3 x 0,5 x 12 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">5</td></tr><tr style="height: 99px"><th id="0R26" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">27</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/m7vejGnvCMbO8DTzFO3sn6vXk7W8qNz5OmVdPzvI18G0vLZ4s8cOfFKqyoDXzjDulthSarWagT0S9d18HiWOi6Siumq14rzPOUHnmFnaI6iQDjdlIxrrc8-smOpIZdO1lVaihRRV6HK9v3A5pcEixqvRfkGkLrTBX1fgTdprtj-2TEixoUfWaMzthRjUfg=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">DIN 912 Allen screw M3 x 0,5 x 20 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">5</td></tr><tr style="height: 99px"><th id="0R27" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">28</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/LUBkxDJgO2voX3FOnofY5hSCKpZmiskEKZKfG3wp_jTVW7Bn8R9i10wxAJps_tNIKzdD_rJ0WdYMxMLunoizmb9GbUkjVFFHm2ZpHNClYiY3_5KOyjxW9_HYSt-kpcgy6tEDNDELWSLfGfcy4P4i2_pgdPkLQGWr-Hcg6NwxSjDaviZAOYgSlmjRVBC-nw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">M3 x 0,5 mm Square nut</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">7</td></tr><tr style="height: 99px"><th id="0R28" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">29</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/gvk4N8cDkNMi2hSkAQZlMnhBcXK9n9vuqkUfVDYVqAjiio3Sc_19xRTW-jIbiqhC3yePCLg0OYsDQkAxVyirg3GmbvUGjBD7bZ4ugD6o1bWKi9qayJtCwxmnpkuUhHWRncaSQOeYuFeeXfWCZdEI-PIdKr6H0kE2NDRkU7PGzhzNQ1faEycCElKgDfMbpw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Parker self tapping screw #4 D2,9 mm x L3/8&quot;</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">12</td></tr><tr style="height: 99px"><th id="0R29" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">30</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/v0sVLynMx51TpD7krTzuZ18ooDBjzbV7Wy0RSdwNuXGT6mIP8wW5YJYwXuCbWTA7PeQkxnSWhbdTZhQYe7HmnIplihftNCn4rZ80ytU8qdxyfvyhyqeJVOCbaQMbkiC7QBZtf3Q7DMuz6W5SpnIJf4-oKMlnz4lMURSK0ZAXwxPfCG89i0CVI_vby10rLA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">Threaded rod M8 x 1,25 mm, lenght 95 mm</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td></tr><tr style="height: 99px"><th id="0R30" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">31</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/P7i-CsRb3Jzu4oquPRs4Z05mwjOsWNbjx8q97m7DGu8dtzyGS4JppNOnMQSG6WJRH_LxOhAt-I8pbY92xptwy8dYRMaVpx7EAGu99ws1hqZVtHMHbQIy9fIwjkY391wZ7SWL23zzXKRDF_rw3BBm_hoHyNqnJkK-Roh2lnoIe4ZOkwFmsh-t0UnLSOYubA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">dry_box_baseBottom_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R31" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">32</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/4tV19dNmEsOz8kHQNxUa_raQveOX9YinspvigIT6P0d3o-LimB51vRsb4o5sNe2OnedmYYlN0Wvp02AxOjFDITOvkUkfjtr4QZwzcHgLvC4bFcKR69vNxjRBdJN2Y2CdFqHxeuNqhdALd902n8b3t4tr25t7PRXh65votQXaeNFJ1le0vlXkUNk0HqPnmQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">bearing_top_cover_v1.0</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">4</td></tr><tr style="height: 99px"><th id="0R32" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">33</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/Tcwf2il_PoS9zGlWYIH_PkGKeRVObfhtOj9PBUTdJ8wzUwXyNPzEQP9SLnSsUz-RBY_5mY0LMaQRsAJO8SputqZ2TFSoOlIz1hzhPGubNM1c_ANjESrUgk5Bx7sd-cNASj6l1LfV4TDvSDdOa9hDHsG79i4vTkWeweDEiZXJSYSmkztWKWnGBLKkrYwulA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">optic_support_v1.0</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R33" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">34</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/5Eq20TkeBzubBiUlrZKj4O0tOWoenLyuPO_YLuVl1Stm61H-jZn3Df7TtdWqGRo29IymvmlYzlAISixjwXCx1v7KbMoxfXJF6YOh9GdNtqAbWOIqNmFBjqw35tYYXY0QyAUyv56sWg8folSU13t_6Dn8GjTyK_qmBtupYD-0WqVQPlxMPXgp8I3CmHTLFQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">racord_base_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R34" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">35</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/61BGx3YgsP_Sp8KiFqriONd3Yo_MDEC-4sEh1f__bdT43LEoOVCExt3sk4Oaah8Ukm_nJHOcHlu8_wwplWGQfEnB3JlhpCevZvalJNKFidhGaNufwi4EcTTnyZjhBjQqqwr1dtIb-x86nmZMdNrYDgfMu4a0TZ_b951L-8fa5dAic6MW-P4QeUlBWveG4A=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">special_screw_v1</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R35" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">36</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/Cv-ThMHMkXoH_Q-sXzu1X8FMevgbVGehM5m8UH2aGBFlXTZ1CwYJuVfbQLJFhW-EkYGejrxvVDBN2UnsTm9HRQduqSJBtYvEwyvus9dnIjNprKhSH6Yo7Nr47NX9IiSxakb4NMm1nfOVnLlDpf3oB9Cwv6WYLOqXWigziuMh0qFxmw8W1Jl45-iBQgtfTw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">handle_base_v1.1</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R36" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">37</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/c__EBnQFYWC5E1tPpreT6WS29NzvztWSj_NWxMB4tIXxOULqdX-FDClyetrmkp2herfHGYZgfh0AW427L13TMxhMimYv3ybj2kEByJr9-X3nlp2ymhNu6rIBVeV--J3E7qxm-GQgwHvEZ_kqjHzz4ZXqDj2xt5VoinHnwFFCQzqGuhpa_bp-lQc85ioW7w=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">handle_v1.1</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R37" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">38</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/oP2a-xtIVJoz0fDDNSo44OaAHoEFAQ0QmlefT7w8GwzinqXo0AN5CZ6PvZ6q4oa4fUOtZgya5wBQjofv7V3PEG2xQfeBvZ4VCWmWF0wSZCaduDHblk4cszWuKJVRG4cLhcR6mCUHtwXajh0Twqg-KW7npz1L-oA7qu8Jj1D1zh7tlKhhZtpvy8oAKJXhIw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">dry_box_baseTop_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R38" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">39</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/t6rULMZcm_pGgbld7nxh4-_V-6WpnQUUnSYM7zQSVnxmvHnFf2pS41CbUrXi6OwmJQAFv6ay8D9knVRD8EFeMAVFIMaKgCp_9zvBD7wWMhk5skud22Db1ZlRnb4tsL0Ydctky9TmCuRlTUGF0Kv_94vj3QIrEFLARgoBhKyeCAajRJk7tFfqNJesosMZJA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">dry_box_top_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R39" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">40</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/sJlt5CiCjD5igBk79RfzoqYB372p6Dn2XQ3Zt0PgS1KAyYhCN4NhFqJSzAfhej8ogKleTe0A4srK4AFIqx9jEZIIQ3zPfNF6ZLEsiT69hrCyvAfYLuDUadmaG7nTmIzU1Pr3cX6Uiqa8mNyMvP4ipgiEFp9LvlYLX_aCO6O0yKxhj7uLQsPLsofd7U2ufQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">magnet_gate_base_v1.0</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R40" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">41</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/AC0NlWk6VJe9UOy_pOZz40GGWmKMKIeoaoCTfM95KtSD0JwoSeUXPuDzNK7KaxBOjl9Z9u0zqX1dFcAlzXGAlgpdwaP3YTgO_q-o9FHuPhnUTnNM0O9Io5-Ty4u8n9ogbF0Z-DeaNaBvoIdI8IKosIRAaV29i6x8J8O0ptmM4UUxLgZERszRgRJcosMAVA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">arm_servo</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R41" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">42</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/iKwbVaKiq2fkPnyUFD6S0DKlfgOiK3y8-aOiRKGESseItrWGcfCWeZvVi55p9qMkkEM76FNPC6kFnzjs8_6z1X-okIruay2Gq04JZkp-dULmKitKLk3aMb_p_Skb12aUWpWGs7qVAJaVpmkLH_FSHIhYRTZaXbYe47Vw8Vb17sz827pGZfFylqY3khdSAQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">dry_box_gate_v2.0</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R42" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">43</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/HKqmWNZ5wjmzQQgTtSYk7kRj_38AGa8osbNCe6ERnTnHhZpfja9FO9iKNEMuSTxmVrHHnOmWRhQRshq0WOhVHSDb2cle8olkR74wMWAt3wdsTNxH4-K4pNL7dGSPdxOhy9S1WAAAwC3sjklzqmhjRP8hIiUrsRDUznXYyPgxj83YA0ED-Wmy2GgvDu_buA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">electronics_case_mb_cover_v1.5</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R43" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">44</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh3.googleusercontent.com/pCeToKEqRlBYcB_E3SmwgHbhTvGQsmYUVdfu40TYPoJrS_8xphZo9jRYUWwtgtPt17BY3dgCHmzFu0BRXIXL-Pseyk8oF9_um1ABu16lIUMzYf5Ljc0RzOFMMPrC5WWzMHXf1MdtkznIEwwNUEkMOaAP7aXQz64s9pck-4iCurfzlYLTW5y0FRhzwAz3RQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">electronics_case_mb_bottom_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R44" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">45</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/c4C2ytusi411DujoV9rCy8nZS0qU4PZrgiI2nHOKUwLh4LRiWGz9ko56LxPzECGjp67qlc64R9S_UfQslg-FaCrs9SKYjU1m3FeEmsPKfOZ4yzWZzL-mNr-fr1VRoZa0DdRQPmCVHwjjstCGaHq7HQQFJrcDT1o14VKyB_geQT8Ty7YkEzthAdusbuJeBw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">stepdown_support_v1.2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R45" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">46</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh6.googleusercontent.com/J1aPUu4zLMofWzvQKSwiin6jGis-nhrVYbEQUijoDtswYFpwN_ZYYWEnOg_3yNi6Joi8NqqmZpSgqCpfzKZqCB9DmjSOZkEaSQk_YAe_PoSCrfRfh4ce9Q3nJaChh3MPYfJtzNM7BA-55OhHrIL6R9AxwGNIz3VsSnd3zDSRlrPd9xg0w4KnxLHNYJH1rA=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">LCD-knob_v2 v2</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R46" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">47</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/sTXnIywtm76u0BF7R5GSwzXJ-cozxNtiODWHuOHzZPmMydB35X-x_N6-rwQ5pmjmu0ul9eyBJrNOdy9Va_NXz6fnU7G1HW0iBYUGKChWo7B4ztQXhh-qb-7jeF2BOWS89jlXKQ_awsHjdt2JkxJ4EwBWQbJ_kTKFRVUx79PgqxWZTxGzJoM0PdG7UveTsg=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">axis_cover_short</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R47" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">48</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/-Yi_7bylYSbpYTR-EGIEzSYwJwYxAh0M_KOTnOfwzDC-58BY2EMEtiipCwebj5HbrCCS_GJjHgQGqeohbl7z7diskvOqmzGLxNEQoKFtm9OFO4ZEmxAftcO0csce4ZF5VHCcGMp3c2cnoEB8QDkeysh7r1-8xoXZOP3q0UZ5h-y1NAOp_G0VcHH0MM_Yvg=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">axis_cover_joint</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R48" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">49</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh5.googleusercontent.com/P5GdxXETQHdrxp722Q5NkWtIoLo1-JlpfSkHzZh1timw3aTTLIwfS0zEIu5yuLBOElZuoJnA6tqWnGjPfAZbMpjY4cObbiBLyF12fUjgy7OO3Q2YCBEzncbeimMsLFqS108rmsy__OVv_HJfuaGPIejEEt9nC4yoproblrPLg93Uxne4BxjhZbkgng-RkQ=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">axis_cover_long</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">1</td></tr><tr style="height: 99px"><th id="0R49" style="height: 99px;" class="row-headers-background"><div class="row-header-wrapper" style="line-height: 99px">50</div></th><td class="s1"><div style="width:257px;height:99px;"><img src="https://lh4.googleusercontent.com/uuTD1wX2cSlpurkD4LdPISIk9i-aHDgxYBPOCyglfvyRJpjvygHw_a_Yn8pusg5CK4nE681QwDv40y3vrEWFMcWogf_vsrCP5aGwKCGPF4Czy7w2SMiVSPi2-4vu-e19iT1Xrs-sEoFUo_JV-1x_Pvnri9pxT8owU8E537PLA0pulnRvNpa7nCnoC0_rEw=w257-h99" title="Imagen" style="width:inherit;height:inherit;object-fit:scale-down;object-position:center center;"/></div></td><td class="s2" dir="ltr">axis_cover_nut</td><td class="s3" dir="ltr">PCS</td><td class="s4" dir="ltr">2</td></tr></tbody></table></div>
