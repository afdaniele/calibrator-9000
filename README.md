# Device: Calibrator-9000
Firmwares and drivers for an Arduino-based 6-DOF calibrator tool.

**Concept art:**
![concept art
rendering](https://github.com/afdaniele/calibrator-9000/blob/master/renderings/22june2018.png?raw=true)

**Final prototype:**
![final prototype](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/full.png?raw=true)

## I want one
Do you want to make one yourself? Nice!

### What you need to buy

| # | Name                      | Qty   | Link          |
| - | ----                      | ---   | ----          |
| 1 | 3D model - 2 parts (.STL) | 1     | https://www.tinkercad.com/things/1XgNqGJ9kwN |
| 2 | PCB board (34*26 holes)   | 1     | any           |
| 3 | Resistance 180 Ohm        | 1     | any           |
| 4 | Resistance 220 Ohm        | 3     | any           |
| 5 | Resistance 270 Ohm        | 1     | any           |
| 6 | Resistance 510 Ohm        | 1     | any           |
| 7 | M3x8 screws (PCB holders) | 4     | any           |
| 8 | M3x16 screws (Bottom part)| 4     | any           |
| 9 | M3 nuts                   | 8     | any           |
|10 | 5mm RGB LED               | 6     | any           |
|11 | 10K linear potentiometer  | 6     | https://www.adafruit.com/product/562  |
|12 | Potentiometer knob 30 x 10mm  | 6 | https://www.aliexpress.com/item/Potentiometer-Knob-30-x-10mm-1-Pieces/32745439004.html |
|13 | Adafruit Itsy Bitsy M0 Express | 1    | https://www.adafruit.com/product/3727 |
|14 | 3D printable PCB holder   | 4     | https://www.tinkercad.com/things/asoKI28gtia |
|15 | Header pins and wires     | *     | any           |

### Instructions

##### Step 1. 3D print the PCB holders, find screws and nuts
![step1](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/screws_and_nuts.png?raw=true)

##### Step 2. Prepare the potentiometers
![step2](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/potentiometers.png?raw=true)

##### Step 3. Prepare the LEDs for X(red), Y(green), Z(blue), R(cyan), P(magenta), W(yellow)
![step3a](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/xyz_leds.png?raw=true)
![step3b](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/rpw_leds.png?raw=true)

##### Step 4. Prepare the PCB
![step4](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/pcb.png?raw=true)


##### Step 5. Use the 3D printed holders to secure the PCB to the bottom plate
![step5](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/pcb_plate.png?raw=true)

##### Step 6. Mount potentiometers and LEDs on the top plate
![step6](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/top_plate.png?raw=true)

##### Step 7. Use 4 M3x16 screws and 4 nuts to secure the case
![step7](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/full.png?raw=true)

##### Step 8. Flash the firmware using the Arduino IDE

##### Step 9. Use the python drivers to calibrate/initialize the device and receive data from it
![step9](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/full.png?raw=true)

##### Step 10. Enjoy the lights
![step10](https://github.com/afdaniele/calibrator-9000/blob/master/pictures/full_lights.png?raw=true)
