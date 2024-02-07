# turntable-controller
Digital stand-alone turntable controller with automated indexing and stepper motor

A. Introduction
  1. The purpose of this project is to create a stand-alone and automated model railroad (any scale) turntable controller
  2. Hardware: 

  a. stepper motor: project is usable for any type of stepper motor but prefered option is the use of a 17HS2408 motor

  b. stepper motor driver: any motor driver module can be used but prefered option is a module with an onboard A4988 chip
    
  c. indexing sensor: optical sensor with accurate positioning sensing. Hall effect is exluded due to its inacurate sensing repeatability and wide detection range.
    
  d. 4x4 keypad with pushbuttons (no membrane keypad)
    
  e. a custom designed PCB (all required info for manufacture is included in this project file)
    
  f. LCD display with 20x4 characters
    
  g. a 3D printable casing for this project, downloadable .stl files
    
  3. Software
    a. C++ custom software with a very wide range of options and features
    b. All software is freely downloadable from this project file

B. Description of features:
  1. Any size of turntable can be controlled
  2. Automated initial indexing and storage in eeprom is provided. Due to the very high accuracy (+/- 1 micrometer) of the optical sensor the repeatability is < 5 micrometer.
  3. Fast index location and programming for outgoing/incoming tracks is provided, storage in eeprom. The turntable will always move CW or CCW after the algorithm included calculates the shortest/fastest rotation
  4. Automated "shortest rotation" algorithm is included in the software
  5. Photo-electric sensor (based on HC-020K module) http://tinyurl.com/2ce3v4j9 or http://tinyurl.com/3nykkvw3 used for indexing calibration
  6. button A: assign the current position to an index number in eeprom
  7. button B: move to whatever position by using the keypad
  8. button C: calibration
  9. button D: designate a position (current position) from eeprom to current turntable indexed position
  11. button #: select an index position to be moved to
  12. Button *: select index position + 180° to be moved to.
  13. enter # or * before index selection so that the turntable alligns at 0° or 180°
  14. Rotary encoder rotation of turntable; RE button serves to set speed x10 or regular
  15. a "long press" of (adjustable) 500ms is included to prevent inadvertent keypresses
  16. include, upon calibration, the option to make a full 360° in order to calculate the stepper motor steps for a full 360° rotation.
  17. EEPROM storage of programmed track index positions

![stepper motor HS2236_1](https://github.com/Erik84750/turntable-controller/assets/20128852/0df68b92-2e0d-4379-9515-5cf681c18422)

![a4988-stepper-motor-controler](https://github.com/Erik84750/turntable-controller/assets/20128852/c50b7870-e508-422d-878e-d73ceac1acc1)

![md-hc-020k](https://github.com/Erik84750/turntable-controller/assets/20128852/56b6b7ec-ffee-40d6-be44-9f8bce4e5b92)

![keypad 4x4](https://github.com/Erik84750/turntable-controller/assets/20128852/72cb4111-b025-4728-82b0-08ddbe2f0e2e)

[Uploading stepper motor 28BYJ-45 12V.pdf…]()

[stepper motor 17HS2408-MotionKing.pdf](https://github.com/Erik84750/turntable-controller/files/14192960/stepper.motor.17HS2408-MotionKing.pdf)

<img width="804" alt="turntable_controller_complete" src="https://github.com/Erik84750/turntable-controller/assets/20128852/87b85869-a899-421c-a504-99f51ce4470f">

[stepper motor driver A4988.pdf](https://github.com/Erik84750/turntable-controller/files/14192971/stepper.motor.driver.A4988.pdf)

