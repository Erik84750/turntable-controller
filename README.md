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
  4. Software
    a. C++ custom software with a very wide range of options and features
    b. All software is freely downloadable from this project file

B. Description of features:
  1. Any size of turntable can be controlled
  2. Automated initial indexing and storage in eeprom is provided. Due to the very high accuracy (+/- 1 micrometer) of the optical sensor the repeatability is < 5 micrometer.
  3. Fast index location and programming for outgoing/incoming tracks is provided, storage in eeprom. The turntable will always move CW or CCW after the algorithm included calculates the shortest/fastest rotation
  4. Automated "shortest rotation" algorithm is included in the software
  5. Photo-electric sensor http://tinyurl.com/2ce3v4j9 used for indexing calibration
  6. button A: assign the current position to an index number in eeprom
  7. button B: move to whatever position by using the keypad
  8. button C: calibration
  9. button D: designate a position (current position) from eeprom to current turntable indexed position
  10. button #: select an index position to be moved to
  11. Button *: select index position + 180° to be moved to.
  12. enter # or * before index selection so that the turntable alligns at 0° or 180°
  13. a "long press" of (adjustable) 500ms is included to prevent inadvertent keypresses
  14. include, upon calibration, the option to make a full 360° in order to calculate the stepper motor steps for a full 360° rotation.
