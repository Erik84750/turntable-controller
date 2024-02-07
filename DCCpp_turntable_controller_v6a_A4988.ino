/*
  Read keypad: if number pressed then two keypad strokes required; else just one stroke
  Store keypad numerical entry for
     OBSOLETE 1. either turntable index position (moveTo
     OBSOLETE 2. or store in EEPROM after 'A' is pressed
  Store character entry for:
     A = Assign current position (currentPosition) into eeprom for an index number (index to be entered)
     B = move to whatever position value is entered in keypad
          # = validate numerical entry (for >2 digits only; ie: case B
            = erase last numerical entry, for case B only
     C = calibration of '0' index position; optionally turn table 360 degrees to calculate number of steps needed for 180 degrees
     D = Designate a position (setCurrentPosition) from eeprom to current turntable indexed position (index to be entered):
         done at program startup when stepper is in a known and indexed position
     # = select index position to be moved to
     * = select index position + 180 degrees to be moved to

       NOTE: v5 : precede index with either # or * to decide allignement of turntable at 0 or 180 degrees

  Version v5:
  1. also allow indexposition + 180 degrees: allow the turntable to make a half turn on the same index
  2. index selection: now precede with either # or  * to decide whether turntable alligns at 0 degrees ort 180 degrees

  Version v6:
     1. done: to do: on calibration give option to make a full 360 degree turn, to count the number of required steps.
     This in order to calculate the number of steps needed to make a 18 degree swing
     2. to do: index selection: now precede with either # or  * to decide whether turntable alligns at 0 degrees or 180 degrees
     3. done: to do: optimize index search (see 2.) for minimum distance movement (algorithm)
     4. done: introduce a "long press" for keypad characters: https://arduinogetstarted.com/tutorials/arduino-button-long-press-short-press
     5. introduce a turntable movement interruption, for example when wrong index is selected; to avoid having to wait for turntable to finalise the wrong move

  Version 6a:
     1. modified algorithm "shortest move" to accomodate for the special case (re Thomas 9/6/23)

  Turntable notes: 120' scale Diamond Scale Model
  Circumference : 141.3cm
  Stepper: 28BYJ-48. gear ratio = 1/63,68395 approx 1/64.
  32 steps/revolution with gear : 32 x 64 = 2038 steps/revolution
  With 1/50 worm gear: 2038 x 50 = 101.900 steps / revolution
  with circumference of 1413mm gives 0,01387mm / step
  With 1ms/step and 101.900 steps/revolution and 60.000 ms/minute = 0,588 rpm
  Without 1/50 wormgear: 29rpm
  Measured max stepper speed: 2:26' for 50 revolutions, = 1 revolution per 2.92 second, = 20 rpm
  Actual speed measurement: 10 revolutions 32" (with accel and decel)
  With 50/1 gear ratio: 0.411 rpm

  NOTE: from accelstepper.h documentation, fastest speed is 4000 steps/second at 16MHz
  https://forum.arduino.cc/t/maximum-speed-using-accelstepper/1059990/6

  video on a Diamond Scale stepper-driven turntable: https://www.youtube.com/watch?v=9iM3EE5VJwk
  How to input multiple digit numbers on keypad https://forum.arduino.cc/t/keypad-input-2-digit-numbers/604861
  How to deal wth both 2-digit numbers as with character keypad input https://forum.arduino.cc/t/keypad-numericals-ok-now-letters-how-to/1128871/8
  How to accumulate and store two numerical entries: https://forum.arduino.cc/t/keypad-how-to-accumulate-and-store-two-numerical-entries/1127280
  How to enter letters besides numericals: https://forum.arduino.cc/t/keypad-numericals-ok-now-letters-how-to/1128871/8
  In-depth analysis of stepper 28BYJ-48 https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
  How to simulate a program workflow: https://forum.arduino.cc/t/blocking-while/1129972/8
  How to reduce dynamic memory consumption: https://forum.arduino.cc/t/uno-global-variables-consume-84/1130251/4
  How to reduce dynamic memory size: https://forum.arduino.cc/t/uno-global-variables-consume-84/1130251/8
  How to long press: https://arduinogetstarted.com/tutorials/arduino-button-long-press-short-press
  PROGMEM reference: http://www.gammon.com.au/progmem
  EEPROM: https://roboticsbackend.com/arduino-store-int-into-eeprom/
          https://docs.arduino.cc/learn/built-in-libraries/eeprom
          https://stackoverflow.com/questions/67201612/differences-for-eeprom-read-and-eeprom-get
          https://www.best-microcontroller-projects.com/arduino-eeprom.html#:%7E:text=Arduino%20EEPROM%20get%20vs%20read,bytes%20starting%20from%20an%20address
  A4988 stepper driver: https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/
                        https://all3dp.com/2/vref-calculator-tmc2209-tmc2208-a4988/
  A4988 microstep resolution
  MS1 MS2 MS3
  L   L   L   full step (default)
  H   L   L   1/2 step
  L   H   L   1/4 step
  H   H   L   1/8 step
  H   H   H   1/16 step

  NEMA17 stepper connections: https://mecha4makers.co.nz/product/17hs2408-stepper-motor-5mm-shaft-without-flat/
  6 Pin JST PH6 (6 pin, 2mm pitch) male socket:
    Pin 1: A+ / 1A
    Pin 2: NC
    Pin 3: B+ / 2A
    Pin 4: A- / 1B
    Pin 5: NC
    Pin 6: B- / 2B





*/
#include "Wire.h"
#include "I2CKeyPad.h"  // https://github.com/RobTillaart/I2CKeyPad
#include <EEPROM.h>
#include <LCD.h> // https://forum.arduino.cc/t/lcd-i2c-libraries-incompatibility/968205
#include <LiquidCrystal_I2C.h>  // https://github.com/fmalpartida/New-LiquidCrystal/blob/master/LiquidCrystal_I2C.h
#include <AccelStepper.h>
#include <avr/pgmspace.h>

#define motorInterfaceType 1
/*
  FUNCTION  = 0, ///< Use the functional interface, implementing your own driver functions (internal use only)
  DRIVER    = 1, ///< Stepper Driver, 2 driver pins required
  FULL2WIRE = 2, ///< 2 wire stepper, 2 motor pins required
  FULL3WIRE = 3, ///< 3 wire stepper, such as HDD spindle, 3 motor pins required
  FULL4WIRE = 4, ///< 4 wire full stepper, 4 motor pins required
  HALF3WIRE = 6, ///< 3 wire half stepper, such as HDD spindle, 3 motor pins required
  HALF4WIRE = 8  ///< 4 wire half stepper, 4 motor pins required
*/
#define MP1  8 // IN1 on the ULN2003
#define MP2  9 // IN2 on the ULN2003
#define MP3  10 // IN3 on the ULN2003
#define MP4  11 // IN4 on the ULN2003
#define ledIR 4 // input for IR led detector, HIGH when NO detection
#define LCD_ADDR     0x27 // LCD I2C address
#define clk 6   //Clock pin connected to D6
#define data 7    //Data pin connected to D7
#define ENC_SWITCH 5   //Push button pin connected to D5
#define led 13  // built-in led
#define enable A0

const uint8_t KEYPAD_ADDRESS = 0x38; // PCF8574 = 0x20  PCF8574AP = 0x38
const int dirPin = 2;
const int stepPin = 3;

char keys[] = "123A456B789C*0#DNF";  // N = NoKey, F = Fail (e.g. >1 keys pressed)
uint8_t i; // identifier for either number (i = 1 ... = number of numbers), or character : i = (0, or) 50
uint8_t ttMaxPos = 15; // maximum number of indexed turntable positions
// Set Encoder values
volatile int32_t stepperPosition = 0;
int32_t prevPosition;
const uint16_t disableDelay = 1000;
const uint16_t keyLongPress = 500;  // minimum time in ms for keypad long press
bool newData, runOk, tempBtn, btnState, ledState, currentBtnState; // booleans for new data from serial, and run allowed flag
bool stepperInUse = false;
bool driverEnabled = false;
bool caseIdx = false;
bool caseReady = false;
bool valueReady = false;
bool indexReady = false;

// char keyVal = 0;   // the actual key pressed
uint8_t index = 0;  // index for keypad array

uint8_t speed = 1;
uint32_t totalSteps180, pressedTime, releasedTime, pressDuration;  // number of steps for half a turntable turnstored in eeprom address 500
bool currentState;  // keypad long-press variables

// Create myStepper instance
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
// AccelStepper myStepper(MotorInterfaceType, 8, 10, 9, 11);
//  AccelStepper myStepper(MotorInterfaceType, MP1, MP3, MP2, MP4);  //Define the pin sequence (IN1-IN3-IN2-IN4). Coil sequence: 1A-1B-2B-2A
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
I2CKeyPad keyPad(KEYPAD_ADDRESS);
LiquidCrystal_I2C lcd(LCD_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

///|||||||||||||||||||||||||||||||||||||||||||||||||||///
///                   setup                           ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println(F("DCCpp_turntable_controller_v6a_A4988"));


  // Encoder interupt pins
  pinMode(ENC_SWITCH, INPUT_PULLUP);
  pinMode(clk, INPUT_PULLUP);
  pinMode(data, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH);

  // Activate pin change interruptions on pin D6 and D7
  PCICR |= (1 << PCIE2);      // set PCICR register bit 2 (PCIE2: PCINT[23:16], 0B00000100) this is for PD0 to PD7
  PCMSK2 |= (1 << PCINT22);   // pin 6 PCINT22, 0B01000000    interrupt from pin 6
  PCMSK2 |= (1 << PCINT23);   // pin 7 PCINT23, 0B10000000    interrupt from pin 7

  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  myStepper.setMaxSpeed(700.0);
  myStepper.setAcceleration(100.0);
  //myStepper.setSpeed(200);
  myStepper.moveTo(0);

  lcd.begin(20, 4);               // initialize the LCD
  lcd.setCursor(1, 0); // starts  column 0..19, row 0..3
  lcd.print(F("DCCpp_tt_ctrlr_v6a"));

  if (keyPad.begin() == false)
  {
    Serial.println(F("\nERROR: cannot communicate to keypad.\nPlease reboot.\n"));
    lcd.setCursor(1, 0); // starts  column 0..19, row 0..3
    lcd.print(F("keypad comms error"));
    while (1);
  }

}

///|||||||||||||||||||||||||||||||||||||||||||||||||||///  // each pulse from the RE causes one interrupt
///               IRQ routine                         ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

ISR(PCINT2_vect) {
  static uint8_t lastStateClock;
  uint8_t currentStateClock = (PIND & 0B01000000);  //  Pin D6 state
  uint8_t stateData = (PIND & 0B10000000);  //  Pin D7 state
  stateData = stateData >> 1;   // shift right one bit: for PD7 move 0Bx0000000 to 0B0x000000 to be on the same level as PD6
  if ( currentStateClock != lastStateClock )
  {
    // If D7 state is different from D6 state, then encoder is rotating clockwise
    if (stateData != currentStateClock)
      stepperPosition += speed;                        //  increment
    //Else, the encoder is rotating counter-clockwise
    else
      stepperPosition -= speed;                        //  decrement
    lastStateClock = currentStateClock;         // Updates the previous state of the clock with the current state
  }
}

///|||||||||||||||||||||||||||||||||||||||||||||||||||///
///                   loop                            ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

void loop() {
  static bool keyPress = true;
  static int myValue = 0;
  static bool pressed = false;
  static char str = 0;

  myStepper.run();  // activate stepper if distanceToGo is not yet 0


  // Check for encoder update from interrupt routine; requires one pulse (one interrupt) for this to be executed
  if (stepperPosition != prevPosition) {
    digitalWrite(enable, LOW);  // enable stepper output    // driverEnabled = true;
    myStepper.moveTo(stepperPosition);
    stepperInUse = true;
    myStepper.enableOutputs();
    driverEnabled = true;
    prevPosition = stepperPosition;
  }

  // check if rotary encoder button is pressed: RE speed selection
  if (readEncSwitch()) {
    if ( !pressed ) {
      speed = (speed == 1 ? 10 : 1);
      digitalWrite(led, (speed == 10 ? HIGH : LOW));
      pressed = true;
      Serial.println(speed);
    }
  }
  else pressed = false;

  disableStepper();

  // press a character key:
  // since v5: preceed with # or * to decide on straight or 180 degrees reversed turntable positioning DONE
  // !! Also calculate if CW or CCW is shortest distance !! DONE

  if (!keyPad.isPressed()) keyPress = true; // set flag keyPress: keypad released. Long press is detected after release.
  if (keyPad.isPressed () && keyPress == true) {  // execute only once
    keyPress = false;
    pressedTime = millis();  // button is pressed
    currentState = true;
    Serial.println(F("pressedTime is set"));
    myValue = getKeypadInput(myValue); // read keypad being pressed, is always a number; if character then the ASCII number for that character
    lcd.clear();
    lcd.setCursor(0, 0);  // starts  column 0..19, row 0..3
    lcd.print(F("apply long key press"));
  }
  if (currentState) { // button has been pressed and may still be pressed
    if (!keyPad.isPressed()) { // keypad is released
      releasedTime = millis();  // release time
      pressDuration = releasedTime - pressedTime;

      if (pressDuration > keyLongPress) { // button was pressed long enough
        Serial.print(F("pressduration = "));
        Serial.println(pressDuration);
        Serial.println(F("keypad is pressed long enough"));
        // myValue = getKeypadPress(myValue); // read keypad being pressed, is always a number; if character then the ASCII number for that character

        Serial.print(F("charater pressed = "));
        Serial.println(myValue);
        Serial.print(F("i = "));
        Serial.println(i);
        Serial.print(F("i in currenState = "));
        Serial.println(i);
        if (i < 50) { // a numerical was pressed
          Serial.println("wrong key, press character key");
          lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
          lcd.print(F("wrong key, use char"));
        }
        else {  // a character or # or * was pressed
          str = myValue; // convert number to character
          myValue = 0;
          lcd.clear();
          lcd.setCursor(0, 0);  // starts  column 0..19, row 0..3
          lcd.print(F("key request:      "));
          lcd.setCursor(16, 0);  // starts  column 0..19, row 0..3
          lcd.print(str);  // print character on line 0
          Serial.print(F("letter obtained : "));
          Serial.println(str);
          keypadAction(str);  // go to switch case
          // str = 0;
        }
        i = 0;
      }
      else {
        Serial.println(F("button press not long enough"));
        lcd.print(F("key not pressed long"));
      }
      currentState = false;
    }
  }
}

///|||||||||||||||||||||||||||||||||||||||||||||||||||///
///               disable stepper output             ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

void disableStepper() {
  static uint32_t timeStepped = 0;

  // Disable outputs once stepperPosition reached
  if (stepperInUse) {  // initialize ouput disable
    if ( myStepper.distanceToGo() == 0 ) {
      stepperInUse = false;
      timeStepped = millis();
      Serial.println(F("stepperInUse = true"));
    }
  }
  else if (driverEnabled && (millis() - timeStepped) >= disableDelay) {     // start disable after timeout disableDelay
    digitalWrite(enable, HIGH);
    myStepper.disableOutputs();
    driverEnabled = false;
    caseReady = true;
    valueReady = false;
    stepperPosition = myStepper.currentPosition(); // set global variable equal to current stepper position
    prevPosition = stepperPosition;
    Serial.print(F("Position: "));
    Serial.println(stepperPosition);
    lcd.clear();
    lcd.setCursor(0, 0);  // starts  column 0..19, row 0..3
    lcd.print(F("stepper disabled"));
    lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
    lcd.print(F("Position: "));
    lcd.setCursor(10, 1);  // starts  column 0..19, row 0..3
    lcd.print(stepperPosition);
  }
}


///|||||||||||||||||||||||||||||||||||||||||||||||||||///
///          get integer or char from keypad          ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

int getKeypadInput(int value) {
  bool isNum = 0;
  char keyVal = 0;   // the actual key pressed

  do  {
    index = keyPad.getKey();                          // input the key
    keyVal = keys[index];
    isNum = (keyVal >= '0' && keyVal <= '9');         // is it a digit?
    char str = keyVal;
    Serial.print(F("index in getKeypadPress = "));
    Serial.println(index);
    Serial.print(F("str in getKeypadPress = "));
    Serial.println(str);

    if (isNum) {
      // i = i + 1;
      i += 1;
      //Serial.print(keyVal - '0');
      value = value * 10 + keyVal - '0';               // accumulate the input number
    }
    if (keyVal == 'A' || keyVal == 'B' || keyVal == 'C' || keyVal == 'D' || keyVal == '*' || keyVal == '#') {
      i = 50;  // sets the flag for non-numerical value entered
      Serial.print(F("i in getKeypadPress = "));
      Serial.println(i);
      value = keyVal;
    }
  }
  while ( !keyVal);       // exit do when a kay was pressed
  return value;
}


///|||||||||||||||||||||||||||||||||||||||||||||||||||///
///          read rotary encoder button               ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

int readEncSwitch() {
  // Switch debounce values
  uint16_t debounce = 30;    // maybe 50 is enough
  static uint32_t lTimeOfChange = 0;  // Time when button state changed
  static int16_t lastStatus = 0;

  if ((millis() - lTimeOfChange) >= debounce) {
    int encSwitch = !digitalRead(ENC_SWITCH);  // Read the Encoder switch (low when pressed so invert)

    if (encSwitch != lastStatus) {
      lastStatus = encSwitch;
      lTimeOfChange = millis();
    }
  }
  return lastStatus;
}


///|||||||||||||||||||||||||||||||||||||||||||||||||||///  https://roboticsbackend.com/arduino-store-int-into-eeprom/
///   writing integers to eeprom, max. 65.535       ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

void intToEEPROM(uint16_t address, int16_t number) {
  EEPROM.update(address, number >> 8);
  EEPROM.update(address + 1, number & 0xFF);
}

///|||||||||||||||||||||||||||||||||||||||||||||||||||///  https://roboticsbackend.com/arduino-store-int-into-eeprom/
///   reading integers from eeprom, max. 65.535       ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

uint16_t intFromEEPROM(uint16_t address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

///|||||||||||||||||||||||||||||||||||||||||||||||||||///  https://roboticsbackend.com/arduino-store-int-into-eeprom/
///      writing long to eeprom, max. 4x10sq9         ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

void writeLongIntoEEPROM(uint16_t address, int32_t number) {
  EEPROM.update(address, (number >> 24) & 0xFF);
  EEPROM.update(address + 1, (number >> 16) & 0xFF);
  EEPROM.update(address + 2, (number >> 8) & 0xFF);
  EEPROM.update(address + 3, number & 0xFF);
}

///|||||||||||||||||||||||||||||||||||||||||||||||||||///  https://roboticsbackend.com/arduino-store-int-into-eeprom/
///      reading long from eeprom, max. 4x10sq9       ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

int32_t readLongFromEEPROM(uint16_t address) {
  return ((long)EEPROM.read(address) << 24) +
         ((long)EEPROM.read(address + 1) << 16) +
         ((long)EEPROM.read(address + 2) << 8) +
         (long)EEPROM.read(address + 3);
}

///|||||||||||||||||||||||||||||||||||||||||||||||||||///
///          keypad character entries                 ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

void keypadAction(char keyStroke) {
  uint8_t inputInt = 0;
  static bool keyPress = 0;  // initial status = keypad not pressed
  static bool secNum = 0;
  static int myValue = 0;
  // static uint32_t timeStepped = 0;
  static bool pressed = false;
  bool caseA, caseB, caseC, caseD, caseStar, caseNr = false;
  // int index = 0;
  long value = 0;
  char keyVal = 0;             // the key press
  int key = keyStroke;
  uint8_t caseValue = 0;
  digitalWrite(enable, LOW);  // enable stepper output

  switch (keyStroke) {
    case 'A': { // Assign current position (currentPosition) into eeprom for an index position (index to be entered)
        uint8_t myIndex = 0;
        Serial.println(F("Case A entered")); // assign current position to an index and write to eeprom
        caseA = true;
        while (caseA) {
          // Serial.println("while started");
          if (!keyPad.isPressed()) keyPress = true; // set flag keyPress: keypad released
          if (keyPad.isPressed () && keyPress == true) { // execute only once; determine index ID
            keyPress = false;
            myIndex = getKeypadInput(myIndex); // read keypad being pressed
            Serial.print(F("A myValue = "));
            Serial.println(myIndex);
            Serial.print(F("A i = "));
            Serial.println(i);
            if (i == 2) { // 2 valid digits have been entered on keypad
              i = 0;
              Serial.print(F("A full value of two digits obtained : "));
              Serial.println(myIndex);
              caseValue = myIndex; // copy to final variable
              myValue = 0;
              valueReady = true;
            }
          }
          if ((caseValue <= ttMaxPos) && valueReady) {  // execute only and once if index is ready and valid: write eeprom
            lcd.clear();
            Serial.print(F("requested index ok, indexReady = "));
            Serial.println(valueReady);
            Serial.print(F("eeprom value being programmed : "));
            Serial.println(myStepper.currentPosition());
            Serial.print(F("eeprom address being programmed : "));
            Serial.println((caseValue * 4));
            lcd.setCursor(0, 0);  // starts  column 0..19, row 0..3
            lcd.print(F("Value programmed for index"));
            lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
            lcd.print(F("Index value ="));
            lcd.setCursor(15, 1);  // starts  column 0..19, row 0..3
            lcd.print(caseValue);
            lcd.setCursor(0, 2);  // starts  column 0..19, row 0..3
            lcd.print(F("Position ="));
            lcd.setCursor(15, 2);  // starts  column 0..19, row 0..3
            lcd.print(myStepper.currentPosition());
            lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
            lcd.print(F("EEPROM address :"));
            lcd.setCursor(17, 3);  // starts  column 0..19, row 0..3
            lcd.print(caseValue * 2);
            writeLongIntoEEPROM((caseValue * 4), myStepper.currentPosition()); // store current position in eeprom address = indexNumber * 4
            caseReady = true;
            valueReady = false;  // execute this if once only
          }
          else if (caseValue > ttMaxPos && valueReady) { // execute if the index entered is too high
            Serial.println(F("requested index too high"));
            lcd.clear();
            lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
            lcd.print(F("Index too high: "));
            lcd.setCursor(17, 1);  // starts  column 0..19, row 0..3
            lcd.print(caseValue);
            lcd.setCursor(0, 2);  // starts  column 0..19, row 0..3
            lcd.print(F("Retry, max ="));
            lcd.setCursor(14, 2);  // starts  column 0..19, row 0..3
            lcd.print(ttMaxPos);
            caseReady = true;
            valueReady = false;  // execute this if once only

          }
          else {  // invalid number entered

          }
          if (caseReady) {
            caseA = false;
            Serial.println("caseA = false");
          }
        }
      }
      break;
    case 'B': { // move to whatever position value is entered in keypad
        // # = validate numerical entry (for >2 digits only; ie: case B
        // *  = erase last numerical entry, for case B only
        Serial.println(F("Case B entered")); // move stepper to value entered in keypad
        caseB = true;
        lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
        lcd.print(F("enter digits"));
        lcd.setCursor(0, 2);  // starts  column 0..19, row 0..3
        lcd.print(F("* = erase # = enter"));
        while (caseB) {
          // Serial.println("while started");
          if (!keyPad.isPressed()) keyPress = true; // set flag keyPress: keypad released
          // Serial.println(F("key 'B' released, while(caseB) entered, Keypress = true")); //
          if (keyPad.isPressed () && keyPress == true) { // execute as long as final value is not obtained
            
            Serial.println(F("keypad is pressed, verify if digit or # or *"));
            // keyPress = false;
            index = keyPad.getKey();                          // input the key
            delay(250);
            keyVal = keys[index];                             // get character from keys array

            if (keyVal >= '0' && keyVal <= '9') {
              value = value * 10 + keyVal - '0';               // accumulate the input number
              Serial.print(F("current value = "));
              Serial.println(value);
              lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
              lcd.print(F("number = "));
              lcd.setCursor(9, 3);  // starts  column 0..19, row 0..3
              lcd.print(value);
            }
            if (keyVal == '#') {                              // final value obtained, validate multidigit entry on keypad
              myValue = value;
              value = 0;
              Serial.print("final value = ");
              Serial.println(myValue);
              Serial.println(F("number is validated"));
              valueReady = true;  // the enteredd number is validated
              keyPress = false;  // the keypad is released and number is ready to be used by steppermotor

            }
            if (keyVal == '*') {                              // erase latest keypad digit entered
              value = value / 10;
              Serial.print(F("current value = "));
              Serial.println(value);
              lcd.setCursor(9, 3);  // starts  column 0..19, row 0..3
              lcd.print("       ");
              delay(100);
              lcd.setCursor(9, 3);  // starts  column 0..19, row 0..3
              lcd.print(value);

            }
          }
          // Serial.println(F("Case B stepper enabled"));
          driverEnabled = true;
          stepperInUse = true;
          caseReady = false;
          while (valueReady) {  // run stepper as long as when valueReady is true
            myStepper.run();
            myStepper.enableOutputs();
            speed = 100;
            myStepper.moveTo(myValue);


            /*
                        if ( myStepper.distanceToGo() == 0 )  // finalise this while when stepper has stopped running
                        {
                          valueReady = false;
                          Serial.print("destination reached : ");
                          Serial.println(myValue);
                        }
            */
            // Disble outputs once stepperPosition reached for RE
            disableStepper();
            // Serial.println(F("case b stepper finished move")); // move stepper to value entered in keypad

            /*
                        if (stepperInUse ) // initialize ouput disable
                        {
                          if ( myStepper.distanceToGo() == 0 ) {
                            stepperInUse = false;
                            timeStepped = millis();
                            Serial.print(F("destination reached : "));
                            Serial.println(myValue);
                            Serial.print(F("distance to go : "));
                            Serial.println(myStepper.distanceToGo());
                          }
                        }
                        else if ( driverEnabled && (millis() - timeStepped) >= disableDelay)  // start disable after timeout disableDelay
                        {
                          myStepper.disableOutputs();
                          driverEnabled = false;
                          caseReady = true;
                          valueReady = false;
                          stepperPosition = myStepper.currentPosition(); // set global variable equal to current stepper position
                          prevPosition = stepperPosition;
                          Serial.println(F("output disabled"));
                          digitalWrite(enable, LOW);  // enable stepper output
                        }
            */

          }
          if (caseReady) {
            caseB = false;
            caseReady = false;
            Serial.println(F("caseB = false, caseReady = false"));
            Serial.println(F("caseB = done"));
          }
        }
      }
      break;

    case 'C': {  // calibrate stepper to sensor (IR detector) for absolute "0" postioning of the turntable, press # to acknowledge, or..
        // press * and allow to optionally turn table 360 degrees so that number of steps needed for 180 degrees can be measured
        bool runOnce1 = true;
        bool runOnce2 = true;


        Serial.println(F("Case C entered")); // enter an index value, retrieve the corresponding eeprom value and store in currentPosition
        caseC = true;
        while (caseC) {
          stepperInUse = true;
          driverEnabled = true;
          Serial.print(F(" value of ledIR = "));
          Serial.println(digitalRead(ledIR));
          Serial.print(F(" keystroke = "));
          Serial.println(keyStroke);
          Serial.print(F(" isRunning = "));
          //int runStatus = myStepper.isRunning();
          Serial.println( myStepper.isRunning(), BIN);  // ??? value is 0 ?? maybe because at this point not running yet ?
          lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
          lcd.print(F("searching '0' index"));
          myStepper.setSpeed(500);
          while (!digitalRead(ledIR)) {  // IR sensor reads HIGH when no detection
            myStepper.runSpeed();
          }
          // bool runStatus = myStepper.isRunning(); // equals 0 if stepper no longer running; equals distanceToGpo = 0
          Serial.print(F(" current position = "));
          Serial.println(stepperPosition);
          Serial.print(F(" value of stepperInUse = "));
          Serial.println(stepperInUse, BIN);
          lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
          lcd.print(F("'0' index found     "));
          lcd.setCursor(0, 2);  // starts  column 0..19, row 0..3
          lcd.print(F("'360' test press *"));
          lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
          lcd.print(F("finalise press #"));
          myStepper.setCurrentPosition(0); // reset current position to 0
          stepperPosition = 0;
          prevPosition = stepperPosition;
          while (runOnce2) {  // execute only after initial calibration, select either # when ready, or * when 360° turn is needed.
            // Serial.println(F("stepperPosition = 0 AND runOnce2 = true"));
            if (!keyPad.isPressed()) keyPress = true; // set flag keyPress: keypad released
            if (keyPad.isPressed() && keyPress == true) {  // execute only after keypad is pressed, after keypad was released prior
              // myStepper.run();  // activate stepper if distanceToGo is not yet 0
              keyPress = false;
              index = keyPad.getKey();                          // input the key
              keyVal = keys[index];                             // get character from keys array
              Serial.print(F("keyVal = "));
              Serial.println(keyVal);
              if (keyVal == '*' && runOnce1) {  // start 360 degree run and calculate number of steps needed for 180 degrees. Run once.
                Serial.println(F("start 360 degree algorithm"));
                totalSteps180 = step360();  // function to measure total number of steps for a 180 degree turn; is blocking until finished
                writeLongIntoEEPROM(500, totalSteps180);
                Serial.print(F("totalStepper180 = "));
                Serial.println(totalSteps180);
                myStepper.setCurrentPosition(0); // reset current position to 0
                stepperPosition = 0;
                prevPosition = stepperPosition;
                Serial.print(F("distanceToGo = "));
                Serial.println(myStepper.distanceToGo());  // sets distanceToGo = 0
                runOnce1 = false;
              }
              else if (keyVal == '#' ) { // validate calibration
                Serial.print(F("Calibration finalised, current position = "));
                Serial.println(stepperPosition);
              }
              else {
              }
              if (myStepper.distanceToGo() == 0) {
                stepperPosition = myStepper.currentPosition();
                prevPosition = stepperPosition;
                Serial.println(F("caseC && runOnce2 = false"));
                caseC = false;
                runOnce2 = false;
                digitalWrite(enable, LOW);  // enable stepper output

              }
            }
          }
        }
        break;
      }
    case 'D': {  // Designate a position (setCurrentPosition) from eeprom to current turntable indexed position (index to be entered):
        //   done at program startup when stepper is in a know and indexed position
        Serial.println(F("Case D entered")); // enter an index value, retrieve the corresponding eeprom value and store in currentPosition
        caseD = true;
        lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
        lcd.print(F("Assign index"));
        while (caseD) {
          myStepper.stop();
          // Serial.println("while started");
          if (!keyPad.isPressed()) keyPress = true; // set flag keyPress: keypad released
          if (keyPad.isPressed () && keyPress == true) {  // execute only once; determine index ID
            keyPress = false;
            myValue = getKeypadInput(myValue); // read keypad being pressed
            Serial.print(F("D myValue = "));
            Serial.println(myValue);
            Serial.print(F("D i = "));
            Serial.println(i);
            lcd.setCursor(14, 1);  // starts  column 0..19, row 0..3
            lcd.print(myValue);
            if (i == 2) {  // 2 valid digits have been entered on keypad
              i = 0;
              Serial.print("D full value of two digits obtained : ");
              Serial.println(myValue);
              caseValue = myValue; // copy to final variable
              myValue = 0;
              valueReady = true;
              delay(750);
            }
          }
          if (caseValue <= ttMaxPos && valueReady) {  // execute only and once if index is ready and valid
            stepperPosition = readLongFromEEPROM(caseValue * 4); // read current position from corresponding eeprom address = indexNumber * 4
            myStepper.setCurrentPosition(stepperPosition);
            Serial.print(F("requested index ok : "));
            Serial.print(valueReady);
            Serial.println(F("eeprom being retrieved"));
            lcd.clear();
            lcd.setCursor(0, 0);  // starts  column 0..19, row 0..3
            lcd.print(F("Value retrieved for index"));
            lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
            lcd.print(F("Index value ="));
            lcd.setCursor(15, 1);  // starts  column 0..19, row 0..3
            lcd.print(caseValue);
            lcd.setCursor(0, 2);  // starts  column 0..19, row 0..3
            lcd.print(F("Position ="));
            lcd.setCursor(11, 2);  // starts  column 0..19, row 0..3
            lcd.print(myStepper.currentPosition());
            lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
            lcd.print(F("EEPROM address :"));
            lcd.setCursor(17, 3);  // starts  column 0..19, row 0..3
            lcd.print(caseValue * 4);
            caseReady = true;
            valueReady = false;  // execute this if once only
          }
          else if (caseValue > ttMaxPos && valueReady) { // requested index too high
            Serial.println(F("requested index too high"));
            lcd.clear();
            lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
            lcd.print(F("Index too high: "));
            lcd.setCursor(17, 1);  // starts  column 0..19, row 0..3
            lcd.print(caseValue);
            lcd.setCursor(0, 2);  // starts  column 0..19, row 0..3
            lcd.print(F("Retry, max ="));
            lcd.setCursor(14, 2);  // starts  column 0..19, row 0..3
            lcd.print(ttMaxPos);
            caseReady = true;
            valueReady = false;  // execute this if once only
          }
          else {  // invalid number entered
          }
          if (caseReady) {
            caseD = false;
            Serial.println(F("caseD = false"));
            stepperPosition = myStepper.currentPosition();
            prevPosition = stepperPosition;
          }
        }
      }
      break;
    case '#': {  // # has been pressed: move from current position to position called by index number from following keypad entry
        caseNr = true;
        caseReady = false;
        uint32_t targetPos = 0;
        uint32_t currentPos = 0;
        uint32_t tailPos = 0, c = 0, t = 0, F;
        int32_t cw, ccw = 0;
        totalSteps180 = readLongFromEEPROM(500);
        F = totalSteps180 * 2;
        lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
        lcd.print(F("enter 2-digit index "));
        lcd.setCursor(0, 2);  // starts  column 0..19, row 0..3
        lcd.print(F("straight through"));
        while (caseNr) {
          myStepper.stop();  // ???
          // Serial.println("while started");
          if (!keyPad.isPressed()) keyPress = true; // set flag keyPress: keypad released
          if (keyPad.isPressed () && keyPress == true) {  // execute only once; determine index ID
            keyPress = false;
            myValue = getKeypadInput(myValue); // read keypad being pressed
            Serial.print(F("# myValue = "));
            Serial.println(myValue);
            Serial.print(F("# i = "));
            Serial.println(i);

            if (i == 2) {  // 2 valid digits have been entered on keypad, target index known
              Serial.print(F("# full value of two digits obtained : "));
              Serial.println(myValue);
              lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
              lcd.print(F("straight, idx ="));
              lcd.setCursor(17, 3);  // starts  column 0..19, row 0..3
              lcd.print(myValue);  // print retrieved index value
              caseValue = myValue; // copy 2-digit index number to final variable: index number to move to
              myValue = 0;  // reset retrieved keypad value
              i = 0;  // reset digit counter
              valueReady = true; // 2-digit target indexnumber has been entered
              delay(750);
            }
            if (i = 50) {
              Serial.println(F("a character was pressed; repeat, press a digit"));
              lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
              lcd.print(F("press digits"));
            }
          }
          while (valueReady) {  //
            targetPos = readLongFromEEPROM(caseValue * 4); // read target position from corresponding eeprom address = indexNumber * 4
            currentPos = myStepper.currentPosition(); // set currentPosition to current position
            if (currentPos == targetPos) {
              cw = -F / 2;
              ccw = F / 2;
              Serial.println(F("current equals target"));
              Serial.print(F("CW = "));
              Serial.println(cw);
              Serial.print(F("CCW = "));
              Serial.println(ccw);
            }
            if (c < F / 2) tailPos = c + F / 2;
            else tailPos = c - F / 2;
            if (tailPos > t) {
              cw = -(tailPos - t);
              ccw = (F - tailPos) + t;
            }
            else {
              cw = -(tailPos + (F - t));
              ccw = t - tailPos;
            }
            Serial.println(F("straight"));
            Serial.print(F("CW = "));
            Serial.println(cw);
            Serial.print(F("CCW = "));
            Serial.println(ccw);
            myStepper.run();
            myStepper.enableOutputs();
            if (abs(cw) > ccw) {  // ccw <= cw: move CCW
              myStepper.move(ccw);
            }
            else {  // move CW
              myStepper.move(cw);
            }
            if (myStepper.distanceToGo() == 0) {
              caseReady = true;  // if move is finished set caseReady flag
              lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
              lcd.print(F("TT move finished    "));
            }
          }

          if (caseReady) {
            caseNr = false;
            Serial.println(F("caseNr = false"));
            disableStepper();  // diosable stepper outputs
          }
        }
      }
      break;
    case'*': {  // move from current position to position called by index number from following keypad entry, plus 180 degree offset
        caseStar = true;
        caseReady = false;
        uint32_t targetPos = 0;
        uint32_t currentPos = 0;
        uint32_t c = 0, t = 0, F;
        int32_t cw, ccw = 0;
        totalSteps180 = readLongFromEEPROM(500);
        F = totalSteps180 * 2;
        lcd.setCursor(0, 1);  // starts  column 0..19, row 0..3
        lcd.print(F("enter 2-digit index "));
        lcd.setCursor(0, 2);  // starts  column 0..19, row 0..3
        lcd.print(F("reversing through"));
        while (caseStar) {
          myStepper.stop();  // ???
          // Serial.println("while started");
          if (!keyPad.isPressed()) keyPress = true; // set flag keyPress: keypad released
          if (keyPad.isPressed () && keyPress == true) {  // execute only once; determine index ID
            keyPress = false;
            myValue = getKeypadInput(myValue); // read keypad being pressed
            Serial.print(F("# myValue = "));
            Serial.println(myValue);
            Serial.print(F("# i = "));
            Serial.println(i);
            lcd.setCursor(14, 1);  // starts  column 0..19, row 0..3
            lcd.print(myValue);
            if (i == 2) {  // 2 valid digits have been entered on keypad, target index known
              i = 0;
              Serial.print("# full value of two digits obtained : ");
              Serial.println(myValue);
              lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
              lcd.print(F("reversing, idx ="));
              lcd.setCursor(17, 3);  // starts  column 0..19, row 0..3
              lcd.print(myValue);  // print retrieved index value
              caseValue = myValue; // copy 2-digit index number to final variable: index number to move to
              myValue = 0;
              valueReady = true; // 2-digit target indexnumber has been entered
              delay(750);
            }
            if (i = 50) {
              Serial.println(F("a character was pressed; repeat, press a digit"));
              lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
              lcd.print(F("press digits"));
            }
          }

          while (valueReady) { //
            targetPos = readLongFromEEPROM(caseValue * 4); // read target position from corresponding eeprom address = indexNumber * 4
            currentPos = myStepper.currentPosition(); // set currentPosition to current position
            if (currentPos == targetPos) {
              cw = -F / 2;
              ccw = F / 2;
              Serial.println(F("current equals target"));
              Serial.print(F("CW = "));
              Serial.println(cw);
              Serial.print(F("CCW = "));
              Serial.println(ccw);
            }
            if (c < t) {
              cw = -(c + F - t);
              ccw = t - c;
            }
            else {
              cw = -(c - t);
              ccw = F - c + t;
            }

            Serial.println(F("reversing"));
            Serial.print(F("CW = "));
            Serial.println(cw);
            Serial.print(F("CCW = "));
            Serial.println(ccw);
            myStepper.run();
            myStepper.enableOutputs();
            if (abs(cw) > ccw) {  // ccw <= cw: move CCW
              myStepper.move(ccw);
            }
            else {  // move CW
              myStepper.move(cw);
            }
            // if (myStepper.distanceToGo() == 0) caseReady = true;  // if move is finished set caseReady flag
            if (myStepper.distanceToGo() == 0) {
              caseReady = true;  // if move is finished set caseReady flag
              lcd.setCursor(0, 3);  // starts  column 0..19, row 0..3
              lcd.print(F("TT move finished    "));
            }
          }

          if (caseReady) {
            caseStar = false;
            Serial.println(F("caseNr = false"));
            disableStepper();  // diosable stepper outputs
          }
        }
        break;
      }
  }
}


///|||||||||||||||||||||||||||||||||||||||||||||||||||///
///       keypad 'C' 360 degree stepcounter           ///
///|||||||||||||||||||||||||||||||||||||||||||||||||||///

uint32_t step360()
{
  Serial.println(F("Moving to sensor"));
  myStepper.enableOutputs();
  myStepper.setSpeed(500);
  while ( digitalRead(ledIR) )
    myStepper.runSpeed();
  while ( !digitalRead(ledIR) )
    myStepper.runSpeed();
  myStepper.disableOutputs();
  // Serial.print(F("End pos = "));
  // Serial.println(myStepper.currentPosition() / 2);
  totalSteps180 = myStepper.currentPosition() / 2;
  myStepper.setCurrentPosition(0); // reset current position to 0
  stepperPosition = 0;
  prevPosition = stepperPosition;
  return totalSteps180;
}
