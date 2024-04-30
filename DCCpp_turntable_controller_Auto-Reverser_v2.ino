//Autoreverser 1.0 (2020-12-25)
// DCC Isolated Autoreversing circuit for turntable track
// A0 pin connected to the out pin of the ACS712 current sensor
// D2 pin connected to the gate of the MOSFET, driving the relay
/*

  https://www.trainboard.com/highball/index.php?threads/turntable-controller-finalised-project-proposed.154256/page-3

*/

//
//    FILE: ACS712_20_DC.ino
// Adapted from AUTHOR: Rob Tillaart
// PURPOSE: demo for AutoReversing circuit for DCC turntable track polarity reversal
//     URL: https://github.com/RobTillaart/ACS712
//
//  use with Arduino Serial Plotter/Monitor


#include "ACS712.h"
#define relay 2
#define led 13
bool relayState = false;

//  Arduino UNO has 5.0 volt with a max ADC value of 1023 steps
//  ACS712 5A  uses 185 mV per A
//  ACS712 20A uses 100 mV per A
//  ACS712 30A uses  66 mV per A

ACS712  ACS(A0, 5.0, 1023, 185);
//  ESP 32 example (might requires resistors to step down the logic voltage)
//  ACS712  ACS(25, 3.3, 4095, 185);

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println(__FILE__);
  Serial.print("ACS712_LIB_VERSION: ");
  Serial.println(ACS712_LIB_VERSION);

  ACS.autoMidPoint();
  //  Serial.println(ACS.getMidPoint());
  pinMode(relay, OUTPUT);
  pinMode(led, OUTPUT);
}

void loop()
{
  int mA = abs(ACS.mA_DC());
  Serial.println(mA);    // comment out to increase loop speed

  if (mA > 1500 && relayState == false)
  {
    digitalWrite(relay, HIGH);
    digitalWrite(led, HIGH);
    relayState = true;
    delay(500);
  }
  else if (mA > 1500 && relayState == true)
  {
    digitalWrite(relay, LOW);
    digitalWrite(led, LOW);
    relayState = false;
    delay(500);
  }
  // delay(50);  // reduce delay to less than the time required by the DCC++/EX power booster to shut down due to a true short circuit.
}


//  -- END OF FILE --
