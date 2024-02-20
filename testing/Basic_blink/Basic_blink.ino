// SensorChecker_RevA or RevB blink routine
// PD7 = sensor select (digital pin 19)
// PA2 = SDA1
// PA3 = SCL1
// PD1 = green LED (digital pin 13)
// PD2 = red LED  (digital pin 14)
// PC2 = Hall sensor SLEEP line (digital pin 10)
// PD0 = Hall sensor voltage out (analog pin A0)

/*
 *  *   You will need MegaCoreX board definitions installed, available
 *   at https://github.com/MCUdude/MegaCoreX
 *   
 *   Reminder for burning bootloader with microUPDI programmer:
 *   Reference https://github.com/MCUdude/microUPDI
 *   The microUPDI will show up in Arduino as Arduino UNO WiFi Rev2
 *   Select that device in the Tools>Port menu
 *   Under Tools>Programmer choose Atmel mEDBG (ATmega32U4)
 *   Select Tools>Board>MegaCoreX>Atmega4808
 *   Clock internal 8MHz, BOD 2.6V, Pinout 32 pin standard,
 *   Reset pin "Reset", Bootloader "Optiboot (UART0 default pins)"
 *   Then hit burn bootloader, it should upload 512k bootloader
 * 
 */

#include "Arduino.h"
#define GRNLED 13
#define REDLED 14



void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF

}

void loop() {
  digitalWrite(REDLED, !(digitalRead(REDLED)));
  delay(500);
  digitalWrite(GRNLED, !(digitalRead(GRNLED)));
  delay(500);

}
