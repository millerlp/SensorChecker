// SensorChecker_RevA OLED test
// Check that the 128x64 pixel OLED is working

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
#include <Wire.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii

#define GRNLED 13
#define REDLED 14

// OLED setup
SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here

void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF

  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
  oled.set2X();
  oled.print("Hello");
  oled.println();
  oled.println(F("Working?"));
  oled.println(F("Third line"));
  oled.println(F("Fourth line"));
  delay(1000);
  
}

void loop() {


}
