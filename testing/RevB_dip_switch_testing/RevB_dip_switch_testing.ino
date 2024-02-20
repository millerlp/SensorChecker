/*
 Testing function of the 4-position DIP switch on RevB board

 PD7 = sensor select (digital pin 19)
 PA2 = SDA1
 PA3 = SCL1
 PD1 = green LED (digital pin 13)
 PD2 = red LED  (digital pin 14)
 PC2 = Hall sensor SLEEP line (digital pin 10)
 PD0 = Hall sensor voltage out (analog pin A0)
*/

#include "Arduino.h"
#include <Wire.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii

// Pin number definitions
#define GRNLED 13
#define REDLED 14

#define DIP1 15  // PD3
#define DIP2 16  // PD4
#define DIP3 17  // PD5
#define DIP4 18  // PD6


// OLED setup
SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here


uint32_t loopDelayMS = 50; // Loop time in milliseconds
uint32_t oldMillis = 0; // time keeping variable
bool DIP1pos = false;
bool DIP2pos = false;
bool DIP3pos = false;
bool DIP4pos = false; 

bool DIPs = 0;

void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  pinMode(DIP1, INPUT_PULLUP); 
  pinMode(DIP2, INPUT_PULLUP); 
  pinMode(DIP3, INPUT_PULLUP); 
  pinMode(DIP4, INPUT_PULLUP); 
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


  oldMillis = millis();

}

void loop() {
  if (millis() - oldMillis > loopDelayMS) {
    oldMillis = millis(); //update oldMillis
    
    oled.home();
    if (digitalRead(DIP1)){
      oled.print("DIP1 = high");
    } else {
      oled.print("DIP1 = low ");
    }
    oled.setCursor(0,2); // 2nd row
    if (digitalRead(DIP2)) {
      oled.print("DIP2 = high");
    } else {
      oled.print("DIP2 = low ");
    }
    oled.setCursor(0,4); // 3rd row
    if (digitalRead(DIP3)) {
      oled.print("DIP3 = high");
    } else {
      oled.print("DIP3 = low ");
    }
    oled.setCursor(0,6); // 4th row
    if (digitalRead(DIP4)) {
      oled.print("DIP4 = high");
    } else {
      oled.print("DIP4 = low ");
    }    
    
    
  }
  

}
