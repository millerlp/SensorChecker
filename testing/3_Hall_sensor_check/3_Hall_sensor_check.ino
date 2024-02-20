/* SensorChecker_RevA Basic Hall sensor check
 *  Assumes that OLED screen is already working

// PD7 = sensor select (digital pin 19)
// PA2 = SDA1
// PA3 = SCL1
// PD1 = green LED (digital pin 13)
// PD2 = red LED  (digital pin 14)
// PC2 = Hall sensor SLEEP line (digital pin 10)
// PD0 = Hall sensor voltage out (analog pin A0)
*/

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
#define SENSOR_SELECT 19 // high = Gape power, low = heart power
#define HALL_SLEEP 25
#define ANALOG_IN A10

char sensorMode = 0; // Used to set gape or heart mode

// OLED setup
SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here


void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  pinMode(SENSOR_SELECT, INPUT_PULLUP);
  sensorMode = digitalRead(SENSOR_SELECT);
  
  pinMode(HALL_SLEEP, OUTPUT);
  digitalWrite(HALL_SLEEP, LOW); // set high to wake, set low to sleep (~60usec to wake)

  analogReference(VDD); // VDD = use supply voltage as analog reference
  pinMode(ANALOG_IN, INPUT); // Hall sensor input channel
  
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
  oled.set2X();
//  oled.print("Hello");
//  oled.println();
  if (sensorMode == 1) {
    oled.println("Gape mode");
    digitalWrite(REDLED, LOW);
  } else if (sensorMode == 0) {
    oled.println("Heart mode");
    digitalWrite(GRNLED, LOW);
  }
  delay(1000);

}


// Function to take a few readings from hall sensor and average them
unsigned int readHall(byte ANALOG_IN){
  unsigned int rawAnalog = 0;
  analogRead(ANALOG_IN); // throw away 1st reading
  for (byte i = 0; i<4; i++){
    rawAnalog = rawAnalog + analogRead(ANALOG_IN);
    delay(1);
  }
  // Do a 2-bit right shift to divide rawAnalog
  // by 4 to get the average of the 4 readings
  rawAnalog = rawAnalog >> 2;   
  return rawAnalog;
}


void loop() {
  if (sensorMode == 1) {
    digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
    unsigned int HallValue = readHall(ANALOG_IN);
//    oled.clear();
//    oled.home();
    oled.clear(0,128,2,3); // Clear rows 2&3 (when using 2x font this clears the 2nd row)
    oled.setCursor(0,2); // Set cursor to column 0, 2nd row (for 2x font)
    oled.print(HallValue);
    digitalWrite(HALL_SLEEP, LOW); // put hall sensor to sleep
    delay(100);
  } else if (sensorMode == 0) {
    oled.clear();
    oled.home();
    oled.println("Heart mode");
    delay(100);
  }

}
