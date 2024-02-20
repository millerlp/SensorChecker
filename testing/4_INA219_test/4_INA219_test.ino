/* SensorChecker_RevA INA219 current monitor check
 *  Assumes that OLED screen is already working

// PD7 = sensor select (digital pin 19)
// PA2 = SDA1
// PA3 = SCL1
// PD1 = green LED (digital pin 13)
// PD2 = red LED  (digital pin 14)
// PC2 = Hall sensor SLEEP line (digital pin 10)
// PD0 = Hall sensor voltage out (analog pin A0)
*/

#include "Arduino.h"
#include <Wire.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "INA219.h" // https://github.com/millerlp/INA219


#define GRNLED 13
#define REDLED 14
#define SENSOR_SELECT 19 // high = Gape power, low = heart power
#define HALL_SLEEP 25
#define ANALOG_IN A10
char sensorMode = 0; // Used to set gape or heart mode
//************************
// OLED setup
SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here

//******************************
// Set up INA219 current/voltage monitor (default I2C address is 0x40)
Adafruit_INA219 ina219(0x40);
float currentShuntVoltage = 0; // Voltage drop across shunt resistor (0.01ohm on heater board)
float currentBusVoltage = 0;  // Voltage at the load (heater)
float currentCurrentValue = 0; // Current in mA
float currentPowerValue = 0; // Power in mW
float loadVoltage = 0;  // Estimated battery voltage (prior to the shunt resistor + load)
float Watts = 0; // Estimated power output, Watts
float movingAverageCurr = 0;
float movingAverageCurrSum = 0;
// Number of samples for moving average:
const byte averageCount = 8;
byte averageCounter = 0;
float actualShuntResistance = 0.75; // ideally 0.1, but enter actual number here


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

  // Initialize the INA219 current sensor
  // By default the initialization will use the range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a 16V, 400mA range:
  ina219.setCalibration_16V_400mA();
//  ina219.setCalibration_32V_32A();
  
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
    oled.println("Gape");
    oled.print(" On  Sleep");
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

//---------------------------------------------
void loop() {

  if (sensorMode == 1){ // Gape mode
    movingAverageCurrSum = 0; // reset
    digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
    unsigned int HallValue = readHall(ANALOG_IN);
    oled.clear(50,128,0,1); // Clear latter half of row 0
    oled.setCursor(60,0); // set cursor to column 60, row 0 (for 2x font)
//    oled.clear(0,128,2,3); // Clear rows 2&3 (when using 2x font this clears the 2nd row)
//    oled.setCursor(0,2); // Set cursor to column 0, 2nd row (for 2x font)
    oled.print(HallValue);

    
  
    
    for (int x=0; x < averageCount; x++){
        movingAverageCurrSum += ina219.getCurrent_mA();
        delay(1);
    }
    movingAverageCurr = movingAverageCurrSum / averageCount; // Calculate average, mA
    // If shunt resistor isn't 0.1ohm, correct the calculated current
    movingAverageCurr = movingAverageCurr / (actualShuntResistance / 0.1); 
    oled.clear(0,128,4,5); // Clear rows 4&5 (when using 2x font this clears the 3rd row)
    oled.setCursor(0,4); // Set cursor to column 0, 2nd row (for 2x font)
    oled.print(movingAverageCurr); oled.print(" ");
  //  oled.print(" mA");
    // Now take reading while asleep
    digitalWrite(HALL_SLEEP, LOW); delay(5); // turn off hall effect sensor
    movingAverageCurrSum = 0; // reset
    for (int x=0; x < averageCount; x++){
        movingAverageCurrSum += ina219.getCurrent_mA();
        delay(1);
    }
    movingAverageCurr = movingAverageCurrSum / averageCount; // Calculate average, mA
    // If shunt resistor isn't 0.1ohm, correct the calculated current
    movingAverageCurr = movingAverageCurr / (actualShuntResistance / 0.1);
    oled.print(movingAverageCurr);;
  //  oled.print(ina219.getCurrent_mA()); oled.print(" mA");
//      oled.clear(0,128,6,7); // Clear rows 4&5 (when using 2x font this clears the 3rd row)
//    oled.setCursor(0,6); // Set cursor to column 0, 2nd row (for 2x font)

//    oled.print(ina219.getBusVoltage_V());
//    oled.print("  ");
//    oled.print(ina219.getShuntVoltage_mV());
    delay(100); 
  }
}
