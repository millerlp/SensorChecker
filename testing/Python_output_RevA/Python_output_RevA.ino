/* Python_output_RevA  - designed to send a heart sensor signal over serial to a Python script for display

  // PD7 = sensor select (digital pin 19)
  // PA2 = SDA1
  // PA3 = SCL1
  // PD1 = green LED (digital pin 13)
  // PD2 = red LED  (digital pin 14)
  // PC2 = Hall sensor SLEEP line (digital pin 10)
  // PD0 = Hall sensor voltage out (analog pin A0)
*/

#include "Arduino.h"
#include "MAX30105.h"         // https://github.com/millerlp/SparkFun_MAX3010x_Sensor_Library
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

uint32_t loopDelayMS = 100; // Loop time in milliseconds
uint32_t longloopDelayMS = 1000; // Time for updating OLED current measure
uint32_t oldMillis = 0; // time keeping variable
uint32_t oldlongloopMillis = 0; // time keeping variable
int loopCounter = 0; 

//************************
// OLED setup
SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here
//*********************************

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
float movingAverageSleepCurr = 0;
float movingAverageSleepCurrSum = 0;
float actualShuntResistance = 0.1; // ideally 0.1, but enter actual number here


//--------------------------------------
// MAX30105 sensor parameters
MAX30105 max3010x;
// sensor configurations
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green. Only use 2
byte REDledBrightness = 1; // low value of 0 shuts it off, 1 is barely on
byte IRledBrightness = 30;  // 0 = off, 255 = fully on. A value of ~30 is good on a human finger
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32, but only use 1. The others are too slow
int pulseWidth = 215; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs. Recommend 215
// For 118us, max sampleRate = 1000; for 215us, max sampleRate = 800, for 411us, max sampleRate = 400
int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384. 4096 is standard

float val1 = 1.0; // Used to send data to Python via serial

// Prototype for the restartAndSampleMAX3010x function (see full function at bottom of file)
uint32_t restartAndSampleMAX3010x(MAX30105 &max3010x, byte IRledBrightness, \
                                  byte sampleAverage, byte ledMode, int sampleRate, \
                                  int pulseWidth, int adcRange, byte REDledBrightness, \
                                  bool EnableTemp = false);

//-------------------------------------------
// Prototype for reading the Hall sensor (averages 4 readings)
unsigned int readHall(byte ANALOG_IN);


//------------ SETUP -----------------------
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

  Serial.begin(57600);

  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS1);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.home();
  oled.set2X();

  if (sensorMode == 0) {
    /*****************************************
       Start up MAX3010x sensor (heart sensor)
     *****************************************/
    if (max3010x.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 100kHz speed
    {
      //    Serial.println(F("Heart found"));
      digitalWrite(GRNLED, LOW); // set low to turn on
      delay(250);
      digitalWrite(GRNLED, HIGH); // set high to turn off
      //    oled.println("Heart sensor on");
    } else {
      //    oled.println("Heart sensor fail");
      oled.println(F("No heart"));
      for (int c = 0; c < 10; c++) {
        digitalWrite(REDLED, LOW); // set low to turn on, leave on due to the error
        delay(250);
        digitalWrite(REDLED, HIGH); // turn off
        delay(500);
      }
      digitalWrite(REDLED, LOW); // set low to turn on, leave on due to the error
    }
  
    delay(1000);
  }

  //-----------------------------------------------
  // Initialize the INA219 current sensor
  // By default the initialization will use the range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a 16V, 400mA range:
  ina219.setCalibration_16V_400mA();


  oled.clear();
  oled.home();
  if (sensorMode == 1) { // gape mode
    oled.println("Gape");
    oled.println(" On  Sleep");
    oled.println();
    oled.print(" mA  mA");
    digitalWrite(REDLED, LOW);
  } else if (sensorMode == 0) {  // heart mode
    oled.println("Heart");
    oled.println(" On  Sleep");
    oled.println();
    oled.print(" mA  mA");
    digitalWrite(GRNLED, LOW);
    loopDelayMS = 50; // speed up main loop
  }
  delay(1000);
  oldMillis = millis();

}


//---------------------------------------------
void loop() {

  if (millis() - oldMillis > loopDelayMS) {

    oldMillis = millis(); //update oldMillis
    
    if (sensorMode == 0) { // Heart mode
      //------------- Heart mode --------------------------
      //--- Wake and sample the heart sensor
      uint32_t heartBuffer = restartAndSampleMAX3010x(max3010x, IRledBrightness, \
                             sampleAverage, ledMode, sampleRate, pulseWidth, adcRange, \
                             REDledBrightness, false);
//      Serial.println(heartBuffer);
      val1 = float(heartBuffer); // convert to float value
      sendToPC(&val1);  // Send over serial, formatted for Python
          
    } else if (sensorMode == 1) { // Gape mode
      //------------- Gape mode ------------------------
//            loopCounter++; 
      digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
      digitalWrite(GRNLED, !digitalRead(GRNLED));  // toggle green LED to show activity
      unsigned int HallValue = readHall(ANALOG_IN);

      
      
      oled.clear(50,128,0,1); // Clear latter half of row 0
      oled.setCursor(60,0); // set cursor to column 60, row 0 (for 2x font)
//    oled.clear(0,128,2,3); // Clear rows 2&3 (when using 2x font this clears the 2nd row)
//    oled.setCursor(0,2); // Set cursor to column 0, 2nd row (for 2x font)
      oled.print(HallValue);
      Serial.println(HallValue); // also output to Serial Monitor or Plotter

      movingAverageCurrSum += ina219.getCurrent_mA();
      // Now take reading while asleep
      digitalWrite(HALL_SLEEP, LOW); delay(5); // turn off hall effect sensor
      movingAverageSleepCurrSum += ina219.getCurrent_mA();
      
      if ( millis() - oldlongloopMillis > longloopDelayMS) {
          oldlongloopMillis = millis();
          movingAverageCurr = movingAverageCurrSum / loopCounter; // Calculate average, mA
        // If shunt resistor isn't 0.1ohm, correct the calculated current
//        movingAverageCurr = movingAverageCurr / (actualShuntResistance / 0.1); 
          movingAverageSleepCurr = movingAverageSleepCurrSum / loopCounter; // Calculate average, mA
          // If shunt resistor isn't 0.1ohm, correct the calculated current
//        movingAverageSleepCurr = movingAverageSleepCurr / (actualShuntResistance / 0.1); 
        
        oled.clear(0,128,4,5); // Clear rows 4&5 (when using 2x font this clears the 3rd row)
        oled.setCursor(0,4); // Set cursor to column 0, 2nd row (for 2x font)
        oled.print(movingAverageCurr); oled.print(" ");
        oled.print(movingAverageSleepCurr);
        movingAverageCurrSum = 0; // reset
        movingAverageSleepCurrSum = 0; // reset
        loopCounter = 0;    // reset
        if (movingAverageCurr > 4.0) {
          // Flash the Red LED if the current draw is higher than the expected 3mA for a A1395 sensor
          digitalWrite(REDLED, !digitalRead(REDLED));
        } else {
          digitalWrite(REDLED, HIGH);  // make sure it's off if there's no problem
        }
      }
    }
  }

}  // end of main loop

///--------------------------------------------------------------
//                  End of main loop
///--------------------------------------------------------------

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



////-------------------------------------------------------------- 
//void timeSync(unsigned long deltaT)
//{
//  unsigned long currTime = micros();
//  long timeToDelay = deltaT - (currTime - timer);
//  if (timeToDelay > 5000)
//  {
//    delay(timeToDelay / 1000);
//    delayMicroseconds(timeToDelay % 1000);
//  }
//  else if (timeToDelay > 0)
//  {
//    delayMicroseconds(timeToDelay);
//  }
//  else
//  {
//      // timeToDelay is negative so we start immediately
//  }
//  timer = currTime + timeToDelay;
//}

//------------------------------------------------------
//        sendToPC function - writing data formatted for Python
//------------------------------------------------------
// A float is 4 bytes, while a double is also 4 bytes on 8-bit processors like the Atmega4808
void sendToPC(float* data1)
{
  byte* byteData1 = (byte*)(data1);

  byte buf[4] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3]};
  Serial.write(buf, 4);
}

//  This version takes 3 channels of data and sends them
void sendToPC(float* data1, float* data2, float* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}



/**********************************************************************
   restartMAX3010x

   Re-set the MAX3010x sampling settings, assuming it has just returned
   from a hard power-down and restart. Take 1 sample and return the
   uint32_t 4-byte value from the IR channel
 **********************************************************************/
uint32_t restartAndSampleMAX3010x(MAX30105 &max3010x, byte IRledBrightness, byte sampleAverage, \
                                  byte ledMode, int sampleRate, int pulseWidth, int adcRange, \
                                  byte REDledBrightness, bool EnableTemp)
{
  uint32_t heartValue = 0;

  max3010x.setup(IRledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  if (EnableTemp) {
    max3010x.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
  }
  //
  // Tweak individual settings
  max3010x.setPulseAmplitudeRed(REDledBrightness); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
  max3010x.setPulseAmplitudeIR(IRledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
  delay(6); // This delay seems to be necessary to give the sensor time to start reading reasonable values (5ms minimum)
  max3010x.clearFIFO(); // Clearing FIFO potentially lets you only have to grab one value from the FIFO once it starts to refill

  long watchdog = millis();
  bool sampleFlag = false;
  // Here we use a watchdog to make sure the collection of a new sample doesn't take
  // longer than our pre-defined watchdog time (in milliseconds)
  while ( millis() - watchdog < 5) {
    // With a cleared FIFO these two pointers will match initially
    byte readPointer = max3010x.getReadPointer();
    byte writePointer = max3010x.getWritePointer();

    if (readPointer != writePointer) {
      // If they don't match, that means a new sample arrived in the FIFO buffer on the MAX3010x
      sampleFlag = true;
      break; // escape the while loop once a new sample has appeared
    }
    delayMicroseconds(20);
    sampleFlag = false;
  }
  if (sampleFlag) {
    max3010x.check();  // retrieve the new sample(s) and put them in the max3010x private buffer
    // Calling getIR() should get the most recent value from the buffer of values
    heartValue = max3010x.getIR();
    //        Serial.println(heartValue);  // modify getIR in the library to remove safeCheck() function
  } else {
    // If sampleFlag was still false, a new sample didn't arrive from the MAX3010x sensor in time
    //        Serial.println("0");  // modify getIR in the library to remove safeCheck() function
    heartValue = 0;
  }

  return (heartValue); // Return the heartValue as output

} // end of restartAndSampleMAX3010x()
