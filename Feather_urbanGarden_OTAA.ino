/* TTN Payload function:
 *
 * function Decoder(bytes, port) {
 *   var decoded = {};
 *   if (true) {
 *     decoded.temperature = ((bytes[2] & 0x80 ? 0xFFFF<<8 : 0) | bytes[2] | bytes[3])/10;
 *     decoded.batVoltage = (bytes[1]/50);
 *     decoded.moisture= (bytes[0])
 *   }
 *   return decoded;
 * }
 */



// Wire lib required for communication with the soil moisture sensor
#include <Wire.h>

//required for deepsleep. Thanks Adafruit
#include "Adafruit_SleepyDog.h"
#include <avr/power.h>

//Libs for LoRaWAN and the connection of the RFM95 with SPI 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/****************************************************************************
 ********************* USER SETTINGS ****************************************
 ****************************************************************************/
 // There is a double-100K resistor divider on the BAT pin and internally connected to A9
#define VBATPIN A9

//enable deepsleep (true)
#define deepsleep true

//define I2C address of the soil moisture sensor 
// https://www.tindie.com/products/miceuz/i2c-soil-moisture-sensor/
#define I2CADDR 0x20

// define values from ADC that represent complete dry (sensor in air) or wet (sensor in water) values. 
#define DRY_LEVEL 550
#define WET_LEVEL 330

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations) 
#define TX_INTERVAL 900


//******************

/* UPLINK SPREADING FACTOR
 *  Example payload of 2 Bytes, note that a fixed SF of 12 is not allowed 
 *  SpreadingFactor         SF7     SF8     SF9     SF10    SF11    SF12        spreading factor; higher means more range and better reception, but also more airtime     
 *  Tpacket                 46.336  92.672  164.864 329.728 659.456 1155.072    ms total airtime to send a full packet inc. overhead     
 *  TTN Fair Access Policy  647     323     181     90      45      25          average messages/day for maximum of 30 seconds airtime on The Things Network   
 *                          27.0    13.5    7.6     3.8     1.9     1.1         average messages/hour when sending all day  
 *  
 *  see https://docs.google.com/spreadsheets/d/1QvcKsGeTTPpr9icj4XkKXq4r2zTc2j0gsHLrnplzM3I/edit#gid=0 
 *  or
 *  https://www.thethingsnetwork.org/forum/t/spreadsheet-for-lora-airtime-calculation/1190/12
 */
#define SPREADING_FACTOR DR_SF11

// set to 2 for more LMIC and other debug messages
#define LMIC_DEBUG_LEVEL 0

// This EUI must be in little-endian format, so least-significant-byte (LSB)
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. 
static const u1_t PROGMEM APPEUI[8]={ ................................ };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above i.e. LSB
static const u1_t PROGMEM DEVEUI[8]={ ...................................... };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format MSB (or, since it is not really a
// number but a block of memory, endianness does not really apply). 
static const u1_t PROGMEM APPKEY[16] = { ............................................ };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

/*************************************************************************
**************************************************************************
**************************************************************************/
// byte array that holds the payload for sending to TTN
static byte mydata[4] = {};

// required by LMIC to send payload
static osjob_t sendjob;

// Pin mapping
// the feather requires an additionally connection between IO1 pin with pin 6 using a wire bridge
const lmic_pinmap lmic_pins = { 
   .nss = 8, 
   .rxtx = LMIC_UNUSED_PIN, 
   .rst = 4, 
   .dio = {7, 6, LMIC_UNUSED_PIN}, 
};


/* ****************************************************************************** 
 *  *************************** function prototypes - see below main loop **********
 *  *****************************************************************************
 */

// get LiPo battery level
byte getBat();

// sensor communication functions
unsigned int getMoisture();
byte convertMoisture(int rawMoisture);
int getTemp();
unsigned int getLight();
void resetSensor();
byte getVersion();
void goSleep();
bool isBusy();
bool waitFree();

// I2C write to and read from sensor
void writeI2CRegister8bit(int addr, int value);
unsigned int readI2CRegister16bit(int addr, byte reg);
int readI2CRegister16bitSigned(int addr, byte reg);
byte readI2CRegister8bit(int addr, int reg);


//Lora related stuff
void onEvent (ev_t ev);
void do_send(osjob_t* j);
void sensor_loop();


/* ======================================================================
Function: setup
Purpose : initialize all the stuff we need
Input   : -
Output  : -
Comments: the setup function runs once when you press reset or power the board
====================================================================== */
void setup() {
  // init I2C
  Wire.begin();

  // init serial console
  Serial.begin(9600);
  if (LMIC_DEBUG_LEVEL > 0) {
    while (!Serial);             // wait for serial monitor for debugging
    Serial.println("Starting....");
  }

  //init the I2CSensor
  void resetSensor();
  if (LMIC_DEBUG_LEVEL > 0) {
   Serial.println("Sensor Init done");
  }
  
  // leave this if you put your controller into deepsleep. Otherwise it will be 
  // hard to connect to it as it goes very quick to deep sleep and does not respond then 
  delay(10000);
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // write some LMIC/LoRa settings
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  LMIC_setLinkCheckMode(1);
  LMIC.dn2Dr = SPREADING_FACTOR;

  if (LMIC_DEBUG_LEVEL > 0) {
    Serial.println("Starting first submission and after OTAA join");
  }

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);

  if (LMIC_DEBUG_LEVEL > 0) {
    Serial.println("LMIC Init done");
    Serial.println("leaving Setup");
  }
}


/* ======================================================================
Function: main loop
Purpose : -
Input   : -
Output  : -
Comments: all functions are in the sensor loop which will run based on schedule 
          set in 'void onEvent (ev_t ev)' 
====================================================================== */
void loop() {
     
    //
    //https://github.com/matthijskooijman/arduino-lmic/blob/master/src/lmic/oslmic.c#L102
    os_runloop_once();
}


/* ======================================================================
Function: main sensor loop
Purpose : -
Input   : -
Output  : -
Comments: runs based on schedule set in 'void onEvent (ev_t ev)' -> 
          in case of a EV_COMPLETE response.
		  The next run is delayed by TX_INTERVAL
          by delay() or deepsleep
====================================================================== */
void sensor_loop() {
  // we don't care what the version is but it wakes the sensor from sleep
  getVersion();

  // start a new moisture measurement and wait until the measurement is done
  getMoisture();
  if (waitFree() == true) {

    // if the sensor measurement finished successfull, read all the values
    unsigned int moist = getMoisture();
    byte conMoist = convertMoisture(moist);
    int temp = getTemp();
    // Serial.println(getLight());  // it is dark as the sensor is covered by water protection
    byte batLevel = getBat();

    // put the data into the array that will be send to TTN
    mydata[0] = conMoist;
    mydata[1] = batLevel;
    mydata[2] = (temp >> 8) & 255;
    mydata[3] = temp & 255;
    
    // provide debug info if required
    if (LMIC_DEBUG_LEVEL > 0) {
      Serial.print(" Moisture : ");
      Serial.println(moist);
      Serial.print(" ConvMoist: ");
      Serial.println(conMoist);
      Serial.print(" Temp     : ");
      Serial.println(temp);
      Serial.print(" Batterie : ");
      Serial.println(batLevel);
      Serial.print(" Payload  : ");
      for (int j=0;j<4;j++) {
        Serial.print(mydata[j]);
        Serial.print(" ");
      }
      Serial.println();
      Serial.println();
    }
  }  
}


/* ======================================================================
Function: onEvent
Purpose : event handler for LoRa events
Input   : events
Output  : 
Comments: taken from the example. schedules sensor loop if it gets EV_TXCOMPLETE
====================================================================== */
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Submission to TTN was successfull. Schedule next transmission
            if (!deepsleep) {
              // if no deepsleep is used, schedule the next run of 'do_send'
              // which updates the sensor values and submits the value array in TX_INTERVAL
              os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            } else {
              // prepare for deepsleep. First tell the sensor to sleep and wait a moment
              goSleep();
              delay(50);
              // max deepsleep is 8s, so run a loop until TX_INTERVAL is reached 
              for (int i=0; i < int(TX_INTERVAL/8+1); i++) {
                Watchdog.sleep(8000);
              }
              // schedule the next run of 'do_send' which updates the sensor values 
              // and submits the value array immediately (1s)
              os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}


/* ======================================================================
Function: getBat
Purpose : get the voltage from the analog pin
Input   : nothing except the definition of the analog input pint is required
Output  : the voltage as byte. Therefore the float is multiplied by 50
          as the range is only 0-5V -> 0-250
Comments: calls the do_send with the byta array mydata. 
====================================================================== */
byte getBat() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return byte(measuredvbat*50);
}


/* ======================================================================
Function: do_send
Purpose : updating the sensor values, writing it to the mydata array and sending the payload to TTN
Input   : it send the data that given to LMIC_setTxData2 as 2nd argument
Output  : 
Comments: taken from the example
====================================================================== */
void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        sensor_loop();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


/* SOIL MOISTURE RELATED FUNCTIONS
 *  https://github.com/Miceuz/i2c-moisture-sensor/blob/master/README.md
 *  I2C protocol
 *  Available registers for reading and writing.
 *  Name                            Register  R/W   Data length
 *  GET_CAPACITANCE                 0x00      (r)   2
 *  SET_ADDRESS                     0x01      (w)   1
 *  GET_ADDRESS                     0x02      (r)   1
 *  MEASURE_LIGHT                   0x03      (w)   0
 *  GET_LIGHT                       0x04      (r)   2
 *  GET_TEMPERATURE                 0x05      (r)   2
 *  RESET                           0x06      (w)   0
 *  GET_VERSION                     0x07      (r)   1
 *  SLEEP                           0x08      (w)   0
 *  GET_BUSY                        0x09      (r)   1
 *  
 *  
 *  Soil moisture reading process:
 *  1. read from GET_CAPACITANCE, 
 *  2. discard results, 
 *  3. read from GET_BUSY until you get '0' (not busy) as an answer
 *  4. read form GET_CAPACITANCE again - the returned value is the soil moisture NOW.
 */


/* ======================================================================
Function: writeI2CRegister8bit
Purpose : writing a value to the I2C soil moisture sensor
Input   : I2C address of the device and the value to be written
Output  : -
Comments: taken from the example
====================================================================== */
void writeI2CRegister8bit(int addr, int value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}


/* ======================================================================
Function: readI2CRegister16bit
Purpose : reading a value to the I2C soil moisture sensor
Input   : I2C address of the device and the register to be read
Output  : 2 byte unsigned integer value read from the register
Comments: taken from the example - see documentation link above
====================================================================== */
unsigned int readI2CRegister16bit(int addr, byte reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  delay(20);
  Wire.requestFrom(addr, 2);
  unsigned int t = Wire.read() << 8;
  t = t | Wire.read();
  return t;
}


/* ======================================================================
Function: readI2CRegister16bitSigned
Purpose : reading a value to the I2C soil moisture sensor
Input   : I2C address of the device and the register to be read
Output  : 2 byte signed integer value read from the register
Comments: the referred Arduino lib has a similar function for temp which requires neg values
          https://github.com/Apollon77/I2CSoilMoistureSensor/blob/master/I2CSoilMoistureSensor.cpp
====================================================================== */
int readI2CRegister16bitSigned(int addr, byte reg) {
  return (int)readI2CRegister16bit(addr, reg);
}


/* ======================================================================
Function: readI2CRegister8bit
Purpose : reading a value to the I2C soil moisture sensor
Input   : I2C address of the device and the register to be read
Output  : 1 byte "byte" value read from the register
Comments: from the example
====================================================================== */
byte readI2CRegister8bit(int addr, int reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  delay(20);
  Wire.requestFrom(addr, 1);
  byte t = Wire.read();
  return t;
}


/* ======================================================================
Function: resetSensor()
Purpose : resetting the sensor
Input   : none
Output  : none
Comments: provides the I2C address and register number to the I2C functions
====================================================================== */
void resetSensor() {
  writeI2CRegister8bit(I2CADDR, 6); //reset
}



/* ======================================================================
Function: getMoisture()
Purpose : reading the moisture value
Input   : none
Output  : raw moisture value 2 bytes
Comments: provides the I2C address and register number to the I2C functions
====================================================================== */
unsigned int getMoisture() {
  return readI2CRegister16bit(I2CADDR, 0); //read capacitance register
}


/* ======================================================================
Function: getTemp()
Purpose : reading the temperature value
Input   : none
Output  : temperature value 2 bytes (note: it is temp*10)
Comments: provides the I2C address and register number to the I2C functions
====================================================================== */
int getTemp() {
  return readI2CRegister16bitSigned(I2CADDR, 5); //temperature register
}


/* ======================================================================
Function: resetLight()
Purpose : reading the light value
Input   : none
Output  : raw analog light value 2 bytes (65k is no light)
Comments: provides the I2C address and register number to the I2C functions. 
          Sensor is covered by water protection so the function is not used
====================================================================== */
unsigned int getLight() {
  writeI2CRegister8bit(I2CADDR, 3); //request light measurement
  return readI2CRegister16bit(I2CADDR, 4); //read light register
}


/* ======================================================================
Function: getVersiont()
Purpose : reading the sensor version
Input   : none
Output  : 1 byte "byte" value for the sensor version
Comments: provides the I2C address and register number to the I2C functions. 
          function is used to wake the sensor up from sleep
====================================================================== */
byte getVersion() {
  return readI2CRegister8bit(I2CADDR, 7);
}


/* ======================================================================
Function: goSleep()
Purpose : telling the sensor to sleep
Input   : none
Output  : none
Comments: provides the I2C address and register number to the I2C functions. 
          function is used to save power
====================================================================== */
void goSleep() {
  writeI2CRegister8bit(I2CADDR, 8);
}


/* ======================================================================
Function: isBusy()
Purpose : getting the info wether the sensor is busy or has finished measurement
Input   : none
Output  : boolean true for busy
Comments: provides the I2C address and register number to the I2C functions. 
====================================================================== */
bool isBusy() {
  if ( readI2CRegister8bit(I2CADDR, 9) !=  0) {
    return true;
  } else {
    return false;
  } 
}

/* ======================================================================
Function: waitFree()
Purpose : wait until the sensor has finished the measurement. In case it stuck (50 * 50ms
          it resets the sensor
Input   : none
Output  : boolean true if measrement was done or false after a reset was required
Comments: - 
====================================================================== */
bool waitFree() {
  byte busyCounter = 0;
  while ( isBusy() ) {
    busyCounter++;
    Serial.println("busy");
    delay(50);
    if (busyCounter >50) {
      resetSensor();
      delay(1000);
      return false;
    }
  }
  return true;
}


/* ======================================================================
Function: convertMoisture
Purpose : convert the raw moisture value to a 0-100% value
Input   : raw moisture value 2 bytes int
Output  : soil moisture byte value between 0...100
Comments: - 
====================================================================== */
byte convertMoisture(int rawMoisture) {
  rawMoisture = rawMoisture - WET_LEVEL;
  
  //set the values back to the range between DRY_LEVEL and WET LEVEL if outside
  if (rawMoisture <0) {
    rawMoisture = 0;
  }
  if ( rawMoisture > (DRY_LEVEL - WET_LEVEL) ){
    rawMoisture = (DRY_LEVEL - WET_LEVEL);
  }
  
  return byte( float(rawMoisture)/(DRY_LEVEL-WET_LEVEL)*100); 
}

  




