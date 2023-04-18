#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <DHTesp.h>
#include <Ticker.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <QMC5883LCompass.h>

//Unions for vlaues larger than one byte
union
{
  float value;
  byte Byte[4];
} gpsLat, gpsLng, gpsAlt, gpsSpeed, gpsHDOP, bmpAltitude, bmpPressure, bmpTemperature;

union 
{
  uint8_t value;
  byte Byte[2];
} gpsSats, gpsAge,compassX, compassY, compassZ;


QMC5883LCompass compass;
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

int counter = 0;

// SX1276 requires the following connections:
int pin_cs = 26;
int pin_dio0 = 27;
int pin_nrst = 25;
int pin_dio1 = 14;

int pin_rx_enable = 13;
int pin_tx_enable = 12;
SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);


/*
//New radio pin defs
int radio_CS    = 26;
int radio_D0    = 27;
int radio_D1    = 14;
int radio_RESET = 25;
int radio_TX_ENA= 12;
int radio_RX_ENA= 13;
*/

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200); //Open serial
  ss.begin(GPSBaud); //Open software serial to gps
  compass.init(); //Compass module init
  //Init the barometer
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  //Begin radio
  Serial.print(F("[SX1276] Initializing ... "));
  //int state = radio.begin(); //-121dBm
  //int state = radio.begin(868.0); //-20dBm
  int state = radio.begin(915.0); //-23dBm
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("init success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set output power to 10 dBm (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission
  //       duty cycle MUST NOT exceed 1%
//  if (radio.setOutputPower(20) == ERR_INVALID_OUTPUT_POWER) {
//    Serial.println(F("Selected output power is invalid for this module!"));
//    while (true);
//  }
  // some modules have an external RF switch
  // controlled via two pins (RX enable, TX enable)
  // to enable automatic control of the switch,
  // call the following method

  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);
  radio.setOutputPower(15);
  
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));


}

void loop()
{
  compass.read();
  // Return XYZ readings
  compassX.value = compass.getX();
  compassY.value = compass.getY();
  compassZ.value = compass.getZ();

  if (! bmp.performReading()) {
    Serial.println("BMP328 - FAILED TO READ");
    return;
  }

  bmpTemperature.value=bmp.temperature;//Unit deg C
  bmpPressure.value=bmp.pressure; //Unit hPa
  bmpAltitude.value=bmp.readAltitude(SEALEVELPRESSURE_HPA); //Unit meters

  gpsSats.value= gps.satellites.value();
  gpsAge.value = gps.location.age();
  gpsLat.value = gps.location.lat();
  gpsLng.value = gps.location.lng();
  gpsAlt.value = gps.altitude.meters();
  gpsSpeed.value=gps.speed.kmph();
  gpsHDOP.value= gps.hdop.hdop();

  Serial.print("GPS Sats: ");  
  Serial.println(gpsSats.value);  

  Serial.print("GPS Fix Age: ");  
  Serial.println(gpsAge.value);  

  Serial.print("GPS Fix Age: ");  
  Serial.println(gpsAge.value);  
  
  Serial.print("BMP Temp: ");
  Serial.println(bmpTemperature.value);





  byte packet[36] = {
              gpsLat.Byte[0], //Byte 0
              gpsLat.Byte[1], //Byte 1
              gpsLat.Byte[2], //Byte 2
              gpsLat.Byte[3], //Byte 3

              gpsLng.Byte[0], //Byte 4
              gpsLng.Byte[1], //Byte 5
              gpsLng.Byte[2], //Byte 6
              gpsLng.Byte[3], //Byte 7

              gpsAlt.Byte[0], //Byte 8
              gpsAlt.Byte[1], //Byte 9
              gpsAlt.Byte[2], //Byte 10
              gpsAlt.Byte[3], //Byte 11

              gpsSpeed.Byte[0], //Byte 12
              gpsSpeed.Byte[1], //Byte 13
              gpsSpeed.Byte[2], //Byte 14
              gpsSpeed.Byte[3], //Byte 15

              gpsHDOP.Byte[0], //Byte 16
              gpsHDOP.Byte[1], //Byte 17
              gpsHDOP.Byte[2], //Byte 18
              gpsHDOP.Byte[3], //Byte 19

              bmpAltitude.Byte[0], //Byte 20
              bmpAltitude.Byte[1], //Byte 21
              bmpAltitude.Byte[2], //Byte 22
              bmpAltitude.Byte[3], //Byte 23

              bmpPressure.Byte[0], //Byte 24
              bmpPressure.Byte[1], //Byte 25
              bmpPressure.Byte[2], //Byte 26
              bmpPressure.Byte[3], //Byte 27

              bmpTemperature.Byte[0], //Byte 28
              bmpTemperature.Byte[1], //Byte 29
              bmpTemperature.Byte[2], //Byte 30
              bmpTemperature.Byte[3], //Byte 31

              gpsSats.Byte[0], //Byte 32
              gpsSats.Byte[1], //Byte 33

              gpsAge.Byte[0], //Byte 34
              gpsAge.Byte[1] //Byte 35

             };
  
//} gpsLat, gpsLng, gpsAlt, gpsSpeed, gpsHDOP;
/*
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);
  */

  //printInt(gps.charsProcessed(), true, 6);
  //printInt(gps.sentencesWithFix(), true, 10);
  //printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));




  Serial.print(F("[SX1276] Transmitting packet ... "));
  //char output[50];
  //sprintf(output, "Counter: %d", counter++);
  //int state = radio.transmit(output);
  int state = radio.transmit(packet, 36);
  
  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));

    // print measured data rate
    Serial.print(F("[SX1276] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }









    
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
