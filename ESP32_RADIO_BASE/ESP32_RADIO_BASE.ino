#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <Ticker.h>
//#include <Servo.h>
#include <RadioLib.h>
#include <Wire.h>

int counter = 0;

byte packet[36];

char buffer[64];

// SX1276 requires the following connections:
int pin_cs = 26;
int pin_dio0 = 27;
int pin_nrst = 25;
int pin_dio1 = 14;

int pin_rx_enable = 13;
int pin_tx_enable = 12;
SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);

Ticker periodicTicker;

const char* ssid = "Blah";
const char* password =  "yourNetworkPass";

AsyncWebServer server(80);

//Coordinates to Bismarck Airport
static const double BISMARCK_AIRPORT_LAT = 46.775200, BISMARCK_AIRPORT_LNG = -100.7572;

//Unions for values larger than one byte
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




/******   Set up ESP Dash dashboard and cards    ********/
ESPDash dashboard(&server); 

//Buttons
Card button(&dashboard, BUTTON_CARD, "Self Destruct");

//GPS data
Card temperature(&dashboard, TEMPERATURE_CARD, "GPS Satilites", "Sats locked");
Card lat(&dashboard, GENERIC_CARD, "Latitude", "deg");
Card lon(&dashboard, GENERIC_CARD, "Longitude", "deg");
Card alt(&dashboard, GENERIC_CARD, "Altitude", "Meters");
Card speed(&dashboard, GENERIC_CARD, "Speed", "kmph");
Card pressure(&dashboard, GENERIC_CARD, "Pressure", "kmph");
Card tempcard(&dashboard, TEMPERATURE_CARD, "Air temp", "degrees");
Card magX(&dashboard, GENERIC_CARD, "Magnetic X", ".");
Card magY(&dashboard, GENERIC_CARD, "Magnetic y", ".");
Card magZ(&dashboard, GENERIC_CARD, "Magnetic Z", ".");

//Testing
Card humidity(&dashboard, HUMIDITY_CARD, "Humidity", "%");

/*
   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 16, TXPin = 17;

void updateCards() {
  float temp = 1;
  temperature.update(temp);
  lat.update((float)gpsLat.value);
  lon.update((float)gpsLng.value);
  alt.update((int)gpsAlt.value);
  speed.update((int)gpsSpeed.value);
  pressure.update((float)(50 / 100.0));
  tempcard.update((float)(20));
  magX.update(99);
  magY.update(99);
  magZ.update(99);
  
  
  dashboard.sendUpdates();
}
 

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, NULL);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("Connecting to WiFi network: ");
    Serial.println(ssid);
  }
  Serial.println(WiFi.localIP());
  server.begin();
  periodicTicker.attach_ms(500, updateCards);
  
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

  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);

  // set output power to 10 dBm (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission
  //       duty cycle MUST NOT exceed 1%
//  if (radio.setOutputPower(20) == ERR_INVALID_OUTPUT_POWER) {
//    Serial.println(F("Selected output power is invalid for this module!"));
//    while (true);
//  }
}

void loop()
{
  Serial.print(F("[SX1276] Receiving packet ... "));
  //byte packet[20];
  int state = radio.receive(packet, 35);
  
  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));
    
    //Update the telemetry
    telemetryUpdate();

    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1278] RSSI:\t\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1278] SNR:\t\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

    // print frequency error
    // of the last received packet
    Serial.print(F("[SX1278] Frequency error:\t"));
    Serial.print(radio.getFrequencyError());
    Serial.println(F(" Hz"));

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }







}

void telemetryUpdate()
{

  gpsLat.Byte[0] = packet[0];
  gpsLat.Byte[1] = packet[1];
  gpsLat.Byte[2] = packet[2];
  gpsLat.Byte[3] = packet[3];
  
  gpsLng.Byte[0] = packet[4];
  gpsLng.Byte[1] = packet[5];
  gpsLng.Byte[2] = packet[6];
  gpsLng.Byte[3] = packet[7];

  gpsAlt.Byte[0] = packet[8];
  gpsAlt.Byte[1] = packet[9];
  gpsAlt.Byte[2] = packet[10];
  gpsAlt.Byte[3] = packet[11];

  gpsSpeed.Byte[0] = packet[12];
  gpsSpeed.Byte[1] = packet[13];
  gpsSpeed.Byte[2] = packet[14];
  gpsSpeed.Byte[3] = packet[15];

  gpsHDOP.Byte[0] = packet[16];
  gpsHDOP.Byte[1] = packet[17];
  gpsHDOP.Byte[2] = packet[18];
  gpsHDOP.Byte[3] = packet[19];

  bmpAltitude.Byte[0] = packet[20];
  bmpAltitude.Byte[1] = packet[21];
  bmpAltitude.Byte[2] = packet[22];
  bmpAltitude.Byte[3] = packet[23];
  
  bmpPressure.Byte[0] = packet[24];
  bmpPressure.Byte[1] = packet[25];
  bmpPressure.Byte[2] = packet[26];
  bmpPressure.Byte[3] = packet[27];

  bmpTemperature.Byte[0] = packet[28];
  bmpTemperature.Byte[1] = packet[29];
  bmpTemperature.Byte[2] = packet[30];
  bmpTemperature.Byte[3] = packet[31];

  gpsSats.Byte[0] = packet[32];
  gpsSats.Byte[1] = packet[33];

  gpsAge.Byte[0] = packet[34];
  gpsAge.Byte[1] = packet[35];


  unsigned long distanceAirport =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gpsLat.value,
      gpsLng.value,
      BISMARCK_AIRPORT_LAT,
      BISMARCK_AIRPORT_LNG) / 1000;

  Serial.println();
  
  Serial.print("GPS Lat: ");
  Serial.println(gpsLat.value);

  Serial.print("GPS Lng: ");
  Serial.println(gpsLng.value);

  Serial.print("Distance to Airport (km): ");
  Serial.println(distanceAirport);

  Serial.print("GPS Alt: ");
  Serial.println(gpsAlt.value);

  Serial.print("GPS HDOP: ");
  Serial.println(gpsHDOP.value);

  Serial.print("Temp: ");
  Serial.println(bmpTemperature.value);

  sprintf(buffer,"GPS Latitude: %f, GPS Longitude ",gpsLat.value);
  Serial.println(buffer);
 // ltoa(gpsLat.value,buffer,10)
  



}