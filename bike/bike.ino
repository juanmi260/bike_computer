#include "Conections.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <FreeDefaultFonts.h>
#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSans12pt7b.h"
#include "Fonts/FreeSerif12pt7b.h"

// Firmaware
#define DEBUG

// BME values
//SCL_BME A5 21(esp32)
//SDA_BME A4 22(esp32)
Adafruit_BME280 bme; // I2C
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_TIMER 1000
float temperature = 0, pressure = 0, altitude = 0, humidity = 0;
unsigned long lastMillisBME = 0;

// GPS values
#define GPS_RX 34
#define GPS_TX 35
#define GPS_BAUD 9600
#define GPS_TIMER 1000
#define MIN_SATELLITES 5
#define GPS_DELAY 500
TinyGPSPlus gps;
SoftwareSerial ss;
int satellites = -1, fix = 0, age = 0;
unsigned long lastMillisGPS = 0;
float latitude = 0, longitude = 0, speed = 0, hdop = 0;
TinyGPSDate date;
TinyGPSTime timegps;

// TFT-LCD-TouchScreen values
MCUFRIEND_kbv tft;

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define RGB(r, g, b) (((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3))

#define GREY      RGB(127, 127, 127)
#define DARKGREY  RGB(64, 64, 64)
#define TURQUOISE RGB(0, 128, 128)
#define PINK      RGB(255, 128, 192)
#define OLIVE     RGB(128, 128, 0)
#define PURPLE    RGB(128, 0, 128)
#define AZURE     RGB(0, 128, 255)
#define ORANGE    RGB(255,128,64)

#define TFT_TIMER 1000
unsigned long lastMillisTFT = 0;



// Print vars
unsigned long lastMillisOUT = 0;
#define OUT_TIMER 1000

// Log vars
unsigned long lastMillisLOG = 0;
#define LOG_TIMER 1000

//Accumulate vars
  // Speed
  // Distance
  // Cadence
  // Altitude
  

/**
 * Setup sensors and pins
 */
void setup() {
    
    Serial.begin(9600);
    while(!Serial);    // time to get serial running

    Serial.println("Starting gps tracker");

    tftSetup();
    showSpashScreen();

    //Setup sensors
    bmeSetup();
    gpsSetup();
    
    tft.fillScreen(BLACK);
}


void loop() {     
    readBME();
    gpsRead();
    int menu = 0;
    switch(menu) {
      case 0:
          refreshScreen();
          break;
      case 1:
        refreshStatusScreen();
        break;
      case 2:
        refreshScreen();
        break;
    }
    
    refreshScreen();
    //printValues();
    #ifdef DEBUG
      //printValues();
    #endif
      //logParameters();
}

/**
 * Setup BMESensor
 */
void bmeSetup()
{
  unsigned status;
    
    Serial.println("BME setup....: inicializando");
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1);
    }
    
    Serial.println("BME setup....: ok");
}

/**
 * Read BME sensor values and refresh BME vars
 */
void readBME()
{
  if (millis() - lastMillisBME >= BME_TIMER) {
    lastMillisBME = millis();
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    //altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();
    Serial.println(temperature);
  }
}

/**
 * Gps setup
 */
void gpsSetup()
{
  Serial.println("GPS setup....: inicializando");
  ss.begin(GPS_BAUD, GPS_RX, GPS_TX);
  Serial.println("GPS setup....: ok");
}

/**
 * Read GPS values and refresh GPS vars
 */
void gpsRead()
{
    unsigned long start = millis();
    do {
      while (ss.available())
        gps.encode(ss.read());
    } while (millis() - start < GPS_DELAY);
      
    satellites = gps.satellites.value();
    latitude = gps.location.lat();
    altitude = gps.altitude.meters();
    longitude = gps.location.lng();
    speed = gps.speed.kmph();
    date = gps.date;
    timegps = gps.time;
    fix = gps.sentencesWithFix();

    Serial.print("GPS satellites: ");
    Serial.println(satellites);
}

/**
  * Output data in serial
  */
void printValues()
{
  if (millis() - lastMillisOUT >= OUT_TIMER) {
    lastMillisOUT = millis();
    char lat[15], lng[15], spd[15];
    char buffer[200];
    sprintf(buffer, "Satelites: %i Latitud: %s Longitud: %s Velocidad: %s", satellites, dtostrf(latitude, 3, 7, lat), dtostrf(longitude, 3, 7, lng), dtostrf(latitude, 3, 7, spd));
    Serial.println(buffer);
  }
}

/**
  * Log string with the following params
  * - latitude
  * - longitude
  * - elevation
  * - speed
  * - satellites
  * - date-time
  *
  *
  *
  *
  *
  *
  *
  *
  */
void logParameters()
{
    if (millis() - lastMillisLOG >= LOG_TIMER) {
        lastMillisLOG = millis();
        int numberOfParams = 6;
        int maxParamLength = 15;
        char data[numberOfParams][maxParamLength];
        char lat[15], lng[15], spd[15], ele[15];
        char buffer[500];
        //sprintf(buffer, "Satelites: %i Latitud: %s Longitud: %s Velocidad: %s", satellites, dtostrf(latitude, 3, 7, lat), dtostrf(longitude, 3, 7, lng), dtostrf(latitude, 3, 7, spd));
        sprintf(
            buffer,
            "%s,%s,%s,%d-%d-%dT%02d:%02d%:%02d,%d,%d,%s,%d,%s,%s,%s",
            dtostrf(latitude, 3, 7, data[0]),       //Latitude
            dtostrf(longitude, 3, 7, data[1]),      //longitude
            dtostrf(altitude, 3, 7, data[2]),      //elevation
            //Time ISO 8601 2019-10-12T17:52:01Z
            date.year(),
            date.month(),                            
            date.day(),
            timegps.hour(),
            timegps.minute(),
            timegps.second(),
            fix,
            satellites,
            dtostrf(hdop, 1, 6, data[3]),
            age,
            dtostrf(temperature, 3, 2, data[4]),
            dtostrf(pressure, 3, 2, data[5]),
            dtostrf(humidity, 3, 2, data[6])
        );
        Serial.println(buffer);
    }
}

/**
 * TFT setup
 */
void tftSetup() {
    Serial.println("TFT setup....: inicializando");
    uint16_t ID;
    tft.reset();
    ID = tft.readID();
    Serial.print("TFT ID = 0x");
    Serial.println(ID, HEX);
    //    if (ID == 0xD3D3) ID = 0x9481; // write-only shield
    if (ID == 0xD3D3) ID = 0x9486; // write-only shield
    tft.begin(ID);
    tft.setRotation(0);
    tft.fillScreen(BLACK);
    showmsgXY(5, 180, 1, &FreeSevenSegNumFont, "Hola");
    Serial.println("GPS setup....: ok");
}

/**
 * Print message in LCD
 */
void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg)
{
    int16_t x1, y1;
    uint16_t wid, ht;
    //tft.drawFastHLine(0, y, tft.width(), RED);
    tft.setFont(f);
    tft.setCursor(x, y);
    tft.setTextColor(WHITE,RED);
    tft.setTextSize(sz);
    tft.print(msg);
    delay(1000);
}

/**
 * Load splash screen while setting up other sensors
 */
void showSpashScreen()
{
  showmsgXY(20, 200, 1, &FreeSans12pt7b, "Opensource Bike Computer");
  showmsgXY(130, 220, 1, &FreeSans9pt7b, "Rev1.0");
  showmsgXY(90, 240, 1, &FreeSans9pt7b, "By Juanmi_260");
  delay(1000);
}

void refreshScreen()
{
  if (millis() - lastMillisTFT >= TFT_TIMER) {
    char buffer[20];
    char tmp[20];
    lastMillisTFT = millis();
    sprintf(buffer, "Temperature: %s", dtostrf(temperature, 3, 2, tmp));
    showmsgXY(20, 200, 1, &FreeSans12pt7b, buffer);
    /*char lat[15], lng[15], spd[15];
    char buffer[200];
    sprintf(buffer, "Satelites: %i Latitud: %s Longitud: %s Velocidad: %s", satellites, dtostrf(latitude, 3, 7, lat), dtostrf(longitude, 3, 7, lng), dtostrf(latitude, 3, 7, spd));
    Serial.println(buffer);*/
  }
}

void refreshStatusScreen()
{
  tft.fillScreen(RED);
}
