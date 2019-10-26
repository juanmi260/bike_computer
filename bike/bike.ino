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
#include "Fonts/FreeMono24pt7b.h"
#include "Fonts/FreeSans12pt7b.h"
#include "Fonts/FreeSans18pt7b.h"
#include "Fonts/FreeSerif12pt7b.h"
#include "SevenSegmenstClassic20p.h"


/**********************************************************************************************************
 *                                          VARS ZONE                                                     *
 **********************************************************************************************************/
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



// UI Vars
unsigned long lastMillisOUT = 0;
#define OUT_TIMER 1000
int menu = 0;
boolean initLiveDataUI = false;
boolean initStatusBar = false;

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
    // Read sensors data
    readBME();
    gpsRead();

    // Refresh screen UI
    switch(menu) {
      case 0: // Status screen
        refreshStatusScreen();
        break;
      case 1: // Live data
        refreshLiveData();
        break;
      default: // Test screen
        refreshScreen();
        break;
    }
}

/**********************************************************************************************************
 *                                          Set up functions                                              *
 **********************************************************************************************************/
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
 * Gps setup
 */
void gpsSetup()
{
  Serial.println("GPS setup....: inicializando");
  ss.begin(GPS_BAUD, GPS_RX, GPS_TX);
  Serial.println("GPS setup....: ok");
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
    Serial.println("GPS setup....: ok");

    /*Adafruit_GFX_Button on_btn;
    on_btn.initButton(&tft,  60, 350, 100, 40, WHITE, CYAN, BLACK, "ON", 2);
    on_btn.drawButton(false);
    delay(5000);
    on_btn.drawButton(true);
    delay(5000);*/
}

/**********************************************************************************************************
 *                                  Retrieve sensors data functions                                       *
 **********************************************************************************************************/
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
/**********************************************************************************************************
 *                                        Serial debug functions                                          *
 **********************************************************************************************************/
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

/**********************************************************************************************************
 *                                              UI Helpers                                                *
 **********************************************************************************************************/
/**
 * Print message in LCD
 */
void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg, uint16_t fColor=WHITE, uint16_t bColor=BLACK)
{
    int16_t x1, y1;
    uint16_t wid, ht;
    //tft.drawFastHLine(0, y, tft.width(), RED);
    tft.setFont(f);
    tft.setCursor(x, y);
    tft.setTextColor(fColor, bColor);
    tft.setTextSize(sz);
    tft.print(msg);
    //delay(1000);
}

/**
 * Print message in LCD
 */
void showmsgXYVCenter(int x, int y, int sz, const GFXfont *f, const char *msg, uint16_t fColor=WHITE, uint16_t bColor=BLACK)
{
    int16_t x1, y1;
    uint16_t w, h;
    //tft.drawFastHLine(0, y, tft.width(), RED);
    tft.getTextBounds(msg, x, y, &x1, &y1, &w, &h);
    x = (tft.width()-w)/2;
    tft.setTextColor(fColor, bColor);
    //center
    showmsgXY(x, y, sz, f, msg, fColor, bColor);
}

/**
 * Print message in LCD
 */
void showmsgXYVCenterWidth(int x, int y, int sz, const GFXfont *f, const char *msg, int width, uint16_t fColor=WHITE, uint16_t bColor=BLACK)
{
    int16_t x1, y1;
    uint16_t w, h;
    //tft.drawFastHLine(0, y, tft.width(), RED);
    tft.getTextBounds(msg, x, y, &x1, &y1, &w, &h);
    //x = ((tft.width()-w+x)/2);
    //x = (tft.width()-width)+((width-w)/2);
    x = (width-w)/2 + x;
    tft.setTextColor(fColor, bColor);
    //center
    showmsgXY(x, y, sz, f, msg, fColor, bColor);
}

/**********************************************************************************************************
 *                                              UI Screens                                                *
 **********************************************************************************************************/
/**
 * Load splash screen while setting up other sensors
 */
/**
 * Splash screen UI
 */
void showSpashScreen()
{
  showmsgXYVCenter(0, 200, 1, &FreeSans9pt7b, "OpenSource Bike Computer");
  showmsgXYVCenter(0, 220, 1, &FreeSans9pt7b, "Rev1.0");
  showmsgXYVCenter(0, 240, 1, &FreeSans9pt7b, "By Juanmi_260");
  delay(1000);
}

/**
 * TEST Screen UI
 */
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

/**
 * Status screen UI
 */
void refreshStatusScreen()
{
  drawStatusBar();
  //TODO draw some data
}

/**
 * Live data UI
 */
void refreshLiveData()
{
  if (millis() - lastMillisTFT >= TFT_TIMER) {
    lastMillisTFT = millis();
    //Init live data UI only once
    if (!initLiveDataUI) {
      initLiveDataUI = true;
      showmsgXYVCenter(0, 19, 1, &FreeSans12pt7b, "TIME");
      tft.drawFastHLine(0, 96, tft.width(), WHITE);
      showmsgXYVCenterWidth(220, 98, 1, &FreeSans12pt7b, "AVG", 100);
      tft.drawFastVLine(220, 96, 96, WHITE);
      showmsgXYVCenterWidth(220, 98+32, 1, &FreeSans12pt7b, "MAX", 100);
      tft.drawFastHLine(220, 96+32, 100, WHITE);
      showmsgXYVCenterWidth(220, 98+64, 1, &FreeSans12pt7b, "MIN", 100);
      tft.drawFastHLine(220, 96+64, 100, WHITE);
      showmsgXYVCenterWidth(0, 117, 1, &FreeSans12pt7b, "SPEED", 220);
      
      tft.drawFastHLine(0, 192, tft.width(), WHITE);
      showmsgXYVCenter(0, 213, 1, &FreeSans12pt7b, "DISTANCE");
      
      tft.drawFastHLine(0, 288, tft.width(), WHITE);
      showmsgXYVCenter(0, 309, 1, &FreeSans12pt7b, "ACTIVITY TIME");
      
      tft.drawFastHLine(0, 384, tft.width(), WHITE);
      showmsgXYVCenter(0, 405, 1, &FreeSans12pt7b, "CAL");
    }

    //Refresh Time
    char buffer[32], oldBuffer[32]={};
    
    sprintf(buffer, "%02d:%02d:%02d ", timegps.hour(), timegps.minute(), timegps.second());
    showmsgXYVCenter(0, 87, 1, &FreeBigFont, buffer);
    //Refresh Speed
    showmsgXYVCenter(0, 183, 1, &FreeMono24pt7b, "20");
    //Refresh Distance
    showmsgXYVCenter(0, 279, 1, &FreeMono24pt7b, "000179");
    //Refresh ActivityTime
    showmsgXYVCenter(0, 375, 1, &FreeMono24pt7b, buffer);
    //Refresh CAL
    showmsgXYVCenter(0, 471, 1, &FreeMono24pt7b, "50");

    delay(2000);
    sprintf(buffer, "%02d:%02d:%02d ", timegps.hour(), timegps.minute(), timegps.second()+1);
    showmsgXYVCenter(0, 87, 1, &FreeBigFont, buffer);
  }
}

/**********************************************************************************************************
 *                                              UI Components                                             *
 **********************************************************************************************************/
void drawBattery(int x, int y, int percentage, bool charging, uint16_t bColor)
{
  // Draw battery
  int heigh = 26;
  int width = 40;
  int margin = 8;
  //Battery perimeter color set to blue if charging
  //bool charging = true;
  uint16_t batteryColor = WHITE;  
  if (charging) {
    batteryColor = RGB(0, 127, 255);
  }
  //Battery perimeter
  tft.drawRect(x, y, width, heigh, batteryColor);
  //Battery pin
  tft.fillRect(x+width, y+8, 4, 8, batteryColor);
  
  //Infill percentage and color
  uint16_t color = GREEN;
  if (percentage <= 20 && percentage > 10) {
    color = YELLOW;
  } else if (percentage <=10) {
    color = RED;
  }
  tft.fillRect(x+2, y+2, ((width-4)*percentage)/100, heigh-4, color);
  tft.fillRect(x+2+(((width-4)*percentage)/100), y+2, width-4-(((width-4)*percentage)/100), heigh-4, bColor);
  //tft.fillRect(x+2+(((width-4)*percentage)/100), y+2, heigh-4, bColor);

  //Battery percentage number
  char buffer[10];
  sprintf(buffer, "%3d%s", percentage, "%");
  tft.setFont();
  tft.setCursor(x+width+margin, y+4);
  tft.setTextColor(WHITE, bColor);
  tft.setTextSize(2);
  tft.print(buffer);
}

void drawGps(int x, int y, int satellites, uint16_t bColor)
{
  int margin = 2;
  int r = 8;
  //Draw perimeter
  tft.drawCircle(x+2+r,y+2+r,r, WHITE);
  tft.drawFastHLine((x+(2+r+r+1)), y+2+r, 2, WHITE);
  tft.drawFastHLine(x, y+2+r, 2, WHITE);
  tft.drawFastVLine(x+2+r, y, 2, WHITE);
  tft.drawFastVLine(x+2+r, y+2+r+r+1, 2, WHITE);

  //Draw Fix BLACK|WHITE|GREEN
  uint16_t satColor = bColor;
  if (satellites >= 3 && satellites < 5){
    satColor = WHITE;
  }else if (satellites >= 5) {
    satColor = GREEN;
  }
  tft.fillCircle(x+2+r,y+2+r,r-2, satColor);

  //Draw number of satellites
  char buffer[10];
  sprintf(buffer, "%2d",  satellites);
  tft.setFont();
  tft.setCursor(x+2+r+r+2+margin*2, y+3);
  tft.setTextColor(WHITE, bColor);
  tft.setTextSize(2);
  tft.print(buffer);
}

void drawTime(uint16_t bColor)
{
  char buffer[6];
  sprintf(buffer, "%02d:%02d ", timegps.hour(), timegps.minute());
  int16_t x=0, y=0, x1, y1;
    uint16_t w, h;
    //tft.drawFastHLine(0, y, tft.width(), RED);
  tft.getTextBounds(buffer, x, y, &x1, &y1, &w, &h);
  tft.setFont();
  tft.setCursor(tft.width()-w, 5+3);
  tft.setTextColor(WHITE, bColor);
  tft.setTextSize(2);
  tft.print(buffer);
}

/**
 * Draw status bar in screens where it's needed.
 */
void drawStatusBar()
{
  uint16_t statusBarColor = RGB(20,20,20);
  if (!initStatusBar) {
    initStatusBar = !initStatusBar;
    tft.fillRect(0,0, tft.width(), 32, statusBarColor);
  }
  drawBattery(5, 4, 100, false, statusBarColor);
  drawGps(120, 4, satellites, statusBarColor);
  drawTime(statusBarColor);
}
