#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Firmaware
#define DEBUG

// BME vars
//SCL A5
//SDA A4
Adafruit_BME280 bme; // I2C
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_TIMER 1000
float temperature = 0, pressure = 0, altitude = 0, humidity = 0;
unsigned long lastMillisBME = 0;

// GPS vars
#define GPS_RX 3
#define GPS_TX 4
#define GPS_BAUD 9600
#define GPS_TIMER 1000
#define MIN_SATELLITES 5
#define GPS_DELAY 500
TinyGPSPlus gps;
SoftwareSerial ss(GPS_TX, GPS_RX);
int satellites = 0, fix = 0, age = 0;
unsigned long lastMillisGPS = 0;
float latitude = 0, longitude = 0, speed = 0, hdop = 0;
TinyGPSDate date;
TinyGPSTime time;

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

    bmeSetup();
    gpsSetup();
}


void loop() {     
    readBME();
    gpsRead();
    #ifdef DEBUG
      //printValues();
    #endif
      logParameters();
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
  ss.begin(GPS_BAUD);
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
    time = gps.time;
    fix = gps.sentencesWithFix();
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
            time.hour(),
            time.minute(),
            time.second(),
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
