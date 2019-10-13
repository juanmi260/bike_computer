# Open source bike computer
Open source bike computer

## Project propurse
The project objetive is create an opensource bike computer to get the main training rider parameters and export a gpx file to be analyzed in third party application.

Another objetive is create a Progresive web app to sync data between bike computer and smartphone, making some direct integrations with third party apps like Strava.

### Measured parameters
* GPS
    - Location
    - Speed
    - Altitude
    - Distance
    - Dete and time
* Hall sersors
    - Cadence
    - Speed*
    - Distance*
* Barometric + temperature sensor (BMP280)
    - Altitude
    - Temperature
    - Atmospheric pressure

(\*) Optional: only when gps is not available

### Working modes
Bike computer will have an interface to make some settings in the system, watch system status and change between some modes adapted to rider activity.
#### Free mode
Bike computer will provide live data.

#### Route mode
The computer will provide data of route like averages data for:
    * Route.
    * Last kilometer.
    * Actual kilometer with forecast for the end of the kilometer.

#### Track mode
After configure the track start/finish point, the computer will provide:
    * Lap statistics.
    * Current lap comparision with best and last lap.
    * Current lap forecast.

#### Tracking Mode
Like route mode with activity traking in gpx format saved in SD card.

## Hardware
### Bill of materials
|Uds.|Item                           |Description                    |
|----|-------------------------------|-------------------------------|
|1   |ESP32                          |ESP32                          |
|1   |Pantalla TFT                   |Tactil screen tft 3.5" RGB + SD|
|2   |Hall sensor                    |Digital halll sensor           |
|1   |GPS                            |GPS+GLONNAS                    |
|2   |Magnet                         |Neodimiun magnet               |
|3   |Wires                          |AWG28 3 pin wire               |
|1   |Battery                        |LG18650                        |
|1   |Barometric + temperature sensor|BMP280                         |
|2   |Capacitor                      |Capacitor 1 microF             |

### Conections

## Software
### Libraries
* TinyGPS++

### Sketch notes
```cpp
```


