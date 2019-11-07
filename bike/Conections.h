// The ESP32 and TFT the pins used for testing are:
#define TFT_CS   33  // Chip select control pin (library pulls permanently low
#define TFT_DC   15  // Data Command control pin - must use a pin in the range 0-31
#define TFT_RST  32  // Reset pin, toggles on startup

#define TFT_WR    4  // Write strobe control pin - must use a pin in the range 0-31
#define TFT_RD    2  // Read strobe control pin

#define TFT_D0   12  // Must use pins in the range 0-31 for the data bus
#define TFT_D1   13  // so a single register write sets/clears all bits.
#define TFT_D2   26  // Pins can be randomly assigned, this does not affect
#define TFT_D3   25  // TFT screen update performance.
#define TFT_D4   17
#define TFT_D5   16
#define TFT_D6   27
#define TFT_D7   14

//Languaje
//EN
#define SPEED "SPEED"
#define ACTIVITY_TIME "ACTIVITY TIME"
