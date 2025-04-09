#include "Wire.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#define SDA_PIN 5 // SDA pin for ESP32
#define SCL_PIN 6 // SCL pin for ESP32

#define RXD_PIN 8 // RXD pin for ESP32
#define TXD_PIN 7 // TXD pin for ESP32

struct SensorData {
  float x;
  float y;
  float z;
};
SensorData sensorData;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

void drawinfo(void) {
 display.clearDisplay();

 display.setTextSize(1);             
 display.setTextColor(SSD1306_WHITE);        // Draw white text
 display.setCursor(0,0);             // Start at top-left corner
 display.println(F("Hello Fab Academy!"));
 display.println(F("Sensor Data:"));
 display.print(F("X: ")); display.print(sensorData.x); display.println(F(" m/s^2"));
 display.print(F("Y: ")); display.print(sensorData.y); display.println(F(" m/s^2"));
 display.print(F("Z: ")); display.print(sensorData.z); display.println(F(" m/s^2"));
 display.display();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN); // Initialize Serial2 with RXD and TXD pins

  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C with specified SDA and SCL pins

 // I2C Scanner
 Serial.println(F("Scanning for I2C devices..."));
 byte count = 0;
 for (byte address = 1; address < 127; address++) {
   Wire.beginTransmission(address);
   if (Wire.endTransmission() == 0) {
    Serial.print(F("Found I2C device at address 0x"));
    if (address < 16) Serial.print(F("0"));
    Serial.println(address, HEX);
    count++;
   }
 }
 if (count == 0) Serial.println(F("No I2C devices found."));
 else Serial.print(F("Found ")); Serial.print(count); Serial.println(F(" device(s)."));

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  drawinfo();
}

void loop() {
  if(Serial2.available()){
    uint8_t data[sizeof(sensorData)];
    Serial2.readBytes((char*)data, sizeof(sensorData)); // Read the data from Serial2 into the buffer
    memcpy(&sensorData, data, sizeof(sensorData)); // Copy the buffer into the sensorData struct
    drawinfo();
  }else{
    delay(100);
  }
}
