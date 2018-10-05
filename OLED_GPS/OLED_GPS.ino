

/*****************************************
  ESP32 GPS VKEL 9600 Bds
******************************************/

#include <TinyGPS++.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#define LED_BUILTIN 14


#define OLED_SDA 21
#define OLED_SCL 22

Adafruit_SH1106 display(OLED_SDA, OLED_SCL);

TinyGPSPlus gps;
HardwareSerial Serial1(1);
#define gpsSerial Serial1

#define PMTK_SET_NMEA_UPDATE_05HZ  "$PMTK220,2000*1C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// Flashy stuff
#define VERY_FAST 50
#define FAST 200
#define SLOW 500
#define FOREVER -1

void flash(int times, unsigned int speed) {
  while (times == -1 || times--) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(speed);
    digitalWrite(LED_BUILTIN, LOW);
    delay(speed);
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

  flash(10, VERY_FAST);
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
#if 0
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("GPS Tracker");
  display.display();
#endif
  flash(2, SLOW);

}

void loop()
{
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      flash(2, FAST);
      Serial.print("Latitude  : ");
      Serial.println(gps.location.lat(), 5);
      Serial.print("Longitude : ");
      Serial.println(gps.location.lng(), 4);
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
      Serial.print("Altitude  : ");
      Serial.print(gps.altitude.feet() / 3.2808);
      Serial.println("M");
      Serial.print("Time      : ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());
      Serial.println("**********************");

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.print("LAT ");
      display.println(gps.location.lat(), 5);
      display.print("LNG ");
      display.println(gps.location.lng(), 5);
      display.print("SAT ");
      display.println(gps.satellites.value(), 5);
      display.print("Time ");
      display.print(gps.time.hour());
      display.print(":");
      display.print(gps.time.minute());
      display.print(":");
      display.println(gps.time.second());
      display.display();

      delay(1000);
    }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
      flash(FOREVER, VERY_FAST);
    }
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

