
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"

#define DHTPIN 5     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
const int oneWireBus = 4;     
int lcdColumns = 16;
int lcdRows = 2;

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
void setup() {
    lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  Serial.begin(115200);
  Serial.println(F("DHTxx test!"));

  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);

  //read humidity
  float h = dht.readHumidity();


  // Check if any reads failed and exit early (to try again).
  if (isnan(h)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.println("%");
  lcd.setCursor(0,0);
  lcd.print("Humidity:");
   lcd.setCursor(10,0);
  lcd.print(h);
  lcd.setCursor(15,0);
  lcd.print("%");
  

}
