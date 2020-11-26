#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     
int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

void setup() {
   lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  // Start the Serial Monitor
  Serial.begin(115200);
  // Start the DS18B20 sensor
  sensors.begin();
}

void loop() {
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");


  // set cursor to first column, second row
  lcd.setCursor(0,0);
  lcd.print("Temp in C:");
   lcd.setCursor(11,0);
  lcd.print(temperatureC);
  lcd.setCursor(0,1);
  lcd.print("Temp in F:");
   lcd.setCursor(11,1);
  lcd.print(temperatureF);
   

}
