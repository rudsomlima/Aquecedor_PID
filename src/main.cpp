#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans12pt7b.h>
#include <TimeLib.h>

///////// VARIÁVEIS
int32 cron, timer;
int h, m, s;

///////// DEFINIÇÃO DO DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define I2C_SDA 5
#define I2C_SCL 4

///////// DEFINIÇÃO DO SENSOR
int TMP75_ADDR = 0x49;
const byte TMP75_CONFIG_REG = 0x01;
const byte TMP75_TEMP_REG = 0x00;

float read_tm75()
{
  Wire.requestFrom(TMP75_ADDR, 2);
  int tm75_reg = (byte)Wire.read() << 8;
  tm75_reg |= (byte)Wire.read();
  tm75_reg >>= 4;
  // In 12-bit mode, resolution is 0.0625°C (1/16th)
  return tm75_reg / float (16);
}
extern volatile unsigned long timer0_millis; //Pra poder zerar o timer 0 lá na frente 

void setup()
{
  delay(100);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  //digitalWrite(led, HIGH);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(9600);
  Serial.println("");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("LIGANDO...");
  display.display();

  // TMP75 CONFIG ---------------------------------------------
  Wire.beginTransmission(TMP75_ADDR); // Address the TMP75 sensor
  Wire.write(TMP75_CONFIG_REG);       // Select the configuration register
  Wire.write(0b01100000);             // Write desired config: No Shutdown mode, Termostat in Comp. mode, Def. Polarity, Fault queue 1, 12 bit resolution, One-Shot disabled
  Wire.endTransmission();
  Wire.beginTransmission(TMP75_ADDR); // Address the TMP75 sensor
  Wire.write(TMP75_TEMP_REG);         // Select the temperature register, so further read requests will retrieve from it.
  Wire.endTransmission();

  // TIMER ------------------------------------------------------
  time_t t = now();
  setTime(0, 0, 0, 4, 9, 2021);
}

void loop()
{
  display.clearDisplay();
  float temp = read_tm75();
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" C");
  delay(500);
  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 33);
  display.printf("%4.1f", temp);
  display.setCursor(0, 52);
  display.printf("%2d:%2d:%2d",2-hour(),60-minute(),60-second());
  display.display();
}