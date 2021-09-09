#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans12pt7b.h>
#include <TimeLib.h>
#include <PID_v1.h>

///////// VARIÁVEIS
int32 cron, timer;
int h, m, s;
float pwm;
int saida = 1;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 100, 10, 10, DIRECT);

///////// DEFINIÇÃO DO DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define I2C_SDA 5  //D1
#define I2C_SCL 4  //D2

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

///////// DEFINIÇÃO DO PID
// #define POT_PIN A0
#define OUTPUT_PIN 2


void setup()
{
  delay(100);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(OUTPUT_PIN, OUTPUT);
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
  display.println("TEMPERATURA");
  display.display();
  // display.startscrollleft(0x00,0x07);

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
  h = 1;
  m = 59;
  s = 59;

  // PID ------------------------------------------------------
  Setpoint = 30.0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = read_tm75();
  display.clearDisplay();
  // float temp = read_tm75();
  // display.setFont(&FreeSans12pt7b);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.printf("ATUAL:%4.1f", Input);
  display.printf("SET:  %4.1f", Setpoint);
  display.printf("PWM: %5.1f", Output);
  uint hora = h - hour();
  uint minuto = m - minute();
  uint segundo = s - second();
  if(hora==0 & minuto==0 & segundo==0)
  {
    Output = 0;
    analogWrite(OUTPUT_PIN, Output);  // desliga saida
    saida = 0;
  }
  if(saida==0) display.println("DESLIGADO");
  else
  {
    myPID.Compute();
    display.printf("%02dh%02dm%02ds", hora, minuto, segundo);
    Serial.print(Input);
    Serial.print(" ");
    Serial.print(second());
    Serial.print(" ");
    Serial.println(Output);
  }
  display.display();
}
