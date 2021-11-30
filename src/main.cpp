#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <PID_v1.h>
#include <ESP8266WiFi.h>

///////// VARIÁVEIS
uint h, m, s, t;
bool inicio=0;
unsigned long previousMillis = 0;
uint contador=1, incremento;
uint setpoint_max=70;
bool click=1;
unsigned long currentMillis;
double tempo, input_ant;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 100, 8, 0, DIRECT);

///////// DEFINIÇÃO DO DISPLAY
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define I2C_SDA 5 //D1
#define I2C_SCL 4 //D2
#define OUTPUT_PIN 14 //PWM resistencia via MOSFET
#define COOLER_PIN 13 //COOLER via MOSFET
#define BUTTON_PIN 12

///////// DEFINIÇÃO DO SENSOR
int TMP75_ADDR = 0x49;
const byte TMP75_CONFIG_REG = 0x01;
const byte TMP75_TEMP_REG = 0x00;

float read_tm75() {
  Wire.requestFrom(TMP75_ADDR, 2);
  int tm75_reg = (byte)Wire.read() << 8;
  tm75_reg |= (byte)Wire.read();
  tm75_reg >>= 4;
  // In 12-bit mode, resolution is 0.0625°C (1/16th)
  return tm75_reg / float(16);
}

void atualiza_display(void) {
  uint hora = t / 3600;
  uint minuto = (t / 60) % 60;
  uint segundo = t % 60;
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.printf("TEMP: %4.1f\r\n", Input);
  display.printf("SET:  %4.1f\r\n", Setpoint);
  display.printf("PWM: %5.0f\r\n", Output);
  if(t>0) display.printf(" %2dh%02dm%02ds", hora, minuto, segundo);
  else display.printf("DESLIGADO");
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.print(Input);
  Serial.print(" ");
  Serial.println(Output);
  display.display();
}

void le_botao(void) {
  currentMillis = millis();
  tempo = currentMillis - previousMillis;
  if ((!digitalRead(BUTTON_PIN)) && (tempo >= 100) && (tempo < 2000))
  {
    incremento++;
    click = 1;
    previousMillis = currentMillis;
    Serial.println(tempo);
  }
  if ((tempo >= 3000))
  {
    contador++;
    click = 1;
    previousMillis = currentMillis;
    Serial.println(tempo);
  }
}

void setup() {
  delay(100);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  analogWriteFreq(100);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(COOLER_PIN, OUTPUT); //pino D6
  digitalWrite(COOLER_PIN, LOW); // desliga cooler
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
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("TEMPERATURA");
  display.display();

  // TMP75 CONFIG ---------------------------------------------
  Wire.beginTransmission(TMP75_ADDR); // Address the TMP75 sensor
  Wire.write(TMP75_CONFIG_REG);       // Select the configuration register
  Wire.write(0b01100000);             // Write desired config: No Shutdown mode, Termostat in Comp. mode, Def. Polarity, Fault queue 1, 12 bit resolution, One-Shot disabled
  Wire.endTransmission();
  Wire.beginTransmission(TMP75_ADDR); // Address the TMP75 sensor
  Wire.write(TMP75_TEMP_REG);         // Select the temperature register, so further read requests will retrieve from it.
  Wire.endTransmission();

  // PID ------------------------------------------------------
  myPID.SetOutputLimits(0, 1023);
  Setpoint = 30;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()  {
  if (inicio == 0)
  { //se zerar o relógio ou é início desliga PWM e cooler
    display.clearDisplay();
    Output = 0;
    analogWrite(OUTPUT_PIN, Output); // desliga saida
    digitalWrite(COOLER_PIN, LOW); // se desligar resistencia desliga cooler
    contador=1;
    Serial.println(contador);
    Serial.println("CONTADOR = 1");
    incremento = 30;
    while (contador==1) {
      le_botao();
      if(click) {  
        click = 0;   
        if(incremento > setpoint_max) incremento = 30;        
        Setpoint = incremento;
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println("SETPOINT");
        display.println(" ");
        display.setTextSize(4);
        display.printf("%u", (uint)Setpoint);
        display.write(9);        
        display.println("C");
        display.display();
      }
    delay(200);
    }

    Serial.println("CONTADOR = 2");
    click=1;
    incremento = 0;
    while (contador == 2) {
      // Serial.println("CONTADOR = 2");
      le_botao();
      if (click)  {
        click = 0;
        if (incremento > 8) incremento = 0;
        h = incremento;
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println("HORAS");
        display.println(" ");
        display.setTextSize(3);
        display.printf("%2uh%02um", h, m);
        display.display();
      }
    delay(200);
    }

    Serial.println("CONTADOR = 3");
    click=1;
    incremento = 0;
    while (contador == 3)  {
      // Serial.println("CONTADOR = 3");
      le_botao();
      if (click)  {
        click=0;
        if (incremento > 60) incremento = 0;
        m = incremento;
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println("MINUTOS");
        display.println(" ");
        display.setTextSize(3);
        display.printf("%2uh%02um", h, m);
        display.display();
      }
    delay(200);
    }
    t = m * 60 + h * 3600;
    h=0,m=0,s=0;
    atualiza_display();
    digitalWrite(COOLER_PIN, HIGH);
  }


  inicio = 1; //comeca a rodar o programa
  if(!digitalRead(BUTTON_PIN)) inicio=0;
  if(t>0) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 1000) {
      previousMillis = currentMillis;
      t--;
      atualiza_display();
    }
    Input = read_tm75();
    myPID.Compute();
    if (Input != input_ant) {
      atualiza_display();
      input_ant = Input;
    }
    analogWrite(OUTPUT_PIN, Output);
  }
  else {
    Output = 0;
    analogWrite(OUTPUT_PIN, Output); // desliga saida
    digitalWrite(COOLER_PIN, LOW); // se desligar resistencia desliga cooler
    Input = read_tm75();
    atualiza_display();
  }
}