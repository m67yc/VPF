#include <Arduino.h>
#include <Wire.h>
#include <MS5837.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <FastLED.h>

RF24 radio(8, 7); /* CE, CSN */
hd44780_I2Cexp lcd;
CRGB leds[2];
MS5837 pressure_sensor;

/* communication */
const byte address[6] = "14514";
const byte oaddress[6] = "19198";

const int QUEUE_LEN = 180;
char queue[QUEUE_LEN + 10];
byte serial_reader[1];
int st = 0, en = 0, size = 0;
int tick = 0;
int status = 0;

/*  */
constexpr uint8_t MANUAL_SWITCH = A1;
constexpr uint8_t PUSH_SWITCH = A2;
constexpr uint8_t PULL_SWITCH = A3;

const uint8_t MotorPin1 = 5;
const uint8_t MotorPin2 = 6;

float currentPressure;
float pastPressure = 9999;
const uint16_t tgtPressure = 1055;
float startPressure = 1023;
float minPressure = 10000;

bool autoMovement = 0;
bool first_time_pulling = 0;
bool downing = 0, uping = 0;

uint16_t max_errorPressure = tgtPressure, min_errorPressure = tgtPressure;
uint8_t errorPressure = 2;

uint32_t last_time;

void moveMotor(uint8_t Speed, char mode)
{
  if (mode == 'U')
  {
    lcd.setCursor(0, 1);
    lcd.print("Floating!");
    analogWrite(MotorPin1, 0);
    analogWrite(MotorPin2, Speed);
  }
  else if (mode == 'D')
  {
    lcd.setCursor(0, 1);
    lcd.print("Sinking!");
    analogWrite(MotorPin1, Speed);
    analogWrite(MotorPin2, 0);
  }
  else if (mode == 'S')
  {
    lcd.setCursor(0, 1);
    lcd.print("Stopping!");
    analogWrite(MotorPin1, 0);
    analogWrite(MotorPin2, 0);
  }
}

void ManualMode()
{
  if (!digitalRead(PULL_SWITCH) && digitalRead(PUSH_SWITCH)) /* A2 high -> sinking */
  {
    moveMotor(255, 'D');
  }
  else if (digitalRead(PULL_SWITCH) && !digitalRead(PUSH_SWITCH)) /* A3 high -> rising */
  {
    moveMotor(255, 'U');
  }
  else
  {
    moveMotor(0, 'S');
  }
}

void setup()
{
  lcd.begin(20, 4);
  lcd.clear();
  lcd.init();
  Serial.begin(9600);

  // while (!pressure_sensor.init())
  // {
  //   Serial.println("Init failed!");
  //   Serial.println("Are SDA/SCL connected correctly?");
  //   Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
  //   Serial.println("\n\n\n");
  //   delay(5000);
  // }
  pressure_sensor.setFluidDensity(997);
  pressure_sensor.setModel(0);

  LEDS.addLeds<WS2812, A0, GRB>(leds, 2);

  pinMode(MANUAL_SWITCH, INPUT_PULLUP);
  pinMode(PUSH_SWITCH, INPUT_PULLUP);
  pinMode(PULL_SWITCH, INPUT_PULLUP);
}

void loop()
{
  moveMotor(255, 'D');
  lcd.setCursor(0, 1);
  lcd.print("12345678901234567890");
  lcd.setCursor(0, 2);
  lcd.print("12345678901234567890");
  lcd.setCursor(0, 3);
  lcd.print("12345678901234567890");
  lcd.setCursor(0, 0);
  lcd.print("12345678901234567890");

  leds[0] = CRGB::White;
  leds[1] = CRGB::White;
  LEDS.show();
}
