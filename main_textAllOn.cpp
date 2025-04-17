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
#define CE_PIN 8
#define CSN_PIN 7

const byte slaveAddress[6] = "88806";

RF24 radio(CE_PIN, CSN_PIN);  // Create a Radio

char dataToSend[10] = "Message 0";
char txNum = '0';

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000;

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

//================

void send() {

  bool rslt;
  rslt = radio.write(&dataToSend, sizeof(dataToSend));
  // Always use sizeof() as it gives the size as the number of bytes.
  // For example if dataToSend was an int sizeof() would correctly return 2

  Serial.print("Data Sent ");
  Serial.print(dataToSend);
  if (rslt) {
      Serial.println("  Acknowledge received");
      updateMessage();
  }
  else {
      Serial.println("  Tx failed");
      updateMessage();
  }
}

//================

void updateMessage() {
  // so you can see that new data is being sent
  txNum += 1;
  if (txNum > '9') {
    txNum = '0';
  }
  dataToSend[8] = txNum;
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

  Serial.println("SimpleTx Starting");

  while (!radio.begin())
  {
    Serial.println("radio hardware is not responding!!");
  }
  
  radio.enableAckPayload();
  radio.setChannel(90);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(slaveAddress);
  radio.stopListening();
}

void loop()
{
  moveMotor(255, 'D');
  delay(1000);
  moveMotor(255, 'U');
  delay(1000);

  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    send();
    prevMillis = millis();
  }

  lcd.setCursor(0, 1);
  lcd.print("12345678901234567890");
  lcd.setCursor(0, 2);
  lcd.print("12345678901234567890");
  lcd.setCursor(0, 3);
  lcd.print("12345678901234567890");
  lcd.setCursor(0, 0);
  lcd.print("12345678901234567890");

  FastLED.setBrightness(1);
  leds[0] = CRGB::Amethyst;
  leds[1] = CRGB::Aquamarine;
  LEDS.show();

  Serial.print(digitalRead(MANUAL_SWITCH));
  Serial.print("  ");
  Serial.print(digitalRead(PULL_SWITCH));
  Serial.print("  ");
  Serial.println(digitalRead(PUSH_SWITCH));
}
