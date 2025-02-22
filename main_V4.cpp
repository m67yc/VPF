#include <Arduino.h>
#include <Wire.h>
#include <MS5837.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

hd44780_I2Cexp lcd;
MS5837 pressure_sensor;

constexpr uint8_t MANUAL_SWITCH = A1;
constexpr uint8_t PUSH_SWITCH = A2;
constexpr uint8_t PULL_SWITCH = A3;

const uint8_t MotorPin1 = 5;
const uint8_t MotorPin2 = 6;

float currentPressure, previousPressure;
const uint16_t tgtPressure = 1055;
float startPressure = 1023;
float minPressure = 0;

bool autoMovement = 0;
bool first_time_pulling = 0;

long previousMillis = millis();

uint16_t max_errorPressure = tgtPressure, min_errorPressure = tgtPressure;
uint8_t errorPressure = 1;

void moveMotor(uint8_t MotorSpeed1, uint8_t MotorSpeed2)
{
  MotorSpeed1 = constrain(MotorSpeed1, 0, 255);
  MotorSpeed2 = constrain(MotorSpeed2, 0, 255);
  analogWrite(MotorPin1, MotorSpeed1);
  analogWrite(MotorPin2, MotorSpeed2);
}

void setup()
{
  lcd.begin(20, 4);
  lcd.clear();
  lcd.init();
  Serial.begin(9600);

  while (!pressure_sensor.init())
  {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  pressure_sensor.setFluidDensity(997);
  pressure_sensor.setModel(0);

  pinMode(MANUAL_SWITCH, INPUT_PULLUP);
  pinMode(PUSH_SWITCH, INPUT_PULLUP);
  pinMode(PULL_SWITCH, INPUT_PULLUP);
}

void loop()
{
  pressure_sensor.read();
  lcd.clear();
  currentPressure = pressure_sensor.pressure();
  lcd.setCursor(0, 0);
  lcd.print("Pressure: ");
  lcd.print(pressure_sensor.pressure());

  if (millis() - previousMillis == 10)
  {
    previousPressure = currentPressure;
    previousMillis = millis();
  }

  if (autoMovement == 0)
  {
    lcd.setCursor(0, 2);
    lcd.print("Manual mode");
    if (digitalRead(MANUAL_SWITCH)) /* A1 LOW -> Auto mode */
      autoMovement = 1;
    else
    {
      if (!digitalRead(PULL_SWITCH) && digitalRead(PUSH_SWITCH)) /* A2 high -> sinking */
      {
        lcd.setCursor(0, 1);
        lcd.print("Sinking!");
        moveMotor(255, 0);
      }
      else if (digitalRead(PULL_SWITCH) && !digitalRead(PUSH_SWITCH)) /* A3 high -> rising */
      {
        lcd.setCursor(0, 1);
        lcd.print("Rising!");
        moveMotor(0, 255);
      }
      else
      {
        lcd.setCursor(0, 1);
        lcd.print("Stopping!");
        moveMotor(0, 0);
      }
    }
  }
  else
  {
    lcd.setCursor(0, 2);
    lcd.print("Auto mode!");

    if (currentPressure < startPressure) /* not work before in water */
      moveMotor(0, 0);
    else
    {
      if (first_time_pulling == 0)
      {
        lcd.setCursor(0, 1);
        lcd.print("Sinking!");
        moveMotor(255, 0);
        minPressure = currentPressure;

        if (currentPressure > previousPressure + 2) /* now higher than pass -> it is sinking -> stop */
        {
          first_time_pulling = 1;
          moveMotor(0, 0);
        }
      }
      else if (currentPressure > (tgtPressure + 5) && currentPressure < (tgtPressure - 5)) /* very neer tgt -> stop */
      {
        lcd.setCursor(0, 1);
        lcd.print("Stopping!");
        moveMotor(0, 0);
      }
      else if (currentPressure > tgtPressure) /* under tgt */
      {
        min_errorPressure = tgtPressure;
        if (max_errorPressure < currentPressure)
          max_errorPressure = currentPressure;

        if (max_errorPressure - currentPressure <= errorPressure + 1) /* now under pass -> it is sinking -> let it float */
        {
          lcd.setCursor(0, 1);
          lcd.print("Floating!");
          moveMotor(0, 100);
        }
        else
        {
          max_errorPressure = currentPressure + (errorPressure + 3);
          lcd.setCursor(0, 1);
          lcd.print("Stopping!");
          moveMotor(90, 0);
          moveMotor(0, 0);
        }
      }
      else /* higher than tgt */
      {
        max_errorPressure = tgtPressure;
        if (min_errorPressure > currentPressure && currentPressure < minPressure)
          min_errorPressure = currentPressure;

        if (currentPressure - min_errorPressure <= errorPressure + 1) /* now higher than pass -> it is floating -> let it sink */
        {
          lcd.setCursor(0, 1);
          lcd.print("Sinking!");
          moveMotor(100, 0);
        }
        else
        {
          min_errorPressure = currentPressure - (errorPressure + 3);
          lcd.setCursor(0, 1);
          lcd.print("Stopping!");
          moveMotor(0, 90);
          moveMotor(0, 0);
        }
      }
    }
  }
}
