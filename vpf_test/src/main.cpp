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


const uint8_t momentumAdjustment = 10;
const uint16_t tgtPressure = 1040;
float currentPressure, previousPressure;
float startPressure = 1030;
uint16_t distance;
uint16_t prevT, eprev;
bool startMovement = 0;
bool autoMovement = 0;
bool stopState = 0;
bool is_rising = 0;
bool first_time_pulling = 0;


uint16_t eintegral = 0;


// uint32_t bestDelay(uint16_t delay_Time)
// {
//   uint32_t currentTime = millis();
//   if (currentTime >= millis() - delay_Time)
// }


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
  currentPressure = pressure_sensor.pressure();
  lcd.setCursor(0, 0);
  lcd.print("Pressure: ");
  lcd.print(pressure_sensor.pressure());


  if (autoMovement == 0)
  {
    lcd.setCursor(0, 2);
    lcd.print("Manual mode");
    if (!digitalRead(MANUAL_SWITCH))
    {
      if (!digitalRead(PULL_SWITCH) && digitalRead(PUSH_SWITCH))
      {
        lcd.setCursor(0, 1);
        lcd.print("Sinking!");
        moveMotor(255, 0);
      }
      else if (digitalRead(PULL_SWITCH) && !digitalRead(PUSH_SWITCH))
      {
        lcd.setCursor(0, 1);
        lcd.print("Rising!");
        moveMotor(0, 255);
      }
      else
      {
        lcd.clear();
        moveMotor(0, 0);
      }
    }
    else
      autoMovement = 1;
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pressure: ");
    lcd.print(pressure_sensor.pressure());
    lcd.setCursor(0, 2);
    lcd.print("Auto mode");
    if (currentPressure >= startPressure)
    {
      startMovement = 1;
    }
    else
    {
      startMovement = 0;
    }


    if (startMovement)
    {
      if (first_time_pulling == 0)
      {
        lcd.setCursor(0, 1);
        lcd.print("Sinking!");
        moveMotor(255, 0);
        first_time_pulling = 1;
      }
      if (currentPressure > tgtPressure && first_time_pulling == 1)
      {
        if (currentPressure > previousPressure)
        {
          lcd.setCursor(0, 1);
          lcd.print("stopped!");
          moveMotor(0, 0);
        }
        else
        {
          lcd.setCursor(0, 1);
          lcd.print("floating!");
          moveMotor(0, 100);
        }
      }
      else
      {
        if (currentPressure > previousPressure)
        {
          lcd.setCursor(0, 1);
          lcd.print("Sinking!");
          moveMotor(100, 0);
        }
        else
        {
          lcd.setCursor(0, 1);
          lcd.print("stopped!");
          moveMotor(0, 0);
        }
      }
    }
  }
}
