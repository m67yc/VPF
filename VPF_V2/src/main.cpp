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

bool autoMovement = 0;
bool first_time_pulling = 0;

long previousMillis = millis();

void moveMotor(uint8_t MotorSpeed1, uint8_t MotorSpeed2)
{
  MotorSpeed1 = constrain(MotorSpeed1, 0, 255);
  MotorSpeed2 = constrain(MotorSpeed2, 0, 255);
  analogWrite(MotorPin1, MotorSpeed1);
  analogWrite(MotorPin2, MotorSpeed2);
}

void LCDprint(uint8_t columns, uint8_t rows, char words, char num = 'q')
{
  lcd.setCursor(columns, rows);
  lcd.print(words);
  if (num != 'q')
    lcd.print(num);
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
  LCDprint(0, 0, "Pressure: ", pressure_sensor.pressure());

  if (millis() - previousMillis = 10)
  {
    previousPressure = currentPressure;
    previousMillis = millis();
  }

  if (autoMovement == 0)
  {
    LCDprint(0, 2, "Manual mode");
    if (digitalRead(MANUAL_SWITCH)) /* A1 LOW -> Auto mode */
      autoMovement = 1;
    else
    {
      if (!digitalRead(PULL_SWITCH) && digitalRead(PUSH_SWITCH)) /* A2 high -> sinking */
      {
        LCDprint(0, 1, "Sinking!");
        moveMotor(255, 0);
      }
      else if (digitalRead(PULL_SWITCH) && !digitalRead(PUSH_SWITCH)) /* A3 high -> rising */
      {
        LCDprint(0, 1, "Rising!");
        moveMotor(0, 255);
      }
      else
      {
        LCDprint(0, 1, "Stopping!");
        moveMotor(0, 0);
      }
    }
  }
  else
  {
    LCDprint(0, 2, "Auto mode");

    if (currentPressure < startPressure) /* not work before in water */
      moveMotor(0, 0);
    else
    {
      if (first_time_pulling == 0)
      {
        LCDprint(0, 1, "Sinking!");
        moveMotor(255, 0);

        if (currentPressure > previousPressure + 2) /* now higher than pass -> it is sinking -> stop */
        {
          first_time_pulling = 1;
          moveMotor(0, 0);
        }
      }
      else if (currentPressure > tgtPressure) /* under tgt */
      {
        if (currentPressure > previousPressure + 1) /* now under pass -> it is sinking -> let it float */
        {
          LCDprint(0, 1, "Floating!");
          moveMotor(0, 100);
        }
        else
        {
          LCDprint(0, 1, "Stopped!");
          moveMotor(0, 0);
        }
      }
      else /* higher than tgt */
      {
        if (currentPressure < previousPressure - 1) /* now higher than pass -> it is floating -> let it sink */
        {
          LCDprint(0, 1, "Sinking!");
          moveMotor(100, 0);
        }
        else
        {
          LCDprint(0, 1, "Stopped!");
          moveMotor(0, 0);
        }
      }
    }
  }
}
