#include <Arduino.h>
#include <Wire.h>
#include <MS5837.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8, 7); /* CE, CSN */
hd44780_I2Cexp lcd;
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

  if (autoMovement == 0)
  {
    lcd.setCursor(0, 2);
    lcd.print("Manual mode");
    if (digitalRead(MANUAL_SWITCH)) /* A1 LOW -> Auto mode */
      autoMovement = 1;
    else
      ManualMode();
  }
  else
  {
    lcd.setCursor(0, 2);
    lcd.print("Auto mode!");

    if (currentPressure < startPressure) /* not work before in water */
      moveMotor(0, 'S');
    else
    {
      if (first_time_pulling == 0)
      {
        if (millis() - last_time >= 2000)
        {
          if (currentPressure - minPressure <= 1)
          {
            first_time_pulling = 1;
          }
          last_time = millis();
        }
        else if (millis() - last_time >= 1000 && millis() - last_time < 1500)
        {
          if (minPressure > currentPressure)
          {
            minPressure = currentPressure;
          }
        }
      }
      else if (currentPressure > tgtPressure - 5 && currentPressure < tgtPressure + 5)
      {
        moveMotor(0, 'S');
      }
      else if (currentPressure > tgtPressure)
      {
        uping = 0;
        if (downing == 0)
        {
          if (millis() - last_time >= 1800)
          {
            moveMotor(0, 'S');
            last_time = millis();
            if (currentPressure - pastPressure <= 1)
              downing = 1;
          }
          else if (millis() - last_time >= 900)
          {
            moveMotor(255, 'D');
            pastPressure = currentPressure;
          }
        }
        else if (millis() - last_time <= 900)
        {
          moveMotor(255, 'U');
        }
        else if (millis() - last_time >= 5000 && (currentPressure > tgtPressure - 5 && currentPressure < tgtPressure + 5))
        {
          downing = 0;
        }
      }
      else if (currentPressure < tgtPressure)
      {
        downing = 0;
        if (uping == 0)
        {
          if (millis() - last_time >= 1800)
          {
            moveMotor(0, 'S');
            last_time = millis();
            if (pastPressure - currentPressure <= 1)
              uping = 1;
          }
          else if (millis() - last_time >= 900)
          {
            moveMotor(255, 'U');
            pastPressure = currentPressure;
          }
        }
        else if (millis() - last_time <= 900)
        {
          moveMotor(255, 'D');
        }
        else if (millis() - last_time >= 5000 && (currentPressure > tgtPressure - 5 && currentPressure < tgtPressure + 5))
        {
          uping = 0;
        }
      }

      // if (first_time_pulling == 0 && (currentPressure - minPressure >= 1 || minPressure - currentPressure >= 1)) /* first time pulling */
      // {
      //   moveMotor(255, 'D');

      //   if (currentPressure > minPressure + 2) /* now higher than pass -> it is sinking -> stop */
      //   {
      //     minPressure = currentPressure + 2;
      //     first_time_pulling = 1;
      //     moveMotor(0, 'S');
      //   }
      // }

      // if (first_time_pulling == 0)
      //   minPressure = currentPressure;
      // else if (currentPressure > (tgtPressure + 5) && currentPressure < (tgtPressure - 5)) /* very neer tgt -> stop */
      // {
      //   moveMotor(0, 'S');
      // }
      // else if (currentPressure > tgtPressure) /* under tgt */
      // {
      //   min_errorPressure = tgtPressure;
      //   if (max_errorPressure < currentPressure && currentPressure - max_errorPressure < errorPressure)
      //     max_errorPressure = currentPressure;

      //   if (max_errorPressure - currentPressure <= errorPressure) /* now under pass -> it is sinking -> let it float */
      //   {
      //     moveMotor(100, 'U');
      //   }
      //   else
      //   {
      //     max_errorPressure = currentPressure + (errorPressure + 3);
      //     moveMotor(100, 'D');
      //     moveMotor(0, 'S');
      //   }
      // }
      // else /* higher than tgt */
      // {
      //   max_errorPressure = tgtPressure;
      //   if (min_errorPressure > currentPressure && currentPressure < minPressure)
      //     min_errorPressure = currentPressure;

      //   if (currentPressure - min_errorPressure <= errorPressure) /* now higher than pass -> it is floating -> let it sink */
      //   {
      //     moveMotor(100, 'D');
      //   }
      //   else
      //   {
      //     min_errorPressure = currentPressure - (errorPressure + 3);
      //     moveMotor(100, 'U');
      //     moveMotor(0, 'S');
      //   }
      // }
    }
  }
}

// // #if 1
// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>

// RF24 radio(8, 7); // CE, CSN

// const byte address[6] = "14514";
// const byte oaddress[6] = "19198";

// const int QUEUE_LEN = 180;
// char queue[QUEUE_LEN + 10];
// byte serial_reader[1];
// int st = 0, en = 0, size = 0;
// int tick = 0;
// int status = 0;

// void push(char c){
//   queue[en++] = c;
//   en %= QUEUE_LEN;
//   size ++;
// }

// char getChar() {
//   int l_st = st;
//   st = (st + 1) % QUEUE_LEN;
//   size--;
//   return queue[l_st];
// }

// void setup() {
//   Serial.begin(9600);
//   radio.begin();
//   radio.setChannel(66);
//   radio.openWritingPipe(address);
//   radio.openReadingPipe(0, oaddress);
//   radio.setPALevel(RF24_PA_HIGH);
//   radio.setDataRate(RF24_250KBPS);
// }

// void setNum(int n, char c){
//   while (n) {
//     push(n % 10 + '0');
//     n /= 10;
//   }
//   push(c);
// }

// void write(int t, int v){
//   setNum(t, '@');
//   setNum(v, '#');
// }

// void loop() {

//   while(size){
//     radio.stopListening();
//     char text[2] = "6";
//     text[0] = getChar();
//     while(digitalRead(8)){
//       Serial.println("Radio unavailable!");
//       radio.begin();
//       radio.setChannel(66);
//       radio.openWritingPipe(address);
//       radio.openReadingPipe(0, oaddress);
//       radio.setPALevel(RF24_PA_HIGH);
//       radio.setDataRate(RF24_250KBPS);
//     }
//     radio.write(&text, sizeof(text));
//   }
//   radio.startListening();
//   while (radio.available()) {
//     radio.read(serial_reader, sizeof(serial_reader));
//     switch (serial_reader[0]) {
//       case 'U':
//         status = 1;
//         break;
//       case 'S':
//         status = 0;
//         break;
//       case 'D':
//         status = -1;
//         break;
//       // motor control
//     }
//   }
//   switch (status) {
//     case -1:
//       Serial.println("Sinking...");
//       break;
//     case 0:
//       Serial.println("Stopping...");
//       break;
//     case 1:
//       Serial.println("Floating...");
//       break;
//   }
//   tick += 1;
//   write(tick * 5, abs(tick * tick % 500 - 250));
//   delay(50);
// }
// // #else
