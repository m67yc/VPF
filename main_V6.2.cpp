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

/* 推桿馬達引脚 */
const uint8_t MotorPin1 = 5;
const uint8_t MotorPin2 = 6;

/* 按鈕引脚 */
constexpr uint8_t MANUAL_SWITCH = A1;
constexpr uint8_t PUSH_SWITCH = A2;
constexpr uint8_t PULL_SWITCH = A3;

/* 编码器配置 */
volatile long currentPos = 0;
volatile int8_t prevPhase1;
const uint8_t ENCODER_M1_1 = 3;
const uint8_t ENCODER_M1_2 = 2;

/* 中斷實現 */
const char PHASE_COMPARE[4][4] = {
    {0, -1, 1, 0},
    {1, 0, 0, -1},
    {-1, 0, 0, 1},
    {0, 1, -1, 0}};

ISR(PCINT2_vect)
{
  int8_t currentPhase;
  currentPhase = (PIND >> digitalPinToPCMSKbit(ENCODER_M1_2)) & B11;
  currentPos += PHASE_COMPARE[currentPhase][prevPhase1];
  prevPhase1 = currentPhase;
}

/* communication */
const byte address[6] = "14514";
const byte oaddress[6] = "19198";

const int QUEUE_LEN = 180;
char queue[QUEUE_LEN + 10];
byte serial_reader[1];
int st = 0, en = 0, size = 0;
int tick = 0;
int status = 0;

/* pressure數值 */
float currentPressure;
const uint16_t tgtPressure = 1055;
const uint8_t tgtPressureScope = 5;
float startPressure = 1023;

/* 用於自動模式的變量 */
bool autoMovement = 0;

/* 编码器的數值： 一格和neutral狀態 */
int8_t oneGridValue = 0;
int8_t neutralValue = 0;

/* 狀態 */
bool isNeutral = 0;
bool isUping = 0;
bool isDowning = 0;

/* PID */
//數值
uint16_t kp = 1;
uint16_t ki = 0;
uint16_t kd = 0;
//計算變數
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;
// 時間控制（固定時間間隔計算 PID）
unsigned long currentTime, previousTime;
float deltaTime;


/* 控制推桿馬達 */
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
    analogWrite(MotorPin1, 1);
    analogWrite(MotorPin2, 1);
  }
}

/* 手動模式 */
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

/* 移動推桿馬達到對應的编码器數值 */
bool movePos(int tgt_Pos)
{

  const int DEAD_ZONE = 3;

  long error = tgt_Pos - currentPos;
  if (abs(error) <= DEAD_ZONE)
  {
    moveMotor(0, 'S');
    return 1;
  }

  if (currentPos > tgt_Pos)
    moveMotor(100, 'U');
  else if (currentPos < tgt_Pos)
    moveMotor(100, 'D');

  return 0;
}

void setup()
{
  Serial.begin(9600);

  /* 初始化LCD */
  lcd.begin(20, 4);
  lcd.clear();
  lcd.init();

  /* 初始化壓力傳感器 */
  while (!pressure_sensor.init())
  {
    lcd.setCursor(0, 1);
    lcd.print("Pressure Sensor Fail");
    delay(1000);
  }
  pressure_sensor.setFluidDensity(997);
  pressure_sensor.setModel(0);

  /* 初始化推桿馬達 */
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  pinMode(MANUAL_SWITCH, INPUT_PULLUP);
  pinMode(PUSH_SWITCH, INPUT_PULLUP);
  pinMode(PULL_SWITCH, INPUT_PULLUP);

  /* 推桿和編碼器復位 */
  lcd.setCursor(0, 1);
  lcd.print("init...");
  moveMotor(255, 'D');
  delay(5000);
  currentPos = 0;
  lcd.clear();

  previousTime = millis();

  /* 初始化中斷寄存器 */
  cli();
  prevPhase1 = (PINB >> digitalPinToPCMSKbit(ENCODER_M1_2)) & B11;
  PCICR |= (1 << PCIE2);
  PCMSK2 = (1 << digitalPinToPCMSKbit(ENCODER_M1_1) | 1 << digitalPinToPCMSKbit(ENCODER_M1_2));
  sei();
}

/* PID算數 */
int32_t PID(int32_t input)
{
  int32_t Output = 0;
  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0;  // 轉換為秒

  if (deltaTime >= 0.1) {  // 每 100ms 計算一次 PID

    error = tgtPressure - input;//計算誤差

    // 3. 計算積分項（避免積分飽和）
    integral += error * deltaTime;
    if (integral > 255) integral = 255;  // PWM 最大限制
    else if (integral < 0) integral = 0;

    // 4. 計算微分項
    derivative = (error - lastError) / deltaTime;
    lastError = error;

    // 5. 計算 PID 輸出
    Output = kp * error + ki * integral + kd * derivative;

    // 6. 限制輸出範圍（0-255）
    if (Output > 255) Output = 255;
    else if (Output < 0) Output = 0;

    previousTime = currentTime;//更新時間

    return Output;
  }
}

void loop()
{

  pressure_sensor.read();
  currentPressure = pressure_sensor.pressure();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pressure: ");
  lcd.print(pressure_sensor.pressure());
  lcd.setCursor(0, 3);
  lcd.print("Pos: ");
  lcd.print(currentPos);

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
      if (currentPressure > tgtPressure + tgtPressureScope)
      {
        if (!isUping)
          isUping = movePos(neutralValue - 2*oneGridValue);
      }
      else if (currentPressure < tgtPressure - tgtPressureScope)
      {
        if (!isDowning)
          isDowning = movePos(neutralValue + 2*oneGridValue);
      }
      else
      {
        if (!isNeutral)
          isNeutral = movePos(neutralValue);
      }
    }
  }
}
