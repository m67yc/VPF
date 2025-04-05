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
float startPressure = 1020;
float pressureDifference;

/* 速度 */
float currentSpeed;

/* 用於自動模式的變量 */
bool autoMovement = 0;

/* 编码器的數值： 一格和neutral狀態 */
int8_t oneGridValue = 0;
int8_t neutralValue = 0;

/* 狀態 */
bool isNeutral = 0;
bool isUping = 0;
bool isDowning = 0;

// Kalman Setting
float Q_process = 0.01; // 過程噪聲 (系統不確定性)
float Q_measure = 1.0;  // 測量噪聲 (傳感器噪聲)
float P_estimate = 1.0; // 估計誤差協方差
float K_gain = 0;       // 卡爾曼增益
float current_estimate = 0;
float last_estimate = 0;

/* PID */
// 數值
float kp = 1;
float ki = 0;
float kd = 0;
// 計算變數
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;
// 時間控制（固定時間間隔計算 PID）
unsigned long currentTime, previousTime;
float deltaTime;

/* 控制推桿馬達 */
void moveMotor(uint8_t speed, char mode)
{
  if (mode == 'U')
  {
    lcd.setCursor(0, 1);
    lcd.print("Floating!");
    if (speed > 0)
    {
      analogWrite(MotorPin1, abs(speed));
      analogWrite(MotorPin2, 0);
    }
    else if (speed < 0)
    {
      analogWrite(MotorPin1, 0);
      analogWrite(MotorPin2, abs(speed));
    }
    else
    {
      analogWrite(MotorPin1, 1);
      analogWrite(MotorPin2, 1);
    }
  }
  else if (mode == 'D')
  {
    lcd.setCursor(0, 1);
    lcd.print("Sinking!");
    if (speed < 0)
    {
      analogWrite(MotorPin1, abs(speed));
      analogWrite(MotorPin2, 0);
    }
    else if (speed > 0)
    {
      analogWrite(MotorPin1, 0);
      analogWrite(MotorPin2, abs(speed));
    }
    else
    {
      analogWrite(MotorPin1, 1);
      analogWrite(MotorPin2, 1);
    }
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
    moveMotor(100, 'D');
  else if (currentPos < tgt_Pos)
    moveMotor(100, 'U');

  return 0;
}

// Kalman Implement
float kalmanFilter(float measurement)
{
  // 預測階段
  current_estimate = last_estimate;
  P_estimate = P_estimate + Q_process;

  // 更新階段
  K_gain = P_estimate / (P_estimate + Q_measure);
  current_estimate = current_estimate + K_gain * (measurement - current_estimate);
  P_estimate = (1 - K_gain) * P_estimate;

  // 更新狀態
  last_estimate = current_estimate;
  return current_estimate;
}

// 傳感器數據更新
void updateSensors()
{
  static float lastPressure = 1000;
  static unsigned long lastTime = 0;

  pressure_sensor.read();
  currentPressure = kalmanFilter(pressure_sensor.pressure());
  pressureDifference = currentPressure - lastPressure;

  // 计算速度(mbar/s)
  unsigned long elapsed = millis() - lastTime;
  if (elapsed > 0)
  {
    if (pressureDifference < -0.1 || pressureDifference > 0.1)
      currentSpeed = (pressureDifference) * 1000.0 / elapsed;
    else
      currentSpeed = 0.00;

    lastPressure = currentPressure;
    lastTime = millis();
  }
}

/* PID算數 */
float PID(int32_t target, int32_t input)
{
  float Output = 0;

  currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0;

  error = target - input;

  // 計算積分項（避免積分飽和）
  integral += error * deltaTime;
  if (integral > 200)
    integral = 200;
  else if (integral < 0)
    integral = 0;

  // 計算微分項
  derivative = (error - lastError) / deltaTime;
  lastError = error;

  Output = kp * error + ki * integral + kd * derivative; // 計算 PID 輸出

  if (Output > 200)
    Output = 200;
  else if (Output < -200)
    Output = -200;

  previousTime = currentTime;

  return Output;
}

/* 設置速度 */
void setSpeed(uint8_t speed)
{
  // speed < 0 -> 上升
  // speed > 0 -> 下降
  // currentSpeed < 0 -> 上升
  // currentSpeed > 0 -> 下降
  int8_t motorSpeed = abs(PID(speed, currentSpeed));

  if (speed > 0 && currentSpeed < speed) // 要下降但當前上升
    moveMotor(motorSpeed, 'D');
  else if (speed > 0 && currentSpeed > speed) // 要下降但當前下降速度快了
    moveMotor(motorSpeed, 'U');
  else if (speed < 0 && currentSpeed > speed) // 要上升但當前下降
    moveMotor(motorSpeed, 'U');
  else if (speed < 0 && currentSpeed < speed) // 要上升但當前上升速度快了
    moveMotor(motorSpeed, 'D');
  else
    moveMotor(0, 'S');
}

/*正program
 */
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
  moveMotor(255, 'U');
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

void loop()
{

  updateSensors();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pressure: ");
  lcd.print(pressure_sensor.pressure());
  lcd.setCursor(0, 3);
  lcd.print("Speed: ");
  lcd.print(currentSpeed);

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
        setSpeed(-1);
      }
      else if (currentPressure < tgtPressure - tgtPressureScope)
      {
        setSpeed(1);
      }
      else
      {
        setSpeed(0);
      }
    }
  }
}
