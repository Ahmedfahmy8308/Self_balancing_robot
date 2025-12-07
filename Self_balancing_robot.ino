/*Self balancing robot . robot name : عنتر  */

#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20
#define BUZZER_PIN 11
#define BT_RX 12
#define BT_TX 13
#define ANGLE_THRESHOLD 30 // Warning angle threshold

MPU6050 mpu(0x68);                  // Explicitly set I2C address
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD address 0x27, size 16x2
SoftwareSerial bluetooth(BT_RX, BT_TX);

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double originalSetpoint = 175;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState = 0;
char btCommand = '0';            // Bluetooth command
double targetSetpoint = 175;     // Target setpoint for smooth transitions
double turnOffset = 0;           // Offset for turning left/right
double targetTurnOffset = 0;     // Target turn offset
unsigned long moveStartTime = 0; // Timer for auto-stop
bool isMoving = false;           // Movement state
double Kp = 22;
double Kd = 1.5;
double Ki = 140;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.6;

// MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 9;
int IN4 = 8;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// timers
long time1Hz = 0;
long time5Hz = 0;
long lcdUpdateTime = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize Bluetooth
  bluetooth.begin(9600);

  // Initialize LCD - show message only once at startup
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hi I'm ANTER");
  lcd.setCursor(0, 1);
  lcd.print("    Ready!    "); // Centered

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  bool connected = mpu.testConnection();
  Serial.println(connected ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // If standard test fails, force continue anyway (for MPU6500/MPU9250)
  if (!connected)
  {
    Serial.println(F("Note: May be MPU6500/MPU9250 - continuing anyway..."));
  }

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    // ERROR!
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // no mpu data - performing PID calculations and output to motors

    pid.Compute();

    // Apply turn offset for left/right movement
    int leftMotorSpeed = output - turnOffset;
    int rightMotorSpeed = output + turnOffset;

    // Use the motor controller with individual motor speeds
    motorController.move(leftMotorSpeed, rightMotorSpeed, MIN_ABS_SPEED);

    // Read Bluetooth commands during idle time
    if (bluetooth.available())
    {
      btCommand = bluetooth.read();
      if (btCommand == '1') // Forward
      {
        targetSetpoint = originalSetpoint - 5;
        targetTurnOffset = 0;
        moveStartTime = millis();
        isMoving = true;
      }
      else if (btCommand == '2') // Backward
      {
        targetSetpoint = originalSetpoint + 5;
        targetTurnOffset = 0;
        moveStartTime = millis();
        isMoving = true;
      }
      else if (btCommand == '3') // Turn Right
      {
        targetSetpoint = originalSetpoint;
        targetTurnOffset = 30; // Adjust this value for turn speed
        moveStartTime = millis();
        isMoving = true;
      }
      else if (btCommand == '4') // Turn Left
      {
        targetSetpoint = originalSetpoint;
        targetTurnOffset = -30; // Adjust this value for turn speed
        moveStartTime = millis();
        isMoving = true;
      }
      else // Stop/Balance
      {
        targetSetpoint = originalSetpoint;
        targetTurnOffset = 0;
        isMoving = false;
      }
    }

    // Auto-stop after 500ms
    if (isMoving && (millis() - moveStartTime > 500))
    {
      targetSetpoint = originalSetpoint;
      targetTurnOffset = 0;
      isMoving = false;
    }

    // Smooth transition to target setpoint
    if (setpoint < targetSetpoint)
    {
      setpoint += 0.5;
      if (setpoint > targetSetpoint)
        setpoint = targetSetpoint;
    }
    else if (setpoint > targetSetpoint)
    {
      setpoint -= 0.5;
      if (setpoint < targetSetpoint)
        setpoint = targetSetpoint;
    }

    // Smooth transition for turns
    if (turnOffset < targetTurnOffset)
    {
      turnOffset += 2;
      if (turnOffset > targetTurnOffset)
        turnOffset = targetTurnOffset;
    }
    else if (turnOffset > targetTurnOffset)
    {
      turnOffset -= 2;
      if (turnOffset < targetTurnOffset)
        turnOffset = targetTurnOffset;
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if LOG_INPUT
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif
    input = ypr[1] * 180 / M_PI + 180;

    // Activate buzzer when angle is too high
    if (abs(input - originalSetpoint) > ANGLE_THRESHOLD)
    {
      digitalWrite(BUZZER_PIN, HIGH);
    }
    else
    {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}
