/*
 * MPU6050 Sensor Test - Diagnostic Tool
 * Tests MPU6050 connection, calibration, and data quality
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

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

// Test variables
unsigned long lastPrint = 0;
int successfulReadings = 0;
int failedReadings = 0;
int fifoOverflows = 0;
float previousAngle = 0;
float angleChangeRate = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println(F("\n================================="));
  Serial.println(F("MPU6050 DIAGNOSTIC TEST"));
  Serial.println(F("=================================\n"));

  // Initialize device
  Serial.println(F("1. Initializing MPU6050..."));
  mpu.initialize();

  // Verify connection
  Serial.print(F("2. Testing connection... "));
  if (mpu.testConnection())
  {
    Serial.println(F("SUCCESS!"));
  }
  else
  {
    Serial.println(F("FAILED!"));
    Serial.println(F("\n*** CHECK WIRING: SDA->A4, SCL->A5, VCC->5V, GND->GND ***"));
    while (1)
      ; // Stop here
  }

  // Check I2C address
  Serial.print(F("3. I2C Address: 0x"));
  Serial.println(mpu.getDeviceID(), HEX);

  // Load and configure the DMP
  Serial.println(F("4. Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Supply gyro offsets
  Serial.println(F("5. Setting gyro offsets..."));
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  Serial.println(F("   XGyro: 220, YGyro: 76, ZGyro: -85, ZAccel: 1788"));

  if (devStatus == 0)
  {
    Serial.println(F("6. DMP Initialization SUCCESS!"));

    // Turn on the DMP
    Serial.println(F("7. Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    Serial.println(F("8. Enabling interrupt (Pin 2)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Set DMP Ready flag
    Serial.println(F("9. DMP Ready!"));
    dmpReady = true;

    // Get expected DMP packet size
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("10. DMP Packet Size: "));
    Serial.println(packetSize);

    Serial.println(F("\n================================="));
    Serial.println(F("STARTING CONTINUOUS MONITORING"));
    Serial.println(F("================================="));
    Serial.println(F("Format: Time | Angle | Change | Status | Stats"));
    Serial.println(F("Keep sensor STEADY and watch for:"));
    Serial.println(F("  - Angle should be stable (±0.5°)"));
    Serial.println(F("  - No FIFO overflows"));
    Serial.println(F("  - No sudden jumps\n"));
  }
  else
  {
    // ERROR!
    Serial.print(F("DMP Initialization FAILED (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    Serial.println(F("Error codes:"));
    Serial.println(F("  1 = initial memory load failed"));
    Serial.println(F("  2 = DMP configuration updates failed"));
    while (1)
      ; // Stop here
  }
}

void loop()
{
  // If DMP not ready, return
  if (!dmpReady)
    return;

  // Wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // Do nothing while waiting
  }

  // Reset interrupt flag
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // Reset FIFO
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    fifoOverflows++;

    Serial.print(F("*** FIFO OVERFLOW #"));
    Serial.print(fifoOverflows);
    Serial.println(F(" *** (This indicates timing issues!)"));
    failedReadings++;
  }
  // Check for DMP data ready interrupt
  else if (mpuIntStatus & 0x02)
  {
    // Wait for correct available data length
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // Read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Track FIFO count
    fifoCount -= packetSize;

    // Get Quaternion and calculate Yaw/Pitch/Roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convert to degrees
    float angle = ypr[1] * 180 / M_PI + 180;

    // Calculate change rate
    angleChangeRate = angle - previousAngle;
    previousAngle = angle;

    successfulReadings++;

    // Print every 100ms
    if (millis() - lastPrint >= 100)
    {
      lastPrint = millis();

      // Time
      Serial.print(millis() / 1000.0, 2);
      Serial.print("s | ");

      // Angle
      Serial.print("Angle: ");
      Serial.print(angle, 2);
      Serial.print("° | ");

      // Change
      Serial.print("Δ: ");
      if (abs(angleChangeRate) < 0.01)
        Serial.print(" 0.00");
      else
        Serial.print(angleChangeRate, 2);
      Serial.print("° | ");

      // Status
      if (abs(angleChangeRate) > 5.0)
      {
        Serial.print("⚠ JUMP! ");
      }
      else if (abs(angleChangeRate) > 1.0)
      {
        Serial.print("~ Noise ");
      }
      else
      {
        Serial.print("✓ Stable");
      }
      Serial.print(" | ");

      // Stats
      Serial.print("OK:");
      Serial.print(successfulReadings);
      Serial.print(" Err:");
      Serial.print(failedReadings);
      Serial.print(" OF:");
      Serial.print(fifoOverflows);

      // FIFO status
      Serial.print(" | FIFO:");
      Serial.print(fifoCount);

      Serial.println();
    }
  }
}
