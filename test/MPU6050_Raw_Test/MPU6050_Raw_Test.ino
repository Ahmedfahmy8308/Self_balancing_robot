/*
 * MPU6050 Basic Test - Without DMP
 * Tests raw sensor readings to verify hardware functionality
 */

#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of MPU6050

int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("\n=================================");
  Serial.println("MPU6050 RAW DATA TEST");
  Serial.println("=================================\n");

  // Wake up MPU6050 (it starts in sleep mode)
  Serial.println("1. Waking up MPU6050...");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to zero (wakes up the MPU6050)
  byte error = Wire.endTransmission();

  if (error == 0)
  {
    Serial.println("   ✓ MPU6050 woke up successfully!");
  }
  else
  {
    Serial.print("   ✗ Error waking MPU6050: ");
    Serial.println(error);
    Serial.println("\n*** SENSOR COMMUNICATION FAILED ***");
    while (1)
      ;
  }

  // Verify WHO_AM_I register
  Serial.println("2. Reading WHO_AM_I register...");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  byte whoami = Wire.read();

  Serial.print("   WHO_AM_I = 0x");
  Serial.print(whoami, HEX);

  if (whoami == 0x68)
  {
    Serial.println(" ✓ CORRECT! (MPU6050)");
  }
  else
  {
    Serial.println(" ✗ WRONG! Expected 0x68");
    Serial.println("\n*** SENSOR MAY BE DAMAGED ***");
  }

  Serial.println("\n=================================");
  Serial.println("STARTING RAW DATA MONITORING");
  Serial.println("=================================");
  Serial.println("Keep sensor STEADY and observe:");
  Serial.println("  - Accel should be stable");
  Serial.println("  - Gyro should be near zero");
  Serial.println("  - Temp should be reasonable\n");
  Serial.println("Format: Accel(X,Y,Z) | Gyro(X,Y,Z) | Temp");
  Serial.println("-------------------------------------------");

  delay(1000);
}

void loop()
{
  // Read accelerometer and gyroscope data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 registers

  // Read all data (3 accel, 1 temp, 3 gyro)
  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  // Convert and print
  Serial.print("Accel: ");
  Serial.print(accelerometer_x);
  Serial.print(", ");
  Serial.print(accelerometer_y);
  Serial.print(", ");
  Serial.print(accelerometer_z);
  Serial.print(" | Gyro: ");
  Serial.print(gyro_x);
  Serial.print(", ");
  Serial.print(gyro_y);
  Serial.print(", ");
  Serial.print(gyro_z);
  Serial.print(" | Temp: ");

  // Convert temperature to Celsius
  float temp_c = (temperature / 340.0) + 36.53;
  Serial.print(temp_c, 1);
  Serial.print("°C");

  // Check for issues
  bool hasIssue = false;

  // Check if all values are zero (sensor dead)
  if (accelerometer_x == 0 && accelerometer_y == 0 && accelerometer_z == 0 &&
      gyro_x == 0 && gyro_y == 0 && gyro_z == 0)
  {
    Serial.print(" ⚠ ALL ZERO - SENSOR DEAD!");
    hasIssue = true;
  }

  // Check if values are stuck (same as last reading)
  static int16_t last_ax = 0, last_ay = 0, last_az = 0;
  static int count = 0;

  if (accelerometer_x == last_ax && accelerometer_y == last_ay &&
      accelerometer_z == last_az && count > 0)
  {
    count++;
    if (count > 10)
    {
      Serial.print(" ⚠ STUCK VALUES - SENSOR FROZEN!");
      hasIssue = true;
    }
  }
  else
  {
    count = 0;
    last_ax = accelerometer_x;
    last_ay = accelerometer_y;
    last_az = accelerometer_z;
  }

  // Check if values are changing wildly (noise)
  static int16_t prev_ax = 0;
  if (abs(accelerometer_x - prev_ax) > 5000 && prev_ax != 0)
  {
    Serial.print(" ⚠ WILD CHANGE - NOISY!");
    hasIssue = true;
  }
  prev_ax = accelerometer_x;

  if (!hasIssue)
  {
    Serial.print(" ✓");
  }

  Serial.println();

  delay(100); // Update 10 times per second
}
