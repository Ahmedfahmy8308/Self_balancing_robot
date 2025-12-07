/*
 * I2C Scanner - Find all I2C devices on the bus
 * This will help diagnose MPU6050 connection issues
 */

#include <Wire.h>

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("\n=================================");
  Serial.println("I2C DEVICE SCANNER");
  Serial.println("=================================\n");
  Serial.println("Scanning I2C bus...\n");
}

void loop()
{
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for I2C devices...");
  Serial.println("----------------------------");

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("✓ Device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" (");

      // Identify common devices
      if (address == 0x68 || address == 0x69)
        Serial.print("MPU6050/MPU9250");
      else if (address == 0x27)
        Serial.print("LCD 16x2");
      else if (address == 0x3F)
        Serial.print("LCD 16x2");
      else if (address == 0x76 || address == 0x77)
        Serial.print("BMP280/BME280");
      else
        Serial.print("Unknown");

      Serial.println(")");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("✗ Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  Serial.println("----------------------------");
  if (nDevices == 0)
  {
    Serial.println("\n⚠ WARNING: NO I2C DEVICES FOUND!");
    Serial.println("\nPossible issues:");
    Serial.println("  1. Check wiring:");
    Serial.println("     - SDA → A4 (or SDA pin)");
    Serial.println("     - SCL → A5 (or SCL pin)");
    Serial.println("     - VCC → 5V (or 3.3V)");
    Serial.println("     - GND → GND");
    Serial.println("  2. Bad connections/loose wires");
    Serial.println("  3. Device not powered");
    Serial.println("  4. Faulty sensor");
    Serial.println("  5. Wrong voltage (check if 3.3V or 5V)");
  }
  else
  {
    Serial.print("\n✓ Found ");
    Serial.print(nDevices);
    Serial.println(" device(s) on I2C bus");

    // Check for MPU6050
    bool mpuFound = false;
    for (address = 1; address < 127; address++)
    {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0 && (address == 0x68 || address == 0x69))
      {
        mpuFound = true;
        Serial.print("\n✓ MPU6050 detected at 0x");
        Serial.println(address, HEX);
        Serial.println("  → Sensor is connected properly!");
        Serial.println("  → If test still fails, sensor may be damaged");
      }
    }

    if (!mpuFound)
    {
      Serial.println("\n⚠ MPU6050 NOT FOUND!");
      Serial.println("  Expected at 0x68 or 0x69");
      Serial.println("  → Check MPU6050 wiring specifically");
      Serial.println("  → Try different MPU6050 module");
    }
  }

  Serial.println("\n=================================");
  Serial.println("Scan complete. Rescanning in 5 seconds...\n");
  delay(5000);
}
