/*
  - BT RX → Pin 12
  - BT TX → Pin 13
  - BT VCC → 5V
  - BT GND → GND
*/

#include <SoftwareSerial.h>

#define BT_RX 12 // Arduino RX ← BT TX
#define BT_TX 13 // Arduino TX → BT RX

SoftwareSerial bluetooth(BT_RX, BT_TX);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }

  Serial.println("========================================");
  Serial.println("    Bluetooth Test - اختبار البلوتوث");
  Serial.println("========================================");
  Serial.println();

  bluetooth.begin(9600);
  Serial.println("[OK] Bluetooth initialized at 9600 baud");
  Serial.println("[INFO] Waiting for Bluetooth data...");
  Serial.println("[INFO] Send commands from phone app");
  Serial.println("========================================");
  Serial.println();
}

void loop()
{
  if (bluetooth.available())
  {
    char received = bluetooth.read();

    Serial.println("----------------------------------------");
    Serial.print("[BT → Arduino] Received: ");
    Serial.print(received);
    Serial.print(" (ASCII: ");
    Serial.print((int)received);
    Serial.println(")");

    interpretCommand(received);

    bluetooth.print("Received: ");
    bluetooth.println(received);

    Serial.println("----------------------------------------");
    Serial.println();
  }

  if (Serial.available())
  {
    char toSend = Serial.read();
    bluetooth.write(toSend);

    Serial.print("[Arduino → BT] Sent: ");
    Serial.println(toSend);
  }

  delay(10);
}

void interpretCommand(char cmd)
{
  Serial.print("[INFO] Command interpretation: ");

  switch (cmd)
  {
  case 'F':
  case 'f':
    Serial.println("FORWARD - تقدم للأمام");
    break;

  case 'B':
  case 'b':
    Serial.println("BACKWARD - رجوع للخلف");
    break;

  case 'L':
  case 'l':
    Serial.println("LEFT - دوران يسار");
    break;

  case 'R':
  case 'r':
    Serial.println("RIGHT - دوران يمين");
    break;

  case '0':
  case 'S':
  case 's':
    Serial.println("STOP - توقف");
    break;

  case '\n':
  case '\r':
    Serial.println("(Newline character - ignored)");
    break;

  default:
    Serial.print("UNKNOWN - أمر غير معروف: '");
    Serial.print(cmd);
    Serial.println("'");
    break;
  }
}
