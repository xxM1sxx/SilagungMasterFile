#include <Arduino.h>
#include <ModbusMaster.h>

#define RX_PIN 17 // For ESP32 RX2 pin
#define TX_PIN 18 // For ESP32 TX2 pin

ModbusMaster node;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // RS485 komunikasi

  node.begin(1, Serial2);  // Slave ID default = 1

  Serial.println("RS485 Ultrasonic Sensor");
}

void loop()
{
  uint8_t result;
  uint16_t distanceRaw, tempRaw;

  // === Baca jarak (Register 0x0100) ===
  result = node.readHoldingRegisters(0x0100, 1);
  if (result == node.ku8MBSuccess) {
    distanceRaw = node.getResponseBuffer(0);
    Serial.print("Distance: ");
    Serial.print(distanceRaw / 10.0, 1);
    Serial.println(" cm");
  } else {
    Serial.print("Failed to read distance. Error: ");
    Serial.println(result);
  }

  Serial.println("-----------------------------");
  delay(1000);
}
