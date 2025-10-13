#include <ModbusMaster.h>

#define RXD2 18
#define TXD2 17
ModbusMaster node;

float registersToFloat(uint16_t reg1, uint16_t reg2) {
 uint32_t combined = ((uint32_t)reg1 << 16) | reg2;
 float val;
 memcpy(&val, &combined, 4);
 return val;
}

void setup() {
 Serial.begin(115200);
 Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
 node.begin(1, Serial2); // ID 1
 Serial.println("=== Flowmeter Monitor (RS485 Modbus) ===");
}

void loop() {
 uint8_t result = node.readHoldingRegisters(0x0006, 2); // baca 2 register
 if (result == node.ku8MBSuccess) {
   uint16_t r6 = node.getResponseBuffer(0);
   uint16_t r7 = node.getResponseBuffer(1);
   float flow = registersToFloat(r6, r7);
   Serial.print("Flow rate: ");
   Serial.print(flow, 2);
   Serial.println(" L/min");
 } else {
   Serial.print("Error: "); Serial.println(result);
 }

 delay(1000);
}