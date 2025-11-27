#include <Arduino.h>
#include <ModbusMaster.h>

// === Pin Configuration ===
#define RX_PIN 17       // ESP32 RX2 pin
#define TX_PIN 18       // ESP32 TX2 pin
#define RS485_DE_RE 4   // RS485 DE/RE control pin (HIGH = TX, LOW = RX)

// === Serial Configuration ===
#define RS485_SERIAL Serial2
#define SENSOR_ADDRESS 1
#define SENSOR_BAUDRATE 9600
#define READ_RETRY 3

// === Register Map (from latest CWT manual) ===
#define REG_SOIL_HUMIDITY  0x0000
#define REG_SOIL_TEMP       0x0001
#define REG_EC              0x0002
#define REG_PH              0x0003
#define REG_NITROGEN        0x0004
#define REG_PHOSPHORUS      0x0005
#define REG_POTASSIUM       0x0006

ModbusMaster node;

// ------------------------------------------------------------
void preTransmission() {
  digitalWrite(RS485_DE_RE, HIGH);
  delayMicroseconds(500);
}

void postTransmission() {
  delayMicroseconds(500);
  digitalWrite(RS485_DE_RE, LOW);
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== CWT Soil Sensor Reader (Addr=1, Baud=9600, 8N1) ===");

  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);

  RS485_SERIAL.begin(SENSOR_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);

  node.begin(SENSOR_ADDRESS, RS485_SERIAL);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("Mulai membaca data sensor setiap 2 detik...\n");
}

// ------------------------------------------------------------
const char* mbErrName(uint8_t code) {
  switch (code) {
    case ModbusMaster::ku8MBSuccess: return "Success";
    case ModbusMaster::ku8MBIllegalFunction: return "IllegalFunction";
    case ModbusMaster::ku8MBIllegalDataAddress: return "IllegalDataAddress";
    case ModbusMaster::ku8MBIllegalDataValue: return "IllegalDataValue";
    case ModbusMaster::ku8MBSlaveDeviceFailure: return "SlaveDeviceFailure";
    case ModbusMaster::ku8MBInvalidSlaveID: return "InvalidSlaveID";
    case ModbusMaster::ku8MBInvalidFunction: return "InvalidFunction";
    case ModbusMaster::ku8MBResponseTimedOut: return "ResponseTimedOut";
    case ModbusMaster::ku8MBInvalidCRC: return "InvalidCRC";
    default: return "Unknown";
  }
}

uint8_t readRegRetry(uint16_t addr, uint16_t* out) {
  uint8_t rc = 0xFF;
  for (int i = 0; i < READ_RETRY; i++) {
    rc = node.readHoldingRegisters(addr, 1);
    if (rc == node.ku8MBSuccess) {
      *out = node.getResponseBuffer(0);
      return rc;
    }
    delay(100);
  }
  return rc;
}

void debugReadPerRegister() {
  uint16_t v;
  uint8_t rc;

  rc = readRegRetry(REG_SOIL_HUMIDITY, &v);
  if (rc == node.ku8MBSuccess) Serial.printf("Debug Moisture   : %.1f %%RH\n", v / 10.0);
  else Serial.printf("Error Moisture   : %s (0x%02X)\n", mbErrName(rc), rc);

  rc = readRegRetry(REG_SOIL_TEMP, &v);
  if (rc == node.ku8MBSuccess) Serial.printf("Debug Temperature: %.1f °C\n", v / 10.0);
  else Serial.printf("Error Temperature: %s (0x%02X)\n", mbErrName(rc), rc);

  rc = readRegRetry(REG_EC, &v);
  if (rc == node.ku8MBSuccess) Serial.printf("Debug EC         : %u µS/cm\n", v);
  else Serial.printf("Error EC         : %s (0x%02X)\n", mbErrName(rc), rc);

  rc = readRegRetry(REG_PH, &v);
  if (rc == node.ku8MBSuccess) Serial.printf("Debug pH         : %.1f\n", v / 10.0);
  else Serial.printf("Error pH         : %s (0x%02X)\n", mbErrName(rc), rc);

  rc = readRegRetry(REG_NITROGEN, &v);
  if (rc == node.ku8MBSuccess) Serial.printf("Debug Nitrogen   : %u mg/kg\n", v);
  else Serial.printf("Error Nitrogen   : %s (0x%02X)\n", mbErrName(rc), rc);

  rc = readRegRetry(REG_PHOSPHORUS, &v);
  if (rc == node.ku8MBSuccess) Serial.printf("Debug Phosphorus : %u mg/kg\n", v);
  else Serial.printf("Error Phosphorus : %s (0x%02X)\n", mbErrName(rc), rc);

  rc = readRegRetry(REG_POTASSIUM, &v);
  if (rc == node.ku8MBSuccess) Serial.printf("Debug Potassium  : %u mg/kg\n", v);
  else Serial.printf("Error Potassium  : %s (0x%02X)\n", mbErrName(rc), rc);
}

void loop() {
  uint8_t result = 0xFF;
  for (int i = 0; i < READ_RETRY; i++) {
    result = node.readHoldingRegisters(REG_SOIL_HUMIDITY, 7);
    if (result == node.ku8MBSuccess) break;
    delay(100);
  }
  if (result == node.ku8MBSuccess) {
    float soilHumidity = node.getResponseBuffer(0) / 10.0;
    float soilTemp     = node.getResponseBuffer(1) / 10.0;
    uint16_t ec        = node.getResponseBuffer(2);
    float ph           = node.getResponseBuffer(3) / 10.0;
    uint16_t N         = node.getResponseBuffer(4);
    uint16_t P         = node.getResponseBuffer(5);
    uint16_t K         = node.getResponseBuffer(6);
    Serial.println("=== Data Sensor ===");
    Serial.printf("Soil Moisture   : %.1f %%RH\n", soilHumidity);
    Serial.printf("Soil Temperature: %.1f °C\n", soilTemp);
    Serial.printf("Conductivity    : %u µS/cm\n", ec);
    Serial.printf("pH Value        : %.1f\n", ph);
    Serial.printf("Nitrogen (N)    : %u mg/kg\n", N);
    Serial.printf("Phosphorus (P)  : %u mg/kg\n", P);
    Serial.printf("Potassium (K)   : %u mg/kg\n", K);
    Serial.println("--------------------------\n");
  } else {
    Serial.printf("Gagal membaca blok 7 register! %s (0x%02X)\n", mbErrName(result), result);
    debugReadPerRegister();
  }
  delay(2000);
}
