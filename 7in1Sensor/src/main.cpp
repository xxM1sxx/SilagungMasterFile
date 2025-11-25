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
  delayMicroseconds(100);
}

void postTransmission() {
  delayMicroseconds(100);
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
void loop() {
  uint8_t result;

  result = node.readHoldingRegisters(REG_SOIL_HUMIDITY, 7); // read 7 registers in a row
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
    Serial.printf("Conductivity    : %d µS/cm\n", ec);
    Serial.printf("pH Value        : %.1f\n", ph);
    Serial.printf("Nitrogen (N)    : %d mg/kg\n", N);
    Serial.printf("Phosphorus (P)  : %d mg/kg\n", P);
    Serial.printf("Potassium (K)   : %d mg/kg\n", K);
    Serial.println("--------------------------\n");
  } else {
    Serial.printf("Gagal membaca sensor! Error Code: 0x%02X\n", result);
  }

  delay(2000); // baca setiap 2 detik
}
