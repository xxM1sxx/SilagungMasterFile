#include <Arduino.h>
#include <ModbusMaster.h>

// Instantiate ModbusMaster object
ModbusMaster node;

#define RX_PIN 17 // For ESP32 RX2 pin
#define TX_PIN 18 // For ESP32 TX2 pin

void printModbusError(uint8_t result){
  Serial.print("Modbus error: ");
  switch (result)
  {
  case ModbusMaster::ku8MBIllegalFunction:
    Serial.println("Illegal Function");
    break;
  case ModbusMaster::ku8MBIllegalDataAddress:
    Serial.println("Illegal Data Address");
    break;
  case ModbusMaster::ku8MBIllegalDataValue:
    Serial.println("Illegal Data Value");
    break;
  case ModbusMaster::ku8MBSlaveDeviceFailure:
    Serial.println("Slave Device Failure");
    break;
  default:
    Serial.println("Unknown Error");
  }
}

// Tambahkan variabel untuk timing di bagian atas file, setelah definisi pin
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 2000; // Interval pembacaan dalam milidetik
const unsigned long SENSOR_DELAY = 100;   // Delay antar pembacaan sensor

void setup() {
  Serial.begin(115200);                            // Serial monitor
  Serial2.begin(4800, SERIAL_8N1, RX_PIN, TX_PIN); // RS485 port

  // Modbus slave ID 1
  node.begin(1, Serial2);

  Serial.println("Starting Soil NPK Sensor Reader...");
  delay(1000);
  lastReadTime = millis(); // Inisialisasi waktu pertama
}

void loop(){
  // Cek apakah sudah waktunya membaca sensor
  if (millis() - lastReadTime >= READ_INTERVAL) {
    uint8_t result;
    bool success = true;
    unsigned long sensorLastRead = 0;

    Serial.println("\n--- Reading Sensor Values ---");

    // Read pH (Register 0x06)
    result = node.readHoldingRegisters(0x06, 1);
    if (result == node.ku8MBSuccess) {
      float ph = node.getResponseBuffer(0) / 100.0;
      Serial.print("pH: ");
      Serial.println(ph, 2);
    } else {
      Serial.print("Failed to read pH! ");
      printModbusError(result);
      success = false;
    }
    while (millis() - sensorLastRead < SENSOR_DELAY);
    sensorLastRead = millis();

    result = node.readHoldingRegisters(0x12, 1);  // Register 0x12 untuk moisture
  if (result == node.ku8MBSuccess) {
      float moisture = node.getResponseBuffer(0) / 10.0;  // Dibagi 10 sesuai dokumentasi
      
      Serial.print("Soil Moisture: ");
      Serial.print(moisture);
      Serial.println(" %");
    } else {
      Serial.println("Gagal membaca moisture!");
    }

    result = node.readHoldingRegisters(0x13, 1);
    if (result == node.ku8MBSuccess){
      float temperature = node.getResponseBuffer(0) / 10.0;

      Serial.print("Soil Temperature: ");
      Serial.print(temperature, 1);
      Serial.println(" °C");
    } else {
      Serial.print("Failed to read moisture/temperature! ");
      printModbusError(result);
      success = false;
    }
    while (millis() - sensorLastRead < SENSOR_DELAY);
    sensorLastRead = millis();

// Read Air Humidity (Register 0x14)
  result = node.readHoldingRegisters(0x14, 1);
  if (result == node.ku8MBSuccess) {
      float airHumidity = node.getResponseBuffer(0) / 10.0;  // Dibagi 10 sesuai dokumentasi
      
      Serial.print("Air Humidity: ");
      Serial.print(airHumidity);
      Serial.println(" %");
  } else {
      Serial.print("Gagal membaca air humidity! ");
      printModbusError(result);
      success = false;
  }

    while (millis() - sensorLastRead < SENSOR_DELAY);
    sensorLastRead = millis();

    // Read Conductivity (Register 0x15)
    result = node.readHoldingRegisters(0x15, 1);
    if (result == node.ku8MBSuccess) {
      uint16_t conductivity = node.getResponseBuffer(0);
      Serial.print("Soil Conductivity: ");
      Serial.print(conductivity);
      Serial.println(" us/cm");
    } else {
      Serial.print("Failed to read conductivity! ");
      printModbusError(result);
      success = false;
    }

    while (millis() - sensorLastRead < SENSOR_DELAY);
    sensorLastRead = millis();

    // Read NPK Values (Registers 0x1E-0x20)
    result = node.readHoldingRegisters(0x1E, 1);
    if (result == node.ku8MBSuccess){
      uint16_t nitrogen = node.getResponseBuffer(0);
      Serial.println("\nNPK Values:");
      Serial.print("Nitrogen (N): ");
      Serial.print(nitrogen);
      Serial.println(" mg/kg");
    } else {
      Serial.print("Failed to read NPK values! ");
      printModbusError(result);
      success = false;
    }

    while (millis() - sensorLastRead < SENSOR_DELAY);
    sensorLastRead = millis();

    result = node.readHoldingRegisters(0x1F, 1);
    if (result == node.ku8MBSuccess) {
      uint16_t phosphorus = node.getResponseBuffer(0);
      Serial.print("Phosphorus (P): ");
      Serial.print(phosphorus);
      Serial.println(" mg/kg");
    } else {
      Serial.print("Failed to read phosphorus! ");
      printModbusError(result);
      success = false;
    }

    while (millis() - sensorLastRead < SENSOR_DELAY);
    sensorLastRead = millis();

    result = node.readHoldingRegisters(0x20, 1);
    if (result == node.ku8MBSuccess) {
      uint16_t potassium = node.getResponseBuffer(0);
      Serial.print("Potassium / Kalium (K): ");
      Serial.print(potassium);
      Serial.println(" mg/kg");
    } else {
      Serial.print("Failed to read potassium! ");
      printModbusError(result);
      success = false;
    }


    while (millis() - sensorLastRead < SENSOR_DELAY);
    sensorLastRead = millis();


    if (success) {
      Serial.println("\nAll values read successfully!");
    } else {
      Serial.println("\nSome readings failed, check connections and power supply");
    }

    Serial.println("--------------------------------");

    lastReadTime = millis(); // Update waktu pembacaan terakhir
  }
}