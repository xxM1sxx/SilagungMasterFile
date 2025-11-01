#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>

// ===== KONFIGURASI SISTEM =====
// Pin RS485 untuk ESP32-S3
#define RS485_RX_PIN 17
#define RS485_TX_PIN 18
// Konfigurasi Modbus - Alamat RS485 untuk semua device
#define HMI_SLAVE_ID 2
#define RELAY_SLAVE_ID 6
#define SENSOR1_SLAVE_ID 4        // Ultrasonic Sensor 1
#define SENSOR2_SLAVE_ID 5        // Ultrasonic Sensor 2
#define SOIL7IN1_SLAVE_ID 1       // 7in1 Soil Sensor
#define BAUD_RATE 9600

// Konfigurasi Pressure Sensor (4-20mA ADC)
#define PRESSURE_SENSOR_PIN 13    // Pin ADC untuk pressure sensor
#define VREF 3300                 // ADC reference voltage: 3300mV
#define ADC_SAMPLES 10            // Jumlah sample untuk averaging
#define FILTER_ALPHA 0.1          // Low-pass filter coefficient

// Konfigurasi Valve dan Relay
#define VALVE_COUNT 5
#define RELAY_COUNT 10
#define LAMP_DELAY_TIME 30000  // 30 detik delay untuk lampu dalam milliseconds

// Alamat HMI
#define HMI_BUTTON_START_ADDR 0   // LB0-LB4 untuk tombol kontrol
#define HMI_LAMP_START_ADDR 10    // LB10-LB14 untuk indikator lampu

// ===== OBJEK MODBUS =====
ModbusMaster hmiNode;
ModbusMaster relayNode;
ModbusMaster sensor1Node;
ModbusMaster sensor2Node;
ModbusMaster soil7in1Node;
HardwareSerial rs485Serial(1);



// ===== STRUKTUR DATA =====
struct ValveState {
  bool toggleSwitchState;      // Status toggle switch: true=ON, false=OFF
  bool relayOddState;          // Status relay ganjil (kontinyu)
  bool relayEvenState;         // Status relay genap (kontinyu sebagai penutup)
  unsigned long lampDelayStart; // Waktu mulai delay lampu
  bool lampDelayActive;        // Apakah delay lampu sedang aktif
  bool targetLampState;        // Target status lampu setelah delay
};

// ===== VARIABEL GLOBAL =====
ValveState valves[VALVE_COUNT];
bool buttonStates[VALVE_COUNT] = {false};
bool lastButtonStates[VALVE_COUNT] = {false};
bool lampStates[VALVE_COUNT] = {false};

// Status komunikasi
bool hmiCommOK = false;
bool relayCommOK = false;
bool sensorCommOK = false;
bool soil7in1CommOK = false;
bool pressureCommOK = false;
unsigned long lastHMIRead = 0;
unsigned long lastLampUpdate = 0;
unsigned long lastSensorRead = 0;
unsigned long lastSensorWrite = 0;
unsigned long lastSoil7in1Read = 0;
unsigned long lastSoil7in1Write = 0;
unsigned long lastPressureRead = 0;
unsigned long lastPressureWrite = 0;
unsigned long lastStatusPrint = 0;
const unsigned long HMI_READ_INTERVAL = 100;    // Baca HMI setiap 100ms
const unsigned long LAMP_UPDATE_INTERVAL = 500; // Update lampu setiap 500ms
const unsigned long SENSOR_READ_INTERVAL = 1000; // Baca sensor setiap 1 detik
const unsigned long SENSOR_WRITE_INTERVAL = 1500; // Tulis ke HMI setiap 1.5 detik
const unsigned long SOIL7IN1_READ_INTERVAL = 2000; // Baca 7in1 sensor setiap 2 detik
const unsigned long SOIL7IN1_WRITE_INTERVAL = 2500; // Tulis 7in1 ke HMI setiap 2.5 detik
const unsigned long PRESSURE_READ_INTERVAL = 1000; // Baca pressure sensor setiap 1 detik
const unsigned long PRESSURE_WRITE_INTERVAL = 1200; // Tulis pressure ke HMI setiap 1.2 detik
const unsigned long STATUS_PRINT_INTERVAL = 10000; // Print status setiap 10 detik
const unsigned long MODBUS_DELAY = 50;          // Delay antar komunikasi Modbus (ms)

// Error handling dan retry mechanism
uint8_t hmiRetryCount = 0;
uint8_t relayRetryCount = 0;
const uint8_t MAX_RETRY_COUNT = 3;
unsigned long lastErrorReport = 0;
const unsigned long ERROR_REPORT_INTERVAL = 5000; // Report error setiap 5 detik

// Stuck relay detection
struct RelayMonitor {
  bool isStuck;
  unsigned long lastSuccessTime;
  uint8_t failureCount;
};
RelayMonitor relayMonitors[RELAY_COUNT];

// Sensor data validation
struct SensorData {
  uint16_t distance;
  bool isValid;
  unsigned long lastValidRead;
  uint8_t consecutiveErrors;
};

// 7in1 Soil Sensor data structure
struct Soil7in1Data {
  float ec;           // Electrical Conductivity (us/cm)
  float ph;           // pH value
  uint16_t nitrogen;  // Nitrogen (mg/kg)
  uint16_t phosphorus; // Phosphorus (mg/kg)
  uint16_t potassium; // Potassium (mg/kg)
  float temperature;  // Soil Temperature (°C)
  bool isValid;
  unsigned long lastValidRead;
  uint8_t consecutiveErrors;
};

// Pressure Sensor data structure (4-20mA)
struct PressureData {
  float currentmA;         // Current reading (4-20mA)
  float filteredCurrentmA; // Filtered current using exponential moving average
  float pressureBar;       // Pressure in Bar (0-10 Bar)
  uint16_t rawValue;       // Raw ADC value
  bool isValid;
  unsigned long lastValidRead;
  uint8_t consecutiveErrors;
};

SensorData sensor1Data = {0, false, 0, 0};
SensorData sensor2Data = {0, false, 0, 0};
Soil7in1Data soil7in1Data = {0.0, 0.0, 0, 0, 0, 0.0, false, 0, 0};
PressureData pressureData = {0.0, 0.0, 0.0, 0, false, 0, 0};

const uint8_t MAX_SENSOR_ERRORS = 5;
const uint16_t MIN_VALID_DISTANCE = 5;   // cm
const uint16_t MAX_VALID_DISTANCE = 500; // cm

// 7in1 Sensor validation ranges
const float MIN_VALID_PH = 0.0;
const float MAX_VALID_PH = 14.0;
const float MIN_VALID_EC = 0.0;
const float MAX_VALID_EC = 20000.0; // us/cm
const uint16_t MAX_VALID_NPK = 9999; // mg/kg
const float MIN_VALID_TEMP = -40.0;  // °C
const float MAX_VALID_TEMP = 80.0;   // °C

// Pressure Sensor validation ranges (berdasarkan referensi 4-20mA, 0-10 Bar)
const float MIN_CURRENT = 4.0;      // 4mA = 0 Bar
const float MAX_CURRENT = 20.0;     // 20mA = 10 Bar
const float MAX_PRESSURE = 10.0;    // 10 Bar maksimum
const float MIN_PRESSURE = 0.0;     // 0 Bar minimum

// Emergency stop flag
bool emergencyStop = false;

// ===== DEKLARASI FUNGSI =====
void setupModbus();
void setupSystem();
bool readHMIButtons();
void processButtonChanges();
void updateLampDelays();
void updateLamps();
void updateSingleLamp(uint8_t valveIndex);
bool setRelay(uint8_t relayNum, bool state);
void startLampDelay(uint8_t valveIndex, bool targetState);
void printSystemStatus();
void initializeRelayMonitors();
void checkRelayHealth();
void handleCommunicationError(const char* deviceName, uint8_t errorCode);
void emergencyStopAll();
bool isSystemHealthy();

bool readUltrasonicSensor(uint8_t sensorId, SensorData* sensorData) {
  uint8_t result;
  uint16_t distanceRaw;
  ModbusMaster* sensorNode = (sensorId == 1) ? &sensor1Node : &sensor2Node;
  
  // Tambahkan delay sebelum komunikasi untuk mencegah konflik bus
  delay(MODBUS_DELAY);
  
  // Coba beberapa alamat register yang umum digunakan sensor ultrasonik
  uint16_t registerAddresses[] = {0x0000, 0x0001, 0x0100, 0x0101};
  bool readSuccess = false;
  
  for (uint8_t i = 0; i < 4 && !readSuccess; i++) {
    result = sensorNode->readHoldingRegisters(registerAddresses[i], 1);
    
    if (result == sensorNode->ku8MBSuccess) {
      distanceRaw = sensorNode->getResponseBuffer(0);
      
      // Validasi data - coba berbagai format konversi
      uint16_t distanceCm;
      if (distanceRaw > 1000) {
        distanceCm = distanceRaw / 10; // Format mm ke cm
      } else {
        distanceCm = distanceRaw; // Sudah dalam cm
      }
      
      // Validasi range
      if (distanceCm >= MIN_VALID_DISTANCE && distanceCm <= MAX_VALID_DISTANCE) {
        sensorData->distance = distanceCm;
        sensorData->isValid = true;
        sensorData->lastValidRead = millis();
        sensorData->consecutiveErrors = 0;
        readSuccess = true;
        
        Serial.printf("✓ Sensor %d: %d cm (register 0x%04X)\n", sensorId, distanceCm, registerAddresses[i]);
      }
    }
    
    if (!readSuccess) {
      delay(10); // Delay kecil sebelum mencoba register berikutnya
    }
  }
  
  if (!readSuccess) {
    sensorData->consecutiveErrors++;
    if (sensorData->consecutiveErrors >= MAX_SENSOR_ERRORS) {
      sensorData->isValid = false;
      sensorData->distance = 0;
    }
    Serial.printf("❌ Sensor %d gagal dibaca. Error: %d (consecutive: %d)\n", 
                  sensorId, result, sensorData->consecutiveErrors);
  }
  
  return readSuccess;
}

void readUltrasonicSensors() {
  // Baca Sensor 1
  readUltrasonicSensor(1, &sensor1Data);
  delay(MODBUS_DELAY); // Delay antar sensor
  
  // Baca Sensor 2
  readUltrasonicSensor(2, &sensor2Data);
  
  // Update status komunikasi sensor
  sensorCommOK = (sensor1Data.isValid || sensor2Data.isValid);
}

void writeUltrasonicToHMI() {
  // Tambahkan delay sebelum komunikasi HMI
  delay(MODBUS_DELAY);
  
  // Tulis data Sensor 1 ke HMI Local Word 0
  uint16_t sensor1Value = sensor1Data.isValid ? sensor1Data.distance : 0;
  uint8_t result1 = hmiNode.writeSingleRegister(0x0000, sensor1Value);
  
  if (result1 == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW0 (Sensor 1): %d cm\n", sensor1Value);
  } else {
    Serial.printf("❌ Gagal tulis Sensor 1 ke HMI. Error: %d\n", result1);
  }
  
  delay(MODBUS_DELAY); // Delay antar penulisan
  
  // Tulis data Sensor 2 ke HMI Local Word 1
  uint16_t sensor2Value = sensor2Data.isValid ? sensor2Data.distance : 0;
  uint8_t result2 = hmiNode.writeSingleRegister(0x0001, sensor2Value);
  
  if (result2 == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW1 (Sensor 2): %d cm\n", sensor2Value);
  } else {
    Serial.printf("❌ Gagal tulis Sensor 2 ke HMI. Error: %d\n", result2);
  }
  
  // Update status komunikasi HMI berdasarkan hasil penulisan
  if (result1 == hmiNode.ku8MBSuccess || result2 == hmiNode.ku8MBSuccess) {
    hmiCommOK = true;
  }
}

bool readSoil7in1Sensor() {
  uint8_t result;
  
  // Tambahkan delay sebelum komunikasi untuk mencegah konflik bus
  delay(MODBUS_DELAY);
  
  Serial.println("📊 Membaca sensor 7in1...");
  
  // Baca 7 register sekaligus (0x0000-0x0006) seperti pada kode referensi
  result = soil7in1Node.readHoldingRegisters(0x0000, 7);
  
  if (result == soil7in1Node.ku8MBSuccess) {
    // Ekstrak data dari buffer response
    float soilHumidity = soil7in1Node.getResponseBuffer(0) / 10.0;  // Register 0x0000
    float soilTemp = soil7in1Node.getResponseBuffer(1) / 10.0;      // Register 0x0001
    uint16_t ec = soil7in1Node.getResponseBuffer(2);                // Register 0x0002
    float ph = soil7in1Node.getResponseBuffer(3) / 10.0;           // Register 0x0003
    uint16_t nitrogen = soil7in1Node.getResponseBuffer(4);          // Register 0x0004
    uint16_t phosphorus = soil7in1Node.getResponseBuffer(5);        // Register 0x0005
    uint16_t potassium = soil7in1Node.getResponseBuffer(6);         // Register 0x0006
    
    // Validasi data
    bool allValid = true;
    
    // Validasi pH
    if (ph >= MIN_VALID_PH && ph <= MAX_VALID_PH) {
      soil7in1Data.ph = ph;
      Serial.printf("✓ pH: %.1f\n", ph);
    } else {
      Serial.printf("❌ pH di luar range: %.1f\n", ph);
      allValid = false;
    }
    
    // Validasi Temperature
    if (soilTemp >= MIN_VALID_TEMP && soilTemp <= MAX_VALID_TEMP) {
      soil7in1Data.temperature = soilTemp;
      Serial.printf("✓ Soil Temperature: %.1f°C\n", soilTemp);
    } else {
      Serial.printf("❌ Temperature di luar range: %.1f°C\n", soilTemp);
      allValid = false;
    }
    
    // Validasi EC
    if (ec >= MIN_VALID_EC && ec <= MAX_VALID_EC) {
      soil7in1Data.ec = ec;
      Serial.printf("✓ EC: %d µS/cm\n", ec);
    } else {
      Serial.printf("❌ EC di luar range: %d µS/cm\n", ec);
      allValid = false;
    }
    
    // Validasi NPK
    if (nitrogen <= MAX_VALID_NPK && phosphorus <= MAX_VALID_NPK && potassium <= MAX_VALID_NPK) {
      soil7in1Data.nitrogen = nitrogen;
      soil7in1Data.phosphorus = phosphorus;
      soil7in1Data.potassium = potassium;
      Serial.printf("✓ NPK: N=%d P=%d K=%d mg/kg\n", nitrogen, phosphorus, potassium);
    } else {
      Serial.printf("❌ NPK di luar range: N=%d P=%d K=%d mg/kg\n", nitrogen, phosphorus, potassium);
      allValid = false;
    }
    
    // Update status sensor
    if (allValid) {
      soil7in1Data.isValid = true;
      soil7in1Data.lastValidRead = millis();
      soil7in1Data.consecutiveErrors = 0;
      soil7in1CommOK = true;
      Serial.println("✅ Semua data 7in1 sensor berhasil dibaca");
    } else {
      soil7in1Data.consecutiveErrors++;
      if (soil7in1Data.consecutiveErrors >= MAX_SENSOR_ERRORS) {
        soil7in1Data.isValid = false;
        soil7in1CommOK = false;
      }
      Serial.printf("⚠️ Beberapa data 7in1 tidak valid (consecutive errors: %d)\n", 
                    soil7in1Data.consecutiveErrors);
    }
    
    return allValid;
    
  } else {
    Serial.printf("❌ Gagal baca sensor 7in1. Error: 0x%02X\n", result);
    handleCommunicationError("7in1 Sensor", result);
    
    soil7in1Data.consecutiveErrors++;
    if (soil7in1Data.consecutiveErrors >= MAX_SENSOR_ERRORS) {
      soil7in1Data.isValid = false;
      soil7in1CommOK = false;
    }
    
    return false;
  }
}

// Fungsi konversi 4-20mA ke pressure (0-10 Bar)
float currentToPressure(float currentmA) {
  // Convert 4-20mA to 0-10 Bar
  if(currentmA < MIN_CURRENT) {
    return 0.0; // Below 4mA = fault condition
  }
  
  // Linear conversion: (current - 4mA) / (20mA - 4mA) * 10 Bar
  float pressureBar = ((currentmA - MIN_CURRENT) / (MAX_CURRENT - MIN_CURRENT)) * MAX_PRESSURE;
  
  // Clamp to valid range
  if(pressureBar < 0) pressureBar = 0;
  if(pressureBar > MAX_PRESSURE) pressureBar = MAX_PRESSURE;
  
  return pressureBar;
}

bool readPressureSensor() {
  Serial.println("📊 Membaca pressure sensor (ADC 4-20mA)...");
  
  unsigned long voltageSum = 0;
  
  // Baca multiple samples untuk averaging (seperti pada kode referensi)
  for (int i = 0; i < ADC_SAMPLES; i++) {
    voltageSum += analogRead(PRESSURE_SENSOR_PIN);
    delay(5); // Delay untuk sample independence
  }
  
  // Hitung rata-rata voltage
  uint16_t voltage = (voltageSum / ADC_SAMPLES) / 4096.0 * VREF;
  pressureData.rawValue = voltageSum / ADC_SAMPLES;
  
  // Konversi ke current (mA)
  // Berdasarkan kode referensi: current = voltage / 120.0 (Sense Resistor: 120ohm)
  float currentmA = voltage / 120.0;
  pressureData.currentmA = currentmA;
  
  // Apply exponential moving average (low-pass filter)
  if (pressureData.filteredCurrentmA == 0.0) {
    pressureData.filteredCurrentmA = currentmA; // Initialize on first reading
  } else {
    pressureData.filteredCurrentmA = (FILTER_ALPHA * currentmA) + ((1.0 - FILTER_ALPHA) * pressureData.filteredCurrentmA);
  }
  
  // Validasi filtered current dalam range 4-20mA
  if (pressureData.filteredCurrentmA >= MIN_CURRENT && pressureData.filteredCurrentmA <= MAX_CURRENT) {
    // Konversi filtered current ke pressure
    pressureData.pressureBar = currentToPressure(pressureData.filteredCurrentmA);
    
    // Validasi pressure dalam range yang valid
    if (pressureData.pressureBar >= MIN_PRESSURE && pressureData.pressureBar <= MAX_PRESSURE) {
      pressureData.isValid = true;
      pressureData.lastValidRead = millis();
      pressureData.consecutiveErrors = 0;
      pressureCommOK = true;
      
      Serial.printf("✓ Pressure: %.2f Bar (Filtered Current: %.2f mA, Raw Current: %.2f mA, Voltage: %d mV)\n", 
                    pressureData.pressureBar, pressureData.filteredCurrentmA, currentmA, voltage);
      return true;
    } else {
      Serial.printf("❌ Pressure di luar range: %.2f Bar\n", pressureData.pressureBar);
    }
  } else {
    Serial.printf("❌ Filtered current di luar range 4-20mA: %.2f mA (Raw: %.2f mA)\n", 
                  pressureData.filteredCurrentmA, currentmA);
  }
  
  // Jika sampai sini, data tidak valid
  pressureData.consecutiveErrors++;
  if (pressureData.consecutiveErrors >= MAX_SENSOR_ERRORS) {
    pressureData.isValid = false;
    pressureCommOK = false;
  }
  
  Serial.printf("⚠️ Data pressure tidak valid (consecutive errors: %d)\n", 
                pressureData.consecutiveErrors);
  return false;
}

void writeSoil7in1ToHMI() {
  // Tambahkan delay sebelum komunikasi HMI
  delay(MODBUS_DELAY);
  
  Serial.println("📤 Menulis data 7in1 ke HMI...");
  
  // Tulis EC ke HMI Local Word 4 (LW4)
  uint16_t ecValue = soil7in1Data.isValid ? (uint16_t)soil7in1Data.ec : 0;
  uint8_t result1 = hmiNode.writeSingleRegister(0x0004, ecValue);
  
  if (result1 == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW4 (EC): %d us/cm\n", ecValue);
  } else {
    Serial.printf("❌ Gagal tulis EC ke HMI LW4. Error: %d\n", result1);
  }
  delay(MODBUS_DELAY);
  
  // Tulis pH ke HMI Local Word 5 (LW5) - dikali 10 untuk presisi
  uint16_t phValue = soil7in1Data.isValid ? (uint16_t)(soil7in1Data.ph * 10) : 0;
  uint8_t result2 = hmiNode.writeSingleRegister(0x0005, phValue);
  
  if (result2 == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW5 (pH): %.1f (raw: %d)\n", soil7in1Data.ph, phValue);
  } else {
    Serial.printf("❌ Gagal tulis pH ke HMI LW5. Error: %d\n", result2);
  }
  delay(MODBUS_DELAY);
  
  // Tulis Nitrogen ke HMI Local Word 6 (LW6)
  uint16_t nValue = soil7in1Data.isValid ? soil7in1Data.nitrogen : 0;
  uint8_t result3 = hmiNode.writeSingleRegister(0x0006, nValue);
  
  if (result3 == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW6 (Nitrogen): %d mg/kg\n", nValue);
  } else {
    Serial.printf("❌ Gagal tulis Nitrogen ke HMI LW6. Error: %d\n", result3);
  }
  delay(MODBUS_DELAY);
  
  // Tulis Phosphorus ke HMI Local Word 7 (LW7)
  uint16_t pValue = soil7in1Data.isValid ? soil7in1Data.phosphorus : 0;
  uint8_t result4 = hmiNode.writeSingleRegister(0x0007, pValue);
  
  if (result4 == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW7 (Phosphorus): %d mg/kg\n", pValue);
  } else {
    Serial.printf("❌ Gagal tulis Phosphorus ke HMI LW7. Error: %d\n", result4);
  }
  delay(MODBUS_DELAY);
  
  // Tulis Potassium ke HMI Local Word 8 (LW8)
  uint16_t kValue = soil7in1Data.isValid ? soil7in1Data.potassium : 0;
  uint8_t result5 = hmiNode.writeSingleRegister(0x0008, kValue);
  
  if (result5 == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW8 (Potassium): %d mg/kg\n", kValue);
  } else {
    Serial.printf("❌ Gagal tulis Potassium ke HMI LW8. Error: %d\n", result5);
  }
  delay(MODBUS_DELAY);
  
  // Tulis Temperature ke HMI Local Word 9 (LW9) - dikali 10 untuk presisi
  uint16_t tempValue = soil7in1Data.isValid ? (uint16_t)(soil7in1Data.temperature * 10) : 0;
  uint8_t result6 = hmiNode.writeSingleRegister(0x0009, tempValue);
  
  if (result6 == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW9 (Temperature): %.1f°C (raw: %d)\n", soil7in1Data.temperature, tempValue);
  } else {
    Serial.printf("❌ Gagal tulis Temperature ke HMI LW9. Error: %d\n", result6);
  }
  
  // Update status komunikasi HMI berdasarkan hasil penulisan
  uint8_t successCount = 0;
  if (result1 == hmiNode.ku8MBSuccess) successCount++;
  if (result2 == hmiNode.ku8MBSuccess) successCount++;
  if (result3 == hmiNode.ku8MBSuccess) successCount++;
  if (result4 == hmiNode.ku8MBSuccess) successCount++;
  if (result5 == hmiNode.ku8MBSuccess) successCount++;
  if (result6 == hmiNode.ku8MBSuccess) successCount++;
  
  if (successCount >= 3) { // Minimal 50% berhasil
    hmiCommOK = true;
    Serial.printf("✅ Data 7in1 berhasil ditulis ke HMI (%d/6 register)\n", successCount);
  } else {
    Serial.printf("⚠️ Sebagian data 7in1 gagal ditulis ke HMI (%d/6 register)\n", successCount);
  }
}

void writePressureToHMI() {
  // Tambahkan delay sebelum komunikasi HMI
  delay(MODBUS_DELAY);
  
  Serial.println("📤 Menulis data pressure ke HMI...");
  
  // Tulis pressure ke HMI Local Word 2 (LW2)
  // Konversi pressure dari Bar ke format yang sesuai untuk HMI (misal: x100 untuk 2 desimal)
  uint16_t pressureValue = pressureData.isValid ? (uint16_t)(pressureData.pressureBar * 100) : 0;
  uint8_t result = hmiNode.writeSingleRegister(0x0002, pressureValue);
  
  if (result == hmiNode.ku8MBSuccess) {
    Serial.printf("✓ HMI LW2 (Pressure): %.2f Bar (Raw: %d)\n", 
                  pressureData.pressureBar, pressureValue);
    hmiCommOK = true;
  } else {
    Serial.printf("❌ Gagal tulis pressure ke HMI LW2. Error: %d\n", result);
    hmiCommOK = false;
  }
}

void readUltrasonicSensors();
void writeUltrasonicToHMI();
bool readSoil7in1Sensor();
void writeSoil7in1ToHMI();
bool readPressureSensor();
void writePressureToHMI();
float currentToPressure(float currentmA);

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== ESP32-S3 Valve Control System ===");
  
  // Inisialisasi sistem
  setupSystem();
  setupModbus();
  initializeRelayMonitors();
  
  Serial.println("Sistem siap beroperasi!");
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long currentTime = millis();
  
  // Emergency stop check
  if (emergencyStop) {
    Serial.println("🚨 EMERGENCY STOP AKTIF - Sistem dihentikan");
    delay(1000);
    return;
  }
  
  // System health check
  if (!isSystemHealthy()) {
    Serial.println("⚠️ Sistem tidak sehat - menjalankan emergency stop");
    emergencyStopAll();
    return;
  }
  
  // Baca status tombol HMI
  if (currentTime - lastHMIRead >= HMI_READ_INTERVAL) {
    if (readHMIButtons()) {
      processButtonChanges();
      hmiRetryCount = 0; // Reset retry count on success
    } else {
      hmiRetryCount++;
      if (hmiRetryCount >= MAX_RETRY_COUNT) {
        handleCommunicationError("HMI", 0xE2);
        hmiRetryCount = 0;
      }
    }
    lastHMIRead = currentTime;
  }
  
  // Update delay lampu yang sedang berjalan
  updateLampDelays();
  
  // Check relay health
  checkRelayHealth();
  
  // Update lampu indikator
  if (currentTime - lastLampUpdate >= LAMP_UPDATE_INTERVAL) {
    updateLamps();
    lastLampUpdate = currentTime;
  }

  // Baca sensor ultrasonik dengan interval yang tepat
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    readUltrasonicSensors();
    lastSensorRead = currentTime;
  }
  
  // Tulis data sensor ke HMI dengan interval yang berbeda
  if (currentTime - lastSensorWrite >= SENSOR_WRITE_INTERVAL) {
    writeUltrasonicToHMI();
    lastSensorWrite = currentTime;
  }
  
  // Baca sensor 7in1 dengan interval yang tepat
  if (currentTime - lastSoil7in1Read >= SOIL7IN1_READ_INTERVAL) {
    readSoil7in1Sensor();
    lastSoil7in1Read = currentTime;
  }
  
  // Tulis data sensor 7in1 ke HMI dengan interval yang berbeda
  if (currentTime - lastSoil7in1Write >= SOIL7IN1_WRITE_INTERVAL) {
    writeSoil7in1ToHMI();
    lastSoil7in1Write = currentTime;
  }
  
  // Baca pressure sensor dengan interval yang tepat
  if (currentTime - lastPressureRead >= PRESSURE_READ_INTERVAL) {
    readPressureSensor();
    lastPressureRead = currentTime;
  }
  
  // Tulis data pressure ke HMI dengan interval yang berbeda
  if (currentTime - lastPressureWrite >= PRESSURE_WRITE_INTERVAL) {
    writePressureToHMI();
    lastPressureWrite = currentTime;
  }
  
  // Print status sistem secara berkala
  if (currentTime - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
    printSystemStatus();
    lastStatusPrint = currentTime;
  }
  
  delay(10); // Delay kecil untuk stabilitas
}

// ===== IMPLEMENTASI FUNGSI =====

void setupSystem() {
  // Inisialisasi ADC untuk pressure sensor
  analogReadResolution(12); // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db); // 0-3.3V range
  Serial.println("✓ ADC untuk pressure sensor diinisialisasi");
  
  // Inisialisasi semua valve dalam keadaan default
  for (uint8_t i = 0; i < VALVE_COUNT; i++) {
    valves[i].toggleSwitchState = false;
    valves[i].relayOddState = false;
    valves[i].relayEvenState = false;
    valves[i].lampDelayStart = 0;
    valves[i].lampDelayActive = false;
    valves[i].targetLampState = false;
    
    buttonStates[i] = false;
    lastButtonStates[i] = false;
    lampStates[i] = false;
  }
}

void setupModbus() {
  // Inisialisasi komunikasi RS485 untuk semua perangkat Modbus (HMI, Relay, dan sensor ultrasonik) menggunakan satu serial port.
  rs485Serial.begin(BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  // Inisialisasi node Modbus
  hmiNode.begin(HMI_SLAVE_ID, rs485Serial);
  relayNode.begin(RELAY_SLAVE_ID, rs485Serial);
  sensor1Node.begin(SENSOR1_SLAVE_ID, rs485Serial); // Sensor 1 dengan Slave ID 4
  sensor2Node.begin(SENSOR2_SLAVE_ID, rs485Serial); // Sensor 2 dengan Slave ID 5
  soil7in1Node.begin(SOIL7IN1_SLAVE_ID, rs485Serial); // Sensor 7in1 dengan Slave ID 1
  // Note: Pressure sensor menggunakan ADC analog, bukan Modbus
  
  Serial.println("✓ Inisialisasi komunikasi Modbus selesai");
  
  // Test komunikasi relay
  bool relayTestOK = false;
  for (uint8_t i = 1; i <= RELAY_COUNT; i++) {
    if (setRelay(i, false)) {
      relayTestOK = true;
    }
    delay(100);
  }
  
  if (relayTestOK) {
    Serial.println("✓ Komunikasi relay berhasil");
    relayCommOK = true;
  } else {
    Serial.println("⚠️ Komunikasi relay gagal - periksa koneksi");
    relayCommOK = false;
  }
  
  // Test komunikasi sensor 7in1
  Serial.println("🧪 Test komunikasi sensor 7in1...");
  if (readSoil7in1Sensor()) {
    Serial.println("✓ Komunikasi sensor 7in1 berhasil");
  } else {
    Serial.println("⚠️ Komunikasi sensor 7in1 gagal - periksa koneksi");
  }
  
  // Test pembacaan pressure sensor (ADC)
  Serial.println("🧪 Test pembacaan pressure sensor (ADC)...");
  if (readPressureSensor()) {
    Serial.println("✓ Pembacaan pressure sensor berhasil");
  } else {
    Serial.println("⚠️ Pembacaan pressure sensor gagal - periksa koneksi ADC");
  }
  
  // Reset semua tombol dan lampu HMI
  for (uint8_t i = 0; i < VALVE_COUNT; i++) {
    hmiNode.writeSingleCoil(HMI_BUTTON_START_ADDR + i, false);
    delay(50);
    hmiNode.writeSingleCoil(HMI_LAMP_START_ADDR + i, false);
    delay(50);
  }
}

bool readHMIButtons() {
  uint8_t result = hmiNode.readCoils(HMI_BUTTON_START_ADDR, VALVE_COUNT);
  
  if (result == hmiNode.ku8MBSuccess) {
    uint16_t responseData = hmiNode.getResponseBuffer(0);
    
    // Extract status tombol individual
    for (uint8_t i = 0; i < VALVE_COUNT; i++) {
      buttonStates[i] = (responseData >> i) & 0x01;
    }
    
    hmiCommOK = true;
    hmiRetryCount = 0; // Reset retry count on success
    return true;
  } else {
    hmiCommOK = false;
    if (hmiRetryCount < MAX_RETRY_COUNT) {
      hmiRetryCount++;
      Serial.printf("⚠️ HMI read error: 0x%02X (retry %d/%d)\n", result, hmiRetryCount, MAX_RETRY_COUNT);
    } else {
      handleCommunicationError("HMI", result);
      hmiRetryCount = 0;
    }
    return false;
  }
}

void processButtonChanges() {
  for (uint8_t i = 0; i < VALVE_COUNT; i++) {
    // Deteksi perubahan status toggle switch
    if (buttonStates[i] != lastButtonStates[i]) {
      Serial.printf("🔄 Toggle Switch Valve %d: %s\n", i + 1, buttonStates[i] ? "ON" : "OFF");
      
      // Update status toggle switch
      valves[i].toggleSwitchState = buttonStates[i];
      
      if (buttonStates[i]) {
        // Toggle switch ON - aktifkan relay ganjil kontinyu
        uint8_t oddRelay = (i * 2) + 1; // Relay ganjil (1,3,5,7,9)
        setRelay(oddRelay, true);
        valves[i].relayOddState = true;
        valves[i].relayEvenState = false; // Matikan relay genap
        
        // Matikan relay genap jika aktif
        uint8_t evenRelay = (i * 2) + 2;
        setRelay(evenRelay, false);
        
        // Mulai delay lampu untuk kondisi ON
        startLampDelay(i, true);
        
      } else {
        // Toggle switch OFF - aktifkan relay genap kontinyu sebagai penutup
        uint8_t evenRelay = (i * 2) + 2; // Relay genap (2,4,6,8,10)
        setRelay(evenRelay, true);
        valves[i].relayEvenState = true;
        valves[i].relayOddState = false; // Matikan relay ganjil
        
        // Matikan relay ganjil jika aktif
        uint8_t oddRelay = (i * 2) + 1;
        setRelay(oddRelay, false);
        
        // Mulai delay lampu untuk kondisi OFF
        startLampDelay(i, false);
      }
    }
    
    // Update status tombol sebelumnya
    lastButtonStates[i] = buttonStates[i];
  }
}

void startLampDelay(uint8_t valveIndex, bool targetState) {
  if (valveIndex >= VALVE_COUNT) {
    return;
  }
  
  valves[valveIndex].lampDelayStart = millis();
  valves[valveIndex].lampDelayActive = true;
  valves[valveIndex].targetLampState = targetState;
}

void updateLampDelays() {
  unsigned long currentTime = millis();
  
  for (uint8_t i = 0; i < VALVE_COUNT; i++) {
    if (valves[i].lampDelayActive) {
      unsigned long elapsed = currentTime - valves[i].lampDelayStart;
      
      // Cek apakah delay 30 detik sudah selesai
      if (elapsed >= LAMP_DELAY_TIME) {
        // Update status lampu sesuai target
        lampStates[i] = valves[i].targetLampState;
        valves[i].lampDelayActive = false;
        
        // Update lampu di HMI
        updateSingleLamp(i);
      }
    }
  }
}

void updateSingleLamp(uint8_t valveIndex) {
  if (valveIndex >= VALVE_COUNT) return;
  
  uint16_t lampAddress = HMI_LAMP_START_ADDR + valveIndex;
  uint8_t result = hmiNode.writeSingleCoil(lampAddress, lampStates[valveIndex]);
  
  if (result != hmiNode.ku8MBSuccess) {
    Serial.printf("❌ Gagal update lampu Valve %d (Error: 0x%02X)\n", 
                  valveIndex + 1, result);
  }
}

bool setRelay(uint8_t relayNum, bool state) {
  if (relayNum < 1 || relayNum > RELAY_COUNT) {
    Serial.printf("❌ Error: Relay %d tidak valid (1-%d)\n", relayNum, RELAY_COUNT);
    return false;
  }
  
  // Check if relay is marked as stuck
  uint8_t monitorIndex = relayNum - 1;
  if (relayMonitors[monitorIndex].isStuck) {
    Serial.printf("🚨 Relay %d ditandai sebagai macet - operasi dibatalkan\n", relayNum);
    return false;
  }
  
  // Alamat coil Modbus (0-based)
  uint16_t coilAddress = relayNum - 1;
  
  // Kirim perintah Modbus dengan retry mechanism
  uint8_t result;
  uint8_t attempts = 0;
  
  do {
    result = relayNode.writeSingleCoil(coilAddress, state);
    attempts++;
    
    if (result == relayNode.ku8MBSuccess) {
      // Update relay monitor on success
      relayMonitors[monitorIndex].lastSuccessTime = millis();
      relayMonitors[monitorIndex].failureCount = 0;
      relayMonitors[monitorIndex].isStuck = false;
      
      return true;
    } else {
      // Handle failure
      relayMonitors[monitorIndex].failureCount++;
      
      if (attempts < MAX_RETRY_COUNT) {
        delay(100); // Wait before retry
      }
    }
  } while (attempts < MAX_RETRY_COUNT);
  
  // All attempts failed - mark relay as potentially stuck
  if (relayMonitors[monitorIndex].failureCount >= MAX_RETRY_COUNT) {
    relayMonitors[monitorIndex].isStuck = true;
    Serial.printf("🚨 RELAY %d MACET - Ditandai sebagai tidak dapat digunakan\n", relayNum);
    
    // Trigger emergency stop if critical relay fails
    if (state == true) { // Only for turning ON operations
      Serial.printf("🚨 Critical relay failure - triggering emergency stop\n");
      emergencyStopAll();
    }
  }
  
  handleCommunicationError("Relay", result);
  return false;
}

void updateLamps() {
  // Fungsi ini sekarang hanya dipanggil untuk update batch semua lampu
  // Update individual lampu ditangani oleh updateSingleLamp()
  for (uint8_t i = 0; i < VALVE_COUNT; i++) {
    uint16_t lampAddress = HMI_LAMP_START_ADDR + i;
    hmiNode.writeSingleCoil(lampAddress, lampStates[i]);
  }
}

void printSystemStatus() {
  Serial.printf("Status: HMI=%s Relay=%s Sensor=%s 7in1=%s Pressure=%s | ", 
                hmiCommOK ? "OK" : "ERR", 
                relayCommOK ? "OK" : "ERR",
                sensorCommOK ? "OK" : "ERR",
                soil7in1CommOK ? "OK" : "ERR",
                pressureData.isValid ? "OK" : "ERR");
  
  // Tampilkan status sensor ultrasonik
  Serial.printf("S1:%s(%dcm) S2:%s(%dcm) | ",
                sensor1Data.isValid ? "OK" : "ERR", sensor1Data.distance,
                sensor2Data.isValid ? "OK" : "ERR", sensor2Data.distance);
  
  // Tampilkan status sensor 7in1
  if (soil7in1Data.isValid) {
    Serial.printf("7in1: pH=%.2f EC=%.0f T=%.1f°C NPK=%d/%d/%d | ",
                  soil7in1Data.ph, soil7in1Data.ec, soil7in1Data.temperature,
                  soil7in1Data.nitrogen, soil7in1Data.phosphorus, soil7in1Data.potassium);
  } else {
    Serial.printf("7in1: ERR(errors=%d) | ", soil7in1Data.consecutiveErrors);
  }
  
  // Tampilkan status pressure sensor
  if (pressureData.isValid) {
    Serial.printf("Pressure: %.2f bar (%.1fmA) | ",
                  pressureData.pressureBar, pressureData.filteredCurrentmA);
  } else {
    Serial.printf("Pressure: ERR(errors=%d) | ", pressureData.consecutiveErrors);
  }
  
  // Tampilkan status valve dalam satu baris
  for (uint8_t i = 0; i < VALVE_COUNT; i++) {
    Serial.printf("V%d:%s%s ", i + 1, 
                  valves[i].toggleSwitchState ? "ON" : "OFF",
                  valves[i].lampDelayActive ? "*" : "");
  }
  Serial.println();
}

// ===== ERROR HANDLING DAN MONITORING =====

void initializeRelayMonitors() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    relayMonitors[i].isStuck = false;
    relayMonitors[i].lastSuccessTime = millis();
    relayMonitors[i].failureCount = 0;
  }
}

void checkRelayHealth() {
  unsigned long currentTime = millis();
  
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    // Check if relay hasn't been used successfully for too long
    if (currentTime - relayMonitors[i].lastSuccessTime > 300000) { // 5 minutes
      if (!relayMonitors[i].isStuck && relayMonitors[i].failureCount > 0) {
        Serial.printf("⚠️ Relay %d tidak responsif selama 5 menit\n", i + 1);
      }
    }
    
    // Auto-recovery attempt for stuck relays
    if (relayMonitors[i].isStuck && (currentTime - relayMonitors[i].lastSuccessTime > 60000)) {
      Serial.printf("🔄 Mencoba recovery Relay %d yang macet\n", i + 1);
      
      // Try to turn relay off as recovery attempt
      uint8_t result = relayNode.writeSingleCoil(i, false);
      if (result == relayNode.ku8MBSuccess) {
        relayMonitors[i].isStuck = false;
        relayMonitors[i].failureCount = 0;
        relayMonitors[i].lastSuccessTime = currentTime;
        Serial.printf("✅ Relay %d berhasil di-recovery\n", i + 1);
      }
    }
  }
}

void handleCommunicationError(const char* deviceName, uint8_t errorCode) {
  unsigned long currentTime = millis();
  
  // Rate limit error reporting
  if (currentTime - lastErrorReport < ERROR_REPORT_INTERVAL) {
    return;
  }
  
  Serial.printf("🚨 KOMUNIKASI ERROR - %s: 0x%02X\n", deviceName, errorCode);
  
  switch(errorCode) {
    case 0xE0:
      Serial.printf("   → Invalid slave ID untuk %s\n", deviceName);
      break;
    case 0xE1:
      Serial.printf("   → Invalid function code untuk %s\n", deviceName);
      break;
    case 0xE2:
      Serial.printf("   → Timeout - Periksa koneksi %s\n", deviceName);
      break;
    case 0xE3:
      Serial.printf("   → CRC error dari %s\n", deviceName);
      break;
    default:
      Serial.printf("   → Unknown error dari %s\n", deviceName);
      break;
  }
  
  Serial.println("   Solusi:");
  Serial.println("   1. Periksa kabel RS485 (A+, B-, GND)");
  Serial.println("   2. Periksa power supply device");
  Serial.println("   3. Periksa slave ID dan baud rate");
  Serial.println("   4. Restart sistem jika perlu");
  
  // Reset komunikasi jika terlalu banyak error
  static unsigned long lastReset = 0;
  if (millis() - lastReset > 30000) { // Reset setiap 30 detik jika ada error
    Serial.println("🔄 Reset komunikasi RS485...");
    rs485Serial.end();
    delay(100);
    rs485Serial.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    lastReset = millis();
  }
  
  lastErrorReport = currentTime;
}

void emergencyStopAll() {
  Serial.println("🚨 EMERGENCY STOP - Menghentikan semua operasi");
  
  emergencyStop = true;
  
  // Stop all valve operations and reset states
  for (uint8_t i = 0; i < VALVE_COUNT; i++) {
    // Reset valve states
    valves[i].toggleSwitchState = false;
    valves[i].relayOddState = false;
    valves[i].relayEvenState = false;
    valves[i].lampDelayActive = false;
    valves[i].lampDelayStart = 0;
    valves[i].targetLampState = false;
    
    Serial.printf("🛑 Emergency stop: Valve %d state direset\n", i + 1);
  }
  
  // Turn off all relays as safety measure
  Serial.println("🔌 Mematikan semua relay untuk keamanan...");
  for (uint8_t i = 1; i <= RELAY_COUNT; i++) {
    relayNode.writeSingleCoil(i - 1, false);
    delay(50);
  }
  
  // Turn off all HMI lamps
  Serial.println("💡 Mematikan semua lampu HMI...");
  for (uint8_t i = 0; i < VALVE_COUNT; i++) {
    hmiNode.writeSingleCoil(HMI_LAMP_START_ADDR + i, false);
    lampStates[i] = false;
    delay(50);
  }
  
  Serial.println("🚨 EMERGENCY STOP SELESAI");
  Serial.println("   Untuk restart sistem, reset ESP32");
}

bool isSystemHealthy() {
  // Check communication health
  if (!hmiCommOK && !relayCommOK) {
    Serial.println("⚠️ Sistem tidak sehat: HMI dan Relay tidak terhubung");
    return false;
  }
  
  // Check for too many stuck relays
  uint8_t stuckRelayCount = 0;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (relayMonitors[i].isStuck) {
      stuckRelayCount++;
    }
  }
  
  if (stuckRelayCount > (RELAY_COUNT / 2)) {
    Serial.printf("⚠️ Sistem tidak sehat: %d relay macet (>50%%)\n", stuckRelayCount);
    return false;
  }
  
  return true;
}