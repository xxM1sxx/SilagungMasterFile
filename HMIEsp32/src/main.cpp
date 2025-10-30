#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>

// ===== KONFIGURASI SISTEM =====
// Pin RS485 untuk ESP32-S3
#define RS485_RX_PIN 17
#define RS485_TX_PIN 18

// Konfigurasi Modbus
#define HMI_SLAVE_ID 2
#define RELAY_SLAVE_ID 6
#define BAUD_RATE 9600

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
unsigned long lastHMIRead = 0;
unsigned long lastLampUpdate = 0;
const unsigned long HMI_READ_INTERVAL = 100;    // Baca HMI setiap 100ms
const unsigned long LAMP_UPDATE_INTERVAL = 500; // Update lampu setiap 500ms

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
  
  delay(10); // Delay kecil untuk stabilitas
}

// ===== IMPLEMENTASI FUNGSI =====

void setupSystem() {
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
  // Inisialisasi komunikasi RS485
  rs485Serial.begin(BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  // Inisialisasi node Modbus
  hmiNode.begin(HMI_SLAVE_ID, rs485Serial);
  relayNode.begin(RELAY_SLAVE_ID, rs485Serial);
  
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
  Serial.printf("Status: HMI=%s Relay=%s | ", 
                hmiCommOK ? "OK" : "ERR", relayCommOK ? "OK" : "ERR");
  
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