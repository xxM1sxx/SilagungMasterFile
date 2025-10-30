#include <Arduino.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>

// Pin Configuration (sesuai dengan konfigurasi yang sudah bekerja)
#define RX_PIN 17 // For ESP32 RX2 pin
#define TX_PIN 18 // For ESP32 TX2 pin
#define RS485_DE_RE_PIN 4  // Pin untuk mengontrol DE/RE pada RS485 transceiver

// Konfigurasi Serial untuk RS485
#define RS485_SERIAL Serial2
#define RS485_BAUD 9600

// Register addresses untuk Waveshare Relay 16CH (berdasarkan dokumentasi)
#define RELAY_STATUS_REGISTER 0x0000  // Register untuk membaca status relay (Coil Status)
#define DEVICE_ADDRESS_REGISTER 0x4000 // Register untuk mengubah alamat device (sesuai dokumentasi Waveshare)
#define RELAY_COUNT 16 // Jumlah relay pada modul

// Inisialisasi ModbusMaster
ModbusMaster node;

// Function declarations
void preTransmission();
void postTransmission();
bool changeRelayModuleAddress(uint8_t currentAddress, uint8_t newAddress);
void scanForDevices();
bool readRelayStatus(uint8_t deviceAddress);
bool controlRelay(uint8_t deviceAddress, uint8_t relayNumber, bool state);
void printMenu();
void handleSerialInput();
void testRelayControl();

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("=== Program Pengubah Alamat Waveshare Relay 16CH RS485 ===");
  Serial.println("Berdasarkan protokol Modbus RTU");
  Serial.println("Dokumentasi: https://www.waveshare.com/wiki/Modbus_RTU_Relay_16CH");
  Serial.println();
  
  // Konfigurasi pin RS485 DE/RE
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW); // Set ke mode receive
  
  // Inisialisasi komunikasi RS485
  RS485_SERIAL.begin(RS485_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Konfigurasi ModbusMaster
  node.begin(1, RS485_SERIAL); // Default slave ID = 1
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  Serial.println("Sistem siap!");
  Serial.println("Pastikan modul Waveshare Relay 16CH terhubung ke RS485.");
  Serial.println("Default baud rate: 9600, 8N1");
  Serial.println();
  
  printMenu();
}

void loop() {
  handleSerialInput();
  delay(100);
}

// Fungsi untuk mengatur RS485 ke mode transmit
void preTransmission() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(100); // Delay kecil untuk stabilitas
}

// Fungsi untuk mengatur RS485 ke mode receive
void postTransmission() {
  delayMicroseconds(100); // Delay kecil untuk stabilitas
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

// Fungsi untuk mengubah alamat modul Waveshare Relay 16CH
bool changeRelayModuleAddress(uint8_t currentAddress, uint8_t newAddress) {
  Serial.println("=== Mengubah Alamat Modul Waveshare Relay 16CH ===");
  Serial.printf("Alamat saat ini: %d\n", currentAddress);
  Serial.printf("Alamat baru: %d\n", newAddress);
  Serial.println("Berdasarkan dokumentasi Waveshare Modbus RTU Relay 16CH");
  Serial.println();
  
  // Set node ke alamat saat ini
  node.begin(currentAddress, RS485_SERIAL);
  
  // Validasi alamat baru (1-247 untuk Modbus)
  if (newAddress < 1 || newAddress > 247) {
    Serial.println("ERROR: Alamat baru harus antara 1-247!");
    return false;
  }
  
  // Coba baca status relay untuk memastikan modul terhubung
  Serial.println("Memeriksa koneksi dengan modul relay...");
  uint8_t result = node.readCoils(RELAY_STATUS_REGISTER, RELAY_COUNT);
  
  if (result != node.ku8MBSuccess) {
    Serial.printf("ERROR: Tidak dapat terhubung ke modul di alamat %d\n", currentAddress);
    Serial.printf("Kode error: 0x%02X\n", result);
    Serial.println("Pastikan:");
    Serial.println("- Modul mendapat power 12V/24V");
    Serial.println("- Kabel RS485 A-A, B-B terhubung benar");
    Serial.println("- Alamat saat ini benar");
    return false;
  }
  
  Serial.println("Modul relay ditemukan! Melanjutkan pengubahan alamat...");
  
  // Metode sesuai dokumentasi Waveshare: Write Single Register ke alamat 0x4000
  Serial.println("Menggunakan format sesuai dokumentasi Waveshare:");
  Serial.printf("Command: %02X 06 40 00 00 %02X [CRC akan dihitung otomatis]\n", currentAddress, newAddress);
  
  result = node.writeSingleRegister(DEVICE_ADDRESS_REGISTER, newAddress);
  
  if (result == node.ku8MBSuccess) {
    Serial.println("SUCCESS: Perintah pengubahan alamat berhasil dikirim!");
    Serial.println("Format yang dikirim sesuai dengan dokumentasi Waveshare.");
    
    // Tunggu sebentar untuk proses internal modul
    delay(2000);
    
    // Verifikasi dengan mencoba komunikasi di alamat baru
    Serial.println("Memverifikasi alamat baru...");
    node.begin(newAddress, RS485_SERIAL);
    
    result = node.readCoils(RELAY_STATUS_REGISTER, RELAY_COUNT);
    
    if (result == node.ku8MBSuccess) {
      Serial.println("SUCCESS: Alamat berhasil diubah dan diverifikasi!");
      Serial.printf("Modul sekarang menggunakan alamat: %d\n", newAddress);
      
      // Tampilkan status relay saat ini
      Serial.println("Status relay saat ini:");
      for (int i = 0; i < RELAY_COUNT; i++) {
        bool relayState = node.getResponseBuffer(i / 16) & (1 << (i % 16));
        Serial.printf("  Relay %02d: %s\n", i + 1, relayState ? "ON" : "OFF");
      }
      
      return true;
    } else {
      Serial.println("WARNING: Perintah terkirim tapi verifikasi gagal.");
      Serial.printf("Kode error verifikasi: 0x%02X\n", result);
      Serial.println("Modul mungkin perlu restart manual atau waktu tunggu lebih lama.");
      return false;
    }
  } else {
    Serial.printf("ERROR: Gagal mengirim perintah pengubahan alamat. Kode: 0x%02X\n", result);
    Serial.println("Pastikan format command sesuai dengan dokumentasi Waveshare:");
    Serial.printf("Expected: %02X 06 40 00 00 %02X [CRC]\n", currentAddress, newAddress);
    return false;
  }
}

// Fungsi untuk scan perangkat yang terhubung
void scanForDevices() {
  Serial.println("=== Scanning Modul Waveshare Relay RS485 ===");
  Serial.println("Mencari modul di alamat 1-10...");
  Serial.println();
  
  bool deviceFound = false;
  
  for (uint8_t addr = 1; addr <= 10; addr++) {
    Serial.printf("Checking alamat %d... ", addr);
    
    node.begin(addr, RS485_SERIAL);
    // Gunakan read coils untuk deteksi modul relay
    uint8_t result = node.readCoils(RELAY_STATUS_REGISTER, RELAY_COUNT);
    
    if (result == node.ku8MBSuccess) {
      Serial.println("DITEMUKAN!");
      
      // Tampilkan status relay
      Serial.printf("  -> Modul Waveshare Relay 16CH terdeteksi\n");
      Serial.printf("  -> Status relay: ");
      
      for (int i = 0; i < RELAY_COUNT; i++) {
        bool relayState = node.getResponseBuffer(i / 16) & (1 << (i % 16));
        Serial.printf("%d", relayState ? 1 : 0);
        if ((i + 1) % 4 == 0 && i < RELAY_COUNT - 1) Serial.print(" ");
      }
      Serial.println();
      
      deviceFound = true;
    } else {
      Serial.println("tidak ada");
    }
    
    delay(100); // Delay antar scan
  }
  
  if (!deviceFound) {
    Serial.println("Tidak ada modul ditemukan di alamat 1-10.");
    Serial.println("Pastikan:");
    Serial.println("- Kabel RS485 terhubung dengan benar (A-A, B-B)");
    Serial.println("- Modul mendapat power 12V atau 24V");
    Serial.println("- Baud rate sesuai (default: 9600)");
    Serial.println("- Alamat modul dalam range 1-10");
  }
  
  Serial.println();
}

// Fungsi untuk membaca status relay dari modul
bool readRelayStatus(uint8_t deviceAddress) {
  node.begin(deviceAddress, RS485_SERIAL);
  
  uint8_t result = node.readCoils(RELAY_STATUS_REGISTER, RELAY_COUNT);
  
  if (result == node.ku8MBSuccess) {
    Serial.printf("Status relay modul alamat %d:\n", deviceAddress);
    
    for (int i = 0; i < RELAY_COUNT; i++) {
      bool relayState = node.getResponseBuffer(i / 16) & (1 << (i % 16));
      Serial.printf("  Relay %02d: %s\n", i + 1, relayState ? "ON" : "OFF");
    }
    
    return true;
  } else {
    Serial.printf("ERROR: Gagal membaca status relay. Kode: 0x%02X\n", result);
    return false;
  }
}

// Fungsi untuk mengontrol relay individual
bool controlRelay(uint8_t deviceAddress, uint8_t relayNumber, bool state) {
  if (relayNumber < 1 || relayNumber > RELAY_COUNT) {
    Serial.printf("ERROR: Nomor relay harus antara 1-%d\n", RELAY_COUNT);
    return false;
  }
  
  node.begin(deviceAddress, RS485_SERIAL);
  
  // Gunakan Write Single Coil (Function Code 05)
  uint16_t coilAddress = RELAY_STATUS_REGISTER + (relayNumber - 1);
  uint8_t result = node.writeSingleCoil(coilAddress, state);
  
  if (result == node.ku8MBSuccess) {
    Serial.printf("SUCCESS: Relay %d di alamat %d berhasil di-%s\n", 
                  relayNumber, deviceAddress, state ? "ON" : "OFF");
    return true;
  } else {
    Serial.printf("ERROR: Gagal mengontrol relay %d. Kode: 0x%02X\n", relayNumber, result);
    return false;
  }
}

// Fungsi untuk test kontrol relay
void testRelayControl() {
  Serial.println("=== Test Kontrol Relay ===");
  Serial.println("Masukkan alamat modul (1-247):");
  
  while (!Serial.available()) delay(10);
  uint8_t addr = Serial.readStringUntil('\n').toInt();
  
  Serial.printf("Masukkan nomor relay (1-%d):\n", RELAY_COUNT);
  
  while (!Serial.available()) delay(10);
  uint8_t relayNum = Serial.readStringUntil('\n').toInt();
  
  Serial.println("Masukkan state (1=ON, 0=OFF):");
  
  while (!Serial.available()) delay(10);
  bool state = Serial.readStringUntil('\n').toInt() == 1;
  
  controlRelay(addr, relayNum, state);
  Serial.println();
}

// Fungsi untuk menampilkan menu
void printMenu() {
  Serial.println("=== MENU UTAMA - Waveshare Relay 16CH ===");
  Serial.println("1. Scan modul yang terhubung");
  Serial.println("2. Ubah alamat modul");
  Serial.println("3. Baca status relay");
  Serial.println("4. Test kontrol relay");
  Serial.println("5. Tampilkan menu ini");
  Serial.println();
  Serial.println("Ketik nomor pilihan dan tekan Enter:");
}

// Fungsi untuk menangani input dari Serial Monitor
void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "1") {
      scanForDevices();
      printMenu();
    }
    else if (input == "2") {
      Serial.println("=== Ubah Alamat Modul ===");
      Serial.println("Masukkan alamat saat ini (1-247):");
      
      while (!Serial.available()) delay(10);
      uint8_t currentAddr = Serial.readStringUntil('\n').toInt();
      
      Serial.println("Masukkan alamat baru (1-247):");
      
      while (!Serial.available()) delay(10);
      uint8_t newAddr = Serial.readStringUntil('\n').toInt();
      
      changeRelayModuleAddress(currentAddr, newAddr);
      Serial.println();
      printMenu();
    }
    else if (input == "3") {
      Serial.println("=== Baca Status Relay ===");
      Serial.println("Masukkan alamat modul (1-247):");
      
      while (!Serial.available()) delay(10);
      uint8_t addr = Serial.readStringUntil('\n').toInt();
      
      readRelayStatus(addr);
      Serial.println();
      printMenu();
    }
    else if (input == "4") {
      testRelayControl();
      printMenu();
    }
    else if (input == "5") {
      printMenu();
    }
    else {
      Serial.println("Pilihan tidak valid. Ketik 5 untuk melihat menu.");
    }
  }
}