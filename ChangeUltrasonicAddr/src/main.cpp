#include <Arduino.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>

// Pin Configuration (sesuai dengan konfigurasi yang sudah bekerja)
#define RX_PIN 17 // For ESP32 RX2 pin (sesuai contoh yang bekerja)
#define TX_PIN 18 // For ESP32 TX2 pin (sesuai contoh yang bekerja)
#define RS485_DE_RE_PIN 4  // Pin untuk mengontrol DE/RE pada RS485 transceiver (opsional)

// Konfigurasi Serial untuk RS485
#define RS485_SERIAL Serial2
#define RS485_BAUD 9600

// Register addresses (berdasarkan dokumentasi sensor yang benar)
#define DISTANCE_REGISTER 0x0100  // Register untuk membaca jarak (dari contoh yang bekerja)
#define ADDRESS_REGISTER 0x0200   // Register untuk mengubah alamat (sesuai datasheet: 0x0200)

// Inisialisasi ModbusMaster
ModbusMaster node;

// Function declarations
void preTransmission();
void postTransmission();
bool changeUltrasonicAddress(uint8_t currentAddress, uint8_t newAddress);
void scanForDevices();
uint16_t readDistance(uint8_t deviceAddress);
void printMenu();
void handleSerialInput();

void setup() {
  // Inisialisasi Serial Monitor
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("=== Program Pengubah Alamat Sensor Ultrasonik RS485 ===");
  Serial.println("Berdasarkan protokol Modbus RTU");
  Serial.println();
  
  // Konfigurasi pin RS485 DE/RE
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW); // Set ke mode receive
  
  // Inisialisasi komunikasi RS485 (menggunakan pin yang sama dengan contoh yang bekerja)
  RS485_SERIAL.begin(RS485_BAUD, SERIAL_8N1, RX_PIN, TX_PIN); // RX=17, TX=18 seperti contoh
  
  // Konfigurasi ModbusMaster
  node.begin(1, RS485_SERIAL); // Default slave ID = 1
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  Serial.println("Sistem siap!");
  Serial.println("Pastikan sensor ultrasonik terhubung ke RS485.");
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

// Fungsi untuk mengubah alamat sensor ultrasonik
bool changeUltrasonicAddress(uint8_t currentAddress, uint8_t newAddress) {
  Serial.println("=== Mengubah Alamat Sensor ===");
  Serial.printf("Alamat saat ini: %d\n", currentAddress);
  Serial.printf("Alamat baru: %d\n", newAddress);
  Serial.printf("Format datasheet: %02X 06 02 00 00 %02X [CRC]\n", currentAddress, newAddress);
  Serial.println();
  
  // Set node ke alamat saat ini
  node.begin(currentAddress, RS485_SERIAL);
  
  // Validasi alamat baru (1-247 untuk Modbus)
  if (newAddress < 1 || newAddress > 247) {
    Serial.println("ERROR: Alamat baru harus antara 1-247!");
    return false;
  }
  
  // Coba baca data dari alamat saat ini untuk memastikan sensor terhubung
  Serial.println("Memeriksa koneksi dengan sensor...");
  uint8_t result = node.readHoldingRegisters(DISTANCE_REGISTER, 1); // Gunakan register jarak yang pasti ada
  
  if (result != node.ku8MBSuccess) {
    Serial.printf("ERROR: Tidak dapat terhubung ke sensor di alamat %d\n", currentAddress);
    Serial.printf("Kode error: 0x%02X\n", result);
    return false;
  }
  
  Serial.println("Sensor ditemukan! Melanjutkan pengubahan alamat...");
  
  // Metode sesuai datasheet: Function Code 06, Register 0x0200
  Serial.println("Menggunakan format sesuai datasheet:");
  Serial.printf("Command: %02X 06 02 00 00 %02X [CRC akan dihitung otomatis]\n", currentAddress, newAddress);
  
  result = node.writeSingleRegister(ADDRESS_REGISTER, newAddress);
  
  if (result == node.ku8MBSuccess) {
    Serial.println("SUCCESS: Perintah pengubahan alamat berhasil dikirim!");
    Serial.println("Format yang dikirim sesuai dengan datasheet.");
    
    // Tunggu sebentar untuk proses internal sensor
    delay(2000);
    
    // Verifikasi dengan mencoba komunikasi di alamat baru
    Serial.println("Memverifikasi alamat baru...");
    node.begin(newAddress, RS485_SERIAL);
    
    result = node.readHoldingRegisters(DISTANCE_REGISTER, 1);
    
    if (result == node.ku8MBSuccess) {
      Serial.println("SUCCESS: Alamat berhasil diubah dan diverifikasi!");
      Serial.printf("Sensor sekarang menggunakan alamat: %d\n", newAddress);
      return true;
    } else {
      Serial.println("WARNING: Perintah terkirim tapi verifikasi gagal.");
      Serial.printf("Kode error verifikasi: 0x%02X\n", result);
      Serial.println("Sensor mungkin perlu restart manual atau waktu tunggu lebih lama.");
      return false;
    }
  } else {
    Serial.printf("ERROR: Gagal mengirim perintah pengubahan alamat. Kode: 0x%02X\n", result);
    Serial.println("Pastikan format command sesuai dengan datasheet:");
    Serial.printf("Expected: %02X 06 02 00 00 %02X [CRC]\n", currentAddress, newAddress);
    return false;
  }
}

// Fungsi untuk scan perangkat yang terhubung
void scanForDevices() {
  Serial.println("=== Scanning Perangkat RS485 ===");
  Serial.println("Mencari sensor di alamat 1-10...");
  Serial.println();
  
  bool deviceFound = false;
  
  for (uint8_t addr = 1; addr <= 10; addr++) {
    Serial.printf("Checking alamat %d... ", addr);
    
    node.begin(addr, RS485_SERIAL);
    // Gunakan register jarak (0x0100) untuk deteksi karena register ini pasti ada
    uint8_t result = node.readHoldingRegisters(DISTANCE_REGISTER, 1);
    
    if (result == node.ku8MBSuccess) {
      Serial.println("DITEMUKAN!");
      
      // Coba baca jarak untuk memastikan ini sensor ultrasonik
      uint16_t distance = readDistance(addr);
      if (distance > 0) {
        Serial.printf("  -> Jarak terukur: %d raw (%.1f cm)\n", distance, distance/10.0);
      }
      
      deviceFound = true;
    } else {
      Serial.println("tidak ada");
    }
    
    delay(100); // Delay antar scan
  }
  
  if (!deviceFound) {
    Serial.println("Tidak ada perangkat ditemukan di alamat 1-10.");
    Serial.println("Pastikan:");
    Serial.println("- Kabel RS485 terhubung dengan benar");
    Serial.println("- Sensor mendapat power");
    Serial.println("- Baud rate sesuai (default: 9600)");
  }
  
  Serial.println();
}

// Fungsi untuk membaca jarak dari sensor
uint16_t readDistance(uint8_t deviceAddress) {
  node.begin(deviceAddress, RS485_SERIAL);
  
  // Menggunakan register yang sama dengan contoh yang bekerja (0x0100)
  uint8_t result = node.readHoldingRegisters(DISTANCE_REGISTER, 1);
  
  if (result == node.ku8MBSuccess) {
    return node.getResponseBuffer(0);
  }
  
  return 0; // Return 0 jika gagal
}

// Fungsi untuk menampilkan menu
void printMenu() {
  Serial.println("=== MENU UTAMA ===");
  Serial.println("1. Scan perangkat yang terhubung");
  Serial.println("2. Ubah alamat sensor");
  Serial.println("3. Test baca jarak sensor");
  Serial.println("4. Tampilkan menu ini");
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
      Serial.println("=== Ubah Alamat Sensor ===");
      Serial.println("Masukkan alamat saat ini (1-247):");
      
      while (!Serial.available()) delay(10);
      uint8_t currentAddr = Serial.readStringUntil('\n').toInt();
      
      Serial.println("Masukkan alamat baru (1-247):");
      
      while (!Serial.available()) delay(10);
      uint8_t newAddr = Serial.readStringUntil('\n').toInt();
      
      changeUltrasonicAddress(currentAddr, newAddr);
      Serial.println();
      printMenu();
    }
    else if (input == "3") {
      Serial.println("=== Test Baca Jarak ===");
      Serial.println("Masukkan alamat sensor (1-247):");
      
      while (!Serial.available()) delay(10);
      uint8_t addr = Serial.readStringUntil('\n').toInt();
      
      Serial.printf("Membaca jarak dari sensor alamat %d...\n", addr);
      uint16_t distance = readDistance(addr);
      
      if (distance > 0) {
        Serial.printf("Jarak terukur: %d raw (%.1f cm)\n", distance, distance/10.0);
      } else {
        Serial.println("Gagal membaca jarak. Periksa koneksi dan alamat.");
      }
      
      Serial.println();
      printMenu();
    }
    else if (input == "4") {
      printMenu();
    }
    else {
      Serial.println("Pilihan tidak valid. Ketik 4 untuk melihat menu.");
    }
  }
}