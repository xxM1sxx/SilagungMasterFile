# ESP32-S3 Valve Control System

Sistem kontrol valve otomatis menggunakan ESP32-S3 dengan komunikasi Modbus RTU untuk mengontrol 5 valve melalui 10 relay dan HMI interface.

## 🎯 Fitur Utama

- **5 Valve Control**: Kontrol independen untuk 5 valve dengan relay terpisah
- **Smart Relay Mapping**: Relay ganjil untuk buka, relay genap untuk tutup
- **Precise Timing**: Delay 35 detik untuk setiap operasi valve
- **Visual Feedback**: Lampu indikator LB10-LB14 untuk status valve
- **Error Handling**: Deteksi relay macet dan komunikasi error
- **Emergency Stop**: Sistem keamanan untuk kondisi darurat
- **Modular Design**: Struktur kode yang mudah di-maintain

## 🔧 Spesifikasi Hardware

### ESP32-S3 Development Board
- **Microcontroller**: ESP32-S3 (Dual Core, 240MHz)
- **Memory**: 512KB SRAM, 384KB ROM
- **Connectivity**: WiFi, Bluetooth, RS485

### HMI Interface
- **Protocol**: Modbus RTU
- **Slave ID**: 2
- **Baud Rate**: 9600
- **Control Buttons**: LB0-LB4 (Valve 1-5)
- **Indicator Lamps**: LB10-LB14 (Status Valve 1-5)

### Relay Module
- **Model**: Waveshare 10-Channel Relay Module
- **Protocol**: Modbus RTU
- **Slave ID**: 6
- **Baud Rate**: 9600
- **Relay Count**: 10 (2 per valve)

## 📋 Konfigurasi Sistem

### Mapping Valve dan Relay
```
Valve 1: Relay 1 (buka) | Relay 2 (tutup) → Lampu LB10
Valve 2: Relay 3 (buka) | Relay 4 (tutup) → Lampu LB11
Valve 3: Relay 5 (buka) | Relay 6 (tutup) → Lampu LB12
Valve 4: Relay 7 (buka) | Relay 8 (tutup) → Lampu LB13
Valve 5: Relay 9 (buka) | Relay 10 (tutup) → Lampu LB14
```

### Pin Configuration
```cpp
#define RS485_TX_PIN    17  // GPIO17 untuk transmit
#define RS485_RX_PIN    16  // GPIO16 untuk receive
#define RS485_DE_PIN    4   // GPIO4 untuk direction control
```

## 🚀 Quick Start

### 1. Hardware Setup
1. Hubungkan ESP32-S3 dengan HMI dan Relay Module via RS485
2. Pastikan power supply mencukupi untuk semua komponen
3. Verifikasi koneksi A+, B-, dan GND

### 2. Software Setup
1. Install PlatformIO atau Arduino IDE
2. Install library yang diperlukan:
   - ModbusMaster
   - HardwareSerial
3. Upload kode ke ESP32-S3

### 3. Testing
1. Buka Serial Monitor (115200 baud)
2. Verifikasi inisialisasi sistem berhasil
3. Test komunikasi HMI dan relay
4. Ikuti panduan testing lengkap di `TESTING_GUIDE.md`

## 💡 Cara Penggunaan

### Operasi Normal
1. **Buka Valve**: Tekan tombol LBx pada HMI
   - Relay ganjil akan menyala selama 35 detik
   - Lampu indikator menyala setelah operasi selesai

2. **Tutup Valve**: Tekan tombol LBx lagi
   - Relay genap akan menyala selama 35 detik
   - Lampu indikator mati setelah operasi selesai

3. **Stop Operasi**: Tekan tombol yang sama saat operasi berlangsung
   - Operasi akan dihentikan segera
   - Relay akan dimatikan

### Multiple Valve Operation
- Sistem mendukung operasi multiple valve bersamaan
- Setiap valve beroperasi independen
- Maksimal 5 valve dapat beroperasi bersamaan

## 🛡️ Sistem Keamanan

### Error Handling
- **Komunikasi Error**: Auto-retry dengan exponential backoff
- **Relay Stuck**: Deteksi dan marking relay bermasalah
- **System Health**: Monitoring kesehatan sistem real-time

### Emergency Stop
- Aktivasi otomatis saat sistem tidak sehat
- Manual trigger melalui kondisi error critical
- Mematikan semua relay dan lampu indikator

### Recovery Mechanism
- Auto-recovery untuk relay stuck setelah 1 menit
- Reset komunikasi error setelah periode tertentu
- Restart sistem jika diperlukan

## 📊 Monitoring dan Debug

### Serial Output
Sistem menyediakan output serial yang informatif:
```
=== ESP32-S3 Valve Control System ===
✓ Sistem valve diinisialisasi
🔘 Tombol Valve 1 ditekan
🔌 Relay 1: ON
⏳ Valve 1 membuka - sisa 25 detik
✅ Valve 1 operasi selesai - Status: TERBUKA
💡 Lampu LB10 (Valve 1): ON
```

### Error Messages
```
🚨 KOMUNIKASI ERROR - HMI: 0xE2
🚨 RELAY 3 MACET - Ditandai sebagai tidak dapat digunakan
🚨 EMERGENCY STOP - Menghentikan semua operasi
```

## 🔧 Maintenance

### Regular Checks
1. Verifikasi komunikasi RS485 stabil
2. Test semua relay secara berkala
3. Monitor error log untuk pattern issues
4. Update firmware jika ada perbaikan

### Troubleshooting
1. **Komunikasi Error**: 
   - Periksa koneksi RS485
   - Verifikasi baud rate dan slave ID
   - Test dengan Modbus scanner tool

2. **Relay Tidak Respon**:
   - Periksa power supply relay module
   - Test relay secara manual
   - Ganti relay jika perlu

3. **HMI Tidak Respon**:
   - Restart HMI
   - Periksa konfigurasi Modbus
   - Verifikasi address mapping

## 📁 Struktur File

```
ReadWriteMultiSlave/
├── src/
│   └── main.cpp              # Main application code
├── platformio.ini            # PlatformIO configuration
├── README.md                 # This file
└── TESTING_GUIDE.md          # Comprehensive testing guide
```

## 🔄 Version History

### v2.0.0 (Current)
- Complete rewrite dengan struktur modular
- Implementasi error handling yang robust
- Emergency stop mechanism
- Comprehensive testing framework
- Improved documentation

### v1.x.x (Legacy)
- Basic valve control functionality
- Simple Modbus communication
- Limited error handling

## 📞 Support

Untuk pertanyaan teknis atau bug report, silakan:
1. Periksa dokumentasi dan testing guide
2. Review error messages di serial output
3. Test dengan hardware minimal setup
4. Dokumentasikan langkah reproduksi issue

## 📄 License

Project ini dikembangkan untuk keperluan akademik dan industri.
Silakan gunakan dan modifikasi sesuai kebutuhan dengan tetap mempertahankan credit.

---

**Developed with ❤️ for Industrial Automation**