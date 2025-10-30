# Program Pengubah Alamat Sensor Ultrasonik RS485

Program ini dirancang untuk mengubah alamat sensor ultrasonik yang menggunakan protokol Modbus RTU melalui interface RS485.

## 🔧 Fitur Program

1. **Scan Otomatis**: Mencari sensor di alamat 1-10
2. **Ubah Alamat**: Mengubah alamat sensor dengan validasi
3. **Test Jarak**: Membaca dan menampilkan jarak dari sensor
4. **Menu Interaktif**: Interface yang mudah digunakan
5. **Error Handling**: Penanganan error yang komprehensif
6. **Validasi Alamat**: Memastikan alamat dalam range yang valid (1-247)
7. **Verifikasi Datasheet**: Menggunakan register yang sesuai dengan spesifikasi resmi (0x0200)

## Hardware yang Diperlukan

- ESP32-S3 DevKit
- Sensor Ultrasonik RS485 dengan protokol Modbus RTU
- RS485 Transceiver Module (seperti MAX485)
- Kabel jumper

## 🔌 Koneksi Hardware

### Pin Configuration (Berdasarkan Konfigurasi yang Sudah Terbukti Bekerja)
```
ESP32-S3 DevKit          RS485 Transceiver          Sensor Ultrasonik RS485
┌─────────────────┐      ┌─────────────────┐        ┌─────────────────────┐
│                 │      │                 │        │                     │
│ 3.3V ───────────┼──────┤ VCC             │        │                     │
│ GND ────────────┼──────┤ GND             │        │                     │
│                 │      │                 │        │                     │
│ GPIO 18 (TX) ───┼──────┤ DI (Data Input) │        │                     │
│ GPIO 17 (RX) ───┼──────┤ RO (Recv Output)│        │                     │
│ GPIO 4 ─────────┼──────┤ DE (Drive Enable│        │                     │
│                 │      │ RE (Recv Enable)│        │                     │
│                 │      │                 │        │                     │
│                 │      │ A+ ─────────────┼────────┤ A+ (RS485+)         │
│                 │      │ B- ─────────────┼────────┤ B- (RS485-)         │
│                 │      │                 │        │                     │
│                 │      │                 │        │ VCC ────── Power    │
│                 │      │                 │        │ GND ────── Ground   │
└─────────────────┘      └─────────────────┘        └─────────────────────┘
```

**Catatan Penting:** Konfigurasi pin ini sudah terbukti bekerja dengan sensor ultrasonik RS485.

### RS485 Transceiver ke ESP32-S3:
- VCC → 3.3V atau 5V
- GND → GND
- DI (Data Input) → GPIO 18 (TX) ← **Sesuai konfigurasi yang bekerja**
- RO (Receiver Output) → GPIO 17 (RX) ← **Sesuai konfigurasi yang bekerja**
- DE (Driver Enable) → GPIO 4 (opsional)
- RE (Receiver Enable) → GPIO 4 (opsional)

### RS485 Transceiver ke Sensor:
- A+ → A+ sensor
- B- → B- sensor
- Pastikan sensor mendapat power sesuai spesifikasi

## Konfigurasi Software

### Library yang Diperlukan:
- ModbusMaster (sudah dikonfigurasi di platformio.ini)

### Parameter Komunikasi:
- Baud Rate: 9600 (default, sesuaikan dengan sensor)
- Data Bits: 8
- Parity: None
- Stop Bits: 1
- RS485 Control Pin: GPIO 4

## Cara Penggunaan

1. **Upload Program**: Upload program ke ESP32-S3
2. **Buka Serial Monitor**: Set baud rate ke 115200
3. **Pilih Menu**:
   - Ketik `1` untuk scan perangkat
   - Ketik `2` untuk ubah alamat sensor
   - Ketik `3` untuk test baca jarak
   - Ketik `4` untuk tampilkan menu

### Contoh Penggunaan Ubah Alamat:

1. Pilih menu `2`
2. Masukkan alamat saat ini (misal: `1`)
3. Masukkan alamat baru (misal: `5`)
4. Program akan:
   - Verifikasi koneksi ke alamat lama
   - Mengirim perintah ubah alamat
   - Verifikasi koneksi ke alamat baru

## 📊 Spesifikasi Modbus (Sesuai Datasheet)

- **Protokol**: Modbus RTU
- **Baud Rate**: 9600 bps
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Register Jarak**: `0x0100` (raw value, unsigned 16-bit, bagi 10 untuk cm)
- **Register Alamat**: `0x0200` (sesuai datasheet resmi, unsigned 16-bit, range 1-254)
- **Range Alamat**: 1-247 (untuk keamanan, menghindari broadcast address 255)

> **⚠️ Penting**: Register alamat adalah `0x0200` sesuai datasheet resmi sensor. 
> Register ini berbeda dengan implementasi umum yang sering menggunakan `0x0000`

## 🐛 Troubleshooting

### Masalah Umum dan Solusi

1. **Scanning tidak menemukan sensor padahal baca jarak berhasil**
   - ✅ **DIPERBAIKI**: Program sekarang menggunakan register 0x0100 untuk deteksi
   - Register 0x0000 tidak tersedia pada sensor ini (menyebabkan error 0xE2)

2. **Error 0xE2 saat ubah alamat**
   - ✅ **DIPERBAIKI**: Verifikasi koneksi menggunakan register 0x0100 yang terbukti bekerja
   - Error 0xE2 = "Invalid Data Address" (register tidak tersedia)

3. **Sensor tidak terdeteksi**
   - Periksa koneksi RS485 (A ke A+, B ke B-)
   - Pastikan power supply sensor (biasanya 12V atau 24V)
   - Cek baud rate (default: 9600)
   - Pastikan grounding yang baik

4. **Komunikasi tidak stabil**
   - Tambahkan resistor terminasi 120Ω di ujung kabel RS485
   - Gunakan kabel twisted pair untuk RS485
   - Hindari kabel terlalu panjang (max 1200m untuk RS485)

### Register yang Bekerja
- ✅ **0x0100**: Distance register (untuk baca jarak dan deteksi sensor)
- ✅ **0x0200**: Address register (untuk ubah alamat sesuai datasheet)
- ❌ **0x0000**: Tidak tersedia (menyebabkan error 0xE2)

## Modifikasi untuk Sensor Spesifik

Jika sensor Anda menggunakan register yang berbeda, modifikasi bagian berikut di kode:

```cpp
// Untuk register alamat yang berbeda
result = node.writeSingleRegister(0x0200, newAddress); // Ganti 0x0100 ke register yang sesuai

// Untuk register jarak yang berbeda  
uint8_t result = node.readHoldingRegisters(0x0002, 1); // Ganti 0x0001 ke register yang sesuai
```

## Referensi

Program ini dikembangkan berdasarkan:
- Protokol Modbus RTU standar <mcreference link="https://gaslab.com/pages/rs485-modbus-rtu-interface" index="3">3</mcreference>
- Dokumentasi sensor ultrasonik RS485 <mcreference link="https://wiki.seeedstudio.com/RS485_750cm_Ultrasonic_Sensor-1/" index="2">2</mcreference>
- Library ModbusMaster untuk Arduino <mcreference link="https://apgsensors.com/product/modbus-ultrasonic-level-sensor/" index="1">1</mcreference>

## Lisensi

Program ini bebas digunakan untuk keperluan pendidikan dan komersial.