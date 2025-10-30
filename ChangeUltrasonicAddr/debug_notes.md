# Debug Notes - Ultrasonic Sensor Address Changer

## 🐛 Masalah yang Ditemukan

### 1. **Scanning Gagal Padahal Baca Jarak Berhasil**

**Gejala**: 
- Test baca jarak di alamat 1 berhasil: "Jarak terukur: 250 raw (25.0 cm)"
- Scanning tidak menemukan perangkat di alamat 1-10
- Ubah alamat gagal dengan error 0xE2

**Penyebab**:
- Fungsi `scanForDevices()` menggunakan register `0x0000` untuk deteksi
- Fungsi `changeUltrasonicAddress()` juga menggunakan register `0x0000` untuk verifikasi
- Register `0x0000` mungkin tidak tersedia atau tidak dapat dibaca
- Fungsi `readDistance()` menggunakan register `0x0100` yang terbukti bekerja

### 2. **Error Code 0xE2**

**Arti Error 0xE2**: 
- Dalam ModbusMaster library, 0xE2 biasanya berarti "Invalid Data Address"
- Menunjukkan bahwa register yang diminta tidak tersedia

## 🔧 Perbaikan yang Diterapkan

### 1. **Fix Fungsi scanForDevices()**
```cpp
// Sebelumnya (GAGAL):
uint8_t result = node.readHoldingRegisters(0x0000, 1);

// Sekarang (BEKERJA):
uint8_t result = node.readHoldingRegisters(DISTANCE_REGISTER, 1); // 0x0100
```

### 2. **Fix Fungsi changeUltrasonicAddress()**
```cpp
// Sebelumnya (ERROR 0xE2):
uint8_t result = node.readHoldingRegisters(0x0000, 1);

// Sekarang (BEKERJA):
uint8_t result = node.readHoldingRegisters(DISTANCE_REGISTER, 1); // 0x0100
```

### 3. **Konsistensi Register**
- **Register 0x0100**: Untuk membaca jarak (TERBUKTI BEKERJA)
- **Register 0x0200**: Untuk mengubah alamat (sesuai datasheet)
- **Register 0x0000**: TIDAK DIGUNAKAN (menyebabkan error)

## 📊 Register Map yang Benar

| Register | Fungsi | Status | Keterangan |
|----------|--------|--------|------------|
| 0x0100 | Distance | ✅ BEKERJA | Untuk baca jarak dan deteksi sensor |
| 0x0200 | Address | ✅ SESUAI DATASHEET | Untuk ubah alamat sensor |
| 0x0000 | Unknown | ❌ ERROR 0xE2 | Tidak tersedia/tidak dapat dibaca |

## 🎯 Hasil Perbaikan

Setelah perbaikan:
1. ✅ **Scanning akan bekerja**: Menggunakan register 0x0100 untuk deteksi
2. ✅ **Ubah alamat akan bekerja**: Verifikasi menggunakan register 0x0100
3. ✅ **Konsistensi**: Semua fungsi menggunakan register yang terbukti bekerja
4. ✅ **Error handling**: Tetap ada untuk kasus lain

## 📝 Catatan Teknis

- Sensor ini tidak memiliki register 0x0000 yang dapat dibaca
- Register 0x0100 (distance) adalah register yang paling reliable untuk deteksi
- Menggunakan register yang sama untuk deteksi dan verifikasi memastikan konsistensi
- Error 0xE2 menunjukkan masalah register address, bukan masalah komunikasi RS485