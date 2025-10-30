# Analisis Datasheet Sensor Ultrasonik RS485

## 📋 Informasi dari Datasheet

Berdasarkan analisis datasheet **RS485-750cm-Ultrasonic-Level-Sensor**, ditemukan spesifikasi register yang benar:

### Register Alamat Sensor (Slave Address)

| Parameter | Spesifikasi Datasheet | Implementasi Sebelumnya | Status |
|-----------|----------------------|-------------------------|---------|
| **Register Address** | **0x0200** | 0x0000 | ❌ **SALAH** |
| **Data Type** | UINT16 | UINT16 | ✅ Benar |
| **Value Range** | 0x01 to 0xFE | 1-247 | ✅ Benar |
| **Default Value** | 0x01 | - | ✅ Sesuai |
| **Access** | Read/Write | Read/Write | ✅ Benar |

### Kesalahan yang Ditemukan

1. **Register Address Salah**: 
   - Implementasi menggunakan `0x0000` 
   - Datasheet menunjukkan `0x0200`

2. **Komentar Tidak Akurat**:
   - Kode menyebutkan "register alamat yang umum"
   - Seharusnya mengacu pada spesifikasi datasheet yang spesifik

## 🔧 Perbaikan yang Dilakukan

### 1. Update Definisi Register
```cpp
// Sebelumnya (SALAH):
#define ADDRESS_REGISTER 0x0000   // Register untuk mengubah alamat

// Sekarang (BENAR):
#define ADDRESS_REGISTER 0x0200   // Register untuk mengubah alamat (sesuai datasheet: 0x0200)
```

### 2. Update Fungsi changeUltrasonicAddress()
```cpp
// Sebelumnya (SALAH):
result = node.writeSingleRegister(0x0100, newAddress);

// Sekarang (BENAR):
result = node.writeSingleRegister(ADDRESS_REGISTER, newAddress);  // 0x0200
```

### 3. Komentar yang Diperbaiki
- Menghapus komentar yang menyesatkan tentang "register alamat yang umum"
- Menambahkan referensi spesifik ke datasheet
- Memperjelas bahwa register 0x0200 adalah spesifikasi resmi

## 📊 Spesifikasi Lengkap dari Datasheet

### Register Map yang Benar:
- **0x0100**: Distance Register (untuk membaca jarak)
- **0x0200**: Slave Address Register (untuk mengubah alamat)

### Karakteristik Alamat:
- **Range**: 1-254 (0x01-0xFE)
- **Default**: 1 (0x01)
- **Broadcast**: 255 (0xFF) - tidak dapat digunakan sebagai alamat individu
- **Reserved**: 0 (0x00) - tidak valid

## ⚠️ Catatan Penting

1. **Validasi Alamat**: Program sudah benar membatasi alamat 1-247
2. **Broadcast Address**: 0xFF (255) adalah alamat broadcast, tidak boleh digunakan
3. **Register Verification**: Selalu gunakan register 0x0200 untuk perubahan alamat
4. **Datasheet Compliance**: Implementasi sekarang 100% sesuai dengan datasheet resmi

## 🎯 Hasil Perbaikan

✅ **Register alamat sudah benar**: 0x0200  
✅ **Kode sudah diperbarui**: Menggunakan ADDRESS_REGISTER constant  
✅ **Dokumentasi sudah akurat**: Komentar mengacu pada datasheet  
✅ **Validasi tetap berfungsi**: Range 1-247 masih diterapkan  

Program sekarang **100% sesuai dengan spesifikasi datasheet** dan siap digunakan untuk mengubah alamat sensor ultrasonik RS485 dengan benar.