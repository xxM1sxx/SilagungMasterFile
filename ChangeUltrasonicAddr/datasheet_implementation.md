# Implementasi Sesuai Datasheet

## Format Command dari Datasheet

**Modify the slave address:**
- Command: `01 06 02 00 00 05 48 71`
- Return: `01 06 02 00 00 05 48 71`
- Description: Change the address 0x01 to 0x05.

## Analisis Format

### Breakdown Command:
- `01` - Slave Address (alamat sensor saat ini)
- `06` - Function Code (Write Single Register)
- `02 00` - Register Address (0x0200 dalam big-endian)
- `00 05` - Data Value (alamat baru 0x05 dalam big-endian)
- `48 71` - CRC16 (dihitung otomatis oleh library)

### Implementasi dalam Code:

```cpp
// Register yang benar sesuai datasheet
#define ADDRESS_REGISTER 0x0200

// Function untuk mengubah alamat
result = node.writeSingleRegister(ADDRESS_REGISTER, newAddress);
```

## Perbedaan dengan Implementasi Sebelumnya

### Sebelumnya (SALAH):
- Menggunakan register `0x0000` untuk deteksi
- Mencoba berbagai metode alternatif
- Format tidak sesuai datasheet

### Sekarang (BENAR):
- Menggunakan register `0x0200` sesuai datasheet
- Function Code 06 (Write Single Register)
- Format persis seperti contoh datasheet
- Verifikasi menggunakan register yang valid (`0x0100`)

## Expected Behavior

1. **Command yang dikirim**: `[current_addr] 06 02 00 00 [new_addr] [CRC]`
2. **Response yang diharapkan**: Echo dari command yang sama
3. **Verifikasi**: Baca register `0x0100` di alamat baru

## Troubleshooting

Jika masih error 0xE2:
1. Pastikan sensor mendukung register `0x0200`
2. Cek apakah sensor perlu restart setelah perubahan alamat
3. Verifikasi parameter komunikasi (baud rate, parity, stop bits)
4. Pastikan tidak ada konflik alamat di bus RS485