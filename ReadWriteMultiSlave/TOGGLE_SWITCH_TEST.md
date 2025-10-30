# Test Toggle Switch Behavior

## Perubahan yang Dilakukan

### Sebelum (Logika Lama):
- **Edge Detection**: Hanya mendeteksi perubahan dari OFF ke ON
- **Toggle Logic**: Tekan sekali = buka, tekan lagi = tutup
- **Masalah**: Toggle switch ON/OFF tidak sesuai dengan status valve

### Sesudah (Logika Baru):
- **State Following**: Mengikuti status toggle switch secara langsung
- **Direct Mapping**: ON = buka valve, OFF = tutup valve
- **Real-time**: Perubahan toggle switch langsung memicu operasi

## Test Scenarios

### Test 1: Toggle Switch ON → Valve Buka
**Prosedur:**
1. Pastikan valve dalam keadaan tertutup
2. Set toggle switch LB0 ke posisi ON
3. Amati response sistem

**Expected Result:**
```
🔄 Toggle Switch Valve 1: ON
🔄 Memulai operasi Valve 1: BUKA (Relay 1)
🔌 Relay 1: ON
✅ Valve 1 operasi dimulai - akan selesai dalam 35 detik
⏳ Valve 1 membuka - sisa XX detik
✅ Valve 1 operasi selesai - Status: TERBUKA
💡 Lampu LB10 (Valve 1): ON
```

### Test 2: Toggle Switch OFF → Valve Tutup
**Prosedur:**
1. Pastikan valve dalam keadaan terbuka (dari test 1)
2. Set toggle switch LB0 ke posisi OFF
3. Amati response sistem

**Expected Result:**
```
🔄 Toggle Switch Valve 1: OFF
🔄 Memulai operasi Valve 1: TUTUP (Relay 2)
🔌 Relay 2: ON
✅ Valve 1 operasi dimulai - akan selesai dalam 35 detik
⏳ Valve 1 menutup - sisa XX detik
✅ Valve 1 operasi selesai - Status: TERTUTUP
💡 Lampu LB10 (Valve 1): OFF
```

### Test 3: Toggle Switch Tidak Berubah Status
**Prosedur:**
1. Valve sudah terbuka, toggle switch sudah ON
2. Set toggle switch LB0 ke posisi ON lagi (tidak berubah)
3. Amati response sistem

**Expected Result:**
```
ℹ️ Valve 1 sudah dalam posisi TERBUKA - tidak perlu operasi
```

### Test 4: Interrupt Operasi dengan Toggle Switch
**Prosedur:**
1. Mulai operasi buka valve (toggle ON)
2. Setelah 10 detik, ubah toggle ke OFF
3. Amati response sistem

**Expected Result:**
```
🔄 Toggle Switch Valve 1: ON
🔄 Memulai operasi Valve 1: BUKA (Relay 1)
... (setelah 10 detik)
🔄 Toggle Switch Valve 1: OFF
🛑 Menghentikan operasi Valve 1 yang sedang berjalan
🔌 Relay 1: OFF
🔄 Memulai operasi Valve 1: TUTUP (Relay 2)
```

### Test 5: Multiple Valve Bersamaan
**Prosedur:**
1. Set toggle switch LB0, LB1, LB2 ke ON bersamaan
2. Amati 3 valve beroperasi bersamaan
3. Set toggle switch LB0 ke OFF saat operasi berlangsung

**Expected Result:**
- 3 relay berbeda (1, 3, 5) menyala bersamaan
- Valve 1 berhenti dan mulai tutup saat toggle diubah ke OFF
- Valve 2 dan 3 tetap melanjutkan operasi buka

## Debug Information

### Status Display yang Ditambahkan:
```
=== STATUS SISTEM ===
HMI: OK | Relay: OK
Toggle Switch Status:
  SW1: ON   SW2: OFF  SW3: OFF  SW4: OFF  SW5: OFF
Valve Status:
  Valve 1: TERBUKA | Lampu: ON
  Valve 2: TERTUTUP | Lampu: OFF
  Valve 3: TERTUTUP | Lampu: OFF
  Valve 4: TERTUTUP | Lampu: OFF
  Valve 5: TERTUTUP | Lampu: OFF
====================
```

## Troubleshooting

### Jika Toggle Switch Tidak Merespon:
1. Periksa komunikasi HMI: `HMI: OK/ERROR`
2. Periksa address toggle switch: LB0-LB4
3. Periksa konfigurasi Modbus HMI

### Jika Valve Tidak Mengikuti Toggle:
1. Periksa output serial untuk pesan error
2. Verifikasi mapping relay: Valve 1 = Relay 1&2
3. Periksa komunikasi relay module

### Jika Lampu Tidak Sinkron:
1. Lampu akan menyala setelah operasi selesai (35 detik)
2. Periksa address lampu: LB10-LB14
3. Verifikasi komunikasi HMI untuk write operation

## Kode yang Diubah

### File: main.cpp
**Fungsi yang dimodifikasi:**
1. `processButtonChanges()` - Logika toggle switch
2. `readHMIButtons()` - Error handling yang lebih baik
3. `printSystemStatus()` - Debug information yang lebih lengkap

**Perubahan utama:**
- Menghilangkan edge detection (hanya ON)
- Menambahkan state following (ON/OFF)
- Menambahkan check untuk mencegah operasi redundant
- Menambahkan debug output yang lebih informatif