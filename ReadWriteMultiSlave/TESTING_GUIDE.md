# Panduan Testing - Sistem Kontrol Valve ESP32-S3

## Overview
Dokumen ini berisi prosedur testing untuk memverifikasi semua fungsi sistem kontrol valve sesuai dengan persyaratan yang telah ditetapkan.

## Persyaratan Hardware
- ESP32-S3 Development Board
- HMI dengan Modbus RTU (Slave ID: 2)
- Waveshare Relay Module 10-Channel (Slave ID: 6)
- Kabel RS485 (A+, B-, GND)
- Power supply untuk semua komponen

## Konfigurasi Sistem
```
Valve 1: Relay 1 (buka), Relay 2 (tutup) - Lampu LB10
Valve 2: Relay 3 (buka), Relay 4 (tutup) - Lampu LB11
Valve 3: Relay 5 (buka), Relay 6 (tutup) - Lampu LB12
Valve 4: Relay 7 (buka), Relay 8 (tutup) - Lampu LB13
Valve 5: Relay 9 (buka), Relay 10 (tutup) - Lampu LB14

Tombol Kontrol: LB0-LB4 untuk Valve 1-5
```

## Test Cases

### 1. Test Inisialisasi Sistem
**Tujuan:** Memverifikasi sistem startup dengan benar

**Prosedur:**
1. Upload kode ke ESP32-S3
2. Buka Serial Monitor (115200 baud)
3. Reset ESP32-S3

**Expected Result:**
```
=== ESP32-S3 Valve Control System ===
5 Valve Control dengan 10 Relay System
=====================================
✓ Sistem valve diinisialisasi
✓ Modbus RTU diinisialisasi
Testing komunikasi relay...
✓ Komunikasi relay berhasil
✓ HMI direset ke kondisi awal
✓ Relay monitors diinisialisasi
Sistem siap beroperasi!
```

**Status:** [ ] PASS [ ] FAIL

---

### 2. Test Komunikasi HMI
**Tujuan:** Memverifikasi komunikasi dengan HMI berfungsi

**Prosedur:**
1. Pastikan HMI terhubung dengan benar
2. Tekan tombol LB0 pada HMI
3. Amati output serial monitor

**Expected Result:**
- Tidak ada error komunikasi HMI
- Tombol press terdeteksi: "🔘 Tombol Valve 1 ditekan"

**Status:** [ ] PASS [ ] FAIL

---

### 3. Test Komunikasi Relay
**Tujuan:** Memverifikasi komunikasi dengan relay module

**Prosedur:**
1. Pastikan relay module terhubung dengan benar
2. Tekan tombol LB0 pada HMI
3. Amati relay fisik dan output serial

**Expected Result:**
- Relay 1 menyala (LED indikator ON)
- Serial output: "🔌 Relay 1: ON"
- Tidak ada error komunikasi relay

**Status:** [ ] PASS [ ] FAIL

---

### 4. Test Mapping Relay yang Benar
**Tujuan:** Memverifikasi mapping relay sesuai spesifikasi

**Prosedur:**
Untuk setiap valve (1-5):
1. Tekan tombol valve pertama kali (operasi buka)
2. Verifikasi relay ganjil yang menyala
3. Tunggu operasi selesai
4. Tekan tombol valve kedua kali (operasi tutup)
5. Verifikasi relay genap yang menyala

**Expected Result:**
```
Valve 1: Relay 1 (buka) → Relay 2 (tutup)
Valve 2: Relay 3 (buka) → Relay 4 (tutup)
Valve 3: Relay 5 (buka) → Relay 6 (tutup)
Valve 4: Relay 7 (buka) → Relay 8 (tutup)
Valve 5: Relay 9 (buka) → Relay 10 (tutup)
```

**Status:** [ ] PASS [ ] FAIL

---

### 5. Test Delay Time 35 Detik
**Tujuan:** Memverifikasi delay time tepat 35 detik

**Prosedur:**
1. Tekan tombol LB0 pada HMI
2. Catat waktu mulai operasi
3. Amati progress report setiap 10 detik
4. Catat waktu selesai operasi

**Expected Result:**
- Progress report: "⏳ Valve 1 membuka - sisa XX detik"
- Total waktu operasi: 35 detik (±1 detik toleransi)
- Serial output: "✅ Valve 1 operasi selesai - Status: TERBUKA"

**Status:** [ ] PASS [ ] FAIL

---

### 6. Test Sinkronisasi Lampu Indikator
**Tujuan:** Memverifikasi lampu menyala setelah operasi selesai

**Prosedur:**
1. Tekan tombol LB0 (buka valve 1)
2. Amati lampu LB10 selama operasi
3. Tunggu hingga operasi selesai (35 detik)
4. Verifikasi lampu LB10 menyala
5. Tekan tombol LB0 lagi (tutup valve 1)
6. Tunggu hingga operasi selesai
7. Verifikasi lampu LB10 mati

**Expected Result:**
- Lampu LB10 TIDAK menyala selama operasi
- Lampu LB10 menyala SETELAH operasi buka selesai
- Lampu LB10 mati SETELAH operasi tutup selesai
- Serial output: "💡 Lampu LB10 (Valve 1): ON/OFF"

**Status:** [ ] PASS [ ] FAIL

---

### 7. Test Operasi Bersamaan (Stress Test)
**Tujuan:** Memverifikasi sistem dapat menangani multiple valve bersamaan

**Prosedur:**
1. Tekan tombol LB0, LB1, LB2 secara berurutan dengan cepat
2. Amati 3 relay yang berbeda menyala bersamaan
3. Tunggu hingga semua operasi selesai
4. Verifikasi 3 lampu indikator menyala

**Expected Result:**
- 3 relay berbeda (1, 3, 5) menyala bersamaan
- Tidak ada error atau konflik
- Semua operasi selesai dalam 35 detik
- 3 lampu indikator (LB10, LB11, LB12) menyala

**Status:** [ ] PASS [ ] FAIL

---

### 8. Test Immediate Stop
**Tujuan:** Memverifikasi operasi dapat dihentikan dengan menekan tombol lagi

**Prosedur:**
1. Tekan tombol LB0 (mulai operasi valve 1)
2. Tunggu 10 detik
3. Tekan tombol LB0 lagi (stop operasi)
4. Verifikasi relay mati dan operasi berhenti

**Expected Result:**
- Serial output: "🛑 Menghentikan operasi Valve 1"
- Relay 1 mati
- Operasi berhenti sebelum 35 detik
- Status valve tidak berubah

**Status:** [ ] PASS [ ] FAIL

---

### 9. Test Error Handling - Komunikasi Gagal
**Tujuan:** Memverifikasi sistem menangani error komunikasi

**Prosedur:**
1. Cabut kabel RS485 dari relay module
2. Tekan tombol LB0 pada HMI
3. Amati response sistem

**Expected Result:**
- Serial output error: "🚨 KOMUNIKASI ERROR - Relay: 0xE2"
- Solusi troubleshooting ditampilkan
- Sistem tidak crash
- Retry mechanism bekerja

**Status:** [ ] PASS [ ] FAIL

---

### 10. Test Emergency Stop
**Tujuan:** Memverifikasi emergency stop berfungsi

**Prosedur:**
1. Mulai operasi beberapa valve
2. Simulasikan kondisi emergency (cabut kabel relay)
3. Amati response sistem

**Expected Result:**
- Serial output: "🚨 EMERGENCY STOP - Menghentikan semua operasi"
- Semua relay dimatikan
- Semua lampu HMI dimatikan
- Sistem masuk mode emergency stop

**Status:** [ ] PASS [ ] FAIL

---

### 11. Test Relay Stuck Detection
**Tujuan:** Memverifikasi deteksi relay macet

**Prosedur:**
1. Simulasikan relay macet (disconnect relay tertentu)
2. Coba operasi valve yang menggunakan relay tersebut
3. Amati response sistem

**Expected Result:**
- Serial output: "🚨 RELAY X MACET - Ditandai sebagai tidak dapat digunakan"
- Relay ditandai sebagai stuck
- Auto-recovery attempt setelah 1 menit
- Emergency stop jika relay critical

**Status:** [ ] PASS [ ] FAIL

---

### 12. Test System Health Check
**Tujuan:** Memverifikasi monitoring kesehatan sistem

**Prosedur:**
1. Jalankan sistem dalam kondisi normal
2. Simulasikan multiple relay failure
3. Amati health check response

**Expected Result:**
- Sistem berjalan normal saat healthy
- Warning saat >50% relay macet
- Emergency stop saat sistem tidak sehat

**Status:** [ ] PASS [ ] FAIL

---

## Hasil Testing

### Summary
- Total Test Cases: 12
- Passed: ___
- Failed: ___
- Success Rate: ___%

### Issues Found
1. ________________________________
2. ________________________________
3. ________________________________

### Recommendations
1. ________________________________
2. ________________________________
3. ________________________________

---

**Tester:** ________________  
**Date:** ________________  
**Firmware Version:** ________________  
**Hardware Revision:** ________________