# Test Guide - Logika Relay Kontinyu Baru

## Persiapan Testing

### Hardware Setup
1. ESP32-S3 terhubung ke HMI (Modbus ID: 1, Baud: 9600)
2. Relay Module terhubung (Modbus ID: 2, Baud: 9600)
3. Serial Monitor aktif (115200 baud)

### Software Setup
1. Upload kode `main.cpp` yang telah dimodifikasi
2. Buka Serial Monitor
3. Tunggu pesan "Sistem siap beroperasi!"

## Test Scenarios

### Test 1: Toggle Switch ON - Relay Ganjil Kontinyu

**Langkah:**
1. Set toggle switch Valve 1 ke posisi ON di HMI
2. Amati output Serial Monitor
3. Periksa status relay

**Expected Output:**
```
🔄 Toggle Switch Valve 1: ON
🟢 Mengaktifkan relay ganjil Valve 1 (kontinyu)
⏱️ Memulai delay lampu 30 detik untuk Valve 1 (target: ON)
```

**Expected Behavior:**
- Relay 1 (ganjil) menyala kontinyu
- Relay 2 (genap) mati
- Timer delay lampu dimulai
- Progress countdown setiap 10 detik

### Test 2: Toggle Switch OFF - Relay Genap Kontinyu

**Langkah:**
1. Set toggle switch Valve 1 ke posisi OFF di HMI
2. Amati output Serial Monitor
3. Periksa status relay

**Expected Output:**
```
🔄 Toggle Switch Valve 1: OFF
🔴 Mengaktifkan relay genap Valve 1 (penutup kontinyu)
⏱️ Memulai delay lampu 30 detik untuk Valve 1 (target: OFF)
```

**Expected Behavior:**
- Relay 2 (genap) menyala kontinyu
- Relay 1 (ganjil) mati
- Timer delay lampu dimulai

### Test 3: Delay Lampu 30 Detik

**Langkah:**
1. Set toggle switch ke ON
2. Tunggu dan amati countdown
3. Verifikasi lampu menyala setelah 30 detik

**Expected Progress Output:**
```
⏳ Delay lampu Valve 1 - sisa 20 detik (target: ON)
⏳ Delay lampu Valve 1 - sisa 10 detik (target: ON)
💡 Delay lampu selesai - Valve 1 lampu: ON
💡 Lampu Valve 1 berhasil diupdate: ON
```

### Test 4: Multiple Valve Operation

**Langkah:**
1. Set toggle switch Valve 1 ke ON
2. Set toggle switch Valve 2 ke ON
3. Set toggle switch Valve 3 ke OFF
4. Amati semua operasi berjalan bersamaan

**Expected Behavior:**
- Valve 1: Relay 1 ON, delay lampu aktif
- Valve 2: Relay 3 ON, delay lampu aktif  
- Valve 3: Relay 6 ON, delay lampu aktif
- Semua timer berjalan independen

### Test 5: System Status Debug

**Langkah:**
1. Tunggu output status sistem otomatis
2. Verifikasi informasi yang ditampilkan

**Expected Status Output:**
```
=== STATUS SISTEM ===
HMI: OK | Relay: OK
Toggle Switch Status:
  SW1: ON  SW2: ON  SW3: OFF SW4: OFF SW5: OFF
Valve & Relay Status:
  Valve 1: Toggle=ON | Relay Ganjil=ON | Relay Genap=OFF | Delay Lampu: 15s (target: ON) | Lampu: OFF
  Valve 2: Toggle=ON | Relay Ganjil=ON | Relay Genap=OFF | Delay Lampu: 12s (target: ON) | Lampu: OFF
  Valve 3: Toggle=OFF | Relay Ganjil=OFF | Relay Genap=ON | Delay Lampu: 8s (target: OFF) | Lampu: ON
====================
```

## Verification Points

### ✅ Relay Behavior
- [ ] Relay ganjil menyala kontinyu saat toggle ON
- [ ] Relay genap menyala kontinyu saat toggle OFF
- [ ] Hanya satu relay aktif per valve pada satu waktu
- [ ] Relay tetap menyala sampai toggle switch diubah

### ✅ Timer Accuracy
- [ ] Delay lampu tepat 30 detik
- [ ] Progress countdown setiap 10 detik
- [ ] Multiple timer berjalan independen
- [ ] Timer tidak terpengaruh operasi valve lain

### ✅ Lamp Control
- [ ] Lampu menyala setelah 30 detik dari toggle ON
- [ ] Lampu mati setelah 30 detik dari toggle OFF
- [ ] Status lampu sesuai dengan target
- [ ] Update lampu berhasil di HMI

### ✅ Error Handling
- [ ] Emergency stop mematikan semua relay
- [ ] Stuck relay detection tetap aktif
- [ ] Communication error handling
- [ ] System health check

## Troubleshooting

### Relay Tidak Menyala
1. Periksa koneksi Modbus relay
2. Verifikasi power supply relay module
3. Cek stuck relay status
4. Restart sistem jika perlu

### Timer Tidak Akurat
1. Periksa fungsi `millis()` 
2. Verifikasi tidak ada blocking delay
3. Cek overflow handling

### Lampu Tidak Update
1. Periksa koneksi HMI Modbus
2. Verifikasi alamat lampu (LB10-LB14)
3. Cek komunikasi HMI

### Multiple Valve Conflict
1. Verifikasi setiap valve independen
2. Cek tidak ada shared resource
3. Periksa array indexing

## Performance Metrics

- **Response Time**: Toggle switch → Relay activation < 100ms
- **Timer Accuracy**: ±1 second untuk delay 30 detik
- **Communication**: HMI read interval 100ms, Lamp update 500ms
- **Memory Usage**: Struktur data minimal, no memory leak