# Panduan Logika Relay Baru - Sistem Kontrol Valve

## Perubahan Utama

Sistem telah diubah dari logika operasi sementara (35 detik) menjadi logika relay kontinyu dengan delay lampu 30 detik.

## Logika Relay Baru

### 1. Relay Ganjil (1, 3, 5, 7, 9)
- **Fungsi**: Relay pembuka valve
- **Behavior**: Menyala kontinyu selama toggle switch dalam posisi ON
- **Mapping**:
  - Valve 1 → Relay 1
  - Valve 2 → Relay 3  
  - Valve 3 → Relay 5
  - Valve 4 → Relay 7
  - Valve 5 → Relay 9

### 2. Relay Genap (2, 4, 6, 8, 10)
- **Fungsi**: Relay penutup valve (kontinyu)
- **Behavior**: Menyala kontinyu selama toggle switch dalam posisi OFF
- **Mapping**:
  - Valve 1 → Relay 2
  - Valve 2 → Relay 4
  - Valve 3 → Relay 6
  - Valve 4 → Relay 8
  - Valve 5 → Relay 10

## Logika Lampu Indikator

### Delay 30 Detik
- **ON Condition**: Lampu menyala 30 detik SETELAH toggle switch diset ke ON
- **OFF Condition**: Lampu mati 30 detik SETELAH toggle switch diset ke OFF
- **Progress**: Sistem menampilkan countdown setiap 10 detik

### Mapping Lampu
- Valve 1 → LB10
- Valve 2 → LB11
- Valve 3 → LB12
- Valve 4 → LB13
- Valve 5 → LB14

## Skenario Operasi

### Toggle Switch ON
1. Toggle switch diset ke ON
2. Relay ganjil (pembuka) menyala kontinyu
3. Relay genap (penutup) dimatikan
4. Timer delay lampu dimulai (30 detik)
5. Setelah 30 detik: Lampu indikator menyala

### Toggle Switch OFF
1. Toggle switch diset ke OFF
2. Relay genap (penutup) menyala kontinyu
3. Relay ganjil (pembuka) dimatikan
4. Timer delay lampu dimulai (30 detik)
5. Setelah 30 detik: Lampu indikator mati

## Struktur Data Baru

```cpp
struct ValveState {
  bool toggleSwitchState;      // Status toggle switch: true=ON, false=OFF
  bool relayOddState;          // Status relay ganjil (kontinyu)
  bool relayEvenState;         // Status relay genap (kontinyu sebagai penutup)
  unsigned long lampDelayStart; // Waktu mulai delay lampu
  bool lampDelayActive;        // Apakah delay lampu sedang aktif
  bool targetLampState;        // Target status lampu setelah delay
};
```

## Fungsi Utama yang Diubah

### 1. `processButtonChanges()`
- Mendeteksi perubahan toggle switch
- Mengaktifkan relay sesuai logika ganjil/genap
- Memulai timer delay lampu

### 2. `updateLampDelays()`
- Menggantikan `updateValveOperations()`
- Menangani countdown delay lampu 30 detik
- Update status lampu setelah delay selesai

### 3. `startLampDelay()`
- Menggantikan `startValveOperation()`
- Memulai timer delay untuk lampu
- Menyimpan target status lampu

## Keamanan dan Error Handling

### Emergency Stop
- Mematikan semua relay
- Mereset semua status valve
- Menghentikan semua timer delay lampu

### Stuck Relay Detection
- Tetap aktif untuk semua relay
- Mencegah operasi pada relay yang macet

### Communication Error
- Retry mechanism untuk HMI dan Relay
- Error reporting setiap 5 detik

## Debug Information

Serial monitor menampilkan:
- Status toggle switch (ON/OFF)
- Status relay ganjil dan genap
- Progress delay lampu dengan countdown
- Status lampu indikator
- Error komunikasi dan troubleshooting

## Testing Checklist

- [ ] Toggle switch ON → Relay ganjil menyala kontinyu
- [ ] Toggle switch OFF → Relay genap menyala kontinyu  
- [ ] Delay lampu 30 detik untuk kondisi ON
- [ ] Delay lampu 30 detik untuk kondisi OFF
- [ ] Progress countdown setiap 10 detik
- [ ] Emergency stop mematikan semua relay
- [ ] Error handling untuk komunikasi
- [ ] Multiple valve operation bersamaan