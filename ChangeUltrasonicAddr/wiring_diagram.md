# Diagram Koneksi Sensor Ultrasonik RS485

## Skema Koneksi Lengkap

```
ESP32-S3 DevKit          RS485 Transceiver          Sensor Ultrasonik RS485
┌─────────────────┐      ┌─────────────────┐        ┌─────────────────────┐
│                 │      │                 │        │                     │
│ 3.3V ───────────┼──────┤ VCC             │        │                     │
│ GND ────────────┼──────┤ GND             │        │                     │
│                 │      │                 │        │                     │
│ GPIO 17 (TX) ───┼──────┤ DI (Data Input) │        │                     │
│ GPIO 16 (RX) ───┼──────┤ RO (Recv Output)│        │                     │
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

## Detail Koneksi

### 1. ESP32-S3 ke RS485 Transceiver

| ESP32-S3 Pin | RS485 Module Pin | Fungsi |
|--------------|------------------|---------|
| 3.3V         | VCC              | Power supply |
| GND          | GND              | Ground |
| GPIO 17      | DI               | Data Input (TX) |
| GPIO 16      | RO               | Receiver Output (RX) |
| GPIO 4       | DE & RE          | Direction Control |

### 2. RS485 Transceiver ke Sensor

| RS485 Module | Sensor Pin | Fungsi |
|--------------|------------|---------|
| A+           | A+ / RS485+ | Differential Signal + |
| B-           | B- / RS485- | Differential Signal - |

### 3. Power untuk Sensor

| Power Supply | Sensor Pin | Spesifikasi |
|--------------|------------|-------------|
| +12V atau +24V | VCC/Power | Sesuai spesifikasi sensor |
| GND          | GND       | Ground |

## Catatan Penting

### RS485 Transceiver
- Gunakan module seperti MAX485, SP485, atau SN75176
- DE (Driver Enable) dan RE (Receiver Enable) dihubung ke pin yang sama (GPIO 4)
- Pastikan level tegangan sesuai (3.3V untuk ESP32)

### Kabel RS485
- Gunakan twisted pair cable untuk mengurangi noise
- Panjang maksimal: ~1200 meter (tergantung baud rate)
- Untuk jarak pendek (<10m), kabel biasa juga bisa digunakan

### Terminasi
- Untuk jaringan panjang atau banyak device, tambahkan resistor terminasi 120Ω
- Pasang di ujung-ujung jaringan RS485

### Grounding
- Pastikan semua device memiliki ground yang sama
- Hindari ground loop

## Contoh Module RS485 yang Kompatibel

1. **MAX485 Module**
   - Tegangan: 3.3V - 5V
   - Murah dan mudah didapat
   - Pin: VCC, GND, DI, RO, DE, RE, A, B

2. **TTL to RS485 Module**
   - Auto direction control (tidak perlu DE/RE)
   - Lebih mudah digunakan untuk pemula

3. **Industrial RS485 Module**
   - Isolasi galvanik
   - Proteksi surge
   - Untuk aplikasi industri

## Troubleshooting Koneksi

### Tidak Ada Komunikasi:
1. Periksa koneksi A+ dan B- (coba tukar jika perlu)
2. Verifikasi power sensor
3. Cek baud rate dan parameter komunikasi
4. Pastikan DE/RE pin terhubung dengan benar

### Komunikasi Tidak Stabil:
1. Tambahkan kapasitor decoupling di power supply
2. Gunakan kabel twisted pair
3. Periksa grounding
4. Kurangi baud rate

### Error Timeout:
1. Periksa alamat sensor
2. Coba scan alamat 1-247
3. Verifikasi parameter Modbus (8N1, baud rate)
4. Periksa timing DE/RE switching

## Konfigurasi Multiple Sensor

Untuk menghubungkan beberapa sensor dalam satu jaringan RS485:

```
ESP32 ── RS485 Transceiver ── Sensor 1 (Alamat 1)
                           ├── Sensor 2 (Alamat 2)  
                           ├── Sensor 3 (Alamat 3)
                           └── Sensor N (Alamat N)
```

- Setiap sensor harus memiliki alamat unik (1-247)
- Hubungkan A+ ke A+ dan B- ke B- secara paralel
- Maksimal 32 device per jaringan (tergantung driver)
- Gunakan terminasi di ujung jaringan