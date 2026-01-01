# FreeRTOSAPSV2 — Mode AUTO: Penyesuaian EC

Dokumen ini menjelaskan cara kerja penyesuaian EC saat irigasi **mode AUTO** berjalan, supaya gampang diingat saat tuning di lapangan.

## Ringkasnya

- Penyesuaian EC hanya aktif jika `irrigationType` adalah `air_nutrisi` (atau `air+nutrisi`) dan `targetEC > 0`.
- Saat EC di bawah target, sistem akan membuka valve nutrisi; saat EC di atas target, sistem akan menutup valve nutrisi.
- Sistem memakai hysteresis (band) supaya tidak bolak-balik switch relay di sekitar target.
- Karena valve nutrisi dianggap motorized (open/close), relay OPEN/CLOSE nutrisi hanya dipulse sebentar lalu dimatikan lagi.

Implementasi utama ada di [main.cpp](file:///d:/Kuliyeah/Skripshit/Code/FreeRTOSAPSV2/src/main.cpp#L1538-L1590).

## Kapan kontrol EC berjalan

Kontrol EC dieksekusi hanya ketika kondisi ini terpenuhi:

- Irigasi sedang berjalan (`currentIrrigation.isActive`)
- Aktivasi sudah siap (`currentIrrigation.activationReady == true`)
- Tipe irigasi adalah `air_nutrisi` atau `air+nutrisi`
- `targetEC > 0.1`
- Konfigurasi valve/pompa dianggap siap (`configReady == true`)

Kalau salah satu tidak terpenuhi, kontrol EC tidak akan melakukan apa-apa.

## Aturan keputusan: target vs EC aktual

Parameter yang dipakai:

- `EC_CONTROL_HYSTERESIS_US` (default 50)
- `EC_CONTROL_INTERVAL_MS` (default 1500)
- `EC_CONTROL_MIN_SWITCH_MS` (default 3000)

Sistem menghitung band:

- `low = targetEC - EC_CONTROL_HYSTERESIS_US`
- `high = targetEC + EC_CONTROL_HYSTERESIS_US`

Keputusannya:

- Jika `ec < low` → nutrisi perlu **dibuka** (wantNutr = true)
- Jika `ec > high` → nutrisi perlu **ditutup** (wantNutr = false)
- Jika `low <= ec <= high` → nutrisi **dipertahankan** (tidak ganti posisi)

Contoh target 800 µS/cm dan hysteresis 50:

- `low = 750`, `high = 850`
- EC 740 → OPEN nutrisi
- EC 870 → CLOSE nutrisi
- EC 800 → tidak berubah posisi (relay nutrisi dipastikan OFF)

## Kenapa relay tidak nyala terus

Valve nutrisi diperlakukan sebagai **motorized valve** (2 relay: OPEN dan CLOSE). Jadi saat perintah OPEN/CLOSE:

1. Nyalakan relay yang sesuai (OPEN atau CLOSE)
2. Tunggu `NUTRI_MOTOR_DRIVE_MS` agar valve sempat bergerak
3. Matikan relay OPEN dan CLOSE (dua-duanya OFF)

Ini mencegah motor/relay kepake terus saat posisi sudah tercapai.

Implementasi:

- `driveNutrisiMotorTo(open)` dan `setNutrisiRelaysOff()` di [main.cpp](file:///d:/Kuliyeah/Skripshit/Code/FreeRTOSAPSV2/src/main.cpp#L638-L677)

## Kenapa kadang “close bentar lalu mati” padahal EC masih tinggi

Itu normal: perintah CLOSE/OPEN memang hanya **pulse**.

Namun kalau valve belum benar-benar sampai posisi (misal drive time kurang, atau mekanik berat), maka perlu follow-up.

Karena itu ada mekanisme redrive:

- Jika EC tetap di luar band (`ec < low` atau `ec > high`) tetapi posisi nutrisi sudah sama dengan yang diinginkan, sistem akan mengulang pulse OPEN/CLOSE setiap:
  - `NUTRI_MOTOR_REDRIVE_INTERVAL_MS` (default 15000 ms)
  - maksimal `NUTRI_MOTOR_REDRIVE_MAX` kali (default 3) per satu sesi irigasi
- Saat EC sudah masuk band, counter redrive di-reset.

Tujuannya: membantu kasus valve belum “nyampe posisi” dalam satu pulse.

## Batasan switching supaya tidak spam

Walaupun EC sering berubah-ubah, perubahan posisi nutrisi dibatasi:

- Kontrol EC diperiksa tiap `EC_CONTROL_INTERVAL_MS`
- Perubahan posisi nutrisi (switch OPEN↔CLOSE) minimal berjarak `EC_CONTROL_MIN_SWITCH_MS`

Ini penting untuk menjaga bus RS485/Modbus tetap stabil dan mencegah relay chattering.

## Parameter yang sering dituning

Semua ada di [main.cpp](file:///d:/Kuliyeah/Skripshit/Code/FreeRTOSAPSV2/src/main.cpp#L190-L205).

- `EC_CONTROL_HYSTERESIS_US`
  - Naikkan kalau relay terlalu sering switching di sekitar target.
  - Turunkan kalau mau lebih presisi tapi risiko chattering naik.
- `EC_CONTROL_INTERVAL_MS`
  - Turunkan untuk respon lebih cepat (beban polling/modbus naik).
  - Naikkan untuk stabilitas bus lebih tinggi (respon lebih lambat).
- `EC_CONTROL_MIN_SWITCH_MS`
  - Naikkan untuk membatasi switching.
  - Turunkan kalau sistem terlalu lambat koreksi.
- `NUTRI_MOTOR_DRIVE_MS`
  - Naikkan kalau valve belum sempat full open/close.
  - Turunkan kalau valve cepat dan ingin mengurangi “lama nyala” coil.
- `NUTRI_MOTOR_REDRIVE_INTERVAL_MS` dan `NUTRI_MOTOR_REDRIVE_MAX`
  - Naikkan interval / turunkan max kalau tidak ingin banyak redrive.
  - Turunkan interval / naikkan max kalau valve sering tidak mencapai posisi dalam 1 pulse.

## Catatan penting (jenis valve)

Logika ini diasumsikan untuk nutrisi yang memakai **motorized valve** (tahan posisi tanpa daya).

Kalau nutrisi ternyata valve tipe solenoid yang harus diberi listrik agar tetap open, maka logika “pulse lalu OFF” tidak cocok dan harus dibedakan.

