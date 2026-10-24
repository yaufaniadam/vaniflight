# VaniFlight — Autonomous Flying Wing Flight Controller

> **Nama Proyek:** `VaniFlight` *(Yaufani Flight Controller — kebalikan dari INAV)*
> **Dokumen:** `README.md`
> **Versi Firmware & Dokumen:** `v0.4.1`
> **Sejak Versi:** `v0.1.0 (2026-10-18)`
> **Terakhir Diperbarui:** `2026-10-24`
> **Riwayat Lengkap Perubahan:** Lihat [`CHANGELOG.md`](CHANGELOG.md)
> **Peta Jalan Menuju 3-Plane Swarm:** Lihat [`ROADMAP.md`](ROADMAP.md)

Proyek Flight Controller mandiri (*Zero External Library Dependencies* untuk kendali inti) untuk pesawat **Flying Wing / Delta Wing** yang telah teruji terbang (*Flight Proven*) dan dirancang sebagai fondasi menuju **Autonomous Swarm & VTOL**.

Satu kode sumber yang sama otomatis mendukung **4 keluarga mikrokontroler** saat kompilasi:
1. **Waveshare RP2040-Zero / Raspberry Pi Pico (RP2040 / RP2350)** *(Board Utama Plane 1 & Plane 2)*
2. **Omnibus F4 V5 (STM32F405) & STM32F411CEU6 "BlackPill"**
3. **ESP32 / ESP32-S3**
4. **Teensy 4.0 / 4.1**

---

## 1. Fitur Utama & Riwayat Kemampuan (`v0.4.1`)

| Fitur | Sejak Versi | File Terkait | Deskripsi Teknis |
| :--- | :---: | :--- | :--- |
| **Madgwick 6-DOF Quaternion AHRS** | `v0.1.0` | [`imu.h`](flying_wing_fc/imu.h) | Estimasi sikap 500 Hz bebas *gimbal lock*, dilengkapi *anti-NaN guard* (`v0.3.0`) & penyelarasan gravitasi instan (`v0.4.1`). |
| **Cascade PID + Turn Assist** | `v0.1.0` | [`pid.h`](flying_wing_fc/pid.h) | *Outer Angle Loop* $\rightarrow$ *Inner Rate Loop*, *Derivative on Measurement*, *D-term LPF*, & fallback otomatis ke Manual bila IMU/Baro putus (`v0.3.0`/`v0.4.1`). |
| **Hardware PWM + Slew Limiter** | `v0.2.0` | [`mixer.h`](flying_wing_fc/mixer.h) | PWM silikon RP2040 50 Hz dengan *Phase-Staggering* & pembatas laju `6 µs/2 ms` anti-*brownout* BEC 5V. |
| **Servo AutoTrim (INAV Style)** | `v0.4.0` | [`autotrim.h`](flying_wing_fc/autotrim.h) | Saklar `SwB` (`CH8`) di Mode `ANGLE`/`ACRO`: transfer kompensasi integral PID ke tengah servo secara *bumpless* & simpan EEPROM otomatis saat *Disarm*. |
| **Auto-Level Pitch Trim** | `v0.4.1` | [`autotrim.h`](flying_wing_fc/autotrim.h) | Saklar `SwB` (`CH8`) di Mode `MANUAL`: merekam sudut *Pitch* terbang datar (*cruise AoA*) tervalidasi Barometer & penolak belokan pelan. |
| **Navigasi GPS M10 & Baro BMP280** | `v0.2.0` | [`gps.h`](flying_wing_fc/gps.h), [`baro.h`](flying_wing_fc/baro.h) | *Auto-Baudrate Scanner* GPS (`115200`/`38400`/`57600`/`9600`), penguncian titik Home otomatis ($\ge 5$ satelit), `MODE_ALT_HOLD`, & `MODE_RTH`. |
| **EEPROM Flash & CLI Interaktif** | `v0.2.0` | [`settings.h`](flying_wing_fc/settings.h), [`telemetry_cli.h`](flying_wing_fc/telemetry_cli.h) | Konfigurasi *live* via USB Serial Monitor (balik arah servo, kalibrasi, diagnostik GPS/Baro, reboot ke BOOTSEL/DFU tanpa tombol fisik). |

---

## 2. Tabel Wiring (Sambungan Kabel)

> **PENTING (Jalur Daya Servo & BEC ESC):**
> Jangan mengambil listrik 5V untuk menggerakkan Servo dari pin `3.3V` mikrokontroler!
> Sambungkan kabel **Merah (+5V)** dan **Hitam/Coklat (GND)** dari **BEC ESC** langsung ke jalur daya kedua Servo, serta ke pin **5V** dan **GND** pada board mikrokontroler.

### A. Konfigurasi Utama: Waveshare RP2040-Zero / Raspberry Pi Pico

| Komponen | Pin di Komponen | Pin di RP2040-Zero | Keterangan |
| :--- | :--- | :--- | :--- |
| **Sensor IMU (GY-521 MPU6050)** | `VCC` & `GND` | `3V3` & `GND` | Daya bersih 3.3V |
| | `SDA` & `SCL` | **GP4** (`SDA`) & **GP5** (`SCL`) | Bus I2C0 (`0x68`) |
| **Sensor Baro (GY-BMP280 3.3V)** | `VCC` & `GND` | `3V3` & `GND` | **Wajib 3.3V** |
| | `SDA` & `SCL` | **GP4** (`SDA`) & **GP5** (`SCL`) | Paralel dengan MPU6050 (`0x76` / `0x77`) |
| **Servo Sayap Kiri (Elevon L)** | Sinyal (Kuning/Putih) | **GP6** | Hardware PWM Slice 3A |
| **Servo Sayap Kanan (Elevon R)** | Sinyal (Kuning/Putih) | **GP7** | Hardware PWM Slice 3B (*Phase-Staggered*) |
| **ESC Motor Brushless** | Sinyal (Putih) | **GP14** | Hardware PWM Slice 7A (`1000..2000 µs`) |
| **Receiver Serial (SBUS / iBUS / CRSF)** | Sinyal `SBUS` / `TX` | **GP1** (`UART0 RX`) | Inverter silikon internal aktif otomatis untuk SBUS |
| | `RX` Receiver *(khusus ELRS)* | **GP0** (`UART0 TX`) | |
| **GPS RushFPV M10** | `5V` & `GND` | `5V` & `GND` | |
| | `TX` GPS | **GP9** (`UART1 RX`) | Auto-Baudrate (`115200`/`38400`/`57600`/`9600`) |
| | `RX` GPS | **GP8** (`UART1 TX`) | |

---

### B. Pilihan 2: Omnibus F4 V5 (STM32F405 Flight Controller)

| Komponen | Koneksi di Omnibus F4 V5 | Keterangan |
| :--- | :--- | :--- |
| **Gyro / Accel** | *(Internal Onboard)* | Otomatis aktif via **SPI1 MPU6000** (`PA4` CS) |
| **ESC Motor Brushless** | Pin **M1** | Timer `PB0` |
| **Servo Sayap Kiri** | Pin **M2** | Timer `PB1` |
| **Servo Sayap Kanan** | Pin **M3** | Timer `PA3` |
| **Receiver ELRS / iBUS / SBUS** | Pad **RX1** (`PA10`) / **J7 SBUS** (`PC7`) | Multi-port scanner aktif |
| **GPS RushFPV M10** | Pad **TX6** (`PC6`) & **RX6** (`PC7`) | Header UART6 |

---

## 3. Pemetaan Channel Remot (FlySky 10CH SBUS / iBUS)

* **CH1**: Roll (Aileron — Kiri/Kanan)
* **CH2**: Pitch (Elevator — Naik/Turun)
* **CH3**: Throttle (Gas Motor)
* **CH4**: Yaw (Rudder)
* **CH7 (`SwA` — 2 Posisi)**: Saklar **ARM / DISARM** (`< 1500 µs` = Disarmed, `> 1600 µs` = Armed).
  * *Pengaman Arming*: Motor menolak Arm jika stik Throttle belum di posisi paling bawah (`< 1120 µs`).
* **CH8 (`SwB` — 2 Posisi)**: Saklar **AutoTrim & Auto-Level** (`> 1650 µs` = Aktif):
  * Saat di **`MODE_ANGLE` / `MODE_ACRO`**: Menjalankan **Servo AutoTrim** (memindahkan kompensasi integral PID ke titik tengah servo secara *bumpless*).
  * Saat di **`MODE_MANUAL`**: Menjalankan **Auto-Level Pitch Trim** (merekam referensi sudut *Pitch* terbang mendatar dengan validasi Barometer).
  * Semua hasil trim langsung bekerja di RAM dan **otomatis disimpan ke Flash EEPROM saat Anda melakukan DISARM (`SwA` OFF)** setelah mendarat.
* **CH9 (`SwC` — 3 Posisi)**: Saklar **Mode Terbang Utama**:
  1. Atas (`< 1300 µs`): **MANUAL (Passthrough)** — kendali tangan murni.
  2. Tengah (`1300 – 1700 µs`): **ANGLE (Auto-Level / Stabilize)** — sayap otomatis kembali datar saat stik dilepas.
  3. Bawah (`> 1700 µs`): **ACRO (Rate Mode)** — gyro mengunci laju putar sudut.
* **CH10 (`SwD` — 2 Posisi)**: Saklar **Return-to-Home (`MODE_RTH`)** (`> 1700 µs`).

---

## 4. Perintah Cepat USB Serial Monitor (`115200` Baud)

Ketik huruf berikut lalu tekan **Enter** (semua perintah konfigurasi hanya diizinkan saat **DISARMED** demi keselamatan):
* **`[r]`** : Mode Monitor Receiver & Servo (`10 Hz`)
* **`[p]`** : Mode Grafik Arduino Serial Plotter
* **`[g]`** : Panel Diagnostik Lengkap **Barometer BMP280 & GPS RushFPV M10**
* **`[t]`** : Lihat Status **Servo AutoTrim** (`µs`) & **Auto-Level Trim** (`derajat`)
* **`[c]`** : Kalibrasi Gyro & Titik Datar (*Ground Level Calibration*) di meja rata
* **`[z]`** : Reset semua Trim (Servo & Level) kembali ke `0`
* **`[1]` / `[2]`** : Balik arah (*Reverse*) Servo Kiri / Servo Kanan secara *live*
* **`[w]`** : Simpan seluruh pengaturan ke **EEPROM Flash Permanen**
* **`[b]`** : Masuk mode **BOOTSEL / DFU** via software tanpa menekan tombol fisik di board
* **`[h]`** : Tampilkan banner versi firmware & menu bantuan
