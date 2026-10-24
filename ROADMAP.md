# Roadmap Pengembangan VaniFlight — Menuju 3-Plane Flying Wing Swarm

> **Nama Proyek:** `VaniFlight` *(Autonomous Flying Wing Flight Controller)*
> **Dokumen:** `ROADMAP.md`
> **Versi Dokumen:** `v0.4.1`
> **Sejak Versi:** `v0.4.1 (2026-10-24)`
> **Terakhir Diperbarui:** `2026-10-24`

Dokumen ini memetakan tahapan rekayasa dari 1 pesawat *Flying Wing* mandiri hingga mencapai **Swarm Otonom 3 Pesawat (Formasi V / Leader–Wingman)** menggunakan arsitektur **Waveshare RP2040-Zero** + **MPU6050** + **BMP280** + **RushFPV M10 GPS** + **Wemos D1 Mini (ESP-NOW)**.

---

## Peta Jalan Utama (Overview)

| Fase | Target Versi | Fokus Utama | Wahana Aktif | Status |
| :---: | :---: | :--- | :---: | :---: |
| **Fase 1** | `v0.1.0` – `v0.4.1` | Kendali Sikap 500 Hz, Safety Hardening, Servo AutoTrim, & Auto-Level Pitch Trim | **Plane 1** | **SELESAI** |
| **Fase 2** | `v0.5.0` | Integrasi Fisik GPS M10 + Baro BMP280, Uji Terbang `ALT_HOLD`, `RTH`, & `Loiter` | **Plane 1** | **BERIKUTNYA** |
| **Fase 3** | `v0.6.0` | Jaringan Komunikasi Antar-Pesawat **ESP-NOW** (`10 Hz`) via Co-Processor Wemos D1 Mini | **Plane 1 + GCS** | Direncanakan |
| **Fase 4** | `v0.7.0` | Perakitan **Plane 2**, *Vertical Altitude Layering*, & Formasi *Leader–Follower* (2 Pesawat) | **Plane 1 + Plane 2** | Direncanakan |
| **Fase 5** | `v0.8.0` | Algoritma Anti-Tabrakan (*Repulsion Field*) & *Swarm Failsafe* (RTH Berjenjang) | **Plane 1 + Plane 2** | Direncanakan |
| **Fase 6** | `v1.0.0` | Integrasi **Plane 3** & Terbang Formasi Penuh **3-Plane V-Formation Swarm** | **Plane 1, 2, & 3** | Target Utama |

---

## Rincian Teknis Per Fase

### Fase 1: Fondasi Flight Controller & Auto-Trim (`v0.1.0` – `v0.4.1`) — SELESAI
- [x] Loop kendali *real-time* 500 Hz dengan filter Quaternion Madgwick 6-DOF ([`imu.h`](flying_wing_fc/imu.h)).
- [x] Cascade PID (*Angle* $\rightarrow$ *Rate* $\rightarrow$ *Servo*) + *Coordinated Turn Assist* ([`pid.h`](flying_wing_fc/pid.h)).
- [x] Hardware PWM silikon RP2040 50 Hz + *Phase-Staggering* + *Slew-Rate Limiter* ([`mixer.h`](flying_wing_fc/mixer.h)).
- [x] *Maiden Flight* sukses pada **Plane 1**.
- [x] Audit *Fault-Tolerance*: `BOARD_ROTATION`, *Anti-NaN Guard*, *IMU Disconnect Fallback*, & *RTH Anti-Auto-Arm*.
- [x] **Servo AutoTrim** (`SwB` di Mode `ANGLE`/`ACRO`) & **Auto-Level Pitch Trim** (`SwB` di Mode `MANUAL`) ([`autotrim.h`](flying_wing_fc/autotrim.h)).

---

### Fase 2: Navigasi Otonom Tunggal — GPS M10 & Baro BMP280 (`v0.5.0`) — BERIKUTNYA
- [ ] **Integrasi & Uji Meja Sensor Navigasi di Plane 1:**
  - Pemasangan **GY-BMP280** di bus I2C0 (`GP4`/`GP5`) dan **RushFPV M10 GPS** di UART1 (`GP8`/`GP9`).
  - Verifikasi penguncian satelit ($\ge 6$ satelit), akurasi *Course Over Ground (COG)*, dan kestabilan pembacaan ketinggian `AGL` lewat perintah CLI `[g]`.
- [ ] **Uji Terbang `MODE_ALT_HOLD`:**
  - Validasi penguncian ketinggian jelajah konstan (misal `40 m AGL`) dengan redaman *pitch* yang halus tanpa osilasi (*porpoising*).
- [ ] **Penyempurnaan Kendali L1-Navigation & Loiter pada `MODE_RTH`:**
  - Mengganti pengali *bearing* proporsional sederhana dengan hukum panduan **L1 Lateral Guidance** agar lintasan belok menuju titik *Home* dan lingkaran *Loiter* (radius `35 m`) tetap mulus walaupun ada angin samping (*crosswind*).

---

### Fase 3: Link Telemetri & Swarm Mesh via ESP-NOW (`v0.6.0`)
- [ ] **Arsitektur Dual-MCU (RP2040-Zero + Wemos D1 Mini ESP8266):**
  - **RP2040-Zero** tetap murni menangani kendali terbang kritis 500 Hz.
  - **Wemos D1 Mini** bertindak sebagai modem radio **ESP-NOW** *peer-to-peer* (`2.4 GHz`, tanpa router, *latency* $< 5\text{ ms}$) yang terhubung ke RP2040.
- [ ] **Protokol Paket `VaniSwarmPacket` (`10 Hz` Broadcast):**
  - Berisi: `plane_id` (`1..3`), `seq`, `lat_e7`, `lon_e7`, `alt_agl_m`, `ground_speed_mps`, `course_deg`, `roll_deg`, `flight_mode`, dan `crc16`.
- [ ] **Ground Station Monitor (GCS):**
  - Satu unit Wemos D1 Mini terhubung ke laptop untuk menampilkan tabel telemetri & posisi relatif semua pesawat secara *real-time*.

---

### Fase 4: Duplikasi ke Plane 2 & Formasi *Leader–Follower* (`v0.7.0`)
- [ ] **Perakitan & Kalibrasi Plane 2:**
  - Merakit **Plane 2** identik dengan Plane 1, menjalankan kalibrasi meja `[c]`, *Maiden Flight*, **Servo AutoTrim**, dan **Auto-Level Pitch Trim**.
- [ ] **Pemisahan Lapis Ketinggian Aman (*Vertical Altitude Separation*):**
  - Kunci keamanan utama agar pesawat tidak pernah bertabrakan secara fisik saat uji algoritma:
    - **Plane 1 (Leader):** Ketinggian jelajah **`40 m AGL`**
    - **Plane 2 (Wingman Kanan):** Ketinggian jelajah **`48 m AGL`** ($+8\text{ m}$)
    - **Plane 3 (Wingman Kiri):** Ketinggian jelajah **`32 m AGL`** ($-8\text{ m}$)
- [ ] **Kendali Formasi 2D + *Throttle Speed Matching*:**
  - *Wingman* menghitung posisi target relatif di belakang *Leader* ($\Delta x = -15\text{ m}, \Delta y = \pm 15\text{ m}$ terhadap arah terbang *Leader*).
  - Kendali **Roll** mengarahkan pesawat ke titik formasi, sementara kendali **Throttle** otomatis menambah/mengurangi gas (`1350..1750 µs`) berdasarkan jarak sejajar (*along-track error*) agar kecepatan *Wingman* menyamai *Leader*.

---

### Fase 5: Proteksi Anti-Tabrakan & *Swarm Failsafe* (`v0.8.0`)
- [ ] **Medan Tolak Horizontal (*Artificial Potential Field Repulsion*):**
  - Jika jarak horizontal antar dua pesawat kurang dari **`12 meter`**, vektor gaya tolak otomatis menambahkan kompensasi *bank angle* menjauh.
- [ ] **FailsafeKehilangan Sinyal Leader / RC:**
  - Jika *Wingman* kehilangan paket ESP-NOW dari *Leader* selama $> 2.0\text{ detik}$, *Wingman* otomatis beralih ke **RTH / Loiter Mandiri** di lapis ketinggian amannya sendiri (`48 m` atau `32 m`).
  - Saklar darurat pilot (`MODE_MANUAL` atau `MODE_ANGLE`) pada remot masing-masing setiap saat dapat langsung mengambil alih kendali penuh (*instant pilot override*).

---

### Fase 6: Full 3-Plane V-Formation Swarm (`v1.0.0`)
- [ ] **Integrasi Plane 3 (Wingman Kiri):**
  - Perakitan, *auto-trim*, dan integrasi node ke-3 (`plane_id = 3`).
- [ ] **3 Mode Formasi Swarm di Udara:**
  1. **V-Formation (Delta):** Plane 1 di depan (`40 m`), Plane 2 di kanan-belakang (`48 m`), Plane 3 di kiri-belakang (`32 m`).
  2. **Echelon / Trail (Beriringan):** Plane 1 $\rightarrow$ Plane 2 $\rightarrow$ Plane 3 mengikuti jalur satu sama lain.
  3. **Synchronized Loiter Ring:** Ketiga pesawat berputar mengelilingi titik Home pada radius yang sama dengan sudut fasa terpisah $120^\circ$.
