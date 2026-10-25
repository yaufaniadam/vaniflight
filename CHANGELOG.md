# Changelog — VaniFlight (Autonomous Flying Wing Flight Controller)

> **Dokumen:** `CHANGELOG.md`
> **Versi Dokumen:** `v0.4.3`
> **Sejak Versi:** `v0.1.0 (2026-10-18)`
> **Terakhir Diperbarui:** `2026-10-25`

Semua perubahan penting pada arsitektur *firmware*, algoritma kendali, fitur keselamatan, dan dokumentasi dicatat di dokumen ini berdasarkan versi rilis.

---

## [`v0.4.3`] — 2026-10-25 (Modul Non-Blocking Beeper / Buzzer Indikator pada `GP15`)

### Ditambahkan
- **Modul [`beeper.h`](flying_wing_fc/beeper.h) — Non-Blocking Beeper State Machine (`PIN_BEEPER = GP15`):**
  - Mendukung buzzer 2-kaki murah baik jenis **Active (DC HIGH)** maupun **Passive (2.5 kHz Tone)** melalui mode *Hybrid* (`BEEPER_MODE_HYBRID`) dan penguat arus pin silikon RP2040 `12 mA`.
  - Menggunakan *pure SIO GPIO* (`GPIO_FUNC_SIO`) dan *repeating timer* sehingga 100% bebas konflik dengan *Hardware PWM Slice 7A* pada `GP14` (`PIN_MOTOR_ESC`) dan tidak menambah *blocking delay* pada *loop* kendali 500 Hz.
  - Pola bunyi otomatis: **Boot Ready**, **GPS 3D Home Locked** ($\ge 5$ satelit), **Armed / Disarmed**, **Peringatan Arming Throttle Tinggi**, **AutoTrim Selesai**, serta **RTH / Lost Model Beacon** setiap 2 detik.
  - Perintah CLI **`[v]`** pada [`telemetry_cli.h`](flying_wing_fc/telemetry_cli.h) untuk menguji bunyi dan mengganti mode driver (`HYBRID` $\rightarrow$ `ACTIVE` $\rightarrow$ `PASSIVE`).

---

## [`v0.4.2`] — 2026-10-25 (Parser Biner Native u-blox `UBX-NAV-PVT` 230400 Baud untuk RushFPV M10)

### Ditambahkan & Diperbaiki
- **Parser Biner Native u-blox `UBX-NAV-PVT` (`0xB5 0x62 0x01 0x07`, 92-byte payload) ([`gps.h`](flying_wing_fc/gps.h), [`config.h`](flying_wing_fc/config.h)):**
  - Mendukung langsung protokol biner pabrik **RushFPV M10** pada **230400 baud** dengan validasi *checksum* **Fletcher-16 (`CK_A`, `CK_B`)** tanpa memerlukan konfigurasi eksternal via u-center.
  - Mengekstrak langsung waktu satelit UTC (`year-month-day hour:min:sec`), status *Fix* (`2D`/`3D`), jumlah satelit (`numSV`), koordinat presisi tinggi `lat/lon` ($10^{-7}\text{ deg}$), ketinggian `hMSL` ($\text{mm}$), kecepatan tanah `gSpeed`, dan arah gerak `headMot` pada 10 Hz.
  - Mempertahankan kompatibilitas ganda (*dual-protocol*) dengan kalimat ASCII NMEA (`$GNGGA`/`$GNRMC`) dan *Auto-Baudrate Scanner* (`230400` $\rightarrow$ `115200` $\rightarrow$ `57600` $\rightarrow$ `38400` $\rightarrow$ `9600`).
- **Ekspansi Buffer FIFO Hardware UART1 RP2040 (`setFIFOSize(256)`) ([`gps.h`](flying_wing_fc/gps.h)):**
  - Memperbesar *ring buffer* `Serial2` (`GP9 RX / GP8 TX`) dari default 32 byte menjadi **256 byte** agar paket 100-byte `UBX-NAV-PVT` pada 230400 baud (46 byte per siklus loop 2 ms) tidak pernah mengalami *FIFO overflow*.
  - Terverifikasi langsung pada perangkat keras: **BMP280 (`0x76`)** + **RushFPV M10 (`GP9 @ 230400 baud`, 18 satelit 3D Fix)** bekerja simultan pada loop 500 Hz.
- **Pemetaan Ulang Saklar Remot & Peningkatan Navigasi RTH / Alt-Hold ([`rc_input.h`](flying_wing_fc/rc_input.h), [`pid.h`](flying_wing_fc/pid.h), [`telemetry_cli.h`](flying_wing_fc/telemetry_cli.h)):**
  - `SwA (CH7)` = **Return-to-Home (`MODE_RTH`)**
  - `SwB (CH8)` = **ARM / DISARM**
  - `SwC (CH9 - 3 Pos)` = **`MODE_MANUAL`** / **`MODE_ANGLE`** / **`MODE_ALT_HOLD`**
  - `SwD (CH10 - 3 Pos)` = **Servo AutoTrim & Auto-Level Pitch Trim**
  - Menambahkan *Coordinated Turn Assist* pada `MODE_RTH` dan `MODE_ALT_HOLD` serta *fallback* ketinggian GPS MSL pada `MODE_RTH`.
- **Hardening Validitas Navigasi GPS & Alt-Hold Hasil Audit (`[P1]`/`[P2]`) ([`gps.h`](flying_wing_fc/gps.h), [`pid.h`](flying_wing_fc/pid.h), [`rc_input.h`](flying_wing_fc/rc_input.h)):**
  - Memisahkan *timeout* komunikasi umum (`last_sentence_ms_`), posisi valid (`last_fix_ms_`), dan *course over ground* valid (`last_course_ms_`, `course_valid`); kalimat `$GNRMC` berstatus `'V'` langsung membatalkan `fix_valid` & `course_valid`, serta `MODE_RTH` mewajibkan posisi dan *course* yang segar ($\le 2.5\text{ detik}$).
  - Menambahkan verifikasi *checksum* XOR NMEA 0183 (`*XX`) dan pemeriksaan kelengkapan *field* koordinat/ketinggian sebelum memperbarui status GPS atau titik *Home*.
  - Memperbaiki inisialisasi target `MODE_ALT_HOLD` agar langsung mengunci ketinggian aktual barometer (`locked_alt_target_m = baro.altitude_agl_m`) tanpa melompat ke `25 m` saat diaktifkan pada ketinggian rendah ($\le 5\text{ m}$).

---

## [`v0.4.1`] — 2026-10-24 (Auto-Level Pitch Trim, Kalibrasi Quaternion Gravitasi, & Diagnostik GPS/Baro)

### Ditambahkan
- **In-Flight Auto-Level Pitch Trim ([`autotrim.h`](flying_wing_fc/autotrim.h)):**
  - Aktif melalui saklar `SwB` (`CH8 > 1650 us`) saat terbang di `MODE_MANUAL`.
  - Merekam sudut serang terbang mendatar (*cruise pitch AoA*) sebagai referensi `pitch_trim_deg` (`0.0°` untuk `MODE_ANGLE`) tanpa mengubah perintah servo (*100% bumpless*).
  - **Wajib Barometer Aktif (`baro.connected`):** Memvalidasi `|climb_rate_mps| <= 0.8 m/s` dan sebaran ketinggian `(max_alt - min_alt) <= 1.0 m` selama minimal 30 sampel berurutan (`0.6 detik`).
  - **Penolak Belokan Pelan (*Gentle Banked Turn Rejection*):** Mewajibkan sayap datar terhadap referensi darat (`|roll_deg| <= 4.0°`), laju yaw ketat (`|gyro_yaw_dps| < 2.5°/s`), dan hanya mengubah `pitch_trim_deg` (referensi `roll_trim_deg` dikalibrasi khusus di darat).
- **Penyelarasan Quaternion Gravitasi pada Kalibrasi Darat ([`imu.h`](flying_wing_fc/imu.h)):**
  - `calibrateGyro()` kini memfilter guncangan (`peak-to-peak <= 20°/s`) dan langsung menyelaraskan quaternion Madgwick $(q_0, q_1, q_2, q_3)$ ke rata-rata vektor gravitasi accelerometer sehingga bebas jeda konvergensi setelah kalibrasi `[c]`.
- **Auto-Baudrate Scanner GPS RushFPV M10 & Diagnostik ([`gps.h`](flying_wing_fc/gps.h), [`telemetry_cli.h`](flying_wing_fc/telemetry_cli.h)):**
  - Pemindaian otomatis *baudrate* (`115200` $\rightarrow$ `38400` $\rightarrow$ `57600` $\rightarrow$ `9600`) saat *Disarmed* hingga mengunci kalimat NMEA valid (`$GNGGA`/`$GNRMC`).
  - Panel diagnostik `[g]` untuk memantau status I2C BMP280 (`0x76`/`0x77`, suhu, tekanan, AGL, *climb rate*) dan trafik UART GPS M10.
- **Sistem Pencatatan Versi Dokumen & Kode (`FW_VERSION_STR = "v0.4.1"`):**
  - Header versi dan riwayat perubahan di seluruh file `.ino`, `.h`, dan `.md`.

---

## [`v0.4.0`] — 2026-10-24 (Servo AutoTrim Ala INAV dengan Bumpless Transfer)

### Ditambahkan
- **Modul [`autotrim.h`](flying_wing_fc/autotrim.h) — Servo AutoTrim (`MODE_ANGLE` / `MODE_ACRO` via `SwB` CH8):**
  - Mengumpulkan jendela *ring buffer* 50 sampel (`1.0 detik @ 50 Hz`, minimal 30 sampel stabil berurutan) dari kompensasi integral PID saat terbang lurus.
  - Menghitung delta trim servo sesuai arah (`servo_left_dir`, `servo_right_dir`) dan skala (`servo_left_scale`, `servo_right_scale`) pada [`mixer.h`](flying_wing_fc/mixer.h).
  - Membatasi langkah perubahan maksimal `±70 µs` per sesi secara proporsional pasca-pencampuran dan `±200 µs` total.
  - **Bumpless Transfer (`clampTransferHeadroom`):** Memindahkan kompensasi ke trim servo dan mengurangi integral PID sesuai ruang bebas (*headroom*) tanpa mereset riwayat *derivative* (`prev_meas_dps`, `d_lpf`).
  - **Deferred Flash Save:** Menyimpan hasil ke RAM di udara dan menunda penulisan `EEPROM.commit()` (`settings.save()`) hingga pesawat **DISARMED** (`!rc.armed`).
- **Proteksi Perintah CLI Saat Armed ([`telemetry_cli.h`](flying_wing_fc/telemetry_cli.h)):**
  - Memblokir perintah serial yang bersifat *blocking* atau mengubah konfigurasi vital (`[i]`, `[1]`, `[2]`, `[c]`, `[z]`, `[w]`, `[x]`, `[b]`, `[m]`) ketika status `armed == true`.

---

## [`v0.3.0`] — 2026-10-24 (Audit Keamanan & Ketahanan Kondisi Gagal / Fault Tolerance)

### Diperbaiki & Diperkuat
- **Mapping Orientasi Pemasangan Sensor (`BOARD_ROTATION` di [`config.h`](flying_wing_fc/config.h) & [`imu.h`](flying_wing_fc/imu.h)):**
  - Mendukung pemasangan standar (`0°`), `CW90`, `CW180`, `CW270`, `FLIP`, dan `FLIP_CW90` sebelum kalibrasi/filter.
- **Penjaga Pembagian Nol (Anti-NaN Guard) Madgwick ([`imu.h`](flying_wing_fc/imu.h)):**
  - Menambahkan penjaga `s_norm_sq > 1e-9f`, `q_norm_sq > 1e-9f`, dan pemulihan darurat `isnan()` pada sudut Euler.
- **Fallback Otomatis Saat IMU Terputus di Udara ([`pid.h`](flying_wing_fc/pid.h)):**
  - Jika `imu.connected == false`, kendali otomatis beralih ke *Manual Passthrough* agar pilot tetap memegang kendali penuh atas elevon.
- **Pengaman Failsafe RTH Anti-Auto-Arm ([`rc_input.h`](flying_wing_fc/rc_input.h)):**
  - Kehilangan sinyal RC hanya mengaktifkan `MODE_RTH` apabila pesawat **sudah berstatus `armed`** (terbang) dan GPS sudah mengunci titik Home. Jika kehilangan sinyal saat *Disarmed* di darat, motor dijamin tetap terkunci mati (`1000 µs`).

---

## [`v0.2.0`] — 2026-10-19 s/d 2026-10-21 (Migrasi RP2040-Zero, Hardware PWM, & Maiden Flight Sukses)

### Ditambahkan
- **Dukungan Penuh Waveshare RP2040-Zero ([`config.h`](flying_wing_fc/config.h), [`mixer.h`](flying_wing_fc/mixer.h), [`rc_input.h`](flying_wing_fc/rc_input.h)):**
  - Output **Hardware PWM Silikon Murni 50 Hz** (`GP6` Servo Kiri, `GP7` Servo Kanan, `GP14` ESC) menggantikan `Servo.h` berbasis PIO.
  - **Phase-Staggering & Slew-Rate Limiter (`6 µs / 2 ms`)** pada servo untuk meredam lonjakan arus (*inrush current spike*) pada BEC 5V.
  - Inverting UART0 RX silikon (`GP1`) untuk membaca receiver **FlySky FS-iA6C SBUS** tanpa rangkaian inverter eksternal.
  - Penyimpanan konfigurasi permanen di Flash EEPROM ([`settings.h`](flying_wing_fc/settings.h)), modul [`gps.h`](flying_wing_fc/gps.h), dan [`baro.h`](flying_wing_fc/baro.h).
  - **Maiden Flight Berhasil** pada wahana Flying Wing Plane 1.

---

## [`v0.1.0`] — 2026-10-18 (Rilis Awal Arsitektur Flight Controller)

### Ditambahkan
- Struktur dasar modular: [`flying_wing_fc.ino`](flying_wing_fc/flying_wing_fc.ino), [`config.h`](flying_wing_fc/config.h), [`imu.h`](flying_wing_fc/imu.h), [`pid.h`](flying_wing_fc/pid.h), [`mixer.h`](flying_wing_fc/mixer.h), [`rc_input.h`](flying_wing_fc/rc_input.h), dan [`telemetry_cli.h`](flying_wing_fc/telemetry_cli.h).
- Estimasi orientasi Quaternion Madgwick 6-DOF `@ 500 Hz` dan Cascade PID (*Angle* $\rightarrow$ *Rate* $\rightarrow$ *Elevon*).
