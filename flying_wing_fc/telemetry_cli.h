/*
 * ============================================================================
 * @file        telemetry_cli.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.2
 * @since       v0.1.0 (2026-10-18)
 * @updated     2026-10-25
 * @brief       Antarmuka Serial Monitor CLI Interaktif & Streaming Serial Plotter
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.1.0 (2026-10-18): Streaming telemetri 10 Hz, Serial Plotter, & perintah dasar.
 *   - v0.2.0 (2026-10-19): Live reverse servo [1]/[2], ganti protokol [i], simpan Flash [w],
 *                          serta masuk mode BOOTSEL/DFU [b] tanpa tekan tombol fisik.
 *   - v0.4.0 (2026-10-24): Proteksi semua perintah konfigurasi/blocking saat Armed,
 *                          serta menu status [t] dan reset trim [z].
 *   - v0.4.1 (2026-10-24): Tampilan versi firmware di banner, validasi kalibrasi [c],
 *                          dan panel diagnostik lengkap Barometer & GPS M10 di [g].
 *   - v0.4.2 (2026-10-25): Tampilkan telemetri paket biner u-blox UBX-NAV-PVT & MSL di menu [g].
 * ============================================================================
 */
#ifndef TELEMETRY_CLI_H
#define TELEMETRY_CLI_H

#include <Arduino.h>
#include "config.h"
#include "settings.h"
#include "imu.h"
#include "rc_input.h"
#include "pid.h"
#include "mixer.h"
#include "autotrim.h"
#include "gps.h"
#include "baro.h"

enum StreamMode {
  STREAM_PLOTTER,
  STREAM_RC,
  STREAM_STATUS,
  STREAM_PAUSED
};

class TelemetryCLI {
public:
  StreamMode current_mode = STREAM_RC; // Default ke STREAM_RC agar user langsung bisa tes receiver!

  void printBanner(const IMUManager &imu, const BaroManager &baro, const SettingsManager &settings) {
    Serial.println(F("\n============================================================"));
    Serial.println(F("   VANIFLIGHT - FLYING WING FLIGHT CONTROLLER"));
    Serial.print(F("   Firmware Ver  : ")); Serial.print(F(FW_VERSION_STR));
    Serial.print(F(" (Rilis ")); Serial.print(F(FW_RELEASE_DATE)); Serial.println(F(")"));
    Serial.print(F("   Board Aktif   : ")); Serial.println(F(BOARD_NAME));
    Serial.print(F("   Status IMU    : "));
    if (imu.state.connected) {
      #if defined(IMU_USE_SPI) && (IMU_USE_SPI == 1)
        Serial.println(F("TERHUBUNG (MPU6000 SPI OK)"));
      #else
        Serial.println(F("TERHUBUNG (MPU6050 I2C OK)"));
      #endif
    } else {
      Serial.println(F("TIDAK TERDETEKSI (Cek SDA/SCL & Power)"));
    }
    Serial.print(F("   Status Baro   : "));
    if (baro.state.connected) {
      Serial.print(F("TERHUBUNG (BMP280 I2C 0x"));
      Serial.print(baro.state.i2c_addr, HEX);
      Serial.println(F(" OK)"));
    } else {
      Serial.println(F("TIDAK TERDETEKSI (Cek SDA=GP4, SCL=GP5)"));
    }
    Serial.print(F("   Protokol RC   : ")); Serial.println(settings.getProtocolName());
    Serial.print(F("   Arah Servo    : Kiri="));
    Serial.print(settings.data.servo_left_dir > 0 ? F("NORMAL (+1)") : F("REVERSE (-1)"));
    Serial.print(F(" | Kanan="));
    Serial.println(settings.data.servo_right_dir > 0 ? F("NORMAL (+1)") : F("REVERSE (-1)"));
    Serial.print(F("   Trim Servo    : Kiri="));
    Serial.print(settings.data.servo_left_trim_us);
    Serial.print(F(" us | Kanan="));
    Serial.print(settings.data.servo_right_trim_us);
    Serial.println(F(" us"));
    Serial.print(F("   Level Trim    : Roll="));
    Serial.print(imu.state.roll_trim_deg, 2);
    Serial.print(F(" deg | Pitch="));
    Serial.print(imu.state.pitch_trim_deg, 2);
    Serial.println(F(" deg"));
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("MODE TAMPILAN (Ketik huruf lalu Enter):"));
    Serial.println(F("  [r] Mode Monitor Receiver (CH1..CH6)"));
    Serial.println(F("  [p] Mode Grafik IMU & Servo (Arduino Serial Plotter)"));
    Serial.println(F("  [s] Mode Tabel Status Lengkap (IMU + RC + Servo)"));
    Serial.println(F("  [ ] (Spasi) Jeda / Lanjut streaming (Pause/Resume)"));
    Serial.println(F("SETTING LIVE (TANPA PERLU COMPILE / FLASH ULANG!):"));
    Serial.println(F("  [i] Ganti Protokol Receiver (FlySky iBUS / ELRS CRSF / SBUS)"));
    Serial.println(F("  [1] Balik Arah (Reverse) Servo Sayap Kiri"));
    Serial.println(F("  [2] Balik Arah (Reverse) Servo Sayap Kanan"));
    Serial.println(F("  [c] Kalibrasi Gyro & Titik Datar (Level Trim) di Meja/Tanah"));
    Serial.println(F("  [g] Diagnostik Sensor Barometer BMP280 & GPS RushFPV M10"));
    Serial.println(F("  [t] Lihat Status Servo AutoTrim & Auto-Level Trim"));
    Serial.println(F("  [z] Reset Semua Trim (Servo & Level) ke 0"));
    Serial.println(F("  [m] Ganti Mode Terbang Manual -> Acro -> Angle -> AltHold -> RTH"));
    Serial.println(F("SIMPAN & SISTEM (Ala iNav / Betaflight - Tanpa Tombol BOOT!):"));
    Serial.println(F("  [w] SAVE TO FLASH : Simpan semua setting ke EEPROM permanen"));
    Serial.println(F("  [x] REBOOT        : Restart Flight Controller via software"));
    Serial.println(F("  [b] DFU MODE      : Masuk mode Flash DFU TANPA tekan tombol BOOT!"));
    Serial.println(F("  [h] Tampilkan menu bantuan & konfigurasi saat ini"));
    Serial.println(F("============================================================\n"));
  }

  void update(IMUManager &imu, RCInputManager &rc, FlightControllerPID &pid, ElevonMixer &mixer, AutoTrimManager &autotrim, GPSManager &gps, BaroManager &baro, SettingsManager &settings, uint32_t loop_dt_us) {
    // 1. Baca perintah dari pengguna lewat USB Serial Monitor
    while (Serial.available()) {
      char cmd = (char)Serial.read();
      if (cmd == '\n' || cmd == '\r') continue;

      switch (cmd) {
        case 'r':
        case 'R':
          current_mode = STREAM_RC;
          Serial.print(F("\n>>> MODE MONITOR RECEIVER AKTIF ["));
          Serial.print(settings.getProtocolName());
          Serial.println(F("] <<<"));
          break;

        case 'p':
        case 'P':
          current_mode = STREAM_PLOTTER;
          Serial.println(F("\n>>> MODE GRAFIK SERIAL PLOTTER AKTIF <<<"));
          break;

        case 's':
        case 'S':
          current_mode = STREAM_STATUS;
          Serial.println(F("\n>>> MODE STATUS LENGKAP AKTIF <<<"));
          break;

        case ' ':
          if (current_mode == STREAM_PAUSED) {
            current_mode = STREAM_RC;
            Serial.println(F("\n[RESUME] Streaming dilanjutkan."));
          } else {
            current_mode = STREAM_PAUSED;
            Serial.println(F("\n[PAUSE] Streaming dijeda. Ketik [h] untuk menu atau Spasi untuk lanjut."));
          }
          break;

        case 'i':
        case 'I':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum mengubah protokol receiver!\n"));
            break;
          }
          // Toggle protokol receiver: SBUS -> iBUS -> PPM -> CRSF -> SBUS
          if (settings.data.rc_protocol == RC_PROTO_SBUS) {
            settings.data.rc_protocol = RC_PROTO_IBUS;
          } else if (settings.data.rc_protocol == RC_PROTO_IBUS) {
            settings.data.rc_protocol = RC_PROTO_PPM;
          } else if (settings.data.rc_protocol == RC_PROTO_PPM) {
            settings.data.rc_protocol = RC_PROTO_CRSF;
          } else {
            settings.data.rc_protocol = RC_PROTO_SBUS;
          }
          rc.begin(settings.data.rc_protocol);
          Serial.print(F("\n[SETTING] Protokol Receiver diubah LIVE ke: "));
          Serial.println(rc.getProtocolName());
          Serial.println(F("          (Ketik 'w' jika ingin menyimpan permanen ke Flash)\n"));
          break;

        case '1':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum membalik arah servo!\n"));
            break;
          }
          settings.data.servo_left_dir = -settings.data.servo_left_dir;
          mixer.servo_left_dir = settings.data.servo_left_dir;
          Serial.print(F("\n[SETTING] Arah Servo KIRI (M2) diubah ke: "));
          Serial.println(mixer.servo_left_dir > 0 ? F("NORMAL (+1)") : F("REVERSED (-1)"));
          Serial.println(F("          (Ketik 'w' untuk simpan permanen ke Flash)\n"));
          break;

        case '2':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum membalik arah servo!\n"));
            break;
          }
          settings.data.servo_right_dir = -settings.data.servo_right_dir;
          mixer.servo_right_dir = settings.data.servo_right_dir;
          Serial.print(F("\n[SETTING] Arah Servo KANAN (M3) diubah ke: "));
          Serial.println(mixer.servo_right_dir > 0 ? F("NORMAL (+1)") : F("REVERSED (-1)"));
          Serial.println(F("          (Ketik 'w' untuk simpan permanen ke Flash)\n"));
          break;

        case 'c':
        case 'C':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum kalibrasi Gyro & Titik Datar!\n"));
            break;
          }
          Serial.println(F("\n[INFO] Mengkalibrasi Gyro & Titik Datar... Jangan gerakkan pesawat!"));
          if (!imu.calibrateGyro(300)) {
            Serial.println(F("[GAGAL] Kalibrasi Gyro ditolak: IMU tidak terhubung atau pesawat bergerak!\n"));
            break;
          }
          if (!imu.calibrateLevel()) {
            Serial.println(F("[GAGAL] Kalibrasi Titik Datar ditolak: Kemiringan melebihi batas +-15 derajat!\n"));
            break;
          }
          settings.data.roll_trim_deg  = imu.state.roll_trim_deg;
          settings.data.pitch_trim_deg = imu.state.pitch_trim_deg;
          pid.reset();
          Serial.print(F("[OK] Kalibrasi selesai! Level Trim -> Roll: "));
          Serial.print(imu.state.roll_trim_deg, 2);
          Serial.print(F(" deg | Pitch: "));
          Serial.print(imu.state.pitch_trim_deg, 2);
          Serial.println(F(" deg (Ketik 'w' untuk simpan ke Flash)\n"));
          break;

        case 'z':
        case 'Z':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum mereset & menyimpan ke Flash!\n"));
          } else {
            autotrim.resetTrims(mixer, imu.state, settings, false);
          }
          break;

        case 'w':
        case 'W':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum menyimpan ke Flash EEPROM!\n"));
            break;
          }
          Serial.println(F("\n[FLASH] Menyimpan konfigurasi ke EEPROM Flash..."));
          settings.save();
          autotrim.pending_flash_save = false;
          Serial.println(F("[OK] Tersimpan permanen! Setting tidak akan hilang walau cabut baterai/reboot.\n"));
          break;

        case 'x':
        case 'X':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum reboot!\n"));
            break;
          }
          softwareReboot();
          break;

        case 'b':
        case 'B':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum masuk mode DFU/Bootloader!\n"));
            break;
          }
          softwareJumpToDFU();
          break;

        case 'g':
        case 'G':
          Serial.println(F("\n========================================================"));
          Serial.println(F("           DIAGNOSTIK BAROMETER & GPS M10"));
          Serial.println(F("========================================================"));
          Serial.print(F("[BARO] Status  : "));
          if (baro.state.connected) {
            Serial.print(F("TERHUBUNG (0x"));
            Serial.print(baro.state.i2c_addr, HEX);
            Serial.print(F(") | Suhu: "));
            Serial.print(baro.state.temperature_c, 1);
            Serial.print(F(" C | Press: "));
            Serial.print(baro.state.pressure_pa, 0);
            Serial.println(F(" Pa"));
            Serial.print(F("       Alt AGL : "));
            Serial.print(baro.state.altitude_agl_m, 2);
            Serial.print(F(" m | Climb Rate: "));
            Serial.print(baro.state.climb_rate_mps, 2);
            Serial.println(F(" m/s"));
          } else {
            Serial.println(F("TIDAK TERDETEKSI (Pastikan SDA->GP4, SCL->GP5, VCC->3.3V)"));
          }
          Serial.print(F("[GPS]  Status  : "));
          if (gps.state.connected) {
            Serial.print(F("TERHUBUNG di GP"));
            Serial.print(gps.state.active_rx_pin);
            Serial.print(F(" ("));
            Serial.print(gps.state.baud_rate);
            Serial.print(F(" baud, Paket OK: "));
            Serial.print(gps.state.sentences_ok);
            Serial.println(F(")"));
            Serial.print(F("       Frame   : "));
            Serial.println(gps.state.last_nmea);
          } else {
            Serial.print(F("MENUNGGU DATA (Scan GP"));
            Serial.print(gps.state.active_rx_pin);
            Serial.print(F(" @ "));
            Serial.print(gps.state.baud_rate);
            Serial.print(F(" baud, Byte: "));
            Serial.print(gps.state.rx_bytes);
            Serial.println(F(")"));
          }
          Serial.print(F("       Fix     : "));
          if (gps.state.fix_type == 3) Serial.print(F("3D LOCKED"));
          else if (gps.state.fix_type == 2) Serial.print(F("2D LOCKED"));
          else Serial.print(F("SEARCHING SKY"));
          Serial.print(F(" | Satelit: ")); Serial.print(gps.state.satellites);
          Serial.print(F(" | Speed: "));   Serial.print(gps.state.speed_kmh, 1);
          Serial.print(F(" km/h | Alt MSL: ")); Serial.print(gps.state.altitude_m, 1);
          Serial.println(F(" m"));
          Serial.print(F("       Posisi  : Lat "));
          Serial.print(gps.state.latitude_deg, 6);
          Serial.print(F(" | Lon "));
          Serial.print(gps.state.longitude_deg, 6);
          Serial.print(F(" | Home: "));
          Serial.print(gps.state.dist_to_home_m, 1);
          Serial.println(F(" m"));
          Serial.println(F("========================================================\n"));
          break;

        case 'm':
        case 'M':
          if (rc.state.armed) {
            Serial.println(F("\n[PERINGATAN] Disarm terlebih dahulu sebelum mengubah mode via CLI!\n"));
            break;
          }
          if (rc.state.mode == MODE_MANUAL) rc.state.mode = MODE_ACRO;
          else if (rc.state.mode == MODE_ACRO) rc.state.mode = MODE_ANGLE;
          else if (rc.state.mode == MODE_ANGLE) rc.state.mode = MODE_ALT_HOLD;
          else if (rc.state.mode == MODE_ALT_HOLD) rc.state.mode = MODE_RTH;
          else rc.state.mode = MODE_MANUAL;
          Serial.print(F("\n[MODE] Pindah ke: "));
          Serial.println(rc.getModeName());
          break;

        case 't':
        case 'T':
          Serial.println(F("\n========================================================"));
          Serial.println(F("         STATUS SERVO AUTOTRIM & AUTO-LEVEL TRIM"));
          Serial.println(F("========================================================"));
          Serial.print(F("Trim Servo Kiri  : ")); Serial.print(mixer.servo_left_trim_us); Serial.println(F(" us"));
          Serial.print(F("Trim Servo Kanan : ")); Serial.print(mixer.servo_right_trim_us); Serial.println(F(" us"));
          Serial.print(F("Level Trim Roll  : ")); Serial.print(imu.state.roll_trim_deg, 2); Serial.println(F(" deg"));
          Serial.print(F("Level Trim Pitch : ")); Serial.print(imu.state.pitch_trim_deg, 2); Serial.println(F(" deg"));
          Serial.print(F("Status Simpan    : "));
          Serial.println(autotrim.pending_flash_save ? F("MENUNGGU DISARM (Di RAM)") : F("TERSIMPAN DI FLASH"));
          Serial.println(F("\nKetik 'Z' lalu Enter jika ingin RESET semua trim ke 0 (saat Disarmed)."));
          Serial.println(F("========================================================\n"));
          break;

        case 'h':
        case 'H':
          printBanner(imu, baro, settings);
          break;
      }
    }

    // 2. Cetak telemetri setiap 100 ms (10 Hz)
    uint32_t now_ms = millis();
    if (now_ms - last_print_ms_ < 100) return;
    last_print_ms_ = now_ms;

    switch (current_mode) {
      case STREAM_RC:
        if (imu.state.connected) {
          Serial.print(F("IMU[R:"));
          Serial.print(imu.state.roll_deg, 1);
          Serial.print(F("° P:"));
          Serial.print(imu.state.pitch_deg, 1);
          Serial.print(F("°] | "));
        } else {
          Serial.print(F("[IMU DISCONNECTED] | "));
        }

        Serial.print(F("RC -> CH1:"));      Serial.print(rc.state.roll_us);
        Serial.print(F(" CH2:"));           Serial.print(rc.state.pitch_us);
        Serial.print(F(" CH3:"));           Serial.print(rc.state.throttle_us);
        Serial.print(F(" | SwA(RTH):"));    Serial.print(rc.state.rth_sw_us);
        Serial.print(F(" SwB(Arm):"));      Serial.print(rc.state.arm_us);
        Serial.print(F(" SwC(Mode):"));     Serial.print(rc.state.mode_us);
        Serial.print(F(" SwD(Trim):"));     Serial.print(rc.state.autotrim_sw_us);
        Serial.print(F(" ["));              Serial.print(rc.getModeName());
        Serial.print(F("] | "));
        if (rc.state.failsafe) {
          Serial.println(F("[NO RC SIGNAL]"));
        } else {
          Serial.print(F("SRV[L:"));
          Serial.print(mixer.out.servo_left_us);
          Serial.print(F(" R:"));
          Serial.print(mixer.out.servo_right_us);
          Serial.print(F(" | Trim L:"));
          Serial.print(mixer.servo_left_trim_us);
          Serial.print(F(" R:"));
          Serial.print(mixer.servo_right_trim_us);
          Serial.print(F("] ESC:"));
          Serial.print(mixer.out.motor_esc_us);
          Serial.print(F(" | "));
          Serial.println(rc.state.armed ? F("ARMED") : F("DISARMED"));
        }
        break;

      case STREAM_PLOTTER:
        Serial.print(F("Roll_Deg:"));       Serial.print(imu.state.roll_deg, 1);
        Serial.print(F("\tPitch_Deg:"));    Serial.print(imu.state.pitch_deg, 1);
        Serial.print(F("\tRC_Roll:"));      Serial.print((int)rc.state.roll_us - 1500);
        Serial.print(F("\tRC_Pitch:"));     Serial.print((int)rc.state.pitch_us - 1500);
        Serial.print(F("\tServoL_Offset:")); Serial.print((int)mixer.out.servo_left_us - 1500);
        Serial.print(F("\tServoR_Offset:")); Serial.println((int)mixer.out.servo_right_us - 1500);
        break;

      case STREAM_STATUS:
        Serial.print(F("IMU[R:"));
        Serial.print(imu.state.roll_deg, 1);
        Serial.print(F("° P:"));
        Serial.print(imu.state.pitch_deg, 1);
        Serial.print(F("°] | RC[R:"));
        Serial.print(rc.state.roll_us);
        Serial.print(F(" P:"));
        Serial.print(rc.state.pitch_us);
        Serial.print(F(" T:"));
        Serial.print(rc.state.throttle_us);
        Serial.print(F("] | OUT[SrvL:"));
        Serial.print(mixer.out.servo_left_us);
        Serial.print(F(" SrvR:"));
        Serial.print(mixer.out.servo_right_us);
        Serial.print(F(" ESC:"));
        Serial.print(mixer.out.motor_esc_us);
        Serial.print(F("] | "));
        Serial.print(rc.state.failsafe ? F("[FS] ") : F("[OK] "));
        Serial.print(rc.state.armed ? F("ARMED | ") : F("DISARMED | "));
        Serial.println(rc.getModeName());
        break;

      case STREAM_PAUSED:
        break;
    }
  }

private:
  uint32_t last_print_ms_ = 0;
};

#endif // TELEMETRY_CLI_H
