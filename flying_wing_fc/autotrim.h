/*
 * ============================================================================
 * @file        autotrim.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.1
 * @since       v0.4.0 (2026-10-24)
 * @updated     2026-10-24
 * @brief       Sistem Servo AutoTrim (Angle/Acro) & Auto-Level Pitch Trim (Manual)
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.4.0 (2026-10-24): Implementasi awal Servo AutoTrim (CH8 SwB), kompensasi
 *                          polaritas & skala mixer, batas proporsional +-70 us,
 *                          clampTransferHeadroom (bumpless transfer), & simpan
 *                          EEPROM tertunda hingga Disarm (!rc.armed).
 *   - v0.4.1 (2026-10-24): Tambah Auto-Level Pitch Trim di Mode MANUAL (wajib
 *                          Barometer aktif, penolak belokan pelan |roll|<=4 deg &
 *                          |yaw_rate|<2.5 dps, hanya mengubah pitch_trim_deg).
 * ============================================================================
 */
#ifndef AUTOTRIM_H
#define AUTOTRIM_H

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "imu.h"
#include "baro.h"
#include "rc_input.h"
#include "pid.h"
#include "mixer.h"
#include "settings.h"

class AutoTrimManager {
public:
  static constexpr uint8_t  WINDOW_SIZE           = 50;    // Jendela 50 sampel terbaru (1.0 detik @ 50 Hz)
  static constexpr uint8_t  MIN_VALID_SAMPLES     = 30;    // Minimal 30 sampel stabil berurutan (0.6 detik)
  static constexpr uint32_t MAX_SAMPLE_AGE_MS     = 80;    // Umur maksimal sampel terakhir saat finalisasi
  static constexpr float    MAX_LEVEL_SPREAD_DEG  = 2.0f;  // Variasi sudut maksimal selama jendela Auto-Level (derajat)
  static constexpr float    MAX_STICK_SPREAD_NORM = 0.05f; // Variasi stik maksimal selama jendela Auto-Level (5%)
  static constexpr float    MAX_ALT_SPREAD_M      = 1.0f;  // Variasi ketinggian baro maksimal selama jendela Auto-Level (meter)
  static constexpr float    MAX_LEVEL_BANK_DEG    = 4.0f;  // Batas kemiringan sayap (Roll) saat merekam Pitch terbang datar
  static constexpr float    MAX_LEVEL_YAW_DPS     = 2.5f;  // Batas laju belok (Yaw rate) untuk menolak belokan pelan (banked turn)

  bool is_active          = false;
  bool is_completed       = false;
  bool pending_flash_save = false;

  void update(const RCState &rc, IMUState &imu, const BaroState &baro, FlightControllerPID &pid, ElevonMixer &mixer, SettingsManager &settings) {
    // Tulis perubahan trim (Servo maupun Level) ke Flash EEPROM hanya saat status DISARMED (!rc.armed)
    // agar EEPROM.commit() tidak pernah dieksekusi ketika sistem sedang ARMED.
    if (pending_flash_save && !rc.armed) {
      settings.save();
      pending_flash_save = false;
      Serial.println(F(">>> [AUTOTRIM] Status Disarmed -> Trim disimpan permanen ke Flash EEPROM!"));
    }

    bool switch_on    = (rc.autotrim_sw_us > 1650);
    bool mode_valid   = (rc.mode == MODE_ANGLE || rc.mode == MODE_ACRO || rc.mode == MODE_MANUAL);
    bool baro_ready   = (rc.mode != MODE_MANUAL) || baro.connected;
    bool flight_ready = rc.armed && !rc.failsafe && imu.connected && baro_ready && (rc.throttle_norm > 0.15f);

    // 1. Transisi: Saklar baru saja dinyalakan (Start Sesi)
    if (switch_on && !prev_switch_) {
      if (flight_ready && mode_valid) {
        is_active             = true;
        is_completed          = false;
        session_mode_         = rc.mode;
        window_count_         = 0;
        window_head_          = 0;
        last_sample_ms_       = millis();
        last_valid_sample_ms_ = 0;

        if (rc.mode == MODE_MANUAL) {
          Serial.println(F("\n>>> [AUTO-LEVEL PITCH TRIM] DIAKTIFKAN (Mode MANUAL)! Pertahankan terbang lurus & ketinggian konstan..."));
        } else {
          Serial.println(F("\n>>> [SERVO AUTOTRIM] DIAKTIFKAN (Mode ANGLE/ACRO)! Pertahankan terbang lurus & stabil..."));
        }
      } else {
        if (rc.mode == MODE_MANUAL && !baro.connected) {
          Serial.println(F("\n>>> [AUTO-LEVEL] DITOLAK: Barometer wajib aktif untuk memvalidasi terbang mendatar di udara!"));
        } else {
          Serial.println(F("\n>>> [AUTOTRIM] DITOLAK: Harus Armed, gas >15%, IMU sehat, di Mode MANUAL/ANGLE/ACRO."));
        }
      }
    }

    // 2. Saat sesi aktif: validasi kondisi & kumpulkan jendela sampel stabil terbaru
    if (is_active) {
      bool baro_lost = (session_mode_ == MODE_MANUAL) && !baro.connected;
      if (!rc.armed || rc.failsafe || !imu.connected || rc.mode != session_mode_ || baro_lost) {
        cancel(F("Kondisi wajib gagal (Disarm/Failsafe/Sensor/Mode berubah)."));
        prev_switch_ = switch_on;
        return;
      }

      uint32_t now = millis();
      if (now - last_sample_ms_ >= 20) {
        last_sample_ms_ = now;

        if (session_mode_ == MODE_MANUAL) {
          if (isStableLevelFlight(rc, imu, baro)) {
            roll_window_[window_head_]    = imu.roll_deg;                      // Simpan roll terkompensasi untuk cek sebaran
            pitch_window_[window_head_]   = imu.pitch_deg + imu.pitch_trim_deg; // Simpan raw_pitch untuk referensi level
            stick_r_window_[window_head_] = rc.roll_norm;
            stick_p_window_[window_head_] = rc.pitch_norm;
            alt_window_[window_head_]     = baro.altitude_agl_m;
            window_head_ = (window_head_ + 1) % WINDOW_SIZE;
            if (window_count_ < WINDOW_SIZE) {
              window_count_++;
            }
            last_valid_sample_ms_ = now;
          } else {
            window_count_ = 0;
            window_head_  = 0;
          }
        } else {
          if (isStableServoFlight(rc, imu, pid)) {
            roll_window_[window_head_]  = pid.roll_pid.integral;
            pitch_window_[window_head_] = pid.pitch_pid.integral;
            window_head_ = (window_head_ + 1) % WINDOW_SIZE;
            if (window_count_ < WINDOW_SIZE) {
              window_count_++;
            }
            last_valid_sample_ms_ = now;
          } else {
            window_count_ = 0;
            window_head_  = 0;
          }
        }
      }
    }

    // 3. Transisi: Saklar dimatikan (Selesai -> Validasi kondisi akhir & Terapkan ke RAM)
    if (!switch_on && prev_switch_ && is_active) {
      if (session_mode_ == MODE_MANUAL) {
        finalizeLevelTrim(rc, imu, baro, settings);
      } else {
        finalizeServoTrim(rc, imu, pid, mixer, settings);
      }
    }

    prev_switch_ = switch_on;
  }

  void cancel(const __FlashStringHelper *reason = nullptr) {
    is_active     = false;
    window_count_ = 0;
    window_head_  = 0;
    Serial.print(F(">>> [AUTOTRIM] Dibatalkan. "));
    if (reason) Serial.println(reason);
    else Serial.println();
  }

  void resetTrims(ElevonMixer &mixer, IMUState &imu, SettingsManager &settings, bool is_armed) {
    mixer.servo_left_trim_us  = 0;
    mixer.servo_right_trim_us = 0;
    settings.data.servo_left_trim_us  = 0;
    settings.data.servo_right_trim_us = 0;

    float raw_roll      = imu.roll_deg  + imu.roll_trim_deg;
    float raw_pitch     = imu.pitch_deg + imu.pitch_trim_deg;
    imu.roll_trim_deg   = 0.0f;
    imu.pitch_trim_deg  = 0.0f;
    imu.roll_deg        = raw_roll;
    imu.pitch_deg       = raw_pitch;
    settings.data.roll_trim_deg  = 0.0f;
    settings.data.pitch_trim_deg = 0.0f;

    if (!is_armed) {
      settings.save();
      pending_flash_save = false;
      Serial.println(F(">>> [AUTOTRIM] Trim Servo & Auto-Level di-reset ke 0 & disimpan ke EEPROM!"));
    } else {
      pending_flash_save = true;
      Serial.println(F(">>> [AUTOTRIM] Trim Servo & Auto-Level di-reset ke 0 di RAM (disimpan saat Disarm)."));
    }
  }

private:
  bool       prev_switch_          = false;
  FlightMode session_mode_         = MODE_ANGLE;
  uint32_t   last_sample_ms_       = 0;
  uint32_t   last_valid_sample_ms_ = 0;
  uint8_t    window_count_         = 0;
  uint8_t    window_head_          = 0;
  float      roll_window_[WINDOW_SIZE];
  float      pitch_window_[WINDOW_SIZE];
  float      stick_r_window_[WINDOW_SIZE];
  float      stick_p_window_[WINDOW_SIZE];
  float      alt_window_[WINDOW_SIZE];

  bool isStableServoFlight(const RCState &rc, const IMUState &imu, const FlightControllerPID &pid) const {
    if (isnan(imu.roll_deg) || isnan(imu.pitch_deg) ||
        isnan(imu.gyro_roll_dps) || isnan(imu.gyro_pitch_dps) || isnan(imu.gyro_yaw_dps) ||
        isnan(pid.roll_cmd_us) || isnan(pid.pitch_cmd_us) ||
        isnan(pid.roll_pid.integral) || isnan(pid.pitch_pid.integral)) {
      return false;
    }
    bool throttle_ok    = (rc.throttle_norm > 0.15f);
    bool stick_centered = (fabsf(rc.roll_norm) < 0.15f) && (fabsf(rc.pitch_norm) < 0.15f);
    bool attitude_level = (fabsf(imu.roll_deg) < 15.0f) && (fabsf(imu.pitch_deg) < 20.0f);
    bool rates_steady   = (fabsf(imu.gyro_roll_dps) < 25.0f) &&
                          (fabsf(imu.gyro_pitch_dps) < 25.0f) &&
                          (fabsf(imu.gyro_yaw_dps) < 25.0f);
    bool pid_unclamped  = (fabsf(pid.roll_cmd_us) < 390.0f) && (fabsf(pid.pitch_cmd_us) < 390.0f);
    return throttle_ok && stick_centered && attitude_level && rates_steady && pid_unclamped;
  }

  bool isStableLevelFlight(const RCState &rc, const IMUState &imu, const BaroState &baro) const {
    if (!baro.connected) {
      return false;
    }
    if (isnan(rc.roll_norm) || isnan(rc.pitch_norm) ||
        isnan(imu.roll_deg) || isnan(imu.pitch_deg) ||
        isnan(imu.roll_trim_deg) || isnan(imu.pitch_trim_deg) ||
        isnan(imu.gyro_roll_dps) || isnan(imu.gyro_pitch_dps) || isnan(imu.gyro_yaw_dps) ||
        isnan(baro.climb_rate_mps) || isnan(baro.altitude_agl_m)) {
      return false;
    }
    float raw_roll  = imu.roll_deg  + imu.roll_trim_deg;
    float raw_pitch = imu.pitch_deg + imu.pitch_trim_deg;

    bool throttle_ok = (rc.throttle_norm > 0.15f);
    bool stick_ok    = (fabsf(rc.roll_norm) < 0.25f) && (fabsf(rc.pitch_norm) < 0.35f);
    // Sayap wajib datar terhadap referensi darat (|roll_deg| <= 4 derajat) agar tidak merekam saat miring/belok,
    // dan sudut absolut dalam batas wajar (+-15 derajat).
    bool wings_level = (fabsf(imu.roll_deg) <= MAX_LEVEL_BANK_DEG) &&
                       (fabsf(raw_roll) <= MAX_LEVEL_TRIM_DEG) &&
                       (fabsf(raw_pitch) <= MAX_LEVEL_TRIM_DEG);
    // Tolak belokan pelan (gentle banked turn) dengan batas ketat Yaw rate < 2.5 dps dan Roll/Pitch rate < 5.0 dps
    bool rates_ok    = (fabsf(imu.gyro_roll_dps)  < 5.0f) &&
                       (fabsf(imu.gyro_pitch_dps) < 5.0f) &&
                       (fabsf(imu.gyro_yaw_dps)   < MAX_LEVEL_YAW_DPS);
    // Wajib mempertahankan ketinggian (climb rate dekat nol)
    bool baro_ok     = (fabsf(baro.climb_rate_mps) <= 0.8f);
    return throttle_ok && stick_ok && wings_level && rates_ok && baro_ok;
  }

  static float clampTransferHeadroom(float desired, float cur_i, float max_i, float cur_cmd) {
    float safe_max_i = fmaxf(0.0f, max_i - 2.0f);
    if (desired > 0.0f) {
      float avail_pos = fminf(cur_i + safe_max_i, cur_cmd + 395.0f);
      if (avail_pos <= 0.0f) return 0.0f;
      return fminf(desired, avail_pos);
    } else if (desired < 0.0f) {
      float avail_neg = fmaxf(cur_i - safe_max_i, cur_cmd - 395.0f);
      if (avail_neg >= 0.0f) return 0.0f;
      return fmaxf(desired, avail_neg);
    }
    return 0.0f;
  }

  void finalizeLevelTrim(const RCState &rc, IMUState &imu, const BaroState &baro, SettingsManager &settings) {
    is_active = false;

    uint32_t now = millis();
    bool sample_fresh = (last_valid_sample_ms_ > 0) && ((now - last_valid_sample_ms_) <= MAX_SAMPLE_AGE_MS);

    if (!isStableLevelFlight(rc, imu, baro) || !sample_fresh || window_count_ < MIN_VALID_SAMPLES) {
      window_count_ = 0;
      Serial.println(F(">>> [AUTO-LEVEL] GAGAL FINALISASI: Pesawat tidak stabil saat saklar dimatikan atau sampel <30."));
      return;
    }

    // Telusuri mundur dari sampel terbaru di ring buffer, pastikan sebaran (spread) sudut, stik,
    // dan ketinggian barometer termasuk kondisi live saat ini tidak melebihi batas ketat
    float cur_roll_deg  = imu.roll_deg;
    float cur_raw_pitch = imu.pitch_deg + imu.pitch_trim_deg;
    float cur_alt_m     = baro.altitude_agl_m;

    float min_r   = cur_roll_deg,  max_r   = cur_roll_deg;
    float min_p   = cur_raw_pitch, max_p   = cur_raw_pitch;
    float min_sr  = rc.roll_norm,  max_sr  = rc.roll_norm;
    float min_sp  = rc.pitch_norm, max_sp  = rc.pitch_norm;
    float min_alt = cur_alt_m,     max_alt = cur_alt_m;

    float   sum_p       = 0.0f;
    uint8_t valid_count = 0;

    for (uint8_t k = 0; k < window_count_; k++) {
      uint8_t idx = (window_head_ + WINDOW_SIZE - 1 - k) % WINDOW_SIZE;
      float r   = roll_window_[idx];
      float p   = pitch_window_[idx];
      float sr  = stick_r_window_[idx];
      float sp  = stick_p_window_[idx];
      float alt = alt_window_[idx];

      float c_min_r   = fminf(min_r,   r),   c_max_r   = fmaxf(max_r,   r);
      float c_min_p   = fminf(min_p,   p),   c_max_p   = fmaxf(max_p,   p);
      float c_min_sr  = fminf(min_sr,  sr),  c_max_sr  = fmaxf(max_sr,  sr);
      float c_min_sp  = fminf(min_sp,  sp),  c_max_sp  = fmaxf(max_sp,  sp);
      float c_min_alt = fminf(min_alt, alt), c_max_alt = fmaxf(max_alt, alt);

      if ((c_max_r - c_min_r)     > MAX_LEVEL_SPREAD_DEG  ||
          (c_max_p - c_min_p)     > MAX_LEVEL_SPREAD_DEG  ||
          (c_max_sr - c_min_sr)   > MAX_STICK_SPREAD_NORM ||
          (c_max_sp - c_min_sp)   > MAX_STICK_SPREAD_NORM ||
          (c_max_alt - c_min_alt) > MAX_ALT_SPREAD_M) {
        break;
      }

      min_r   = c_min_r;   max_r   = c_max_r;
      min_p   = c_min_p;   max_p   = c_max_p;
      min_sr  = c_min_sr;  max_sr  = c_max_sr;
      min_sp  = c_min_sp;  max_sp  = c_max_sp;
      min_alt = c_min_alt; max_alt = c_max_alt;

      sum_p += p;
      valid_count++;
    }

    window_count_ = 0;

    if (valid_count < MIN_VALID_SAMPLES) {
      Serial.println(F(">>> [AUTO-LEVEL] GAGAL FINALISASI: Sikap/stik/ketinggian berubah selama 0.6 detik terakhir."));
      return;
    }

    // Auto-Level di udara HANYA mengubah referensi Pitch (sudut serang terbang mendatar / cruise AoA).
    // Referensi Roll (0 derajat sayap datar) dikalibrasi di darat via perintah [c] agar kemiringan belok
    // tidak pernah dipelajari sebagai titik datar Roll.
    float avg_raw_pitch  = sum_p / (float)valid_count;
    float old_pitch_trim = imu.pitch_trim_deg;
    float d_pitch_deg    = constrain(avg_raw_pitch - old_pitch_trim, -MAX_LEVEL_STEP_DEG, MAX_LEVEL_STEP_DEG);
    float new_pitch_trim = constrain(old_pitch_trim + d_pitch_deg,   -MAX_LEVEL_TRIM_DEG, MAX_LEVEL_TRIM_DEG);
    float actual_d_pitch = new_pitch_trim - old_pitch_trim;

    // Perbarui titik referensi Pitch datar di RAM (karena sedang di MODE_MANUAL, tidak mengubah output servo!)
    imu.pitch_trim_deg = new_pitch_trim;
    imu.pitch_deg      = cur_raw_pitch - new_pitch_trim;

    settings.data.pitch_trim_deg = new_pitch_trim;
    if (fabsf(actual_d_pitch) > 1e-4f) {
      pending_flash_save = true;
    }
    is_completed = true;

    Serial.println(F("\n========================================================"));
    Serial.println(F("  >>> [AUTO-LEVEL SUKSES] REFERENSI PITCH DITERAPKAN <<<"));
    Serial.print(F("  Sampel Konstan  : ")); Serial.println(valid_count);
    Serial.print(F("  Level Trim Roll : ")); Serial.print(imu.roll_trim_deg, 2);
    Serial.println(F(" deg (Tetap / Kalibrasi Darat)"));
    Serial.print(F("  Level Trim Pitch: ")); Serial.print(imu.pitch_trim_deg, 2);
    Serial.print(F(" deg (Delta: ")); Serial.print(actual_d_pitch, 2); Serial.println(F(" deg)"));
    Serial.println(F("  [INFO] Akan disimpan ke Flash EEPROM otomatis saat DISARM."));
    Serial.println(F("========================================================\n"));
  }

  void finalizeServoTrim(const RCState &rc, const IMUState &imu, FlightControllerPID &pid, ElevonMixer &mixer, SettingsManager &settings) {
    is_active = false;

    uint32_t now = millis();
    bool sample_fresh = (last_valid_sample_ms_ > 0) && ((now - last_valid_sample_ms_) <= MAX_SAMPLE_AGE_MS);

    // Pastikan saat saklar dimatikan pesawat MASIH dalam kondisi stabil, PID tidak saturasi,
    // dan jendela data stabil berurutan memenuhi jumlah minimum
    if (!isStableServoFlight(rc, imu, pid) || !sample_fresh || window_count_ < MIN_VALID_SAMPLES) {
      window_count_ = 0;
      Serial.println(F(">>> [AUTOTRIM] GAGAL FINALISASI: Pesawat tidak stabil saat saklar dimatikan atau sampel stabil <30."));
      return;
    }

    float sum_r = 0.0f;
    float sum_p = 0.0f;
    for (uint8_t i = 0; i < window_count_; i++) {
      sum_r += roll_window_[i];
      sum_p += pitch_window_[i];
    }
    float avg_roll_us  = sum_r / (float)window_count_;
    float avg_pitch_us = sum_p / (float)window_count_;
    uint8_t used_samples = window_count_;
    window_count_ = 0;

    // Batasi jumlah kompensasi yang diminta sesuai ruang bebas (headroom) searah pada integral & output PID
    // agar tidak pernah terjepit di +-max_i atau +-400 us, dan tidak pernah membalik tanda kompensasi.
    float cur_i_roll    = pid.roll_pid.integral;
    float cur_i_pitch   = pid.pitch_pid.integral;
    float cur_cmd_roll  = pid.roll_cmd_us;
    float cur_cmd_pitch = pid.pitch_cmd_us;

    float req_roll_us  = clampTransferHeadroom(avg_roll_us,  cur_i_roll,  pid.roll_pid.max_i,  cur_cmd_roll);
    float req_pitch_us = clampTransferHeadroom(avg_pitch_us, cur_i_pitch, pid.pitch_pid.max_i, cur_cmd_pitch);

    // Hitung delta trim pada masing-masing servo menggunakan rumus arah & skala yang SAMA PERSIS dengan ElevonMixer:
    // left_mix  = (pitch - roll) * servo_left_dir  * servo_left_scale
    // right_mix = (pitch + roll) * servo_right_dir * servo_right_scale
    float left_factor  = (float)mixer.servo_left_dir  * mixer.servo_left_scale;
    float right_factor = (float)mixer.servo_right_dir * mixer.servo_right_scale;

    if (fabsf(left_factor) < 1e-4f || fabsf(right_factor) < 1e-4f) {
      return;
    }

    float raw_d_left  = (req_pitch_us - req_roll_us) * left_factor;
    float raw_d_right = (req_pitch_us + req_roll_us) * right_factor;

    // Batasi perubahan maksimal per sesi setelah pencampuran (maks 70 us pada servo mana pun)
    // dengan penskalaan proporsional agar rasio Pitch:Roll tetap terjaga
    const float max_step_us = 70.0f;
    float max_abs_d = fmaxf(fabsf(raw_d_left), fabsf(raw_d_right));
    if (max_abs_d > max_step_us && max_abs_d > 0.0f) {
      float scale = max_step_us / max_abs_d;
      raw_d_left  *= scale;
      raw_d_right *= scale;
    }

    int16_t old_left_trim  = mixer.servo_left_trim_us;
    int16_t old_right_trim = mixer.servo_right_trim_us;

    int16_t new_left_trim  = (int16_t)constrain((int32_t)old_left_trim  + (int32_t)lroundf(raw_d_left),  -200, 200);
    int16_t new_right_trim = (int16_t)constrain((int32_t)old_right_trim + (int32_t)lroundf(raw_d_right), -200, 200);

    int16_t actual_d_left  = new_left_trim  - old_left_trim;
    int16_t actual_d_right = new_right_trim - old_right_trim;

    // Inversi kembali perubahan aktual integer servo ke domain kompensasi PID (Roll & Pitch)
    float dm_left  = (float)actual_d_left  / left_factor;
    float dm_right = (float)actual_d_right / right_factor;

    float applied_pitch_us = 0.5f * (dm_right + dm_left);
    float applied_roll_us  = 0.5f * (dm_right - dm_left);

    // Verifikasi akhir bahwa integral baru & output baru tidak menyentuh batas jepit (0 loss)
    float new_i_roll    = cur_i_roll    - applied_roll_us;
    float new_i_pitch   = cur_i_pitch   - applied_pitch_us;
    float new_cmd_roll  = cur_cmd_roll  - applied_roll_us;
    float new_cmd_pitch = cur_cmd_pitch - applied_pitch_us;

    if (fabsf(new_i_roll) > pid.roll_pid.max_i || fabsf(new_i_pitch) > pid.pitch_pid.max_i ||
        fabsf(new_cmd_roll) > 400.0f || fabsf(new_cmd_pitch) > 400.0f) {
      Serial.println(F(">>> [AUTOTRIM] Dibatalkan: Transfer melebihi ruang bebas integral/output PID."));
      return;
    }

    // Terapkan trim baru ke RAM dan jadwalkan penyimpanan Flash saat Disarm
    mixer.servo_left_trim_us  = new_left_trim;
    mixer.servo_right_trim_us = new_right_trim;
    settings.data.servo_left_trim_us  = new_left_trim;
    settings.data.servo_right_trim_us = new_right_trim;
    if (actual_d_left != 0 || actual_d_right != 0) {
      pending_flash_save = true;
    }

    // Bumpless Transfer: perbarui integral & output PID tanpa menyentuh state derivative (prev_meas_dps & d_lpf)
    pid.roll_pid.integral  = new_i_roll;
    pid.pitch_pid.integral = new_i_pitch;
    pid.roll_cmd_us        = new_cmd_roll;
    pid.pitch_cmd_us       = new_cmd_pitch;

    is_completed = true;

    Serial.println(F("\n========================================================"));
    Serial.println(F("  >>> [AUTOTRIM SUKSES] DITERAPKAN KE RAM (BUMPLESS) <<<"));
    Serial.print(F("  Sampel Jendela  : ")); Serial.println(used_samples);
    Serial.print(F("  Kompensasi PID  : Roll = ")); Serial.print(applied_roll_us, 1);
    Serial.print(F(" us | Pitch = ")); Serial.print(applied_pitch_us, 1); Serial.println(F(" us"));
    Serial.print(F("  Trim Servo Kiri : ")); Serial.print(mixer.servo_left_trim_us);
    Serial.print(F(" us (Delta: ")); Serial.print(actual_d_left); Serial.println(F(" us)"));
    Serial.print(F("  Trim Servo Kanan: ")); Serial.print(mixer.servo_right_trim_us);
    Serial.print(F(" us (Delta: ")); Serial.print(actual_d_right); Serial.println(F(" us)"));
    Serial.println(F("  [INFO] Akan disimpan ke Flash EEPROM otomatis saat DISARM."));
    Serial.println(F("========================================================\n"));
  }
};

#endif // AUTOTRIM_H
