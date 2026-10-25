/*
 * ============================================================================
 * @file        pid.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.2
 * @since       v0.1.0 (2026-10-18)
 * @updated     2026-10-25
 * @brief       Kendali Cascade PID (Angle -> Rate -> Servo) + Navigasi AltHold & RTH
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.1.0 (2026-10-18): Cascade PID (Outer Angle P + Inner Rate PID, Derivative on
 *                          Measurement, LPF D-term, & Anti-Windup).
 *   - v0.2.0 (2026-10-20): Tambah Coordinated Turn Pitch Assist, Mode ALT_HOLD, & RTH.
 *   - v0.3.0 (2026-10-24): Fallback otomatis ke Manual Passthrough saat IMU terputus.
 *   - v0.4.1 (2026-10-24): Fallback aman ke Mode ANGLE jika Barometer terputus saat ALT_HOLD/RTH.
 *   - v0.4.2 (2026-10-25): Tambah Coordinated Turn Assist & backup GPS MSL pada RTH & ALT_HOLD.
 * ============================================================================
 */
#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "config.h"
#include "imu.h"
#include "rc_input.h"
#include "gps.h"
#include "baro.h"

struct PIDAxis {
  float kp;
  float ki;
  float kd;
  float max_i;

  float integral      = 0.0f;
  float prev_meas_dps = 0.0f;
  float d_lpf         = 0.0f;

  void reset() {
    integral      = 0.0f;
    prev_meas_dps = 0.0f;
    d_lpf         = 0.0f;
  }

  // Menghitung sinyal koreksi (dalam satuan mikrodetik servo, -400..+400 us)
  float compute(float target_rate_dps, float meas_rate_dps, float dt, bool allow_integral) {
    float error = target_rate_dps - meas_rate_dps;

    // 1. Komponen Proportional (P)
    float p_out = kp * error;

    // 2. Komponen Integral (I) dengan Anti-Windup
    //    Integral hanya dijumlahkan saat pesawat sudah terbang (throttle > 15% atau mode RTH aktif)
    //    agar servo tidak mentok miring sendiri saat pesawat masih diam di tanah!
    if (allow_integral && dt > 0.0f) {
      integral += ki * error * dt;
      integral = constrain(integral, -max_i, max_i);
    } else {
      integral *= 0.95f; // Luruhkan perlahan jika di darat
    }

    // 3. Komponen Derivative (D) berdasarkan perubahan Gyro (Derivative on Measurement)
    //    Mencegah servo menyentak kasar ("Derivative Kick") saat stik remot digerakkan cepat
    float d_raw = (dt > 0.0001f) ? -((meas_rate_dps - prev_meas_dps) / dt) : 0.0f;
    prev_meas_dps = meas_rate_dps;
    d_lpf = 0.7f * d_lpf + 0.3f * d_raw; // Filter penghalus komponen D
    float d_out = kd * d_lpf;

    return constrain(p_out + integral + d_out, -400.0f, 400.0f);
  }
};

class FlightControllerPID {
public:
  PIDAxis roll_pid  = {PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  PID_ROLL_MAX_I};
  PIDAxis pitch_pid = {PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_MAX_I};

  float angle_kp_roll  = ANGLE_KP_ROLL;
  float angle_kp_pitch = ANGLE_KP_PITCH;

  // Target ketinggian saat mode Altitude Hold diaktifkan
  float locked_alt_target_m    = 0.0f;
  FlightMode prev_mode_        = MODE_MANUAL;
  bool prev_alt_hold_active_   = false;

  // Hasil akhir perintah kontrol Roll & Pitch (-400..+400 us) untuk dikirim ke Elevon Mixer
  float roll_cmd_us  = 0.0f;
  float pitch_cmd_us = 0.0f;

  void reset() {
    roll_pid.reset();
    pitch_pid.reset();
    roll_cmd_us  = 0.0f;
    pitch_cmd_us = 0.0f;
  }

  void update(const RCState &rc, const IMUState &imu, const GPSState &gps, const BaroState &baro, float dt) {
    // Kunci ketinggian aktual saat ini ketika pilot baru saja masuk ke Mode ALT_HOLD dengan Barometer aktif
    bool alt_hold_active = (rc.mode == MODE_ALT_HOLD && baro.connected);
    if (alt_hold_active && !prev_alt_hold_active_) {
      locked_alt_target_m = baro.altitude_agl_m;
    }
    prev_alt_hold_active_ = alt_hold_active;
    prev_mode_            = rc.mode;

    // FAILSAFE SENSOR: Jika sensor IMU putus/rusak/kabel kendor di udara,
    // langsung fallback ke MANUAL PASSTHROUGH agar servo tidak mengunci miring
    // dan pilot tetap memiliki kendali aerodinamis langsung dari stik!
    if (!imu.connected || rc.mode == MODE_MANUAL) {
      roll_pid.reset();
      pitch_pid.reset();
      roll_cmd_us  = rc.roll_norm  * 400.0f;
      pitch_cmd_us = rc.pitch_norm * 400.0f;
      return;
    }

    float target_roll_rate_dps  = 0.0f;
    float target_pitch_rate_dps = 0.0f;

    if (rc.mode == MODE_RTH) {
      // ======================================================================
      // MODE RTH: Navigasi Otonom Menuju Home (GPS + Barometer)
      // ======================================================================
      float target_roll_deg  = 15.0f; // Default circling jika posisi/course belum valid
      float target_pitch_deg = 2.0f;

      if (gps.fix_valid && gps.course_valid && gps.home_set) {
        // Sudut miring dihitung dari arah menuju Home (dibatasi +-30 derajat)
        target_roll_deg = gps.rth_roll_cmd_deg;
      }

      // Ketinggian dijaga pada 40 meter AGL oleh Barometer BMP280 (atau backup GPS MSL)
      if (baro.connected) {
        float alt_err = RTH_TARGET_ALTITUDE_M - baro.altitude_agl_m;
        target_pitch_deg = constrain(alt_err * ALT_HOLD_KP, -10.0f, MAX_NAV_PITCH_DEG);
      } else if (gps.fix_valid && gps.home_set) {
        target_pitch_deg = gps.rth_pitch_cmd_deg;
      }

      // Coordinated Turn Assist saat berbelok pulang / loiter agar hidung tidak turun
      float roll_rad = imu.roll_deg * DEG_TO_RAD;
      float turn_pitch_comp = (1.0f - cosf(roll_rad)) * 12.0f;
      target_pitch_deg += constrain(turn_pitch_comp, 0.0f, 10.0f);

      // Outer Loop: Hitung target laju putar gyro dari selisih sudut
      target_roll_rate_dps  = constrain((target_roll_deg  - imu.roll_deg)  * angle_kp_roll,
                                        -MAX_ROLL_RATE_DPS, MAX_ROLL_RATE_DPS);
      target_pitch_rate_dps = constrain((target_pitch_deg - imu.pitch_deg) * angle_kp_pitch,
                                        -MAX_PITCH_RATE_DPS, MAX_PITCH_RATE_DPS);

    } else if (rc.mode == MODE_ALT_HOLD && baro.connected) {
      // ======================================================================
      // MODE ALT HOLD: Roll dikontrol pilot, Ketinggian dijaga otomatis oleh Baro
      // ======================================================================
      float target_roll_deg = rc.roll_norm * MAX_ROLL_ANGLE_DEG;

      // Pilot bisa menggeser target ketinggian dari ketinggian aktual dengan stik pitch (+-4 m/detik)
      locked_alt_target_m = constrain(locked_alt_target_m + rc.pitch_norm * 4.0f * dt, -50.0f, 500.0f);
      float alt_err = locked_alt_target_m - baro.altitude_agl_m;
      float target_pitch_deg = constrain(alt_err * ALT_HOLD_KP, -MAX_NAV_PITCH_DEG, MAX_NAV_PITCH_DEG);

      // Coordinated Turn Assist saat miring belok di Mode ALT_HOLD
      float roll_rad = imu.roll_deg * DEG_TO_RAD;
      float turn_pitch_comp = (1.0f - cosf(roll_rad)) * 12.0f;
      target_pitch_deg += constrain(turn_pitch_comp, 0.0f, 10.0f);

      target_roll_rate_dps  = constrain((target_roll_deg  - imu.roll_deg)  * angle_kp_roll,
                                        -MAX_ROLL_RATE_DPS, MAX_ROLL_RATE_DPS);
      target_pitch_rate_dps = constrain((target_pitch_deg - imu.pitch_deg) * angle_kp_pitch,
                                        -MAX_PITCH_RATE_DPS, MAX_PITCH_RATE_DPS);

    } else if (rc.mode == MODE_ANGLE || rc.mode == MODE_ALT_HOLD) {
      // ======================================================================
      // MODE ANGLE (Auto-Level): Posisi stik menentukan target sudut kemiringan
      // ======================================================================
      float target_roll_deg  = rc.roll_norm * MAX_ROLL_ANGLE_DEG;
      float max_pitch_deg    = (rc.pitch_norm >= 0.0f) ? MAX_PITCH_UP_DEG : MAX_PITCH_DOWN_DEG;
      float target_pitch_deg = rc.pitch_norm * max_pitch_deg;

      // Coordinated Turn Assist: saat sayap miring belok, gaya angkat vertikal berkurang
      // sebesar (1 - cos(roll)). Tambahkan sedikit pitch up otomatis proporsional dengan kemiringan
      // agar hidung tidak turun dan sayap berbelok melingkar tajam tanpa radius melebar.
      float roll_rad = imu.roll_deg * DEG_TO_RAD;
      float turn_pitch_comp = (1.0f - cosf(roll_rad)) * 12.0f; // kompensasi hingga ~6 derajat saat miring 60 deg
      target_pitch_deg += constrain(turn_pitch_comp, 0.0f, 10.0f);

      target_roll_rate_dps  = constrain((target_roll_deg  - imu.roll_deg)  * angle_kp_roll,
                                        -MAX_ROLL_RATE_DPS, MAX_ROLL_RATE_DPS);
      target_pitch_rate_dps = constrain((target_pitch_deg - imu.pitch_deg) * angle_kp_pitch,
                                        -MAX_PITCH_RATE_DPS, MAX_PITCH_RATE_DPS);
    } else {
      // ======================================================================
      // MODE ACRO (Rate Mode): Posisi stik langsung menentukan kecepatan putar
      // ======================================================================
      target_roll_rate_dps  = rc.roll_norm  * MAX_ROLL_RATE_DPS;
      target_pitch_rate_dps = rc.pitch_norm * MAX_PITCH_RATE_DPS;
    }

    // Aktifkan Integral hanya saat motor Armed dan terbang (atau saat mode RTH)
    bool flying = rc.armed && (rc.throttle_norm > 0.15f || rc.mode == MODE_RTH);

    // INNER LOOP (Rate PID):
    roll_cmd_us  = roll_pid.compute(target_roll_rate_dps,   imu.gyro_roll_dps,  dt, flying);
    pitch_cmd_us = pitch_pid.compute(target_pitch_rate_dps, imu.gyro_pitch_dps, dt, flying);
  }
};

#endif // PID_H
