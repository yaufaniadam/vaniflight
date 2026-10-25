/*
 * ============================================================================
 * @file        beeper.h
 * @project     Flying Wing Autonomous Flight Controller (VaniFlight)
 * @version     v0.4.3
 * @since       v0.4.3 (2026-10-25)
 * @updated     2026-10-25
 * @brief       Non-Blocking Beeper / Buzzer State Machine (Active & Passive 2-Pin)
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.4.3 (2026-10-25): Implementasi state machine beeper non-blocking di GP15
 *                          (Boot, GPS Home Lock, Arm/Disarm, Throttle Warning,
 *                          AutoTrim, & RTH/Lost Model Beacon) menggunakan SIO GPIO
 *                          murni di dalam loop 500 Hz (0 timer/interrupt, 0 konflik PWM).
 * ============================================================================
 */
#ifndef BEEPER_H
#define BEEPER_H

#include <Arduino.h>
#include "config.h"
#include "rc_input.h"
#include "gps.h"
#include "baro.h"
#include "autotrim.h"

#if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
  #include <hardware/gpio.h>
#endif

enum BeeperDriveMode : uint8_t {
  BEEPER_MODE_ACTIVE  = 0, // Default: DC HIGH (Buzzer 2-kaki Active paling nyaring!)
  BEEPER_MODE_HYBRID  = 1, // Kombinasi DC HIGH + Modulasi Loop 500 Hz
  BEEPER_MODE_PASSIVE = 2  // Modulasi gelombang kotak dari Loop 500 Hz
};

struct BeepStep {
  uint16_t on_ms;
  uint16_t off_ms;
};

class BeeperManager {
public:
  BeeperDriveMode drive_mode = BEEPER_MODE_ACTIVE;

  void begin() {
    pinMode(PIN_BEEPER, OUTPUT);
    digitalWrite(PIN_BEEPER, LOW);

    #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
      // Pastikan GP15 murni memakai fungsi SIO (tidak pernah menyentuh PWM Slice 7 milik GP14 ESC!)
      // dan set arus dorong (drive strength) silikon RP2040 ke maksimal 12 mA agar buzzer nyaring.
      gpio_set_function(PIN_BEEPER, GPIO_FUNC_SIO);
      gpio_set_dir(PIN_BEEPER, GPIO_OUT);
      gpio_set_drive_strength(PIN_BEEPER, GPIO_DRIVE_STRENGTH_12MA);
      gpio_put(PIN_BEEPER, false);
    #endif
  }

  // Bunyi saat FC selesai inisialisasi & siap terbang (Tit - Tit - Tiiit!)
  void playBoot() {
    static const BeepStep seq[] = { {80, 60}, {80, 60}, {180, 100} };
    startSequence(seq, 3);
  }

  // Bunyi saat GPS M10 berhasil mengunci >= 5 satelit & menyimpan titik Home (3x cepat gembira!)
  void playGPSHomeLocked() {
    static const BeepStep seq[] = { {70, 50}, {70, 50}, {70, 50}, {220, 100} };
    startSequence(seq, 4);
  }

  // Bunyi saat Motor ARMED (1x panjang tegas peringatan baling-baling aktif)
  void playArmed() {
    static const BeepStep seq[] = { {400, 100} };
    startSequence(seq, 1);
  }

  // Bunyi saat Motor DISARMED (2x pendek)
  void playDisarmed() {
    static const BeepStep seq[] = { {100, 80}, {120, 100} };
    startSequence(seq, 2);
  }

  // Bunyi saat kalibrasi [c], AutoTrim [SwD], atau simpan Flash [w] berhasil
  void playSuccess() {
    static const BeepStep seq[] = { {80, 60}, {160, 80} };
    startSequence(seq, 2);
  }

  // Bunyi saat kalibrasi gagal / peringatan error
  void playError() {
    static const BeepStep seq[] = { {250, 100}, {250, 100} };
    startSequence(seq, 2);
  }

  // Ganti mode driver buzzer lewat CLI [v] (Active DC -> Hybrid -> Passive)
  void cycleDriveMode() {
    if (drive_mode == BEEPER_MODE_ACTIVE) drive_mode = BEEPER_MODE_HYBRID;
    else if (drive_mode == BEEPER_MODE_HYBRID) drive_mode = BEEPER_MODE_PASSIVE;
    else drive_mode = BEEPER_MODE_ACTIVE;
    playBoot();
  }

  const char* getDriveModeName() const {
    switch (drive_mode) {
      case BEEPER_MODE_HYBRID:  return "HYBRID (DC + Pulse)";
      case BEEPER_MODE_PASSIVE: return "PASSIVE (Pulse Wave)";
      default:                  return "ACTIVE (DC HIGH)";
    }
  }

  void update(const RCState &rc, const GPSState &gps, const AutoTrimManager &autotrim) {
    uint32_t now = millis();

    // 1. Deteksi transisi GPS Home Locked pertama kali
    if (gps.home_set && !prev_home_set_) {
      playGPSHomeLocked();
    }
    prev_home_set_ = gps.home_set;

    // 2. Deteksi transisi Arm / Disarm
    if (rc.armed && !prev_armed_) {
      playArmed();
    } else if (!rc.armed && prev_armed_) {
      playDisarmed();
    }
    prev_armed_ = rc.armed;

    // 3. Deteksi transisi AutoTrim / Auto-Level
    if (autotrim.is_active && !prev_autotrim_active_) {
      static const BeepStep chirp[] = { {60, 40} };
      startSequence(chirp, 1);
    }
    if (autotrim.is_completed && !prev_autotrim_done_) {
      playSuccess();
    }
    prev_autotrim_active_ = autotrim.is_active;
    prev_autotrim_done_   = autotrim.is_completed;

    // 4. Peringatan jika saklar Arm (SwB) dinyalakan tapi Throttle belum di bawah!
    bool arm_switch_on = !rc.failsafe && (rc.arm_us > 1600);
    if (arm_switch_on && !rc.armed && (rc.throttle_us >= ARM_SAFETY_MAX_THR_US)) {
      if (!playing_ && (now - last_warn_ms_ > 600)) {
        last_warn_ms_ = now;
        static const BeepStep warn_seq[] = { {50, 50}, {50, 50}, {50, 50} };
        startSequence(warn_seq, 3);
      }
    }

    // 5. Beacon RTH / Lost Model Finder (Bunyi periodik saat mode RTH aktif)
    if (rc.mode == MODE_RTH && !playing_ && (now - last_beacon_ms_ > 2000)) {
      last_beacon_ms_ = now;
      static const BeepStep beacon_seq[] = { {120, 80}, {120, 80} };
      startSequence(beacon_seq, 2);
    }

    // 6. Jalankan State Machine Non-Blocking
    serviceSequence(now);
  }

private:
  static constexpr uint8_t MAX_STEPS = 6;
  BeepStep steps_[MAX_STEPS];
  uint8_t  step_count_           = 0;
  uint8_t  step_idx_             = 0;
  bool     in_on_phase_          = false;
  bool     playing_              = false;
  bool     toggle_state_         = false;
  uint32_t phase_start_ms_       = 0;
  uint32_t last_warn_ms_         = 0;
  uint32_t last_beacon_ms_       = 0;

  bool     prev_home_set_        = false;
  bool     prev_armed_           = false;
  bool     prev_autotrim_active_ = false;
  bool     prev_autotrim_done_   = false;

  void startSequence(const BeepStep *seq, uint8_t count) {
    if (count == 0 || count > MAX_STEPS) return;
    for (uint8_t i = 0; i < count; i++) {
      steps_[i] = seq[i];
    }
    step_count_     = count;
    step_idx_       = 0;
    in_on_phase_    = true;
    playing_        = true;
    phase_start_ms_ = millis();
    setOutput(true, 0, steps_[0].on_ms);
  }

  void serviceSequence(uint32_t now) {
    if (!playing_) {
      setOutput(false, 0, 0);
      return;
    }

    uint32_t elapsed = now - phase_start_ms_;
    const BeepStep &cur = steps_[step_idx_];

    if (in_on_phase_) {
      if (elapsed >= cur.on_ms) {
        in_on_phase_    = false;
        phase_start_ms_ = now;
        setOutput(false, 0, 0);
      } else {
        setOutput(true, elapsed, cur.on_ms);
      }
    } else {
      if (elapsed >= cur.off_ms) {
        step_idx_++;
        if (step_idx_ >= step_count_) {
          playing_ = false;
          setOutput(false, 0, 0);
        } else {
          in_on_phase_    = true;
          phase_start_ms_ = now;
          setOutput(true, 0, steps_[step_idx_].on_ms);
        }
      }
    }
  }

  void setOutput(bool active, uint32_t elapsed_ms, uint16_t total_on_ms) {
    if (!active) {
      toggle_state_ = false;
      #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
        gpio_put(PIN_BEEPER, false);
      #else
        digitalWrite(PIN_BEEPER, LOW);
      #endif
      return;
    }

    bool pin_high = true;
    if (drive_mode == BEEPER_MODE_PASSIVE) {
      toggle_state_ = !toggle_state_;
      pin_high = toggle_state_;
    } else if (drive_mode == BEEPER_MODE_HYBRID && elapsed_ms >= (uint32_t)(total_on_ms / 2)) {
      toggle_state_ = !toggle_state_;
      pin_high = toggle_state_;
    }

    #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
      gpio_put(PIN_BEEPER, pin_high);
    #else
      digitalWrite(PIN_BEEPER, pin_high ? HIGH : LOW);
    #endif
  }
};

#endif // BEEPER_H
