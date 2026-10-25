/*
 * ============================================================================
 * @file        flying_wing_fc.ino
 * @project     Flying Wing Autonomous Flight Controller (2 Elevon + 1 Motor)
 * @version     v0.4.3
 * @since       v0.1.0 (2026-10-18)
 * @updated     2026-10-25
 * @brief       Program Utama Loop Kendali Real-Time 500 Hz & Orkestrasi Modul
 *
 * Mendukung Multi-Board (Otomatis mendeteksi saat Compile):
 *   1. Waveshare RP2040-Zero / Raspberry Pi Pico (RP2040 / RP2350)
 *   2. Omnibus F4 V5 (STM32F405) & STM32F411CEU6 "BlackPill"
 *   3. ESP32 / ESP32-S3
 *   4. Teensy 4.0 / 4.1
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.1.0 (2026-10-18): Arsitektur awal loop 500 Hz, Madgwick 6-DOF, Cascade PID, Mixer, CLI.
 *   - v0.2.0 (2026-10-19): Integrasi EEPROM Settings, WS2812 RGB LED RP2040-Zero, Maiden Flight.
 *   - v0.3.0 (2026-10-24): Hardening keselamatan (IMU fallback, RTH failsafe lock, Board Rotation).
 *   - v0.4.0 (2026-10-24): Integrasi AutoTrimManager (Servo AutoTrim via SwB CH8 + LED indikator).
 *   - v0.4.1 (2026-10-24): Integrasi Auto-Level Pitch Trim dengan Barometer & Auto-Baud GPS.
 *   - v0.4.3 (2026-10-25): Integrasi BeeperManager non-blocking di GP15 (Boot, GPS Lock, Arm, RTH).
 * ============================================================================
 */

#include <Arduino.h>
#include "config.h"
#include "settings.h"
#include "imu.h"
#include "gps.h"
#include "baro.h"
#include "rc_input.h"
#include "pid.h"
#include "mixer.h"
#include "autotrim.h"
#include "beeper.h"
#include "telemetry_cli.h"

#if defined(STM32F4xx) || defined(ARDUINO_ARCH_STM32)
  Uart Serial1(PA10, PA9);  // UART1: RX1 = PA10, TX1 = PA9
  Uart Serial3(PB11, PB10); // UART3: RX3 = PB11, TX3 = PB10
  Uart Serial6(PC7,  PC6);  // UART6: RX6/SBUS = PC7, TX6 = PC6
#elif defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
  #include <Adafruit_NeoPixel.h>
  static Adafruit_NeoPixel rgb_led(1, 16, NEO_GRB + NEO_KHZ800);
#endif

SettingsManager     settings;
IMUManager          imu;
RCInputManager      rc;
FlightControllerPID pid;
ElevonMixer         mixer;
AutoTrimManager     autotrim;
GPSManager          gps;
BaroManager         baro;
BeeperManager       beeper;
TelemetryCLI        cli;

uint32_t prev_loop_us  = 0;
uint32_t last_blink_ms = 0;
bool     led_state     = false;

void setStatusLED(bool on) {
  #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
    if (!on) {
      rgb_led.setPixelColor(0, 0, 0, 0); // Off (saat kedip)
    } else if (autotrim.is_active) {
      rgb_led.setPixelColor(0, 60, 40, 0); // Kuning/Amber = SERVO AUTOTRIM AKTIF
    } else if (rc.state.armed) {
      rgb_led.setPixelColor(0, 50, 0, 0); // Merah = ARMED (Motor Siap Putar)
    } else if (!rc.state.failsafe) {
      rgb_led.setPixelColor(0, 0, 50, 0); // Hijau = Remote Terkoneksi OK
    } else {
      rgb_led.setPixelColor(0, 0, 20, 50); // Biru/Cyan = Standby (Mencari Sinyal)
    }
    rgb_led.show();
  #else
    digitalWrite(PIN_STATUS_LED, on ? LED_ON_STATE : !LED_ON_STATE);
  #endif
}

void setup() {
  #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
    rgb_led.begin();
    rgb_led.setBrightness(40);
    rgb_led.setPixelColor(0, 0, 30, 50); // Nyala Biru/Cyan tanda power ON
    rgb_led.show();
  #else
    pinMode(PIN_STATUS_LED, OUTPUT);
    setStatusLED(true);
  #endif

  Serial.begin(115200);
  delay(600); // Beri waktu USB CDC siap

  // 0. Muat konfigurasi tersimpan dari EEPROM Flash (jika ada)
  bool has_saved = settings.load();
  mixer.servo_left_dir      = settings.data.servo_left_dir;
  mixer.servo_right_dir     = settings.data.servo_right_dir;
  mixer.servo_left_trim_us  = settings.data.servo_left_trim_us;
  mixer.servo_right_trim_us = settings.data.servo_right_trim_us;
  imu.state.roll_trim_deg   = settings.data.roll_trim_deg;
  imu.state.pitch_trim_deg  = settings.data.pitch_trim_deg;
  pid.roll_pid.kp           = settings.data.roll_kp;
  pid.pitch_pid.kp          = settings.data.pitch_kp;
  pid.angle_kp_roll         = settings.data.angle_kp;
  pid.angle_kp_pitch        = settings.data.angle_kp;

  // 1. Inisialisasi Output Servo Elevon, Motor ESC (dikunci 1000 us), & Beeper GP15
  mixer.begin();
  beeper.begin();

  // 2. Inisialisasi Receiver RC (sesuai protokol di EEPROM) & Modul GPS
  rc.begin(settings.data.rc_protocol);
  gps.begin();

  // 3. Inisialisasi Sensor Gyro/Accelerometer (MPU6000 onboard via SPI / MPU6050 I2C)
  imu.begin();

  #if defined(HAS_BAROMETER) && (HAS_BAROMETER == 1)
    baro.begin();
  #endif

  // 4. Tampilkan panduan di Serial Monitor & bunyikan nada Boot Ready
  if (has_saved) {
    Serial.println(F("\n[EEPROM] Konfigurasi tersimpan berhasil dimuat dari Flash!"));
  }
  cli.printBanner(imu, baro, settings);
  beeper.playBoot();

  prev_loop_us = micros();
}

void loop() {
  uint32_t now_us = micros();
  uint32_t dt_us  = now_us - prev_loop_us;

  // Jaga agar loop kendali berjalan tepat pada frekuensi 500 Hz (setiap 2000 us)
  if (dt_us < LOOP_INTERVAL_US) {
    return;
  }
  prev_loop_us = now_us;
  float dt = (float)dt_us * 1e-6f;

  // Pembatas dt jika terjadi jeda panjang (misal saat barusan kalibrasi ulang)
  if (dt > 0.05f) {
    dt = (float)LOOP_INTERVAL_US * 1e-6f;
  }

  // 1. Baca data sensor GPS dan Barometer
  gps.update(rc.state.armed);
  #if defined(HAS_BAROMETER) && (HAS_BAROMETER == 1)
    baro.update();
  #endif

  // 2. Baca sinyal Receiver RC di semua UART (UART1, UART6/SBUS, UART3) & teruskan NMEA GPS
  rc.update(gps);

  // 3. Baca sensor IMU (Gyro + Accel) & hitung sudut Roll/Pitch dengan filter Madgwick
  imu.update(dt);

  // 4. Hitung kendali PID (Manual / Acro / Angle Auto-Level / Alt-Hold / RTH)
  pid.update(rc.state, imu.state, gps.state, baro.state, dt);

  // 5. Update Servo AutoTrim & Auto-Level Trim (INAV Style via Switch B)
  autotrim.update(rc.state, imu.state, baro.state, pid, mixer, settings);

  // 6. Campurkan (Mix) sinyal Pitch & Roll ke 2 Servo Elevon + 1 ESC Motor Brushless
  mixer.update(pid.roll_cmd_us, pid.pitch_cmd_us, rc.state);

  // 7. Update Beeper Indikator Non-Blocking (GP15) & CLI Serial Monitor
  beeper.update(rc.state, gps.state, autotrim);
  cli.update(imu, rc, pid, mixer, autotrim, gps, baro, beeper, settings, dt_us);

  // 8. Update Indikator LED:
  //    - Berkedip cepat (100 ms) Kuning = AUTOTRIM AKTIF
  //    - Menyala terus (Solid) = ARMED (Motor siap berputar)
  //    - Berkedip normal (250 ms) = DISARMED / Standby
  uint32_t blink_rate_ms = autotrim.is_active ? 100 : 250;
  if (rc.state.armed && !autotrim.is_active) {
    setStatusLED(true);
  } else if (millis() - last_blink_ms > blink_rate_ms) {
    last_blink_ms = millis();
    led_state = !led_state;
    setStatusLED(led_state);
  }
}
