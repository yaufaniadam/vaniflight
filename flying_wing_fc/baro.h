/*
 * ============================================================================
 * @file        baro.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.1
 * @since       v0.2.0 (2026-10-20)
 * @updated     2026-10-24
 * @brief       Driver Barometer I2C (BMP280 / BME280), Estimasi AGL, & Laju Naik/Turun
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.2.0 (2026-10-20): Driver BMP280/BME280 auto-address (0x76/0x77), kalibrasi QFE
 *                          darat (0.0m AGL), & kompensasi suhu/tekanan Bosch.
 *   - v0.4.1 (2026-10-24): Deteksi kegagalan baca I2C/tekanan tidak valid (fail_count_ >= 3
 *                          -> connected = false) untuk keamanan Auto-Level & Alt-Hold.
 * ============================================================================
 */
#ifndef BARO_H
#define BARO_H

#include <Arduino.h>
#include <Wire.h>

struct BaroState {
  bool    connected       = false;
  uint8_t i2c_addr        = 0x76;
  float   temperature_c   = 25.0f;
  float   pressure_pa     = 101325.0f;
  float   ground_press_pa = 101325.0f;
  float   altitude_agl_m  = 0.0f; // Ketinggian relatif dari titik nyala (meter di atas tanah)
  float   climb_rate_mps  = 0.0f; // Kecepatan naik/turun vertikal (m/s)
};

class BaroManager {
public:
  BaroState state;

  bool begin() {
    #if !defined(HAS_BAROMETER) || (HAS_BAROMETER == 0)
      state.connected = false;
      return false;
    #else
      // Deteksi otomatis alamat I2C GY-BMP280 (0x76 atau 0x77)
      if (probeAddress(0x76)) {
        state.i2c_addr = 0x76;
      } else if (probeAddress(0x77)) {
        state.i2c_addr = 0x77;
      } else {
        state.connected = false;
        return false;
      }

    // Baca parameter kalibrasi pabrik dari memori internal BMP280 (Register 0x88..0x9F)
    readCalibrationData();

    // Konfigurasi BMP280:
    // Register 0xF5 (config): Standby 0.5ms, IIR Filter coeff 16 (meredam turbulensi angin di pesawat)
    writeReg(0xF5, 0x10);
    // Register 0xF4 (ctrl_meas): Temp oversampling x2, Pressure oversampling x16, Normal mode
    writeReg(0xF4, 0x57);

    delay(60);
    state.connected = true;
    calibrateGroundAltitude();
    return true;
    #endif
  }

  // Simpan tekanan udara di darat sebagai titik 0.0 meter
  void calibrateGroundAltitude() {
    if (!state.connected) return;
    float sum_p = 0.0f;
    uint8_t valid = 0;
    for (uint8_t i = 0; i < 20; i++) {
      if (readPressureAndTemp()) {
        sum_p += state.pressure_pa;
        valid++;
      }
      delay(10);
    }
    if (valid > 0) {
      state.ground_press_pa = sum_p / (float)valid;
      state.altitude_agl_m  = 0.0f;
      prev_alt_m_           = 0.0f;
    }
  }

  // Update pembacaan barometer pada ~50 Hz agar tidak membebani bus I2C IMU
  void update() {
    if (!state.connected) return;
    uint32_t now_ms = millis();
    if (now_ms - last_read_ms_ < 20) return; // 50 Hz
    float dt = (now_ms - last_read_ms_) * 0.001f;
    last_read_ms_ = now_ms;

    if (readPressureAndTemp() && state.ground_press_pa > 1000.0f) {
      fail_count_ = 0;
      // Rumus barometrik internasional (meter di atas titik lepas landas)
      float raw_alt_m = 44330.0f * (1.0f - powf(state.pressure_pa / state.ground_press_pa, 0.190295f));
      if (!isnan(raw_alt_m)) {
        // Low-pass filter ketinggian
        state.altitude_agl_m = 0.85f * state.altitude_agl_m + 0.15f * raw_alt_m;

        if (dt > 0.005f && dt < 0.5f) {
          float raw_vz = (state.altitude_agl_m - prev_alt_m_) / dt;
          state.climb_rate_mps = 0.8f * state.climb_rate_mps + 0.2f * raw_vz;
        }
        prev_alt_m_ = state.altitude_agl_m;
      }
    } else {
      if (++fail_count_ >= 3) {
        state.connected      = false;
        state.climb_rate_mps = 0.0f;
      }
    }
  }

private:
  uint8_t  fail_count_   = 0;
  uint16_t dig_T1_ = 0;
  int16_t  dig_T2_ = 0, dig_T3_ = 0;
  uint16_t dig_P1_ = 0;
  int16_t  dig_P2_ = 0, dig_P3_ = 0, dig_P4_ = 0, dig_P5_ = 0;
  int16_t  dig_P6_ = 0, dig_P7_ = 0, dig_P8_ = 0, dig_P9_ = 0;

  uint32_t last_read_ms_ = 0;
  float    prev_alt_m_   = 0.0f;

  bool probeAddress(uint8_t addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) return false;
    Wire.beginTransmission(addr);
    Wire.write(0xD0); // Register CHIP_ID (0x58 = BMP280, 0x60 = BME280)
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, (uint8_t)1) < 1) return false;
    uint8_t id = Wire.read();
    return (id == 0x58 || id == 0x60 || id == 0x56 || id == 0x57);
  }

  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(state.i2c_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  uint16_t read16LE(uint8_t reg) {
    Wire.beginTransmission(state.i2c_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(state.i2c_addr, (uint8_t)2);
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    return (uint16_t)((msb << 8) | lsb);
  }

  void readCalibrationData() {
    dig_T1_ = read16LE(0x88);
    dig_T2_ = (int16_t)read16LE(0x8A);
    dig_T3_ = (int16_t)read16LE(0x8C);
    dig_P1_ = read16LE(0x8E);
    dig_P2_ = (int16_t)read16LE(0x90);
    dig_P3_ = (int16_t)read16LE(0x92);
    dig_P4_ = (int16_t)read16LE(0x94);
    dig_P5_ = (int16_t)read16LE(0x96);
    dig_P6_ = (int16_t)read16LE(0x98);
    dig_P7_ = (int16_t)read16LE(0x9A);
    dig_P8_ = (int16_t)read16LE(0x9C);
    dig_P9_ = (int16_t)read16LE(0x9E);
  }

  bool readPressureAndTemp() {
    Wire.beginTransmission(state.i2c_addr);
    Wire.write(0xF7); // Mulai dari PRESS_MSB
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(state.i2c_addr, (uint8_t)6) < 6) return false;

    int32_t adc_P = ((int32_t)Wire.read() << 12) | ((int32_t)Wire.read() << 4) | (Wire.read() >> 4);
    int32_t adc_T = ((int32_t)Wire.read() << 12) | ((int32_t)Wire.read() << 4) | (Wire.read() >> 4);

    // Kompensasi Suhu & Tekanan standar Bosch BMP280
    float var1 = (((float)adc_T) / 16384.0f - ((float)dig_T1_) / 1024.0f) * ((float)dig_T2_);
    float var2 = ((((float)adc_T) / 131072.0f - ((float)dig_T1_) / 8192.0f) *
                  (((float)adc_T) / 131072.0f - ((float)dig_T1_) / 8192.0f)) * ((float)dig_T3_);
    float t_fine = var1 + var2;
    state.temperature_c = t_fine / 5120.0f;

    var1 = (t_fine / 2.0f) - 64000.0f;
    var2 = var1 * var1 * ((float)dig_P6_) / 32768.0f;
    var2 = var2 + var1 * ((float)dig_P5_) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)dig_P4_) * 65536.0f);
    var1 = (((float)dig_P3_) * var1 * var1 / 524288.0f + ((float)dig_P2_) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)dig_P1_);
    if (fabsf(var1) < 1e-6f) return false;

    float p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)dig_P9_) * p * p / 2147483648.0f;
    var2 = p * ((float)dig_P8_) / 32768.0f;
    float p_final = p + (var1 + var2 + ((float)dig_P7_)) / 16.0f;
    if (isnan(p_final) || p_final < 10000.0f || p_final > 130000.0f) {
      return false;
    }
    state.pressure_pa = p_final;
    return true;
  }
};

#endif // BARO_H
