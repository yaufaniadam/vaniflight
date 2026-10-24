/*
 * ============================================================================
 * @file        imu.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.1
 * @since       v0.1.0 (2026-10-18)
 * @updated     2026-10-24
 * @brief       Driver IMU (MPU6050 I2C / MPU6000 SPI) & Filter Quaternion Madgwick 6-DOF
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.1.0 (2026-10-18): Driver awal MPU6050 I2C & algoritma AHRS Madgwick 6-DOF.
 *   - v0.2.0 (2026-10-19): Bus Recovery 9-clock I2C, reinitQuick(), & dukungan SPI MPU6000.
 *   - v0.3.0 (2026-10-24): Transformasi BOARD_ROTATION di readRawScaled() & penjaga
 *                          pembagian nol (anti-NaN guard) pada normalisasi Madgwick.
 *   - v0.4.1 (2026-10-24): Penyelarasan quaternion (q0..q3) langsung ke vektor gravitasi
 *                          saat calibrateGyro() serta validasi gerakan & batas +-15 deg
 *                          pada calibrateLevel().
 * ============================================================================
 */
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "config.h"

struct IMUState {
  bool connected = false;

  // Laju putar sudut terfilter (derajat/detik)
  float gyro_roll_dps  = 0.0f;
  float gyro_pitch_dps = 0.0f;
  float gyro_yaw_dps   = 0.0f;

  // Sudut orientasi hasil sensor fusion Madgwick AHRS (derajat)
  float roll_deg  = 0.0f;
  float pitch_deg = 0.0f;
  float yaw_deg   = 0.0f;

  // Offset kalibrasi titik rata (level trim)
  float roll_trim_deg  = 0.0f;
  float pitch_trim_deg = 0.0f;
};

class IMUManager {
public:
  IMUState state;

  bool begin() {
    #if defined(IMU_USE_SPI) && (IMU_USE_SPI == 1)
      // Inisialisasi SPI1 untuk sensor MPU6000 bawaan board Omnibus F4
      pinMode(PIN_SPI_CS, OUTPUT);
      digitalWrite(PIN_SPI_CS, HIGH);
      SPI.begin();
      delay(50);

      // 1. Device Reset
      writeRegSPI(0x6B, 0x80);
      delay(100);

      // 2. Nonaktifkan interface I2C dan aktifkan SPI (User Control reg 0x6A = 0x10)
      writeRegSPI(0x6A, 0x10);
      delay(10);

      // 3. Bangunkan dan set clock source PLL with X gyro
      writeRegSPI(0x6B, 0x01);
      delay(20);

      // 4. Hardware DLPF ~42 Hz (meredam getaran mesin)
      writeRegSPI(0x1A, 0x03);

      // 5. Gyroscope +-500 dps
      writeRegSPI(0x1B, 0x08);

      // 6. Accelerometer +-8g
      writeRegSPI(0x1C, 0x10);

      // Cek respon chip (WHO_AM_I reg 0x75)
      uint8_t who = readRegSPI(0x75);
      state.connected = (who != 0x00 && who != 0xFF);
      if (state.connected) {
        calibrateGyro(400);
        return true;
      }
      return false;

    #else
      // Jalur I2C untuk sensor eksternal (Pico, BlackPill, ESP32)
      #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
        if (!wire_initialized_) {
          // Bus Clear Routine: toggle SCL 9 kali untuk memaksa MPU6050 melepas jalur SDA jika nyangkut
          pinMode(PIN_I2C_SDA, INPUT_PULLUP);
          pinMode(PIN_I2C_SCL, OUTPUT);
          for (int i = 0; i < 9; i++) {
            digitalWrite(PIN_I2C_SCL, LOW);
            delayMicroseconds(5);
            digitalWrite(PIN_I2C_SCL, HIGH);
            delayMicroseconds(5);
          }
          pinMode(PIN_I2C_SCL, INPUT_PULLUP);
          delayMicroseconds(10);

          Wire.setSDA(PIN_I2C_SDA);
          Wire.setSCL(PIN_I2C_SCL);
          Wire.begin();
          Wire.setClock(100000); // 100 kHz standar aman & toleran kabel
          Wire.setTimeout(5, true);
          wire_initialized_ = true;
        }
      #elif defined(STM32F4xx) || defined(ARDUINO_ARCH_STM32)
        Wire.setSDA(PIN_I2C_SDA);
        Wire.setSCL(PIN_I2C_SCL);
        Wire.begin();
        Wire.setClock(400000);
      #elif defined(ESP32)
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        Wire.setClock(400000);
      #else
        Wire.begin();
        Wire.setClock(400000);
      #endif

      if (!probeAndWake()) {
        state.connected = false;
        return false;
      }

      delay(50);
      configureRegisters();

      state.connected = true;
      fail_count_ = 0;
      calibrateGyro(400);
      return true;
    #endif
  }

  // Re-inisialisasi kilat (<1 ms, tanpa delay/kalibrasi ulang) jika I2C glitch karena arus servo
  bool reinitQuick() {
    #if defined(IMU_USE_SPI) && (IMU_USE_SPI == 1)
      return begin();
    #else
      if (!probeAndWake()) {
        state.connected = false;
        return false;
      }
      configureRegisters();
      state.connected = true;
      fail_count_ = 0;
      return true;
    #endif
  }

  // Kalibrasi offset gyro saat pesawat diam di tanah & selaraskan quaternion awal ke vektor gravitasi
  bool calibrateGyro(uint16_t samples = 400) {
    if (!state.connected) return false;

    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float min_gx = 1e6f, max_gx = -1e6f;
    float min_gy = 1e6f, max_gy = -1e6f;
    float min_gz = 1e6f, max_gz = -1e6f;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < samples; i++) {
      float ax, ay, az, gx, gy, gz;
      if (readRawScaled(ax, ay, az, gx, gy, gz)) {
        if (!isnan(ax) && !isnan(ay) && !isnan(az) &&
            !isnan(gx) && !isnan(gy) && !isnan(gz)) {
          sum_gx += gx;
          sum_gy += gy;
          sum_gz += gz;
          sum_ax += ax;
          sum_ay += ay;
          sum_az += az;
          if (gx < min_gx) min_gx = gx;
          if (gx > max_gx) max_gx = gx;
          if (gy < min_gy) min_gy = gy;
          if (gy > max_gy) max_gy = gy;
          if (gz < min_gz) min_gz = gz;
          if (gz > max_gz) max_gz = gz;
          valid++;
        }
      }
      delay(2);
    }

    if (valid < (samples / 2) || valid == 0) {
      return false;
    }

    // Tolak kalibrasi jika pesawat terguncang / digerakkan selama pengambilan sampel (> 20 dps peak-to-peak)
    if ((max_gx - min_gx) > 20.0f || (max_gy - min_gy) > 20.0f || (max_gz - min_gz) > 20.0f) {
      return false;
    }

    gyro_bias_x_ = sum_gx / (float)valid;
    gyro_bias_y_ = sum_gy / (float)valid;
    gyro_bias_z_ = sum_gz / (float)valid;

    state.gyro_roll_dps  = 0.0f;
    state.gyro_pitch_dps = 0.0f;
    state.gyro_yaw_dps   = 0.0f;

    // Selaraskan quaternion Madgwick langsung ke rata-rata vektor gravitasi agar tidak ada jeda konvergensi
    float avg_ax = sum_ax / (float)valid;
    float avg_ay = sum_ay / (float)valid;
    float avg_az = sum_az / (float)valid;
    float acc_norm = sqrtf(avg_ax * avg_ax + avg_ay * avg_ay + avg_az * avg_az);
    if (acc_norm > 0.7f && acc_norm < 1.3f) {
      float nx = avg_ax / acc_norm;
      float ny = avg_ay / acc_norm;
      float nz = avg_az / acc_norm;

      float roll_rad  = atan2f(ny, nz);
      float pitch_rad = atan2f(-nx, sqrtf(ny * ny + nz * nz));
      float yaw_rad   = state.yaw_deg * DEG_TO_RAD;

      float cr = cosf(0.5f * roll_rad),  sr = sinf(0.5f * roll_rad);
      float cp = cosf(0.5f * pitch_rad), sp = sinf(0.5f * pitch_rad);
      float cy = cosf(0.5f * yaw_rad),   sy = sinf(0.5f * yaw_rad);

      q0_ = cr * cp * cy + sr * sp * sy;
      q1_ = sr * cp * cy - cr * sp * sy;
      q2_ = cr * sp * cy + sr * cp * sy;
      q3_ = cr * cp * sy - sr * sp * cy;

      float raw_roll  = roll_rad  * RAD_TO_DEG;
      float raw_pitch = pitch_rad * RAD_TO_DEG;
      state.roll_deg  = raw_roll  - state.roll_trim_deg;
      state.pitch_deg = raw_pitch - state.pitch_trim_deg;
    }
    return true;
  }

  // Simpan posisi saat ini sebagai titik datar (0 derajat Roll & Pitch).
  // Mengembalikan false jika IMU tidak terhubung, sedang bergerak, atau kemiringan melebihi batas wajar (+-15 derajat).
  bool calibrateLevel() {
    if (!state.connected) return false;
    if (fabsf(state.gyro_roll_dps) > 5.0f ||
        fabsf(state.gyro_pitch_dps) > 5.0f ||
        fabsf(state.gyro_yaw_dps) > 5.0f) {
      return false;
    }
    float raw_roll  = state.roll_deg  + state.roll_trim_deg;
    float raw_pitch = state.pitch_deg + state.pitch_trim_deg;
    if (isnan(raw_roll) || isnan(raw_pitch)) return false;
    if (fabsf(raw_roll) > MAX_LEVEL_TRIM_DEG || fabsf(raw_pitch) > MAX_LEVEL_TRIM_DEG) {
      return false;
    }
    state.roll_trim_deg  = raw_roll;
    state.pitch_trim_deg = raw_pitch;
    state.roll_deg       = 0.0f;
    state.pitch_deg      = 0.0f;
    return true;
  }

  // Dipanggil setiap siklus utama (500 Hz)
  void update(float dt) {
    if (!state.connected) {
      static uint32_t last_check_ms = 0;
      if (millis() - last_check_ms > 500) {
        last_check_ms = millis();
        reinitQuick();
      }
      return;
    }

    float ax, ay, az, gx, gy, gz;
    if (!readRawScaled(ax, ay, az, gx, gy, gz)) {
      if (++fail_count_ >= 3) {
        state.connected = false; // Putus sementara agar loop RC & Servo tidak pernah hang!
      }
      return;
    }
    fail_count_ = 0;

    // Kurangi offset bias hasil kalibrasi
    gx -= gyro_bias_x_;
    gy -= gyro_bias_y_;
    gz -= gyro_bias_z_;

    // Low-Pass Filter (LPF) pada kecepatan sudut Gyro
    state.gyro_roll_dps  = (1.0f - GYRO_LPF_ALPHA) * state.gyro_roll_dps  + GYRO_LPF_ALPHA * gx;
    state.gyro_pitch_dps = (1.0f - GYRO_LPF_ALPHA) * state.gyro_pitch_dps + GYRO_LPF_ALPHA * gy;
    state.gyro_yaw_dps   = (1.0f - GYRO_LPF_ALPHA) * state.gyro_yaw_dps   + GYRO_LPF_ALPHA * gz;

    // Jalankan sensor fusion Madgwick 6-DOF (Quaternion)
    updateMadgwick6DOF(
      state.gyro_roll_dps  * DEG_TO_RAD,
      state.gyro_pitch_dps * DEG_TO_RAD,
      state.gyro_yaw_dps   * DEG_TO_RAD,
      ax, ay, az, dt
    );

    // Konversi Quaternion (q0, q1, q2, q3) ke Euler Angles (Roll, Pitch, Yaw)
    float raw_roll = atan2f(2.0f * (q0_ * q1_ + q2_ * q3_), 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_)) * RAD_TO_DEG;
    float sinp = 2.0f * (q0_ * q2_ - q3_ * q1_);
    float raw_pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * RAD_TO_DEG;
    float raw_yaw = atan2f(2.0f * (q0_ * q3_ + q1_ * q2_), 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_)) * RAD_TO_DEG;

    // Penjaga anti-NaN darurat: jika terjadi glitch sensor ekstrem, pulihkan orientasi
    if (isnan(raw_roll) || isnan(raw_pitch) || isnan(raw_yaw)) {
      q0_ = 1.0f; q1_ = 0.0f; q2_ = 0.0f; q3_ = 0.0f;
      raw_roll  = 0.0f;
      raw_pitch = 0.0f;
      raw_yaw   = 0.0f;
    }

    state.roll_deg  = raw_roll  - state.roll_trim_deg;
    state.pitch_deg = raw_pitch - state.pitch_trim_deg;
    state.yaw_deg   = raw_yaw;
  }

private:
  float gyro_bias_x_ = 0.0f;
  float gyro_bias_y_ = 0.0f;
  float gyro_bias_z_ = 0.0f;

  // Status Quaternion orientasi 3D (bebas Gimbal Lock untuk manuver & VTOL)
  float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;

#if defined(IMU_USE_SPI) && (IMU_USE_SPI == 1)
  void writeRegSPI(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SPI_CS, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();
  }

  uint8_t readRegSPI(uint8_t reg) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SPI_CS, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();
    return val;
  }
#else
  uint8_t imu_i2c_addr_     = IMU_I2C_ADDR;
  bool    wire_initialized_ = false;
  uint8_t fail_count_       = 0;

  bool probeAndWake() {
    uint8_t addr = 0x68;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) {
      addr = 0x69;
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() != 0) {
        return false;
      }
    }
    imu_i2c_addr_ = addr;
    writeReg(0x6B, 0x01); // Bangunkan dari sleep mode
    return true;
  }

  void configureRegisters() {
    writeReg(0x1A, 0x03); // DLPF ~42 Hz
    writeReg(0x1B, 0x08); // Gyro +-500 dps
    writeReg(0x1C, 0x10); // Accel +-8g
  }

  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(imu_i2c_addr_);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }
#endif

  bool readRawScaled(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
#if defined(IMU_USE_SPI) && (IMU_USE_SPI == 1)
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SPI_CS, LOW);
    SPI.transfer(0x3B | 0x80); // Read bit 7 set
    uint8_t buf[14];
    for (uint8_t i = 0; i < 14; i++) {
      buf[i] = SPI.transfer(0x00);
    }
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();

    int16_t raw_ax = (buf[0] << 8) | buf[1];
    int16_t raw_ay = (buf[2] << 8) | buf[3];
    int16_t raw_az = (buf[4] << 8) | buf[5];
    int16_t raw_gx = (buf[8] << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];
#else
    Wire.beginTransmission(imu_i2c_addr_);
    Wire.write(0x3B); // Mulai dari register ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t count = Wire.requestFrom((uint8_t)imu_i2c_addr_, (uint8_t)14);
    if (count < 14) return false;

    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Lewati 2 byte sensor suhu internal
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();
#endif

    // Konversi ke satuan fisis: g (gravitasi) dan derajat/detik
    float in_ax = (float)raw_ax / 4096.0f;
    float in_ay = (float)raw_ay / 4096.0f;
    float in_az = (float)raw_az / 4096.0f;

    float in_gx = (float)raw_gx / 65.5f;
    float in_gy = (float)raw_gy / 65.5f;
    float in_gz = (float)raw_gz / 65.5f;

    // Transformasi Orientasi Board (Mapping Sumbu Sensor MPU6050 terhadap Pesawat)
    #if (BOARD_ROTATION == ROTATION_CW90)
      ax =  in_ay;
      ay = -in_ax;
      az =  in_az;
      gx =  in_gy;
      gy = -in_gx;
      gz =  in_gz;
    #elif (BOARD_ROTATION == ROTATION_CW180)
      ax = -in_ax;
      ay = -in_ay;
      az =  in_az;
      gx = -in_gx;
      gy = -in_gy;
      gz =  in_gz;
    #elif (BOARD_ROTATION == ROTATION_CW270)
      ax = -in_ay;
      ay =  in_ax;
      az =  in_az;
      gx = -in_gy;
      gy =  in_gx;
      gz =  in_gz;
    #elif (BOARD_ROTATION == ROTATION_FLIP)
      ax =  in_ax;
      ay = -in_ay;
      az = -in_az;
      gx =  in_gx;
      gy = -in_gy;
      gz = -in_gz;
    #elif (BOARD_ROTATION == ROTATION_FLIP_CW90)
      ax = -in_ay;
      ay = -in_ax;
      az = -in_az;
      gx = -in_gy;
      gy = -in_gx;
      gz = -in_gz;
    #else // ROTATION_DEFAULT (0 derajat)
      ax = in_ax;
      ay = in_ay;
      az = in_az;
      gx = in_gx;
      gy = in_gy;
      gz = in_gz;
    #endif

    return true;
  }

  void updateMadgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Laju perubahan quaternion dari gyroscope
    qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
    qDot2 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy);
    qDot3 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx);
    qDot4 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx);

    // Koreksi drift menggunakan Accelerometer jika data percepatan valid
    float accel_norm_sq = ax * ax + ay * ay + az * az;
    if (accel_norm_sq > 1e-4f) {
      recipNorm = 1.0f / sqrtf(accel_norm_sq);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      _2q0 = 2.0f * q0_;
      _2q1 = 2.0f * q1_;
      _2q2 = 2.0f * q2_;
      _2q3 = 2.0f * q3_;
      _4q0 = 4.0f * q0_;
      _4q1 = 4.0f * q1_;
      _4q2 = 4.0f * q2_;
      _8q1 = 8.0f * q1_;
      _8q2 = 8.0f * q2_;
      q0q0 = q0_ * q0_;
      q1q1 = q1_ * q1_;
      q2q2 = q2_ * q2_;
      q3q3 = q3_ * q3_;

      // Gradient descent corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;

      // Penjaga pembagian nol (Anti-NaN guard) pada gradient descent
      float s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
      if (s_norm_sq > 1e-9f) {
        recipNorm = 1.0f / sqrtf(s_norm_sq);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= MADGWICK_BETA * s0;
        qDot2 -= MADGWICK_BETA * s1;
        qDot3 -= MADGWICK_BETA * s2;
        qDot4 -= MADGWICK_BETA * s3;
      }
    }

    // Integrasi untuk mendapatkan quaternion terbaru
    q0_ += qDot1 * dt;
    q1_ += qDot2 * dt;
    q2_ += qDot3 * dt;
    q3_ += qDot4 * dt;

    // Normalisasi quaternion dengan penjaga pembagian nol (Anti-NaN guard)
    float q_norm_sq = q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_;
    if (q_norm_sq > 1e-9f) {
      recipNorm = 1.0f / sqrtf(q_norm_sq);
      q0_ *= recipNorm;
      q1_ *= recipNorm;
      q2_ *= recipNorm;
      q3_ *= recipNorm;
    } else {
      // Pulihkan quaternion jika sempat runtuh (collapse ke nol)
      q0_ = 1.0f; q1_ = 0.0f; q2_ = 0.0f; q3_ = 0.0f;
    }
  }
};

#endif // IMU_H
