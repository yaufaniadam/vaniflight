/*
 * ============================================================================
 * @file        rc_input.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.1
 * @since       v0.1.0 (2026-10-18)
 * @updated     2026-10-24
 * @brief       Multi-Receiver Parser (SBUS, iBUS, CRSF, PPM) & Logika Failsafe
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.1.0 (2026-10-18): Parser multi-protokol (iBUS, SBUS, CRSF, PPM) & Arming throttle lock.
 *   - v0.2.0 (2026-10-20): Inverting UART0 RX silikon RP2040 (GP1) untuk FlySky SBUS &
 *                          pemetaan switch FlySky 10CH (SwA, SwB, SwC, SwD).
 *   - v0.3.0 (2026-10-24): Pengaman Failsafe RTH (tidak pernah auto-arm saat pesawat Disarmed).
 *   - v0.4.0 (2026-10-24): Pemetaan CH8 (SwB) ke state.autotrim_sw_us untuk AutoTrim/Auto-Level.
 * ============================================================================
 */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <Arduino.h>
#include "config.h"
#include "gps.h"

#if defined(STM32F4xx) || defined(ARDUINO_ARCH_STM32)
  extern Uart Serial1;
  extern Uart Serial3;
  extern Uart Serial6;
#endif

enum FlightMode : uint8_t {
  MODE_MANUAL   = 0,
  MODE_ACRO     = 1,
  MODE_ANGLE    = 2,
  MODE_ALT_HOLD = 3,
  MODE_RTH      = 4
};

struct RCState {
  uint16_t roll_us     = 1500;
  uint16_t pitch_us    = 1500;
  uint16_t throttle_us    = 1000;
  uint16_t yaw_us         = 1500;
  uint16_t arm_us         = 1000;
  uint16_t autotrim_sw_us = 1000; // CH8 (SwB): AutoTrim Switch
  uint16_t mode_us        = 1500;
  uint16_t rth_sw_us      = 1000;

  bool armed           = false;
  bool failsafe        = true;
  bool bench_test_mode = true;
  FlightMode mode      = MODE_MANUAL;

  // Diagnostik (Omnibus F4: U1=RX1, U6=RX6/J7, U3=RX3)
  uint32_t rx_bytes[3]   = {0, 0, 0};
  uint8_t  last_bytes[4] = {0, 0, 0, 0};
  uint8_t  active_port   = 0;
  const char* active_proto = "NONE";

  float roll_norm      = 0.0f;
  float pitch_norm     = 0.0f;
  float throttle_norm  = 0.0f;
};

struct PortParser {
  uint8_t  buf[64];
  uint8_t  idx = 0;
  uint32_t last_byte_us = 0;
};

namespace RCPPMDecoder {
  volatile uint32_t last_edge_us = 0;
  volatile uint8_t  ch_idx = 0;
  volatile uint16_t ch_raw[8] = {1500, 1500, 1000, 1500, 1000, 1500, 1000, 1500};
  volatile uint32_t last_frame_ms = 0;

  void isr() {
    uint32_t now = micros();
    uint32_t diff = now - last_edge_us;
    last_edge_us = now;
    if (diff > 2500) { // Sync gap > 2.5ms (usually 4-10ms)
      ch_idx = 0;
    } else if (diff >= 850 && diff <= 2200 && ch_idx < 8) {
      ch_raw[ch_idx++] = (uint16_t)diff;
      if (ch_idx >= 4) {
        last_frame_ms = millis();
      }
    }
  }
}

class RCInputManager {
public:
  RCState state;
  uint8_t protocol = RC_PROTO_IBUS; // 1=iBUS, 2=CRSF, 3=SBUS, 4=PPM

  void begin(uint8_t proto = RC_PROTO_IBUS) {
    protocol = proto;
    for (uint8_t p = 0; p < 3; p++) {
      ports_[p].idx = 0;
      ports_[p].last_byte_us = 0;
    }

    uint32_t baud = 115200;
    uint16_t cfg  = SERIAL_8N1;
    bool inv      = false;

    if (protocol == RC_PROTO_SBUS) {
      baud = 100000;
      cfg  = SERIAL_8E2;
      inv  = true;
    } else if (protocol == RC_PROTO_CRSF) {
      baud = 420000;
      cfg  = SERIAL_8N1;
      inv  = false;
    }

    #if defined(STM32F4xx) || defined(ARDUINO_ARCH_STM32)
      if (protocol == RC_PROTO_PPM) {
        if (initialized_) {
          Serial1.end();
          Serial6.end();
          Serial3.end();
        }
        pinMode(PC8, OUTPUT);
        digitalWrite(PC8, LOW); // Disable inverter
        pinMode(PC7, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PC7), RCPPMDecoder::isr, FALLING);
        initialized_ = true;
        return;
      } else {
        detachInterrupt(digitalPinToInterrupt(PC7));
      }

      pinMode(PC8, OUTPUT);
      pinMode(PC9, OUTPUT);
      pinMode(PC0, OUTPUT);
      digitalWrite(PC8, inv ? HIGH : LOW);
      digitalWrite(PC9, LOW);
      digitalWrite(PC0, inv ? HIGH : LOW);

      if (initialized_) {
        Serial1.end();
        Serial6.end();
        Serial3.end();
      }
      Serial1.begin(baud, cfg);
      Serial6.begin(baud, cfg);
      Serial3.begin(baud, cfg);
      initialized_ = true;

    #elif defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
      if (initialized_) RC_SERIAL.end();
      RC_SERIAL.setTX(PIN_RC_UART_TX);
      RC_SERIAL.setRX(PIN_RC_UART_RX);
      RC_SERIAL.setFIFOSize(256);
      RC_SERIAL.begin(baud, cfg);
      gpio_set_inover(PIN_RC_UART_RX, inv ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
      initialized_ = true;
    #elif defined(ESP32)
      if (initialized_) RC_SERIAL.end();
      RC_SERIAL.begin(115200, SERIAL_8N1, PIN_RC_UART_RX, PIN_RC_UART_TX);
      initialized_ = true;
    #else
      if (initialized_) RC_SERIAL.end();
      RC_SERIAL.begin(115200);
      initialized_ = true;
    #endif
  }

  void update(GPSManager &gps_mgr) {
    bool got_frame = false;
    uint32_t now_us = micros();

    #if defined(STM32F4xx) || defined(ARDUINO_ARCH_STM32)
      if (protocol == RC_PROTO_PPM) {
        if (millis() - RCPPMDecoder::last_frame_ms < 300 && RCPPMDecoder::last_frame_ms > 0) {
          noInterrupts();
          for (uint8_t c = 0; c < 8; c++) raw_ch_[c] = RCPPMDecoder::ch_raw[c];
          interrupts();
          last_valid_ms_ = millis();
          state.active_port = 6;
          state.active_proto = "PPM";
          got_frame = true;
        }
      } else {
        Stream *uarts[3]    = { &Serial1, &Serial6, &Serial3 };
        uint8_t port_ids[3] = { 1, 6, 3 };

        for (uint8_t p = 0; p < 3; p++) {
          while (uarts[p]->available()) {
            uint8_t b = (uint8_t)uarts[p]->read();
            state.rx_bytes[p]++;
            state.last_bytes[0] = state.last_bytes[1];
            state.last_bytes[1] = state.last_bytes[2];
            state.last_bytes[2] = state.last_bytes[3];
            state.last_bytes[3] = b;

            gps_mgr.feedChar((char)b);

            bool frame_ok = false;
            if (protocol == RC_PROTO_IBUS) {
              frame_ok = parseIBUSByte(ports_[p], b, now_us);
              if (frame_ok) state.active_proto = "iBUS";
            } else if (protocol == RC_PROTO_SBUS) {
              frame_ok = parseSBUSByte(ports_[p], b, now_us);
              if (frame_ok) state.active_proto = "SBUS";
            } else if (protocol == RC_PROTO_CRSF) {
              frame_ok = parseCRSFByte(ports_[p], b);
              if (frame_ok) state.active_proto = "CRSF";
            }

            if (frame_ok) {
              state.active_port = port_ids[p];
              got_frame = true;
            }
          }
        }
      }
    #else
      while (RC_SERIAL.available()) {
        uint8_t b = (uint8_t)RC_SERIAL.read();
        state.rx_bytes[0]++;
        state.last_bytes[0] = state.last_bytes[1];
        state.last_bytes[1] = state.last_bytes[2];
        state.last_bytes[2] = state.last_bytes[3];
        state.last_bytes[3] = b;

        bool frame_ok = false;
        if (protocol == RC_PROTO_IBUS) frame_ok = parseIBUSByte(ports_[0], b, now_us);
        else if (protocol == RC_PROTO_SBUS) frame_ok = parseSBUSByte(ports_[0], b, now_us);
        else if (protocol == RC_PROTO_CRSF) frame_ok = parseCRSFByte(ports_[0], b);

        if (frame_ok) {
          state.active_port = 1;
          state.active_proto = (protocol == RC_PROTO_IBUS) ? "iBUS" : "SBUS";
          got_frame = true;
        }
      }
    #endif

    if (got_frame) {
      state.bench_test_mode = false;
      state.failsafe = false;

      state.roll_us     = constrain(raw_ch_[0], 1000, 2000); // CH1: Roll
      state.pitch_us    = constrain(raw_ch_[1], 1000, 2000); // CH2: Pitch
      state.throttle_us = constrain(raw_ch_[2], 1000, 2000); // CH3: Throttle
      state.yaw_us      = constrain(raw_ch_[3], 1000, 2000); // CH4: Yaw
      // FlySky 10CH Mapping: CH5=Pot1, CH6=Pot2, CH7=SwA, CH8=SwB, CH9=SwC(3-Pos), CH10=SwD
      state.arm_us         = constrain(raw_ch_[6], 1000, 2000); // CH7 (SwA): Arm / Disarm
      state.autotrim_sw_us = constrain(raw_ch_[7], 1000, 2000); // CH8 (SwB): AutoTrim Switch
      state.mode_us        = constrain(raw_ch_[8], 1000, 2000); // CH9 (SwC 3-Pos): Manual / Angle / Acro
      state.rth_sw_us      = constrain(raw_ch_[9], 1000, 2000); // CH10 (SwD): RTH

      updateArmAndMode();
    } else if (millis() - last_valid_ms_ > 500) {
      state.failsafe = true;

      // KEAMANAN TINGKAT TINGGI (FAILSAFE):
      // RTH otomatis hanya boleh aktif jika pesawat MEMANG SUDAH ARMED di udara sebelum hilang sinyal!
      // Jika remote mati saat pesawat di darat (Disarmed), pesawat WAJIB TETAP DISARMED (Motor MATI 1000 us)!
      if (state.armed && !state.bench_test_mode && gps_mgr.state.home_set && gps_mgr.state.fix_valid) {
        state.mode = MODE_RTH;
        // Tetap armed untuk navigasi otonom terbang pulang ke Home
      } else {
        // Safe Disarm & Level glide passthrough
        state.mode        = MODE_MANUAL;
        state.roll_us     = 1500;
        state.pitch_us    = 1500;
        state.armed       = false;
        state.throttle_us = MOTOR_DISARMED_US;
      }
    }

    state.roll_norm     = applyDeadband((float)state.roll_us  - 1500.0f, 12.0f) / 500.0f;
    state.pitch_norm    = applyDeadband((float)state.pitch_us - 1500.0f, 12.0f) / 500.0f;
    state.throttle_norm = constrain(((float)state.throttle_us - 1000.0f) / 1000.0f, 0.0f, 1.0f);
  }

  const char* getModeName() const {
    switch (state.mode) {
      case MODE_MANUAL:   return "MANUAL";
      case MODE_ACRO:     return "ACRO";
      case MODE_ANGLE:    return "ANGLE";
      case MODE_ALT_HOLD: return "ALT_HOLD";
      case MODE_RTH:      return "RTH";
      default:            return "UNKNOWN";
    }
  }

  const char* getProtocolName() const {
    switch (protocol) {
      case RC_PROTO_IBUS: return "FlySky iBUS (115200 8N1)";
      case RC_PROTO_PPM:  return "PPM Receiver (Pulse J7)";
      case RC_PROTO_SBUS: return "FrSky SBUS (100000 8E2)";
      case RC_PROTO_CRSF: return "ELRS CRSF (420000 8N1)";
      default:            return "UNKNOWN";
    }
  }

private:
  bool       initialized_   = false;
  uint16_t   raw_ch_[10]    = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000};
  uint32_t   last_valid_ms_ = 0;
  PortParser ports_[3];

  float applyDeadband(float val, float deadband) {
    if (fabsf(val) < deadband) return 0.0f;
    return (val > 0.0f) ? (val - deadband) : (val + deadband);
  }

  void updateArmAndMode() {
    if (state.rth_sw_us > 1700) {
      state.mode = MODE_RTH;
    } else {
      if (state.mode_us < 1300) {
        state.mode = MODE_MANUAL;
      } else if (state.mode_us < 1700) {
        state.mode = MODE_ANGLE;
      } else {
        state.mode = MODE_ACRO;
      }
    }

    bool switch_wants_arm = (state.arm_us > 1600);
    if (!switch_wants_arm) {
      state.armed = false;
    } else if (!state.armed && state.throttle_us < ARM_SAFETY_MAX_THR_US) {
      state.armed = true;
    }
  }

  bool parseIBUSByte(PortParser &p, uint8_t b, uint32_t now_us) {
    if (now_us - p.last_byte_us > 3500) {
      p.idx = 0;
    }
    p.last_byte_us = now_us;

    if (p.idx == 0 && b != 0x20) return false;
    if (p.idx == 1 && b != 0x40) { p.idx = 0; return false; }
    p.buf[p.idx++] = b;

    if (p.idx == 32) {
      p.idx = 0;
      uint16_t chksum = 0xFFFF;
      for (uint8_t i = 0; i < 30; i++) chksum -= p.buf[i];
      uint16_t rx_chk = p.buf[30] | (p.buf[31] << 8);

      if (chksum == rx_chk) {
        for (uint8_t c = 0; c < 10; c++) {
          raw_ch_[c] = p.buf[2 + c * 2] | (p.buf[3 + c * 2] << 8);
        }
        last_valid_ms_ = millis();
        return true;
      }
    }
    return false;
  }

  bool parseSBUSByte(PortParser &p, uint8_t b, uint32_t now_us) {
    if (now_us - p.last_byte_us > 3500) {
      p.idx = 0;
    }
    p.last_byte_us = now_us;

    if (p.idx == 0 && b != 0x0F) return false;
    p.buf[p.idx++] = b;

    if (p.idx == 25) {
      p.idx = 0;
      if (p.buf[24] == 0x00 || (p.buf[24] & 0x0F) == 0x04 || p.buf[24] == 0x14 || p.buf[24] == 0x24 || p.buf[24] == 0x34) {
        bool sbus_failsafe = (p.buf[23] & 0x08) != 0;
        if (!sbus_failsafe) {
          const uint8_t* d = &p.buf[1];
          uint16_t ch11[10];
          ch11[0] = ((d[0]     | d[1]  << 8) & 0x07FF);
          ch11[1] = ((d[1]>>3  | d[2]  << 5) & 0x07FF);
          ch11[2] = ((d[2]>>6  | d[3]  << 2 | d[4] << 10) & 0x07FF);
          ch11[3] = ((d[4]>>1  | d[5]  << 7) & 0x07FF);
          ch11[4] = ((d[5]>>4  | d[6]  << 4) & 0x07FF);
          ch11[5] = ((d[6]>>7  | d[7]  << 1 | d[8] << 9) & 0x07FF);
          ch11[6] = ((d[8]>>2  | d[9]  << 6) & 0x07FF);
          ch11[7] = ((d[9]>>5  | d[10] << 3) & 0x07FF);
          ch11[8] = ((d[11]    | d[12] << 8) & 0x07FF);
          ch11[9] = ((d[12]>>3 | d[13] << 5) & 0x07FF);

          for (uint8_t i = 0; i < 10; i++) {
            raw_ch_[i] = (uint16_t)constrain(map(ch11[i], 172, 1811, 1000, 2000), 1000, 2000);
          }
          last_valid_ms_ = millis();
          return true;
        }
      }
    }
    return false;
  }

  bool parseCRSFByte(PortParser &p, uint8_t b) {
    if (p.idx == 0 && b != 0xC8 && b != 0xEE) return false;
    p.buf[p.idx++] = b;

    if (p.idx == 2) {
      if (p.buf[1] > 26 || p.buf[1] < 4) p.idx = 0;
    } else if (p.idx > 2 && p.idx >= p.buf[1] + 2) {
      bool ok = false;
      if (p.buf[2] == 0x16 && p.buf[1] >= 24) {
        const uint8_t* d = &p.buf[3];
        uint16_t ch11[10];
        ch11[0] = ((d[0]     | d[1]  << 8) & 0x07FF);
        ch11[1] = ((d[1]>>3  | d[2]  << 5) & 0x07FF);
        ch11[2] = ((d[2]>>6  | d[3]  << 2 | d[4] << 10) & 0x07FF);
        ch11[3] = ((d[4]>>1  | d[5]  << 7) & 0x07FF);
        ch11[4] = ((d[5]>>4  | d[6]  << 4) & 0x07FF);
        ch11[5] = ((d[6]>>7  | d[7]  << 1 | d[8] << 9) & 0x07FF);
        ch11[6] = ((d[8]>>2  | d[9]  << 6) & 0x07FF);
        ch11[7] = ((d[9]>>5  | d[10] << 3) & 0x07FF);
        ch11[8] = ((d[11]    | d[12] << 8) & 0x07FF);
        ch11[9] = ((d[12]>>3 | d[13] << 5) & 0x07FF);

        for (uint8_t i = 0; i < 10; i++) {
          raw_ch_[i] = (uint16_t)constrain(map(ch11[i], 172, 1811, 1000, 2000), 1000, 2000);
        }
        last_valid_ms_ = millis();
        ok = true;
      }
      p.idx = 0;
      return ok;
    }
    return false;
  }
};

#endif // RC_INPUT_H
