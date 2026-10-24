/*
 * ============================================================================
 * @file        settings.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.1
 * @since       v0.2.0 (2026-10-19)
 * @updated     2026-10-24
 * @brief       Manajemen Penyimpanan EEPROM Flash Permanen & Auto-DFU/BOOTSEL
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.2.0 (2026-10-19): Penyimpanan protokol RC, arah servo, trim servo, & PID ke
 *                          EEPROM Flash (RP2040 & STM32F4) serta reboot ke DFU/BOOTSEL.
 *   - v0.4.1 (2026-10-24): Validasi anti-NaN dan penjepitan batas aman (+-200 us servo,
 *                          +-15 deg level trim) saat memuat dari EEPROM.
 * ============================================================================
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"

#define SETTINGS_MAGIC 0x46574637UL // "FWF7" - Reset EEPROM ke tuned flying wing gains

struct FCSettings {
  uint32_t magic               = SETTINGS_MAGIC;
  uint8_t  rc_protocol         = SELECTED_RC_PROTOCOL; // 1=iBUS, 2=CRSF, 3=SBUS, 4=PPM
  int8_t   servo_left_dir      = SERVO_LEFT_DIR;       // 1 atau -1
  int8_t   servo_right_dir     = SERVO_RIGHT_DIR;      // 1 atau -1
  int16_t  servo_left_trim_us  = SERVO_LEFT_TRIM_US;   // -200 .. +200 us
  int16_t  servo_right_trim_us = SERVO_RIGHT_TRIM_US;  // -200 .. +200 us
  float    roll_trim_deg       = 0.0f;
  float    pitch_trim_deg      = 0.0f;
  float    roll_kp             = PID_ROLL_KP;
  float    pitch_kp            = PID_PITCH_KP;
  float    angle_kp            = ANGLE_KP_ROLL;
};

#if defined(STM32F4xx) || defined(ARDUINO_ARCH_STM32)
// Variabel RAM khusus (.noinit) yang TIDAK terhapus saat MCU Reset
__attribute__((section(".noinit"))) static volatile uint32_t dfu_boot_magic;

// Dipanggil otomatis oleh startup assembly SEBELUM main() / SystemClock_Config() / USB aktif
__attribute__((constructor(100))) void checkBootloaderDFU() {
  if (dfu_boot_magic == 0xDEADBEEFUL) {
    dfu_boot_magic = 0;

    // Pastikan semua interrupt mati & kembalikan clock ke default HSI
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Aktifkan clock SYSCFG & petakan System Memory Bootloader (0x1FFF0000) ke 0x00000000
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __DSB();
    SYSCFG->MEMRMP = 0x01;
    __DSB();
    __ISB();

    // Ambil alamat Stack Pointer & Reset Handler milik ROM DFU Bootloader STM32F4
    uint32_t boot_sp    = *(volatile uint32_t *)0x1FFF0000UL;
    uint32_t boot_entry = *(volatile uint32_t *)0x1FFF0004UL;

    __enable_irq();
    __set_MSP(boot_sp);
    ((void (*)(void))boot_entry)();
    while (1) {}
  }
}

// Pulsa disconnect USB D+ (PA12) agar macOS/Windows mendeteksi cabut-colok USB otomatis
inline void usbDisconnectPulse() {
  pinMode(PA12, OUTPUT);
  digitalWrite(PA12, LOW);
  delay(25);
}

inline void softwareReboot() {
  Serial.println(F("\n[REBOOT] Me-restart Flight Controller..."));
  Serial.flush();
  usbDisconnectPulse();
  NVIC_SystemReset();
}

inline void softwareJumpToDFU() {
  Serial.println(F("\n========================================================"));
  Serial.println(F("[DFU] Masuk ke Mode DFU Bootloader (Tanpa Tombol BOOT)!"));
  Serial.println(F("      Board siap di-flash via dfu-util / flash_dfu.sh"));
  Serial.println(F("========================================================"));
  Serial.flush();
  usbDisconnectPulse();
  dfu_boot_magic = 0xDEADBEEFUL;
  NVIC_SystemReset();
}
#elif defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
inline void softwareReboot() {
  Serial.println(F("\n[REBOOT] Merestart RP2040..."));
  Serial.flush();
  rp2040.reboot();
}
inline void softwareJumpToDFU() {
  Serial.println(F("\n[BOOTSEL] Masuk ke mode Flashing BOOTSEL RP2040..."));
  Serial.flush();
  rp2040.rebootToBootloader();
}
#else
inline void softwareReboot() {
  Serial.println(F("\n[REBOOT] Silakan tekan tombol Reset di board."));
}
inline void softwareJumpToDFU() {
  Serial.println(F("\n[DFU] Fitur Auto-DFU khusus untuk STM32F4/RP2040."));
}
#endif

class SettingsManager {
public:
  FCSettings data;

  void resetDefaults() {
    data = FCSettings();
  }

  bool load() {
    FCSettings tmp;
    #if defined(STM32F4xx) || defined(ARDUINO_ARCH_STM32)
      eeprom_buffer_fill();
      uint8_t *ptr = (uint8_t *)&tmp;
      for (size_t i = 0; i < sizeof(FCSettings); i++) {
        ptr[i] = eeprom_buffered_read_byte(i);
      }
    #elif defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
      EEPROM.begin(256);
      EEPROM.get(0, tmp);
    #else
      EEPROM.get(0, tmp);
    #endif

    if (tmp.magic == SETTINGS_MAGIC &&
        tmp.rc_protocol >= RC_PROTO_IBUS && tmp.rc_protocol <= RC_PROTO_PWM &&
        (tmp.servo_left_dir == 1 || tmp.servo_left_dir == -1) &&
        (tmp.servo_right_dir == 1 || tmp.servo_right_dir == -1) &&
        !isnan(tmp.roll_trim_deg) && !isnan(tmp.pitch_trim_deg)) {
      tmp.servo_left_trim_us  = constrain(tmp.servo_left_trim_us,  -200, 200);
      tmp.servo_right_trim_us = constrain(tmp.servo_right_trim_us, -200, 200);
      tmp.roll_trim_deg       = constrain(tmp.roll_trim_deg,  -MAX_LEVEL_TRIM_DEG, MAX_LEVEL_TRIM_DEG);
      tmp.pitch_trim_deg      = constrain(tmp.pitch_trim_deg, -MAX_LEVEL_TRIM_DEG, MAX_LEVEL_TRIM_DEG);
      data = tmp;
      return true;
    } else {
      resetDefaults();
      return false;
    }
  }

  void save() {
    data.magic = SETTINGS_MAGIC;
    #if defined(STM32F4xx) || defined(ARDUINO_ARCH_STM32)
      // Tulis ke buffer RAM lalu flush 1x ke sektor Flash STM32 (hemat umur Flash & cepat)
      eeprom_buffer_fill();
      const uint8_t *ptr = (const uint8_t *)&data;
      for (size_t i = 0; i < sizeof(FCSettings); i++) {
        eeprom_buffered_write_byte(i, ptr[i]);
      }
      eeprom_buffer_flush();
    #elif defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
      EEPROM.begin(256);
      EEPROM.put(0, data);
      EEPROM.commit();
    #else
      EEPROM.put(0, data);
    #endif
  }

  const char* getProtocolName() const {
    switch (data.rc_protocol) {
      case RC_PROTO_IBUS: return "FlySky iBUS (115200)";
      case RC_PROTO_CRSF: return "ELRS / CRSF (420000)";
      case RC_PROTO_SBUS: return "FrSky SBUS (100000)";
      case RC_PROTO_PWM:  return "Standard PWM";
      default:            return "UNKNOWN";
    }
  }
};

#endif // SETTINGS_H
