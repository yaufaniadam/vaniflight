/*
 * ============================================================================
 * @file        gps.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.2
 * @since       v0.2.0 (2026-10-20)
 * @updated     2026-10-25
 * @brief       Parser Ganda GPS: Biner u-blox UBX-NAV-PVT (230400) + NMEA & Navigasi RTH
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.2.0 (2026-10-20): Parser NMEA u-blox M10 / NEO-6M, Auto-Home lock (>=5 satelit),
 *                          serta perhitungan jarak & bearing Return-to-Home (RTH).
 *   - v0.4.1 (2026-10-24): Tambah Auto-Baudrate Scanner saat Disarmed & diagnostik.
 *   - v0.4.2 (2026-10-25): Tambah Parser Biner Native u-blox UBX-NAV-PVT (0xB5 0x62 0x01 0x07)
 *                          pada 230400 baud untuk RushFPV M10 dengan validasi checksum Fletcher-16.
 * ============================================================================
 */
#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <string.h>
#include "config.h"

struct GPSState {
  bool     connected          = false; // True jika frame UBX / NMEA valid diterima dalam 2.5 detik terakhir
  uint8_t  active_rx_pin      = PIN_GPS_UART_RX;
  uint32_t baud_rate          = GPS_BAUD_RATE;
  uint32_t rx_bytes           = 0;
  uint32_t sentences_ok       = 0;
  char     last_nmea[64]      = {0};
  bool     fix_valid          = false;
  uint8_t  fix_type           = 0;     // 0=No Fix, 2=2D, 3=3D
  uint8_t  satellites         = 0;
  float    latitude_deg       = 0.0f;
  float    longitude_deg      = 0.0f;
  float    altitude_m         = 0.0f;
  float    speed_kmh          = 0.0f;
  float    course_deg         = 0.0f; // Arah gerak pesawat (0..360 derajat dari Utara)

  // Titik Home (Otomatis disimpan saat satelit >= 5)
  bool     home_set           = false;
  float    home_lat_deg       = 0.0f;
  float    home_lon_deg       = 0.0f;
  float    home_alt_m         = 0.0f;

  // Navigasi Return-to-Home (RTH)
  float    dist_to_home_m     = 0.0f;
  float    bearing_to_home_deg= 0.0f;
  float    rth_roll_cmd_deg   = 0.0f; // Sudut miring (bank angle) otomatis untuk berbelok pulang
  float    rth_pitch_cmd_deg  = 0.0f; // Sudut pitch otomatis untuk menjaga ketinggian pulang
};

class GPSManager {
public:
  GPSState state;

  void configureUbloxM10() {
    #if defined(GPS_SERIAL)
      // Paket UBX-CFG-PRT standar untuk menyetel UART1 ke 115200 baud, 8N1, dan aktifkan output NMEA + UBX:
      // Header: B5 62, ID: 06 00, Length: 14 00
      static const uint8_t ubx_cfg_prt_115200[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
        0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
        0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xC0, 0x7E
      };

      // Paket UBX-CFG-RATE untuk 5 Hz (200 ms):
      static const uint8_t ubx_cfg_rate_5hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00,
        0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
      };

      // Kirim konfigurasi di 38400 dulu jika GPS baru bangun di 38400
      GPS_SERIAL.begin(38400);
      delay(50);
      GPS_SERIAL.write(ubx_cfg_prt_115200, sizeof(ubx_cfg_prt_115200));
      GPS_SERIAL.flush();
      delay(50);

      // Kirim juga di 9600 jika GPS baru bangun di 9600
      GPS_SERIAL.begin(9600);
      delay(50);
      GPS_SERIAL.write(ubx_cfg_prt_115200, sizeof(ubx_cfg_prt_115200));
      GPS_SERIAL.flush();
      delay(50);

      // Sekarang buka koneksi permanen di 115200
      GPS_SERIAL.begin(115200);
      delay(50);
      GPS_SERIAL.write(ubx_cfg_prt_115200, sizeof(ubx_cfg_prt_115200));
      GPS_SERIAL.write(ubx_cfg_rate_5hz, sizeof(ubx_cfg_rate_5hz));
      GPS_SERIAL.flush();
    #endif
  }

  void begin() {
    state.baud_rate     = GPS_BAUD_RATE;
    state.active_rx_pin = PIN_GPS_UART_RX;
    last_baud_try_ms_   = 0;
    #if !defined(STM32F4xx) && !defined(ARDUINO_ARCH_STM32)
      #if defined(GPS_SERIAL)
        #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
          startRP2040Port(state.baud_rate);
        #elif defined(ESP32)
          GPS_SERIAL.begin(state.baud_rate, SERIAL_8N1, PIN_GPS_UART_RX, PIN_GPS_UART_TX);
        #else
          GPS_SERIAL.begin(state.baud_rate);
        #endif
      #endif
    #endif
  }

  void feedChar(char c) {
    state.rx_bytes++;
    parseByte((uint8_t)c);
    parseChar(c);
  }

  void update(bool is_armed = false) {
    #if !defined(STM32F4xx) && !defined(ARDUINO_ARCH_STM32)
      #if defined(GPS_SERIAL)
        while (GPS_SERIAL.available()) {
          uint8_t b = (uint8_t)GPS_SERIAL.read();
          state.rx_bytes++;
          parseByte(b);
          parseChar((char)b);
        }
      #endif
    #endif

    uint32_t now = millis();
    if (last_baud_try_ms_ == 0) {
      last_baud_try_ms_ = now;
    }

    // Jika frame UBX/NMEA tidak masuk > 2.5 detik, tandai koneksi & fix tidak aktif
    if (now - last_sentence_ms_ > 2500) {
      state.fix_valid = false;
      state.connected = false;
    }

    // Auto-scanner pintar (230400 -> 115200 -> 57600 -> 38400 -> 9600) saat DISARMED
    // sebelum mengunci paket UBX/NMEA pertama!
    #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
      if (!is_armed && !baud_locked_ && (now - last_baud_try_ms_ > 2000)) {
        last_baud_try_ms_ = now;
        static const uint32_t kBaudList[5] = {230400, 115200, 57600, 38400, 9600};
        scan_step_ = (scan_step_ + 1) % 5;
        state.baud_rate = kBaudList[scan_step_];
        startRP2040Port(state.baud_rate);
      }
    #endif
  }

  // Simpan posisi saat ini sebagai titik Home secara manual/otomatis
  void setHomeNow() {
    if (state.fix_valid && state.satellites >= 5) {
      state.home_lat_deg = state.latitude_deg;
      state.home_lon_deg = state.longitude_deg;
      state.home_alt_m   = state.altitude_m;
      state.home_set     = true;
    }
  }

private:
  char     line_buf_[128];
  uint8_t  line_idx_          = 0;
  uint32_t last_sentence_ms_  = 0;
  uint32_t last_baud_try_ms_  = 0;
  uint8_t  scan_step_         = 0;
  bool     baud_locked_       = false;

  // State machine parser biner u-blox UBX (0xB5 0x62)
  uint8_t  ubx_step_          = 0;
  uint8_t  ubx_class_         = 0;
  uint8_t  ubx_id_            = 0;
  uint16_t ubx_len_           = 0;
  uint16_t ubx_idx_           = 0;
  uint8_t  ubx_ck_a_          = 0;
  uint8_t  ubx_ck_b_          = 0;
  uint8_t  ubx_payload_[100];

  #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
    void startRP2040Port(uint32_t baud) {
      if (hw_started_) {
        GPS_SERIAL.end();
      }
      GPS_SERIAL.setTX(PIN_GPS_UART_TX);
      GPS_SERIAL.setRX(PIN_GPS_UART_RX);
      GPS_SERIAL.setFIFOSize(256); // Wajib 256 byte untuk menampung paket 100-byte UBX-NAV-PVT @ 230400 baud!
      GPS_SERIAL.begin(baud);
      gpio_pull_up(PIN_GPS_UART_RX); // Pull-up pada GP9 (RX) agar bebas noise floating!
      hw_started_ = true;
    }
    bool hw_started_ = false;
  #endif

  static int32_t readI32LE(const uint8_t *p) {
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)u;
  }

  static uint16_t readU16LE(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  }

  // [Since v0.4.2 - 2026-10-25] Parser biner u-blox UBX-NAV-PVT (Class 0x01, ID 0x07, Len 92)
  void parseByte(uint8_t b) {
    switch (ubx_step_) {
      case 0:
        if (b == 0xB5) ubx_step_ = 1;
        break;
      case 1:
        if (b == 0x62) ubx_step_ = 2;
        else ubx_step_ = (b == 0xB5) ? 1 : 0;
        break;
      case 2:
        ubx_class_ = b;
        ubx_ck_a_  = b;
        ubx_ck_b_  = b;
        ubx_step_  = 3;
        break;
      case 3:
        ubx_id_    = b;
        ubx_ck_a_ += b;
        ubx_ck_b_ += ubx_ck_a_;
        ubx_step_  = 4;
        break;
      case 4:
        ubx_len_   = b;
        ubx_ck_a_ += b;
        ubx_ck_b_ += ubx_ck_a_;
        ubx_step_  = 5;
        break;
      case 5:
        ubx_len_  |= ((uint16_t)b << 8);
        ubx_ck_a_ += b;
        ubx_ck_b_ += ubx_ck_a_;
        if (ubx_len_ == 0 || ubx_len_ > sizeof(ubx_payload_)) {
          ubx_step_ = 0;
        } else {
          ubx_idx_  = 0;
          ubx_step_ = 6;
        }
        break;
      case 6:
        ubx_payload_[ubx_idx_++] = b;
        ubx_ck_a_ += b;
        ubx_ck_b_ += ubx_ck_a_;
        if (ubx_idx_ >= ubx_len_) {
          ubx_step_ = 7;
        }
        break;
      case 7:
        if (b == ubx_ck_a_) ubx_step_ = 8;
        else ubx_step_ = 0;
        break;
      case 8:
        if (b == ubx_ck_b_) {
          processUBXPacket();
        }
        ubx_step_ = 0;
        break;
    }
  }

  void processUBXPacket() {
    last_sentence_ms_ = millis();
    state.connected   = true;
    baud_locked_      = true;
    state.sentences_ok++;

    if (ubx_class_ == 0x01 && ubx_id_ == 0x07 && ubx_len_ >= 84) {
      uint16_t yr  = readU16LE(&ubx_payload_[4]);
      uint8_t  mo  = ubx_payload_[6];
      uint8_t  dy  = ubx_payload_[7];
      uint8_t  hr  = ubx_payload_[8];
      uint8_t  mn  = ubx_payload_[9];
      uint8_t  sc  = ubx_payload_[10];

      uint8_t fix_type = ubx_payload_[20];
      uint8_t flags    = ubx_payload_[21];
      uint8_t num_sv   = ubx_payload_[23];
      int32_t lon_e7   = readI32LE(&ubx_payload_[24]);
      int32_t lat_e7   = readI32LE(&ubx_payload_[28]);
      int32_t h_msl_mm = readI32LE(&ubx_payload_[36]);
      int32_t g_spd_mm = readI32LE(&ubx_payload_[60]);
      int32_t head_e5  = readI32LE(&ubx_payload_[64]);

      state.fix_type   = fix_type;
      state.satellites = num_sv;
      state.altitude_m = (float)h_msl_mm * 1e-3f;
      state.speed_kmh  = (g_spd_mm > 0) ? ((float)g_spd_mm * 0.0036f) : 0.0f;

      float cog = (float)head_e5 * 1e-5f;
      while (cog < 0.0f)   cog += 360.0f;
      while (cog >= 360.0f) cog -= 360.0f;
      state.course_deg = cog;

      snprintf(state.last_nmea, sizeof(state.last_nmea),
               "UBX-NAV-PVT [%04u-%02u-%02u %02u:%02u:%02u UTC] Fix:%uD SV:%u",
               yr, mo, dy, hr, mn, sc, fix_type, num_sv);

      bool fix_ok = ((flags & 0x01) != 0) && (fix_type >= 2) && (num_sv >= 4);
      if (fix_ok) {
        state.fix_valid     = true;
        state.latitude_deg  = (float)lat_e7 * 1e-7f;
        state.longitude_deg = (float)lon_e7 * 1e-7f;

        if (!state.home_set && state.satellites >= 5 && fix_type >= 3) {
          setHomeNow();
        }
        if (state.home_set) {
          updateHomeNavigation();
        }
      } else {
        state.fix_valid = false;
      }
    }
  }

  void parseChar(char c) {
    if (c == '$') {
      line_idx_ = 0;
    }
    if (c == '\r' || c == '\n') {
      if (line_idx_ > 6) {
        line_buf_[line_idx_] = '\0';
        processSentence(line_buf_);
      }
      line_idx_ = 0;
      return;
    }
    if (line_idx_ < sizeof(line_buf_) - 1) {
      line_buf_[line_idx_++] = c;
    }
  }

  // Mendukung kalimat NMEA dari GPS u-blox M10 ($GNGGA/$GNRMC) maupun NEO-6M ($GPGGA/$GPRMC)
  void processSentence(char *line) {
    // Kunci baudrate bila menerima kalimat NMEA standar apapun ($GP... / $GN...)
    if (line[0] == '$' && (strncmp(line, "$GN", 3) == 0 || strncmp(line, "$GP", 3) == 0)) {
      baud_locked_      = true;
      state.connected   = true;
      last_sentence_ms_ = millis();
      strncpy(state.last_nmea, line, sizeof(state.last_nmea) - 1);
      state.last_nmea[sizeof(state.last_nmea) - 1] = '\0';
    }

    if (strncmp(line, "$GPGGA,", 7) == 0 || strncmp(line, "$GNGGA,", 7) == 0) {
      parseGGA(line + 7);
      last_sentence_ms_ = millis();
      state.connected   = true;
      state.sentences_ok++;
      baud_locked_      = true;
    } else if (strncmp(line, "$GPRMC,", 7) == 0 || strncmp(line, "$GNRMC,", 7) == 0) {
      parseRMC(line + 7);
      last_sentence_ms_ = millis();
      state.connected   = true;
      state.sentences_ok++;
      baud_locked_      = true;
    }
  }

  void parseGGA(char *p) {
    // Format GGA: time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,...
    char *tok = strsep(&p, ","); // 1: Time
    tok = strsep(&p, ",");       // 2: Latitude (ddmm.mmmm)
    float raw_lat = tok ? atof(tok) : 0.0f;
    tok = strsep(&p, ",");       // 3: N/S
    char ns = (tok && tok[0]) ? tok[0] : 'N';

    tok = strsep(&p, ",");       // 4: Longitude (dddmm.mmmm)
    float raw_lon = tok ? atof(tok) : 0.0f;
    tok = strsep(&p, ",");       // 5: E/W
    char ew = (tok && tok[0]) ? tok[0] : 'E';

    tok = strsep(&p, ",");       // 6: Fix quality (0 = invalid, 1/2 = valid)
    int fix_q = tok ? atoi(tok) : 0;

    tok = strsep(&p, ",");       // 7: Number of satellites
    state.satellites = tok ? (uint8_t)atoi(tok) : 0;

    strsep(&p, ",");             // 8: HDOP
    tok = strsep(&p, ",");       // 9: Altitude ASL (meters)
    if (tok && tok[0]) {
      state.altitude_m = atof(tok);
    }

    if (fix_q > 0 && state.satellites >= 4) {
      state.fix_valid     = true;
      state.latitude_deg  = convertNMEADegMin(raw_lat, ns == 'S');
      state.longitude_deg = convertNMEADegMin(raw_lon, ew == 'W');

      // Kunci titik Home otomatis pertama kali dapat >= 5 satelit
      if (!state.home_set && state.satellites >= 5) {
        setHomeNow();
      }
      if (state.home_set) {
        updateHomeNavigation();
      }
    } else {
      state.fix_valid = false;
    }
  }

  void parseRMC(char *p) {
    // Format RMC: time,status(A/V),lat,N/S,lon,E/W,speed(knots),course(deg),...
    strsep(&p, ","); // 1: Time
    char *status = strsep(&p, ","); // 2: A=Active, V=Void
    if (!status || status[0] != 'A') return;

    strsep(&p, ","); // 3: Lat
    strsep(&p, ","); // 4: N/S
    strsep(&p, ","); // 5: Lon
    strsep(&p, ","); // 6: E/W

    char *spd_knots = strsep(&p, ","); // 7: Ground speed (knots)
    if (spd_knots && spd_knots[0]) {
      state.speed_kmh = atof(spd_knots) * 1.852f;
    }

    char *cog = strsep(&p, ","); // 8: Course Over Ground (Heading 0..360 deg)
    if (cog && cog[0]) {
      state.course_deg = atof(cog);
    }
  }

  float convertNMEADegMin(float val, bool negative) {
    int deg = (int)(val / 100.0f);
    float minutes = val - (deg * 100.0f);
    float dec = (float)deg + (minutes / 60.0f);
    return negative ? -dec : dec;
  }

  void updateHomeNavigation() {
    // Hitung jarak (meter) & arah sudut (bearing) dari posisi pesawat ke titik Home
    float dLat = (state.home_lat_deg - state.latitude_deg) * 111319.5f;
    float cosLat = cosf(state.latitude_deg * DEG_TO_RAD);
    float dLon = (state.home_lon_deg - state.longitude_deg) * 111319.5f * cosLat;

    state.dist_to_home_m = sqrtf(dLat * dLat + dLon * dLon);

    // Bearing menuju Home (0 = Utara, 90 = Timur, 180 = Selatan, 270 = Barat)
    float bearing = atan2f(dLon, dLat) * RAD_TO_DEG;
    if (bearing < 0.0f) bearing += 360.0f;
    state.bearing_to_home_deg = bearing;

    // Hitung selisih arah terbang pesawat (Course Over Ground) dengan arah ke Home (-180..+180)
    float heading_err = state.bearing_to_home_deg - state.course_deg;
    while (heading_err >  180.0f) heading_err -= 360.0f;
    while (heading_err < -180.0f) heading_err += 360.0f;

    // Jika sudah sampai di atas titik Home (< 20 meter), berputar melingkar (Loiter) dengan miring +20 derajat
    if (state.dist_to_home_m < 20.0f) {
      state.rth_roll_cmd_deg = 20.0f;
    } else {
      // Belokkan sayap menuju titik Home (maksimal miring 30 derajat)
      state.rth_roll_cmd_deg = constrain(heading_err * 0.6f, -30.0f, 30.0f);
    }

    // Jaga ketinggian pulang minimal 40 meter di atas titik lepas landas (Home)
    float target_alt = state.home_alt_m + 40.0f;
    float alt_err    = target_alt - state.altitude_m;
    state.rth_pitch_cmd_deg = constrain(alt_err * 0.5f, -10.0f, 15.0f);
  }
};

#endif // GPS_H
