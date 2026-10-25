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
  bool     course_valid       = false; // True jika course over ground (COG) baru & valid (<= 2.5s)
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

    // Pisahkan timeout komunikasi umum, posisi valid, dan course valid (> 2.5 detik)
    if (!has_comm_ || (now - last_sentence_ms_ > 2500)) {
      state.connected    = false;
      state.fix_valid    = false;
      state.course_valid = false;
    }
    if (!has_fix_ || (now - last_fix_ms_ > 2500)) {
      state.fix_valid = false;
    }
    if (!has_course_ || (now - last_course_ms_ > 2500)) {
      state.course_valid = false;
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
  uint32_t last_fix_ms_       = 0;
  uint32_t last_course_ms_    = 0;
  uint32_t last_baud_try_ms_  = 0;
  uint8_t  scan_step_         = 0;
  bool     baud_locked_       = false;
  bool     has_comm_          = false;
  bool     has_fix_           = false;
  bool     has_course_        = false;

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
    uint32_t now      = millis();
    has_comm_         = true;
    last_sentence_ms_ = now;
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

      snprintf(state.last_nmea, sizeof(state.last_nmea),
               "UBX-NAV-PVT [%04u-%02u-%02u %02u:%02u:%02u UTC] Fix:%uD SV:%u",
               yr, mo, dy, hr, mn, sc, fix_type, num_sv);

      float lat_deg = (float)lat_e7 * 1e-7f;
      float lon_deg = (float)lon_e7 * 1e-7f;
      bool coord_ok = (lat_deg >= -90.0f && lat_deg <= 90.0f &&
                       lon_deg >= -180.0f && lon_deg <= 180.0f);
      bool fix_ok   = ((flags & 0x01) != 0) && (fix_type >= 2) && (num_sv >= 4) && coord_ok;

      if (fix_ok) {
        state.latitude_deg  = lat_deg;
        state.longitude_deg = lon_deg;
        state.altitude_m    = (float)h_msl_mm * 1e-3f;
        state.speed_kmh     = (g_spd_mm > 0) ? ((float)g_spd_mm * 0.0036f) : 0.0f;

        float cog = (float)head_e5 * 1e-5f;
        while (cog < 0.0f)    cog += 360.0f;
        while (cog >= 360.0f) cog -= 360.0f;
        state.course_deg    = cog;

        state.fix_valid     = true;
        has_fix_            = true;
        last_fix_ms_        = now;

        state.course_valid  = true;
        has_course_         = true;
        last_course_ms_     = now;

        if (!state.home_set && state.satellites >= 5 && fix_type >= 3) {
          setHomeNow();
        }
        if (state.home_set) {
          updateHomeNavigation();
        }
      } else {
        state.fix_valid    = false;
        has_fix_           = false;
        state.course_valid = false;
        has_course_        = false;
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

  static int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  }

  // Validasi checksum XOR standar NMEA 0183 (*XX di akhir kalimat)
  static bool verifyAndStripNMEAChecksum(char *line) {
    if (!line || line[0] != '$') return false;
    char *star = strchr(line + 1, '*');
    if (!star) return false;

    int hi = hexNibble(star[1]);
    int lo = hexNibble(star[2]);
    if (hi < 0 || lo < 0) return false;

    // Pastikan tidak ada karakter sampah setelah 2 digit hex checksum
    char tail = star[3];
    if (tail != '\0' && tail != '\r' && tail != '\n' && tail != ' ') return false;

    uint8_t calc_ck = 0;
    for (const char *p = line + 1; p < star; ++p) {
      if (*p == '$' || *p < 0x20 || *p > 0x7E) return false;
      calc_ck ^= (uint8_t)(*p);
    }

    uint8_t expected_ck = (uint8_t)((hi << 4) | lo);
    if (calc_ck != expected_ck) return false;

    *star = '\0'; // Potong '*XX' agar field terakhir bersih saat diparsing
    return true;
  }

  static bool parseFloatStrict(const char *s, float &out) {
    if (!s || s[0] == '\0') return false;
    char *endptr = nullptr;
    float val = strtof(s, &endptr);
    if (endptr == s || *endptr != '\0') return false;
    out = val;
    return true;
  }

  bool parseNMEACoord(const char *val_s, const char *dir_s, bool is_lon, float &out_deg) {
    if (!val_s || val_s[0] == '\0' || !dir_s || dir_s[0] == '\0' || dir_s[1] != '\0') {
      return false;
    }
    char dir = dir_s[0];
    if (!is_lon && dir != 'N' && dir != 'S') return false;
    if (is_lon  && dir != 'E' && dir != 'W') return false;

    float raw = 0.0f;
    if (!parseFloatStrict(val_s, raw) || raw < 0.0f) return false;

    int deg = (int)(raw / 100.0f);
    float minutes = raw - ((float)deg * 100.0f);
    if (minutes < 0.0f || minutes >= 60.0f) return false;

    float dec = (float)deg + (minutes / 60.0f);
    float max_deg = is_lon ? 180.0f : 90.0f;
    if (dec > max_deg) return false;

    bool negative = (dir == 'S' || dir == 'W');
    out_deg = negative ? -dec : dec;
    return true;
  }

  // Mendukung kalimat NMEA dari GPS u-blox M10 ($GNGGA/$GNRMC) maupun NEO-6M ($GPGGA/$GPRMC)
  void processSentence(char *line) {
    if (strncmp(line, "$GN", 3) != 0 && strncmp(line, "$GP", 3) != 0) {
      return;
    }

    // Simpan salinan kalimat sebelum '*XX' dipotong untuk tampilan CLI
    char raw_copy[64];
    strncpy(raw_copy, line, sizeof(raw_copy) - 1);
    raw_copy[sizeof(raw_copy) - 1] = '\0';

    // Wajib lulus verifikasi checksum NMEA (*XX) sebelum mengubah state apapun!
    if (!verifyAndStripNMEAChecksum(line)) {
      return;
    }

    has_comm_         = true;
    last_sentence_ms_ = millis();
    state.connected   = true;
    baud_locked_      = true;
    strncpy(state.last_nmea, raw_copy, sizeof(state.last_nmea) - 1);
    state.last_nmea[sizeof(state.last_nmea) - 1] = '\0';

    if (strncmp(line, "$GPGGA,", 7) == 0 || strncmp(line, "$GNGGA,", 7) == 0) {
      state.sentences_ok++;
      parseGGA(line + 7);
    } else if (strncmp(line, "$GPRMC,", 7) == 0 || strncmp(line, "$GNRMC,", 7) == 0) {
      state.sentences_ok++;
      parseRMC(line + 7);
    }
  }

  void parseGGA(char *p) {
    // Format GGA: time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,...
    char *time_s = strsep(&p, ","); // 1: Time
    char *lat_s  = strsep(&p, ","); // 2: Latitude (ddmm.mmmm)
    char *ns_s   = strsep(&p, ","); // 3: N/S
    char *lon_s  = strsep(&p, ","); // 4: Longitude (dddmm.mmmm)
    char *ew_s   = strsep(&p, ","); // 5: E/W
    char *fix_s  = strsep(&p, ","); // 6: Fix quality (0 = invalid, 1/2 = valid)
    char *sat_s  = strsep(&p, ","); // 7: Number of satellites
    char *hdop_s = strsep(&p, ","); // 8: HDOP
    char *alt_s  = strsep(&p, ","); // 9: Altitude ASL (meters)

    if (!time_s || !lat_s || !ns_s || !lon_s || !ew_s || !fix_s || !sat_s || !hdop_s || !alt_s) {
      state.fix_valid    = false;
      has_fix_           = false;
      state.course_valid = false;
      has_course_        = false;
      return;
    }

    int fix_q = (fix_s[0] != '\0') ? atoi(fix_s) : 0;
    int sats  = (sat_s[0] != '\0') ? atoi(sat_s) : 0;
    state.satellites = (uint8_t)((sats > 0) ? sats : 0);

    float lat_deg = 0.0f;
    float lon_deg = 0.0f;
    float alt_m   = 0.0f;
    bool coord_ok = parseNMEACoord(lat_s, ns_s, false, lat_deg) &&
                    parseNMEACoord(lon_s, ew_s, true,  lon_deg);
    bool alt_ok   = parseFloatStrict(alt_s, alt_m);

    if (fix_q > 0 && sats >= 4 && coord_ok && alt_ok) {
      state.fix_type      = 3;
      state.latitude_deg  = lat_deg;
      state.longitude_deg = lon_deg;
      state.altitude_m    = alt_m;
      state.fix_valid     = true;
      has_fix_            = true;
      last_fix_ms_        = millis();

      // Kunci titik Home otomatis pertama kali dapat >= 5 satelit
      if (!state.home_set && state.satellites >= 5) {
        setHomeNow();
      }
      if (state.home_set) {
        updateHomeNavigation();
      }
    } else {
      state.fix_type     = 0;
      state.fix_valid    = false;
      has_fix_           = false;
      state.course_valid = false;
      has_course_        = false;
    }
  }

  void parseRMC(char *p) {
    // Format RMC: time,status(A/V),lat,N/S,lon,E/W,speed(knots),course(deg),...
    char *time_s   = strsep(&p, ","); // 1: Time
    char *status_s = strsep(&p, ","); // 2: A=Active, V=Void
    char *lat_s    = strsep(&p, ","); // 3: Lat
    char *ns_s     = strsep(&p, ","); // 4: N/S
    char *lon_s    = strsep(&p, ","); // 5: Lon
    char *ew_s     = strsep(&p, ","); // 6: E/W
    char *spd_s    = strsep(&p, ","); // 7: Ground speed (knots)
    char *cog_s    = strsep(&p, ","); // 8: Course Over Ground (Heading 0..360 deg)

    if (!time_s || !status_s || status_s[0] != 'A') {
      state.fix_valid    = false;
      has_fix_           = false;
      state.course_valid = false;
      has_course_        = false;
      return;
    }

    float lat_deg = 0.0f;
    float lon_deg = 0.0f;
    if (!parseNMEACoord(lat_s, ns_s, false, lat_deg) ||
        !parseNMEACoord(lon_s, ew_s, true,  lon_deg)) {
      state.fix_valid    = false;
      has_fix_           = false;
      state.course_valid = false;
      has_course_        = false;
      return;
    }

    // Jika fix GGA sudah aktif dan segar, perbarui juga koordinat dari RMC
    if (state.fix_valid && has_fix_ && (millis() - last_fix_ms_ <= 2500)) {
      state.latitude_deg  = lat_deg;
      state.longitude_deg = lon_deg;
    }

    float spd_knots = 0.0f;
    if (parseFloatStrict(spd_s, spd_knots) && spd_knots >= 0.0f) {
      state.speed_kmh = spd_knots * 1.852f;
    }

    float cog = 0.0f;
    if (parseFloatStrict(cog_s, cog)) {
      while (cog < 0.0f)    cog += 360.0f;
      while (cog >= 360.0f) cog -= 360.0f;
      state.course_deg   = cog;
      state.course_valid = true;
      has_course_        = true;
      last_course_ms_    = millis();
      if (state.fix_valid && state.home_set) {
        updateHomeNavigation();
      }
    } else {
      state.course_valid = false;
      has_course_        = false;
    }
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
    if (state.dist_to_home_m < RTH_HOME_RADIUS_M) {
      state.rth_roll_cmd_deg = 20.0f;
    } else if (state.course_valid) {
      // Belokkan sayap menuju titik Home (maksimal miring 30 derajat)
      state.rth_roll_cmd_deg = constrain(heading_err * 0.6f, -MAX_NAV_ROLL_DEG, MAX_NAV_ROLL_DEG);
    } else {
      state.rth_roll_cmd_deg = 15.0f;
    }

    // Jaga ketinggian pulang minimal 40 meter di atas titik lepas landas (Home)
    float target_alt = state.home_alt_m + 40.0f;
    float alt_err    = target_alt - state.altitude_m;
    state.rth_pitch_cmd_deg = constrain(alt_err * 0.5f, -10.0f, 15.0f);
  }
};

#endif // GPS_H
