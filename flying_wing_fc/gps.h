/*
 * ============================================================================
 * @file        gps.h
 * @project     Flying Wing Autonomous Flight Controller
 * @version     v0.4.1
 * @since       v0.2.0 (2026-10-20)
 * @updated     2026-10-24
 * @brief       Parser NMEA GPS ($GNGGA/$GPGGA, $GNRMC/$GPRMC), Auto-Baud, & Navigasi RTH
 *
 * Riwayat Perubahan (Changelog):
 *   - v0.2.0 (2026-10-20): Parser NMEA u-blox M10 / NEO-6M, Auto-Home lock (>=5 satelit),
 *                          serta perhitungan jarak & bearing Return-to-Home (RTH).
 *   - v0.4.1 (2026-10-24): Tambah Auto-Baudrate Scanner (115200/38400/57600/9600) saat
 *                          Disarmed & penghitung diagnostik byte/kalimat NMEA.
 * ============================================================================
 */
#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include "config.h"

struct GPSState {
  bool     connected          = false; // True jika kalimat NMEA valid diterima dalam 2.5 detik terakhir
  uint32_t baud_rate          = GPS_BAUD_RATE;
  uint32_t rx_bytes           = 0;
  uint32_t sentences_ok       = 0;
  bool     fix_valid          = false;
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

  void begin() {
    state.baud_rate = GPS_BAUD_RATE;
    last_baud_try_ms_ = millis();
    #if !defined(STM32F4xx) && !defined(ARDUINO_ARCH_STM32)
      #if defined(GPS_SERIAL)
        #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
          GPS_SERIAL.setTX(PIN_GPS_UART_TX);
          GPS_SERIAL.setRX(PIN_GPS_UART_RX);
          GPS_SERIAL.begin(state.baud_rate);
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
    parseChar(c);
  }

  void update(bool is_armed = false) {
    #if !defined(STM32F4xx) && !defined(ARDUINO_ARCH_STM32)
      #if defined(GPS_SERIAL)
        while (GPS_SERIAL.available()) {
          char c = (char)GPS_SERIAL.read();
          state.rx_bytes++;
          parseChar(c);
        }
      #endif
    #endif

    uint32_t now = millis();
    // Jika kalimat NMEA tidak masuk > 2.5 detik, tandai koneksi & fix tidak aktif
    if (now - last_sentence_ms_ > 2500) {
      state.fix_valid = false;
      state.connected = false;
    }

    // Auto-baudrate scanner (115200 -> 38400 -> 57600 -> 9600) saat DISARMED sebelum mengunci NMEA pertama
    #if defined(ARDUINO_ARCH_RP2040) || defined(TARGET_RP2040)
      if (!is_armed && !baud_locked_ && (now - last_baud_try_ms_ > 2500)) {
        last_baud_try_ms_ = now;
        static const uint32_t kBaudList[4] = {115200, 38400, 57600, 9600};
        baud_idx_ = (baud_idx_ + 1) & 0x03;
        state.baud_rate = kBaudList[baud_idx_];
        GPS_SERIAL.end();
        GPS_SERIAL.setTX(PIN_GPS_UART_TX);
        GPS_SERIAL.setRX(PIN_GPS_UART_RX);
        GPS_SERIAL.begin(state.baud_rate);
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
  char     line_buf_[96];
  uint8_t  line_idx_          = 0;
  uint32_t last_sentence_ms_  = 0;
  uint32_t last_baud_try_ms_  = 0;
  uint8_t  baud_idx_          = 0;
  bool     baud_locked_       = false;

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
