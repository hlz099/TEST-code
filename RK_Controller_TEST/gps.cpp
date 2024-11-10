#include "rc.h"

#define GPS_RX              27
#define GPS_TX              14
#define GPS_FREQ          2000
#define GPS_READ_TIMEOUT    40
#define GPS_REPORT       60000

HardwareSerial gps_uart(1);

float gps_last_lat = NAN;
float gps_last_lon = NAN;
float gps_last_t = 0;

bool gps_signal = false;

bool gps_status = false;

bool checksum(String line) {
  if(line.charAt(0) != '$' and line.charAt(0) != '!') return false;
  int i = line.indexOf('*');
  if(i < 0 or i != line.length() - 3) return false;
  char s = 0;
  for(int j = 1; j < i; j ++) s ^= line.charAt(j);
  return s == (char)strtol(line.substring(i + 1).c_str(), NULL, 16);
}

String get_line_element(String line, int n) {
  int start = 0;
  for(int i = 0; i < n; i ++) {
    start = line.indexOf(',', start) + 1;
    if(start < 0) return "";
  }
  int end = line.indexOf(',', start);
  if(end < 0) end = line.indexOf('*', start);
  if(end < 0) return "";
  return line.substring(start, end);
}

bool parse_gps(String &rmc_line) {
  bool got_coords = false;
  String line = "";
  while(gps_uart.available()) {
    char c = gps_uart.read();
    if(c == '\r') continue;
    if(c == '\n') {
      if(checksum(line) and line.substring(3, 6) == "RMC") {
        rmc_line = line;
        //        log("RMC line: " + line);
        String lat_str = get_line_element(line, 3);
        String lat_sign_str = get_line_element(line, 4);
        String lon_str = get_line_element(line, 5);
        String lon_sign_str = get_line_element(line, 6);
        //        log("Lat: " + lat_str);
        //        log("Lat sign: " + lat_sign_str);
        //        log("Lon: " + lon_str);
        //        log("Lon sign: " + lon_sign_str);
        if(lat_str.length() and lat_sign_str.length() and lat_str.length() and lat_sign_str.length()) {
          gps_last_lat = (lat_str.substring(0, 2).toFloat() + lat_str.substring(2).toFloat() * 0.016666666666666666f) * (lat_sign_str == "N" ? 1.0f : -1.0f);
          gps_last_lon = (lon_str.substring(0, 3).toFloat() + lon_str.substring(3).toFloat() * 0.016666666666666666f) * (lon_sign_str == "E" ? 1.0f : -1.0f);
          got_coords = true;
        }
      }
      line = "";
    } else {
      line += c;
    }
    if(!gps_uart.available()) delay(GPS_READ_TIMEOUT);
  }
  return got_coords;
}

void gps_parser(void *_) {
  long t = millis();
  String rmc_line;
  for(;;) {
    if(parse_gps(rmc_line)) {
      log("GPS: lat=" + String(gps_last_lat, 6) + "; lon=" + String(gps_last_lon, 6));
      gps_last_t = millis();
      if(!gps_signal) {
        gps_signal = true;
        send_sms((String)"Got gps signal at " + millis2str() + "; lat=" + String(gps_last_lat, 6) + "; lon=" + String(gps_last_lon, 6));
      }
    } else if(millis() - t > GPS_REPORT) {
      log("GPS: no signal at " + millis2str() + "; last signal: lat=" + String(gps_last_lat, 6) + "; lon=" + String(gps_last_lon, 6) + "; last RMC line: " + rmc_line);
      t += GPS_REPORT;
    }
    if(millis() - gps_last_t > GPS_FREQ && gps_signal) {
      gps_signal = false;
      send_sms((String)"Lost gps signal at " + millis2str() + "; lat=" + String(gps_last_lat, 6) + "; lon=" + String(gps_last_lon, 6));
    }
    delay(100);
  }
}

void init_gps() {
  gps_uart.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);

  long t = millis();
  String line = "";
  while(millis() - t < 1500) {
    if(gps_uart.available()) {
      char c = gps_uart.read();
      if(c == '\r') continue;
      if(c == '\n') {
        gps_status = checksum(line);
        line = "";
      } else {
        line += c;
      }
    }
    else delay(GPS_READ_TIMEOUT);
  }

  if(gps_status) log("NEO-6M connected");
  else log("NEO-6M not connected");

  xTaskCreatePinnedToCore(gps_parser, "gps_parser", DEFALT_STACK, NULL, 0, NULL, 1);
}
