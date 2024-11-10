#include "rc.h"

#define SD_CS               33
#define SD_SCK              26
#define SD_MISO             32
#define SD_MOSI             25
#define SD_SW               34

#define alphabet ((String)"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789.,")

extern float accel_transform[];
extern float gyro_transform[];
extern float gravity;
extern float approx_gravity;
extern float accel_bx;
extern float accel_by;
extern float accel_bz;
extern float gyro_bx;
extern float gyro_by;
extern float gyro_bz;
extern int gyro_accel_rotations;
extern long gyro_accel_calibration_timeout;
extern long accel_peace_time;
extern long accel_peace_edges;
extern long accel_calibration_timeout;
extern float accel_min_dist;
extern float accel_peace_delta;
extern long gyro_calibration_time;
extern int out1_role;
extern int out2_role;
extern int out3_role;
extern int out4_role;
extern long gsm_call_detector_cooldown;
extern long gsm_ping_cooldown;
extern float start_detect_threshold;
extern float ground_detect_threshold;
extern long start_time;
extern long parachute_time;
extern bool ignite_manually;
extern float R;
extern float mu;
extern float k;
extern long report_cooldown;
extern long beep_on;
extern long beep_off;
extern long beep_freq;
extern int mode;
extern long fall_detection_time;

SPIClass sd_spi(HSPI);

File log_file;

bool sd_status;
bool sd_detect_status;

String phone_number = "";

String millis2str(long t) {
  if(t < 0) t = millis();
  String s = "";
  if(t >= 3600000) {
    s += t / 3600000;
    s += t >= 7200000 ? " hours " : " hour ";
  }
  if(t >= 60000) {
    s += (t / 60000) % 60;
    s += " min ";
  }
  s += (t / 1000) % 60;
  s += ".";
  s += (t / 100) % 10;
  s += (t / 10) % 10;
  s += t % 10;
  s += " sec";
  return s;
}

String message_queue = "";

bool log_deamon_running = false;

SemaphoreHandle_t log_lock = NULL;
SemaphoreHandle_t file_writing_lock = NULL;

extern String run_name;

void log_deamon(void *_) {
  Serial.println("Log deamon started. Run name: " + run_name);
  xSemaphoreTake(file_writing_lock, portMAX_DELAY);
  log_file.println("Log deamon started. Run name: " + run_name);
  log_file.flush();
  xSemaphoreGive(file_writing_lock);
  log_deamon_running = true;
  for(;;) {
    xSemaphoreTake(log_lock, portMAX_DELAY);
    if(message_queue.length()) {
      int i = message_queue.indexOf((char)26);
      String message;
      if(i >= 0) {
        message = message_queue.substring(0, i);
        message_queue = message_queue.substring(i + 1);
      } else {
        message = message_queue;
        message_queue = "";
      }
      xSemaphoreGive(log_lock);
      Serial.println(message);
      xSemaphoreTake(file_writing_lock, portMAX_DELAY);
      log_file.print(millis2str() + ": ");
      log_file.println(message);
      log_file.flush();
      xSemaphoreGive(file_writing_lock);
    } else xSemaphoreGive(log_lock);
    delay(20);
  }
}

void log(String msg) {
  if(log_deamon_running) {
    xSemaphoreTake(log_lock, portMAX_DELAY);
    if(message_queue.length())
      message_queue += (char)26;
    message_queue += msg;
    xSemaphoreGive(log_lock);
  } else {
    Serial.println("(log deamon not started yet): " + msg);
  }
}

int read_config_line(String line, String *name, String *value) {
  if(!line.length()) return 0;
  int i = 0;
  while(line.charAt(i) == ' ' or line.charAt(i) == '\t') {
    i ++;
    if(i >= line.length()) return 0;
  }
  if(line.charAt(i) == '#') return 0;
  int name_start = i;
  while(alphabet.indexOf(line.charAt(i)) >= 0) {
    i ++;
    if(i >= line.length()) return -1;
  }
  int name_end = i;
  if(name_start == name_end) return -1;
  while(line.charAt(i) == ' ' or line.charAt(i) == '\t') {
    i ++;
    if(i >= line.length()) return -1;
  }
  if(line.charAt(i) != '=') return -1;
  i ++; if(i >= line.length()) return -1;
  while(line.charAt(i) == ' ' or line.charAt(i) == '\t') {
    i ++;
    if(i >= line.length()) return -1;
  }
  int value_start = i;
  if(line.charAt(i) == '+' or line.charAt(i) == '-') i ++;
  if(i >= line.length()) return -1;
  while(alphabet.indexOf(line.charAt(i)) >= 0 || (line.charAt(i - 1) == ',' && (line.charAt(i) == '+' || line.charAt(i) == '-'))) {
    i ++;
    if(i >= line.length()) break;
  }
  int value_end = i;
  if(value_start == value_end) return -1;
  if(i >= line.length()) {
    *name = line.substring(name_start, name_end);
    *value = line.substring(value_start, value_end);
    return 1;
  }
  while(line.charAt(i) == ' ' or line.charAt(i) == '\t') {
    i ++;
    if(i >= line.length()) break;
  }
  if(i >= line.length() or line.charAt(i) == '#' or (line.charAt(i) == '\r' and i == line.length() - 1)) {
    *name = line.substring(name_start, name_end);
    *value = line.substring(value_start, value_end);
    return 1;
  }
  return -1;
}

bool contains_float(String s) {
  if(((String)"0123456789+-.").indexOf(s.charAt(0)) < 0) return false;
  if(s.charAt(0) == '+' or s.charAt(0) == '-') s = s.substring(1, s.length());
  for(int i = 0; i < s.length(); i ++) {
    if(((String)"0123456789.").indexOf(s.charAt(i)) < 0) return false;
  }
  if(s.indexOf('.') != s.lastIndexOf('.')) return false;
  if(s == "" or s == ".") return false;
  return true;
}

bool contains_int(String s) {
  if(((String)"0123456789+-").indexOf(s.charAt(0)) < 0) return false;
  if(s.charAt(0) == '+' or s.charAt(0) == '-') s = s.substring(1, s.length());
  for(int i = 0; i < s.length(); i ++) {
    if(((String)"0123456789").indexOf(s.charAt(i)) < 0) return false;
  }
  if(s == "") return false;
  return true;
}

#define names 39
bool read_config() {
  int occurrences[names];
  for(int i = 0; i < names; i ++) occurrences[i] = 0;
  File conf = SD.open(CONFIG, FILE_READ);
  while(conf.available()) {
    String line = conf.readStringUntil('\n');
    String name;
    String value;
    int retval = read_config_line(line, &name, &value);
    if(retval < 0) {
      log("Error reading config line \"" + line + "\"");
      return false;
    }
    if(retval) {
      if(name == "mode")                                occurrences[0] ++;
      else if(name == "phone")                          occurrences[1] ++;
      else if(name == "call_detector_cooldown")         occurrences[2] ++;
      else if(name == "ping_cooldown")                  occurrences[3] ++;
      else if(name == "report_cooldown")                occurrences[4] ++;
      else if(name == "start_detect_threshold")         occurrences[5] ++;
      else if(name == "ground_detect_threshold")        occurrences[6] ++;
      else if(name == "fall_detection_time")            occurrences[7] ++;
      else if(name == "out1_role")                      occurrences[8] ++;
      else if(name == "out2_role")                      occurrences[9] ++;
      else if(name == "out3_role")                      occurrences[10] ++;
      else if(name == "out4_role")                      occurrences[11] ++;
      else if(name == "beep_on")                        occurrences[12] ++;
      else if(name == "beep_off")                       occurrences[13] ++;
      else if(name == "beep_freq")                      occurrences[14] ++;
      else if(name == "start_time")                     occurrences[15] ++;
      else if(name == "ignite_manually")                occurrences[16] ++;
      else if(name == "parachute_time")                 occurrences[17] ++;
      else if(name == "accel_transform")                occurrences[18] ++;
      else if(name == "accel_bx")                       occurrences[19] ++;
      else if(name == "accel_by")                       occurrences[20] ++;
      else if(name == "accel_bz")                       occurrences[21] ++;
      else if(name == "gyro_transform")                 occurrences[22] ++;
      else if(name == "gyro_bx")                        occurrences[23] ++;
      else if(name == "gyro_by")                        occurrences[24] ++;
      else if(name == "gyro_bz")                        occurrences[25] ++;
      else if(name == "R")                              occurrences[26] ++;
      else if(name == "mu")                             occurrences[27] ++;
      else if(name == "k")                              occurrences[28] ++;
      else if(name == "g")                              occurrences[29] ++;
      else if(name == "approx_g")                       occurrences[30] ++;
      else if(name == "accel_peace_time")               occurrences[31] ++;
      else if(name == "accel_peace_edges")              occurrences[32] ++;
      else if(name == "accel_calibration_timeout")      occurrences[33] ++;
      else if(name == "accel_min_dist")                 occurrences[34] ++;
      else if(name == "accel_peace_delta")              occurrences[35] ++;
      else if(name == "gyro_calibration_time")          occurrences[36] ++;
      else if(name == "gyro_accel_calibration_timeout") occurrences[37] ++;
      else if(name == "gyro_accel_rotations")           occurrences[38] ++;
      else {
        log("Unknown config line \"" + line + "\"");
        return false;
      }
    }
  }
  conf.close();
  for(int i = 0; i < names; i ++)
    if(occurrences[i] != 1) {
      log(occurrences[i] ? "Repeating config lines" : "Not all expected config lines are present");
      //log((String)i);
      return false;
    }
  conf = SD.open(CONFIG, FILE_READ);
  while(conf.available()) {
    String line = conf.readStringUntil('\n');
    String name;
    String value;
    int retval = read_config_line(line, &name, &value);
    if(retval < 0) return false;
    if(retval) {
      if(name == "mode") {
        if(value != "monitor" && value != "flight" && value != "test") { log("Illegal value of mode, should be \"monitor\", \"flight\" or \"test\", got \"" + value + "\""); return false; }
        mode = (value == "monitor" ? MODE_MONITOR : (value == "flight" ? MODE_FLIGHT : MODE_TEST));
      } else if(name == "phone") {
        phone_number = value;
      } else if(name == "call_detector_cooldown") {
        if(!contains_int(value)) { log("Illegal value of call_detector_cooldown \"" + value + "\""); return false; }
        gsm_call_detector_cooldown = value.toInt();
      } else if(name == "ping_cooldown") {
        if(!contains_int(value)) { log("Illegal value of ping_cooldown \"" + value + "\""); return false; }
        gsm_ping_cooldown = value.toInt();
      } else if(name == "report_cooldown") {
        if(!contains_int(value)) { log("Illegal value of report_cooldown \"" + value + "\""); return false; }
        report_cooldown = value.toInt();
      } else if(name == "start_detect_threshold") {
        if(!contains_float(value)) { log("Illegal value of start_detect_threshold \"" + value + "\""); return false; }
        start_detect_threshold = value.toFloat();
      } else if(name == "ground_detect_threshold") {
        if(!contains_float(value)) { log("Illegal value of ground_detect_threshold \"" + value + "\""); return false; }
        ground_detect_threshold = value.toFloat();
      } else if(name == "fall_detection_time") {
        if(!contains_int(value)) { log("Illegal value of fall_detection_time \"" + value + "\""); return false; }
        fall_detection_time = value.toInt();
      } else if(name == "out1_role") {
        if(value == "start")          out1_role = OUT_START;
        else if(value == "parachute") out1_role = OUT_PARACHUTE;
        else if(value == "speaker")   out1_role = OUT_SPEAKER;
        else if(value == "buzzer")    out1_role = OUT_BUZZER;
        else if(value == "none")      out1_role = OUT_NONE;
        else { log("Illegal value of out1_role, should be \"start\", \"parachute\", \"speaker\", \"buzzer\" or \"none\", got \"" + value + "\""); return false; }
      } else if(name == "out2_role") {
        if(value == "start")          out2_role = OUT_START;
        else if(value == "parachute") out2_role = OUT_PARACHUTE;
        else if(value == "speaker")   out2_role = OUT_SPEAKER;
        else if(value == "buzzer")    out2_role = OUT_BUZZER;
        else if(value == "none")      out2_role = OUT_NONE;
        else { log("Illegal value of out2_role, should be \"start\", \"parachute\", \"speaker\", \"buzzer\" or \"none\", got \"" + value + "\""); return false; }
      } else if(name == "out3_role") {
        if(value == "start")          out3_role = OUT_START;
        else if(value == "parachute") out3_role = OUT_PARACHUTE;
        else if(value == "speaker")   out3_role = OUT_SPEAKER;
        else if(value == "buzzer")    out3_role = OUT_BUZZER;
        else if(value == "none")      out3_role = OUT_NONE;
        else { log("Illegal value of out3_role, should be \"start\", \"parachute\", \"speaker\", \"buzzer\" or \"none\", got \"" + value + "\""); return false; }
      } else if(name == "out4_role") {
        if(value == "start")          out4_role = OUT_START;
        else if(value == "parachute") out4_role = OUT_PARACHUTE;
        else if(value == "speaker")   out4_role = OUT_SPEAKER;
        else if(value == "buzzer")    out4_role = OUT_BUZZER;
        else if(value == "none")      out4_role = OUT_NONE;
        else { log("Illegal value of out4_role, should be \"start\", \"parachute\", \"speaker\", \"buzzer\" or \"none\", got \"" + value + "\""); return false; }
      } else if(name == "beep_on") {
        if(!contains_int(value)) { log("Illegal value of beep_on \"" + value + "\""); return false; }
        beep_on = value.toInt();
      } else if(name == "beep_off") {
        if(!contains_int(value)) { log("Illegal value of beep_off \"" + value + "\""); return false; }
        beep_off = value.toInt();
      } else if(name == "beep_freq") {
        if(!contains_int(value)) { log("Illegal value of beep_freq \"" + value + "\""); return false; }
        beep_freq = value.toInt();
      } else if(name == "start_time") {
        if(!contains_int(value)) { log("Illegal value of start_time \"" + value + "\""); return false; }
        start_time = value.toInt();
      } else if(name == "ignite_manually") {
        if(value != "true" && value != "false") { log("Illegal value of ignite_manually \"" + value + "\""); return false; }
        ignite_manually = (value == "true");
      } else if(name == "parachute_time") {
        if(!contains_int(value)) { log("Illegal value of parachute_time \"" + value + "\""); return false; }
        parachute_time = value.toInt();
      } else if(name == "accel_transform") {
        for(int i = 0; i < 8; i ++) {
          int j = value.indexOf(',');
          if(j < 0) { log("Illegal value of accel_transform, got " + String(i + 1) + "/9 values"); return false; }
          String value_i = value.substring(0, j);
          value = value.substring(j + 1, value.length());
          if(!contains_float(value_i)) { log("Illegal value of accel_transform's element " + String(i + 1) + ": \"" + value_i + "\""); return false; }
          accel_transform[i] = value_i.toFloat();
        }
        if(value.indexOf(',') >= 0) { log("Illegal value of accel_transform, got more than 9 values"); return false; }
        if(!contains_float(value)) { log("Illegal value of accel_transform's element 9: \"" + value + "\""); return false; }
        accel_transform[8] = value.toFloat();
      } else if(name == "accel_bx") {
        if(!contains_float(value)) { log("Illegal value of accel_bx \"" + value + "\""); return false; }
        accel_bx = value.toFloat();
      } else if(name == "accel_by") {
        if(!contains_float(value)) { log("Illegal value of accel_by \"" + value + "\""); return false; }
        accel_by = value.toFloat();
      } else if(name == "accel_bz") {
        if(!contains_float(value)) { log("Illegal value of accel_bz \"" + value + "\""); return false; }
        accel_bz = value.toFloat();
      } else if(name == "R") {
        if(!contains_float(value)) { log("Illegal value of R \"" + value + "\""); return false; }
        R = value.toFloat();
      } else if(name == "mu") {
        if(!contains_float(value)) { log("Illegal value of mu \"" + value + "\""); return false; }
        mu = value.toFloat() * 1e-3; // kg/mol
      } else if(name == "k") {
        if(!contains_float(value)) { log("Illegal value of k \"" + value + "\""); return false; }
        k = value.toFloat() * 1e-3; // K/m
      } else if(name == "gyro_transform") {
        for(int i = 0; i < 8; i ++) {
          int j = value.indexOf(',');
          if(j < 0) { log("Illegal value of gyro_transform, got " + String(i + 1) + "/9 values"); return false; }
          String value_i = value.substring(0, j);
          value = value.substring(j + 1, value.length());
          if(!contains_float(value_i)) { log("Illegal value of gyro_transform's element " + String(i + 1) + ": \"" + value_i + "\""); return false; }
          gyro_transform[i] = value_i.toFloat();
        }
        if(value.indexOf(',') >= 0) { log("Illegal value of gyro_transform, got more than 9 values"); return false; }
        if(!contains_float(value)) { log("Illegal value of gyro_transform's element 9: \"" + value + "\""); return false; }
        gyro_transform[8] = value.toFloat();
      } else if(name == "gyro_bx") {
        if(!contains_float(value)) { log("Illegal value of gyro_bx \"" + value + "\""); return false; }
        gyro_bx = value.toFloat();
      } else if(name == "gyro_by") {
        if(!contains_float(value)) { log("Illegal value of gyro_by \"" + value + "\""); return false; }
        gyro_by = value.toFloat();
      } else if(name == "gyro_bz") {
        if(!contains_float(value)) { log("Illegal value of gyro_bz \"" + value + "\""); return false; }
        gyro_bz = value.toFloat();
      } else if(name == "g") {
        if(!contains_float(value)) { log("Illegal value of g \"" + value + "\""); return false; }
        gravity = value.toFloat();
      } else if(name == "approx_g") {
        if(!contains_float(value)) { log("Illegal value of approx_g \"" + value + "\""); return false; }
        approx_gravity = value.toFloat();
      } else if(name == "accel_peace_time") {
        if(!contains_int(value)) { log("Illegal value of accel_peace_time \"" + value + "\""); return false; }
        accel_peace_time = value.toInt();
      } else if(name == "accel_peace_edges") {
        if(!contains_int(value)) { log("Illegal value of accel_peace_edges \"" + value + "\""); return false; }
        accel_peace_edges = value.toInt();
      } else if(name == "accel_calibration_timeout") {
        if(!contains_int(value)) { log("Illegal value of accel_calibration_timeout \"" + value + "\""); return false; }
        accel_calibration_timeout = value.toInt();
      } else if(name == "accel_min_dist") {
        if(!contains_float(value)) { log("Illegal value of accel_min_dist \"" + value + "\""); return false; }
        accel_min_dist = value.toFloat();
      } else if(name == "accel_peace_delta") {
        if(!contains_float(value)) { log("Illegal value of accel_peace_delta \"" + value + "\""); return false; }
        accel_peace_delta = value.toFloat();
      } else if(name == "gyro_calibration_time") {
        if(!contains_int(value)) { log("Illegal value of gyro_calibration_time \"" + value + "\""); return false; }
        gyro_calibration_time = value.toInt();
      } else if(name == "gyro_accel_calibration_timeout") {
        if(!contains_int(value)) { log("Illegal value of gyro_accel_calibration_timeout \"" + value + "\""); return false; }
        gyro_accel_calibration_timeout = value.toInt();
      } else if(name == "gyro_accel_rotations") {
        if(!contains_int(value)) { log("Illegal value of gyro_accel_rotations \"" + value + "\""); return false; }
        gyro_accel_rotations = value.toInt();
      } else {
        log("Unknown config line \"" + line + "\"");
        return false;
      }
    }
  }
  conf.close();
  return true;
}

void default_config() {
  SD.remove(CONFIG);
  File conf = SD.open(CONFIG, FILE_WRITE);
  conf.println("# in \"monitor\" mode the system will constantly send it's position to serial port, recieving \"reset\" from serial resets position");
  conf.println("# in \"flight\" mode the system will wait for a phone call then ignight the motor or wait for it to be ignighted, depending on ignite_manually");
  conf.println("# in \"test\" mode the system will wait for a phone call then fire parachute and start signaling");
  conf.println("mode = test");
  conf.println("phone = 0 # phone number");
  conf.println("call_detector_cooldown = 10000 # what time shall pass between phone calls (ms). To fix shitty bug");
  conf.println("ping_cooldown = 60000 # how often to send ping messages before start (ms). To fix a bug where sim800l stops accepting incoming calls. 0 disables ping messages");
  conf.println("report_cooldown = 60000 # how often to send reports during flight (ms). 0 disables reports");
  conf.println("start_detect_threshold = 1.0 # upwards acceleration needed to detect launch (m/s^2)");
  conf.println("ground_detect_threshold = 0.6 # how low should vibratins drop for detecting landing, <0.3 won't work (m/s^2)");
  conf.println("fall_detection_time = 60000 # how long shall vibrations be <ground_detect_threshold in order to detect landing (ms)");
  conf.println("# roles: start - motor ignition (if ignite_manually = true); parachute - parachute firing; speaker - beep after falling to the ground (generates frequency); buzzer - beep after falling (generates long pulces, no frequency)");
  conf.println("out1_role = start");
  conf.println("out2_role = parachute");
  conf.println("out3_role = speaker");
  conf.println("out4_role = none");
  conf.println("beep_on = 1000 # length of beep signal (ms)");
  conf.println("beep_off = 500 # delay between beep signals (ms)");
  conf.println("beep_freq = 700 # beep frequency, for out_role = speaker (hz)");
  conf.println("ignite_manually = false # whether ignite the motor after 1st call (true) or wait to be ignited (false)");
  conf.println("start_time = 1500 # length of the pulse to ignite motor (ms)");
  conf.println("parachute_time = 1500 # length of the pulse to fire parachute (ms)");
  conf.println("# Accel calibratuon:");
  conf.println("accel_transform = 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0 # No spaces after commas pls");
  conf.println("accel_bx = 0.0");
  conf.println("accel_by = 0.0");
  conf.println("accel_bz = 0.0");
  conf.println("# Gyro calibratuon:");
  conf.println("gyro_transform = 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0 # No spaces after commas pls");
  conf.println("gyro_bx = 0.0");
  conf.println("gyro_by = 0.0");
  conf.println("gyro_bz = 0.0");
  conf.println("# Some physics constants:");
  conf.println("R = 8.31446 # universal gas constant (J/(k*mol))");
  conf.println("mu = 28.98 # molar mass of air (g/mol)");
  conf.println("k = 6.5 # temperature gradient, how mach teperature decreaces per kilometer (K/km)");
  conf.println("g = 9.80665");
  conf.println("approx_g = 9.3 # for integrating acceleration assuming rocket is perfectly aligned upwards (which it's not, so fixing with slightly lower value)");
  conf.println("# Some constants that affect the accel/gyro calibration process:");
  conf.println("accel_peace_time = 2000 # time enough to detect a stationary point (ms)");
  conf.println("accel_peace_edges = 300 # the edges of stationary period that do not count as stationary (ms)");
  conf.println("accel_calibration_timeout = 40000 # ms");
  conf.println("accel_min_dist = 8.0 # min euclidean distance between stationary point's acceleration vectors for those points to count as separate ones (m/s^2)");
  conf.println("accel_peace_delta = 0.18 # how much acceleration can vary within a stationary period (m/s^2)");
  conf.println("gyro_calibration_time = 4000 # to calibrate b values (ms)");
  conf.println("gyro_accel_calibration_timeout = 100000 # ms");
  conf.println("gyro_accel_rotations = 9 # how many pi/2 rotations do we need to calibrate gyro and accel");
  conf.println("# this is the default config. It was written here since there were no config or there were an error in the config");
}

void init_sd() {
  log_lock = xSemaphoreCreateMutex();
  file_writing_lock = xSemaphoreCreateMutex();
  //xSemaphoreGive(log_lock);
  //xSemaphoreGive(file_writing_lock);
  
  pinMode(SD_SW, INPUT);
  pinMode(SD_CS, OUTPUT);
  sd_detect_status = digitalRead(SD_SW) == LOW;

  if(sd_detect_status) log("SD detected");
  else log("SD not detected");

  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sd_status = SD.begin(SD_CS, sd_spi);

  if(sd_status) log("SD connected");
  else log("SD not connected");

  int log_number = 0;
  while(SD.exists((String)"/log" + (++log_number)));
  log_file = SD.open((String)"/log" + log_number, FILE_WRITE);

  xTaskCreatePinnedToCore(log_deamon, "log", DEFALT_STACK, NULL, 1, NULL, 1);
 
  if(!read_config()) {
    log("Error reading config");
    default_config();
    if(read_config()) log("Config written succesfully");
    else log("Error writing config");
  } else {
    log("Config read successfully");
  }
}
