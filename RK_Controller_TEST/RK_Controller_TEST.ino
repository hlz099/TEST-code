#include "rc.h"

TwoWire i2c(0);

bool flying = true;

// ====== MicroSD ======
// ====== BMP280 ======

#define BMP280_ADDR     0x77

TwoWire bmp280_i2c(0);
Adafruit_BMP280 bmp280(&i2c);

bool bmp280_status;

float R;
float mu;
float k;
extern float gravity;
extern float max_a;

float pressure2alt(float T0, float p0, float p) {
  return T0 / k * (1. - pow(p / p0, R * k / (mu * gravity)));
}

// ====== LSM6DSO32 (Gyro/Accel) ======

extern float x, y, z;
extern float vx, vy, vz;
extern float smooth_ax, smooth_ay, smooth_az;
extern float smooth_rx, smooth_ry, smooth_rz;
extern double rotation[];
extern float approx_vy;
extern float approx_y;
extern float upwards_acceleration;
extern SemaphoreHandle_t ag_deamon_lock;
extern SemaphoreHandle_t temp_request1;
extern SemaphoreHandle_t temp_request2;
extern SemaphoreHandle_t press_request1;
extern SemaphoreHandle_t press_request2;
extern float temp;
extern float press;

// ====== NEO-6M (GPS) ======

extern float gps_last_lat;
extern float gps_last_lon;

// ====== SIM800L (GSM) ======

extern bool gsm_phone_call_detected;

// ====== OUTS ======

#define OUT1              21
#define OUT2              19
#define OUT3              17
#define OUT4              18
#define VIN_HOLD          23

int out1_role = OUT_NONE;
int out2_role = OUT_NONE;
int out3_role = OUT_NONE;
int out4_role = OUT_NONE;
long start_time = 1500;
long parachute_time = 1500;

hw_timer_t *parachute_timer;

void parachute_end() {
  if(out1_role == OUT_PARACHUTE) digitalWrite(OUT1, LOW);
  if(out2_role == OUT_PARACHUTE) digitalWrite(OUT2, LOW);
  if(out3_role == OUT_PARACHUTE) digitalWrite(OUT3, LOW);
  if(out4_role == OUT_PARACHUTE) digitalWrite(OUT4, LOW);
  log("The parachute should have been fired by now!");
}

void parachute() {
  if(out1_role != OUT_PARACHUTE &&
     out2_role != OUT_PARACHUTE &&
     out3_role != OUT_PARACHUTE &&
     out4_role != OUT_PARACHUTE) return;
  timerWrite(parachute_timer, 0);
  timerAlarmEnable(parachute_timer);
  log("Firing parachute...");
  if(out1_role == OUT_PARACHUTE) digitalWrite(OUT1, HIGH);
  if(out2_role == OUT_PARACHUTE) digitalWrite(OUT2, HIGH);
  if(out3_role == OUT_PARACHUTE) digitalWrite(OUT3, HIGH);
  if(out4_role == OUT_PARACHUTE) digitalWrite(OUT4, HIGH);
}

hw_timer_t *start_timer;

void IRAM_ATTR start_end() {
  if(out1_role == OUT_START) digitalWrite(OUT1, LOW);
  if(out2_role == OUT_START) digitalWrite(OUT2, LOW);
  if(out3_role == OUT_START) digitalWrite(OUT3, LOW);
  if(out4_role == OUT_START) digitalWrite(OUT4, LOW);
  log("The motor should have been ignited by now!");
}

void start() {
  if(out1_role != OUT_START &&
     out2_role != OUT_START &&
     out3_role != OUT_START &&
     out4_role != OUT_START) return;
  timerWrite(start_timer, 0);
  timerAlarmEnable(start_timer);
  log("Igniting motor...");
  if(out1_role == OUT_START) digitalWrite(OUT1, HIGH);
  if(out2_role == OUT_START) digitalWrite(OUT2, HIGH);
  if(out3_role == OUT_START) digitalWrite(OUT3, HIGH);
  if(out4_role == OUT_START) digitalWrite(OUT4, HIGH);
}

long beep_on = 1000;
long beep_off = 500;
long beep_freq = 700;

void _signal(void* _) {
  for(;;) {
    long t = micros();
    if(out1_role == OUT_BUZZER) digitalWrite(OUT1, HIGH);
    if(out2_role == OUT_BUZZER) digitalWrite(OUT2, HIGH);
    if(out3_role == OUT_BUZZER) digitalWrite(OUT3, HIGH);
    if(out4_role == OUT_BUZZER) digitalWrite(OUT4, HIGH);
    long t2 = t;
    int n = 0;
    while(t2 - t < beep_on * 1000) {
      t2 += 500000 / beep_freq;
      if(out1_role == OUT_SPEAKER) digitalWrite(OUT1, HIGH);
      if(out2_role == OUT_SPEAKER) digitalWrite(OUT2, HIGH);
      if(out3_role == OUT_SPEAKER) digitalWrite(OUT3, HIGH);
      if(out4_role == OUT_SPEAKER) digitalWrite(OUT4, HIGH);
      delayMicroseconds(max(min(t2, t + beep_on * 1000) - (long)micros(), 0l));
      if(out1_role == OUT_SPEAKER) digitalWrite(OUT1, LOW);
      if(out2_role == OUT_SPEAKER) digitalWrite(OUT2, LOW);
      if(out3_role == OUT_SPEAKER) digitalWrite(OUT3, LOW);
      if(out4_role == OUT_SPEAKER) digitalWrite(OUT4, LOW);
      t2 += 500000 / beep_freq;
      delayMicroseconds(max(min(t2, t + beep_on * 1000) - (long)micros(), 0l));
      n ++;
    }
    if(out1_role == OUT_BUZZER) digitalWrite(OUT1, LOW);
    if(out2_role == OUT_BUZZER) digitalWrite(OUT2, LOW);
    if(out3_role == OUT_BUZZER) digitalWrite(OUT3, LOW);
    if(out4_role == OUT_BUZZER) digitalWrite(OUT4, LOW);
    while(t + (beep_on + beep_off) * 1000 > micros()) delay(1);
    //delayMicroseconds(max(t + (beep_on + beep_off) * 1000 - (long)micros(), 0l));
  }
}

void signal() {
  if(out1_role != OUT_SPEAKER &&
     out2_role != OUT_SPEAKER &&
     out3_role != OUT_SPEAKER &&
     out4_role != OUT_SPEAKER &&
     out1_role != OUT_BUZZER &&
     out2_role != OUT_BUZZER &&
     out3_role != OUT_BUZZER &&
     out4_role != OUT_BUZZER) return;
  xTaskCreatePinnedToCore(_signal, "signal", DEFALT_STACK, NULL, 1, NULL, 1);
}

// ============

String first_names[] = {"fast", "slow", "high", "nice", "big", "small", "rare", "common", "smart", "dumb", "wize", "lucky", "unlucky", "pretty"};
#define first_names_num   14
String second_names[] = {"beaver", "racoon", "cow", "bear", "cat", "dog", "eagle", "deer", "fish", "tree", "daffodil", "rocket", "turtle", "snail"};
#define second_names_num  14

String gen_name() {
  String first = first_names[random(first_names_num)];
  String second = second_names[random(second_names_num)];
  if(random(2)) first[0] -= 32;
  if(random(2)) second[0] -= 32;
  char joint = ' ' + 63 * random(2);
  return first + joint + second;
}

String run_name;

int mode = MODE_TEST;
//int mode = MODE_MONITOR;
float start_detect_threshold = 1.0;
float ground_detect_threshold = 0.6;
bool ignite_manually;
extern SemaphoreHandle_t file_writing_lock;
extern float vibration;
long report_cooldown = 60000;
long gsm_ping_cooldown = 60000;
#define REPORT ((String)";m_v=" + max_v + ";m_appr_vy=" + max_approx_vy + ";y=" + y + ";appr_y=" + approx_y + ";pres=" + press + ";alt=" + alt +\
                ";m_alt=" + max_alt + ";m_appr_y=" + max_approx_y + ";m_y=" + max_y + ";t=" + (millis() - launch_time) + ";m_a=" + max_a +\
                ";lat=" + String(gps_last_lat, 6) + ";lon=" + String(gps_last_lon, 6))
#define CSV_LINE ((String)(millis() - launch_time) + ';' + approx_y + ';' + approx_vy + ';' + x + ';' + y + ';' + z + ';' + vx + ';' + vy + ';' + vz +\
                  ';' + smooth_ax + ';' + smooth_ay + ';' + smooth_az + ';' + smooth_rx + ';' + smooth_ry + ';' + smooth_rz + ';' + press + ';' +\
                  alt + ';' + String(gps_last_lat, 6) + ';' + String(gps_last_lon, 6))
long fall_detection_time = 60000;

void main_protocol() {
  log("Main protocol started, waiting for a phone call ...");
  send_sms("Main protocol started, waiting for a phone call ...");
  long wait_start = millis();
  while(!gsm_phone_call_detected) {
    delay(10);
    if(gsm_ping_cooldown > 0 && millis() - wait_start >= gsm_ping_cooldown) {
      send_sms("Ping message, t=" + millis2str() + ". Still waiting for a phone call ...");
      wait_start += gsm_ping_cooldown;
    }
  }
  gsm_phone_call_detected = false;
  digitalWrite(VIN_HOLD, LOW);
  int flight_log_number = 0;
  xSemaphoreTake(file_writing_lock, portMAX_DELAY);
  while(SD.exists((String)"/flightlog" + (++flight_log_number) + ".csv"));
  File flight_log = SD.open((String)"/flightlog" + flight_log_number + ".csv", FILE_WRITE);
  flight_log.println(run_name);
  flight_log.println("millis;approx_y;approx_vy;x;y;z;vx;vy;vz;ax;ay;az;rx;ry;rz;pressure;alt;lat;lon");
  xSemaphoreGive(file_writing_lock);
  if(ignite_manually) {
    log("Got a phone call, holding VIN, igniting ...");
    send_sms("Got a phone call, holding VIN, igniting ...");
    reset_position();
    start();
  } else {
    log("Got a phone call, holding VIN, waiting for launch (detecting via accelerometer) ...");
    send_sms("Got a phone call, holding VIN, waiting for launch (detecting via accelerometer) ...");
    bool waiting = true;
    while(waiting) {
       //long t = millis();
      reset_position();
      //log(String(millis() - t));
      //t = millis();
      for(int i = 0; i < 10; i ++) {
        delay(50);
        if(upwards_acceleration >= start_detect_threshold) {
          waiting = false;
          break;
        }
      }
    }
    log("Launch detected!");
    send_sms("Launch detected!");
  }
  long launch_time = millis();
  long prev_report = launch_time;
  xSemaphoreGive(press_request1);
  xSemaphoreTake(press_request2, portMAX_DELAY);
  float start_pressure = press;
  xSemaphoreGive(temp_request1);
  xSemaphoreTake(temp_request2, portMAX_DELAY);
  float start_temp = temp;
  float max_approx_vy = 0.;
  float max_v = 0.;
  float max_alt = 0.;
  float max_approx_y = 0.;
  float max_y = 0.;
  float alt;
  for(;;) {
    xSemaphoreGive(press_request1);
    xSemaphoreTake(press_request2, portMAX_DELAY);
    alt = pressure2alt(start_temp, start_pressure, press);
    if(alt > max_alt) max_alt = alt;
    if(approx_y > max_approx_y) max_approx_y = approx_y;
    if(y > max_y) max_y = y;
    xSemaphoreTake(file_writing_lock, portMAX_DELAY);
    flight_log.println(CSV_LINE);
    flight_log.flush();
    //Serial.println(CSV_LINE);
    xSemaphoreGive(file_writing_lock);
    float v = sqrt(vx * vx + vy * vy + vz * vz);
    if(max_v < v) max_v = v;
    if(max_approx_vy < approx_vy) max_approx_vy = approx_vy;
    if(approx_vy < 0) {
      parachute();
      String report = (String)"Reached apogee, firing parachute" + REPORT;
      log(report);
      send_sms(report);
      break;
    }
    long t = millis();
    if(t - prev_report >= report_cooldown) {
      prev_report = t;
      String report = (String)"Flying upwards" + REPORT;
      log(report);
      send_sms(report);
    }
    delay(100);
  }
  long last_vibration = millis();
  for(;;) {
    xSemaphoreGive(press_request1);
    xSemaphoreTake(press_request2, portMAX_DELAY);
    alt = pressure2alt(start_temp, start_pressure, press);
    xSemaphoreTake(file_writing_lock, portMAX_DELAY);
    flight_log.println(CSV_LINE);
    flight_log.flush();
    //Serial.println(CSV_LINE);
    xSemaphoreGive(file_writing_lock);
    if(vibration > ground_detect_threshold) last_vibration = millis();
    if(millis() - last_vibration >= fall_detection_time) break;
    long t = millis();
    if(t - prev_report >= report_cooldown) {
      prev_report = t;
      String report = (String)"Falling" + REPORT;
      log(report);
      send_sms(report);
    }
    delay(100);
  }
  String report = (String)"Fell" + REPORT;
  log(report);
  send_sms(report);
  xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
  signal();
  log("Main protocol terminating.");
}

void test_protocol() {
  log("Test protocol started, waiting for a phone call ...");
  send_sms("Test protocol started, waiting for a phone call ...");
  while(!gsm_phone_call_detected)
    delay(100);
  gsm_phone_call_detected = false;
  log("Got a phone call");
  send_sms("Got a phone call, firing parachute ...");
  parachute();
  signal();
  log("Test protocol terminating.");
}

void monitor_protocol() {
  reset_position();
  for(;;) {
    Serial.println(String(x, 5) + " " + String(y, 5) + " " + String(z, 5) + " " +
                   String(rotation[0], 5) + " " + String(rotation[1], 5) + " " + String(rotation[2], 5) + " " +
                   String(rotation[3], 5) + " " + String(rotation[4], 5) + " " + String(rotation[5], 5) + " " +
                   String(rotation[6], 5) + " " + String(rotation[7], 5) + " " + String(rotation[8], 5));
    delay(50);
  }
}

void setup() {
  Serial.begin(115200);
  i2c.begin(SDA, SCL, 1800000); // note that 1.8 mhz is overclocking for LSM6DSO32 (max speed according to datasheet is 400 khz)
  run_name = gen_name();

  // ====== MicroSD ======
  init_sd();
  // ====== BMP280 ======

  //  bmp280_i2c.begin(BMP280_SDA, BMP280_SCL);
  bmp280_status = bmp280.begin(BMP280_ADDR);

  if (bmp280_status) log("BMP280 connected");
  else log("BMP280 not connected");

  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_OFF,   /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_1);/* Standby time. */

  // ====== NEO-6M (GPS) ======
  init_gps();
  // ====== LSM6DSO32 (Gyro/Accel) ======
  init_gyro_accel();
  // ====== SIM800L (GSM) ======
  init_gsm();
  // ====== OUTS ======

  pinMode(OUT1, OUTPUT); digitalWrite(OUT1, LOW);
  pinMode(OUT2, OUTPUT); digitalWrite(OUT2, LOW);
  pinMode(OUT3, OUTPUT); digitalWrite(OUT3, LOW);
  pinMode(OUT4, OUTPUT); digitalWrite(OUT4, LOW);
  pinMode(VIN_HOLD, OUTPUT); digitalWrite(VIN_HOLD, HIGH);
  
  start_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(start_timer, &start_end, true);
  timerAlarmWrite(start_timer, 1000 * start_time, false);
  
  parachute_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(parachute_timer, &parachute_end, true);
  timerAlarmWrite(parachute_timer, 1000 * parachute_time, false);
  
  // ====== main protocol =======
//  log((String)"out1 role: " + (out1_role == OUT_BUZZER || out1_role == OUT_SPEAKER ? (out1_role == OUT_BUZZER ? "buzzer" : "speaker") : (out1_role == OUT_PARACHUTE ? "parachute" : "start")));
//  log((String)"out2 role: " + (out2_role == OUT_BUZZER || out2_role == OUT_SPEAKER ? (out2_role == OUT_BUZZER ? "buzzer" : "speaker") : (out2_role == OUT_PARACHUTE ? "parachute" : "start")));
//  log((String)"out3 role: " + (out3_role == OUT_BUZZER || out3_role == OUT_SPEAKER ? (out3_role == OUT_BUZZER ? "buzzer" : "speaker") : (out3_role == OUT_PARACHUTE ? "parachute" : "start")));
//  log((String)"out4 role: " + (out4_role == OUT_BUZZER || out4_role == OUT_SPEAKER ? (out4_role == OUT_BUZZER ? "buzzer" : "speaker") : (out4_role == OUT_PARACHUTE ? "parachute" : "start")));

  //mode = MODE_FLIGHT;
  if(mode == MODE_FLIGHT) main_protocol();
  else if(mode == MODE_TEST) test_protocol();
  else monitor_protocol();
}

void loop() {
  if(report_cooldown <= 0 || mode != MODE_FLIGHT || gsm_phone_call_detected) {
    for(;;) {
      if(gsm_phone_call_detected) digitalWrite(VIN_HOLD, HIGH);
      delay(100);
    }
  }
  for(int i = 0; i < report_cooldown / 10; i ++) delay(10);
  resend_sms();
  send_sms("I'm alive! time=" + millis2str() + " lat=" + String(gps_last_lat, 6) + ", lon=" + String(gps_last_lon, 6));
}
