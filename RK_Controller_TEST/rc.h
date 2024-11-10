#pragma once

#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <Adafruit_LSM6DSO32.h>
#include <math.h>

#define DEFALT_STACK      6144

#define SDA                  4
#define SCL                 16

#define CONFIG       "/config"

#define sign(x) ((x) > 0. ? 1. : ((x) < 0. ? -1. : 0.))

#define OUT_NONE             0
#define OUT_START            1
#define OUT_PARACHUTE        2
#define OUT_SPEAKER          3
#define OUT_BUZZER           4

#define MODE_MONITOR        10
#define MODE_FLIGHT         11
#define MODE_TEST           12

//bool test_mode = false;

void log(String);
void save_calibration();
void get_rotation_matrix(double, double, double, double*);
int read_config_line(String, String*, String*);
void calibration_deamon(void*);
void reset_position();
void gyro_accel_deamon(void*);
bool calibrate_gyro_a_accel_ab();

void init_gyro_accel();
String millis2str(long t = -1);
void init_sd();
void init_gps();
void send_sms(String);
void resend_sms();

void init_gsm();
String get_gsm_responce();
