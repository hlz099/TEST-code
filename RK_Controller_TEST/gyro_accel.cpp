#include "rc.h"

//#define LSM6DSO32_SDA       23
//#define LSM6DSO32_SCL       22
#define LSM6DSO32_ADDR    0x6A
#define ACCEL_K        78.4532
#define GYRO_K   0.01745329252
#define ACCEL_AVG          100
#define SOLVED_DELTA      1e-6
#define MAX_XYZ_BUFFSIZE  9000
#define SMOOTH_K0         0.99
#define SMOOTH_K1         0.01

//TwoWire lsm6dso32_i2c(1);
Adafruit_LSM6DSO32 lsm6dso32;

extern TwoWire i2c;

bool lsm6dso32_status;

int gyro_accel_rotations = 9;
long gyro_accel_calibration_timeout = 100000;
long accel_peace_time = 2000;
long accel_peace_edges = 300;
long accel_calibration_timeout = 40000;
float accel_min_dist = 8.;
float accel_peace_delta = .18;
long gyro_calibration_time = 4000;
float accel_bx = 0.;
float accel_by = 0.;
float accel_bz = 0.;
float accel_transform[9] = {1., 0., 0.,
                            0., 1., 0.,
                            0., 0., 1.};
float gyro_bx = 0.;
float gyro_by = 0.;
float gyro_bz = 0.;
float gyro_transform[9] = {1., 0., 0.,
                           0., 1., 0.,
                           0., 0., 1.};
float &accel_ax = accel_transform[0];
float &accel_ay = accel_transform[4];
float &accel_az = accel_transform[8];
float &gyro_ax = gyro_transform[0];
float &gyro_ay = gyro_transform[4];
float &gyro_az = gyro_transform[8];
float gravity = 9.80665;
float approx_gravity = 9.3;
float max_a = 0;

SemaphoreHandle_t ag_deamon_lock = NULL;
SemaphoreHandle_t temp_request1 = NULL;
SemaphoreHandle_t temp_request2 = NULL;
SemaphoreHandle_t press_request1 = NULL;
SemaphoreHandle_t press_request2 = NULL;
float temp;
float press;

void init_gyro_accel() {
    //  lsm6dso32_i2c.begin(LSM6DSO32_SDA, LSM6DSO32_SCL);
  //  lsm6dso32_status = lsm6dso32.begin_I2C(LSM6DSO32_ADDR, &lsm6dso32_i2c);

  ag_deamon_lock = xSemaphoreCreateMutex();
  temp_request1 = xSemaphoreCreateBinary();
  temp_request2 = xSemaphoreCreateBinary();
  press_request1 = xSemaphoreCreateBinary();
  press_request2 = xSemaphoreCreateBinary();
  xSemaphoreTake(temp_request1, 0);
  xSemaphoreTake(temp_request2, 0);
  xSemaphoreTake(press_request1, 0);
  xSemaphoreTake(press_request2, 0);
  //xSemaphoreGive(ag_deamon_lock);
  
  lsm6dso32_status = lsm6dso32.begin_I2C(LSM6DSO32_ADDR, &i2c);

  if(lsm6dso32_status) log("LSM6DSO32 connected");
  else log("LSM6DSO32 not connected");
//  lsm6dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
//  lsm6dso32.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
//  lsm6dso32.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);
  lsm6dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
  lsm6dso32.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  lsm6dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6dso32.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
  //  calibrate_accel();

  xTaskCreatePinnedToCore(gyro_accel_deamon, "ga_d", DEFALT_STACK, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(calibration_deamon, "cal_d", DEFALT_STACK, NULL, 5, NULL, 1);
}

float det(double *A) {
  return A[0] * A[4] * A[8] +
         A[1] * A[5] * A[6] +
         A[2] * A[3] * A[7] -
         A[2] * A[4] * A[6] -
         A[1] * A[3] * A[8] -
         A[0] * A[5] * A[7];
}

void linsolve(double *A, double *B, int n) {
  for(int ij = 0; ij < n; ij ++) {
    if(A[ij * (n + 1)] == 0.) A[ij * (n + 1)] = 1e-30;
    double k = 1. / A[ij * (n + 1)];
    for(int j = 0; j < n; j ++)
      A[ij * n + j] *= k;
    B[ij] *= k;
    for(int i = 0; i < n; i ++) {
      if(i == ij) continue;
      double k2 = -A[i * n + ij];
      for(int j = 0; j < n; j ++)
        A[i * n + j] += A[ij * n + j] * k2;
      B[i] += B[ij] * k2;
    }
  }
}

void rmat(float *A, float *B, int n = 3) {
  for(int ij = 0; ij < n; ij ++) {
    if(A[ij * (n + 1)] == 0.) A[ij * (n + 1)] = 1e-30;
    float k = 1. / A[ij * (n + 1)];
    for(int j = 0; j < n; j ++) {
      A[ij * n + j] *= k;
      B[ij * n + j] *= k;
    }
    for(int i = 0; i < n; i ++) {
      if(i == ij) continue;
      float k2 = -A[i * n + ij];
      for(int j = 0; j < n; j ++) {
        A[i * n + j] += A[ij * n + j] * k2;
        B[i * n + j] += B[ij * n + j] * k2;
      }
    }
  }
}

void matmul(double *A, double *B, double *C, int n = 3) {
  for(int i = 0; i < n; i ++) {
    for(int j = 0; j < n; j ++) {
      double c = 0.;
      for(int k = 0; k < n; k ++)
        c += A[i * n + k] * B[k * n + j];
      C[i * n + j] = c;
    }
  }
}

void arrmul(double *A, float k, double *B, int n = 9) {
  for(int i = 0; i < n; i ++) B[i] = A[i] * k;
}

void arrsum(double *A, double *B, double *C, int n = 9) {
  for(int i = 0; i < n; i ++) C[i] = A[i] + B[i];
}

void arrsum(float *A, float *B, float *C, int n = 9) {
  for(int i = 0; i < n; i ++) C[i] = A[i] + B[i];
}

float test_accel_calibration(float *x_examples, float *y_examples, float *z_examples,
                             float ax, float bx, float ay, float by, float az, float bz) {
  float error = 0.;
  for(int i = 0; i < 6; i ++) {
    float x = x_examples[i] * ax + bx;
    float y = y_examples[i] * ay + by;
    float z = z_examples[i] * az + bz;
    float error2 = gravity * gravity - x * x - y * y - z * z;
    error += error2 * error2;
  }
  return error;
}

void _solve_accel_calibration(float *x_examples, float *y_examples, float *z_examples,
                              float *ax, float *bx, float *ay, float *by, float *az, float *bz) {
  double error[6];
  for(int i = 0; i < 6; i ++) {
    float x = x_examples[i] * *ax + *bx;
    float y = y_examples[i] * *ay + *by;
    float z = z_examples[i] * *az + *bz;
    error[i] = gravity * gravity - x * x - y * y - z * z;
  }
  double A[36];
  for(int i = 0; i < 6; i ++) {
    float dbx = 2. * (x_examples[i] * *ax + *bx);
    float dby = 2. * (y_examples[i] * *ay + *by);
    float dbz = 2. * (z_examples[i] * *az + *bz);
    float dax = dbx * x_examples[i];
    float day = dby * y_examples[i];
    float daz = dbz * z_examples[i];
    A[i * 6    ] = dax;
    A[i * 6 + 1] = day;
    A[i * 6 + 2] = daz;
    A[i * 6 + 3] = dbx;
    A[i * 6 + 4] = dby;
    A[i * 6 + 5] = dbz;
  }
  linsolve(A, error, 6);
  *ax += error[0];
  *ay += error[1];
  *az += error[2];
  *bx += error[3];
  *by += error[4];
  *bz += error[5];
}

bool solve_accel_calibration(float *x_examples, float *y_examples, float *z_examples) {
  float ax = 1.;
  float bx = 0.;
  float ay = 1.;
  float by = 0.;
  float az = 1.;
  float bz = 0.;
  for(int i = 0; i < 10; i ++) {
    _solve_accel_calibration(x_examples, y_examples, z_examples, &ax, &bx, &ay, &by, &az, &bz);
    float error = test_accel_calibration(x_examples, y_examples, z_examples, ax, bx, ay, by, az, bz);
    if(error < SOLVED_DELTA) {
      for(int j = 0; j < 9; j ++) accel_transform[j] = 0.;
      accel_ax = ax;
      accel_bx = bx / ax;
      accel_ay = ay;
      accel_by = by / ay;
      accel_az = az;
      accel_bz = bz / az;
      return true;
    }
  }
  return false;
}

bool check_peace(float *x_buffer, float *y_buffer, float *z_buffer, long *t_buffer, long buffsize, long buffer_start, long buffer_end, long *start, long *end) {
//  log("check_peace started");
  bool found = false;
  long end_t = t_buffer[(buffer_end + buffsize - 1) % buffsize];
  bool just_started = true;
  for(*start = buffer_start; (*start != buffer_end or just_started) and end_t - t_buffer[*start] >= accel_peace_time; *start = (*start + 1) % buffsize) {
    float x_min = 1e+10; float x_max = -1e+10;
    float y_min = 1e+10; float y_max = -1e+10;
    float z_min = 1e+10; float z_max = -1e+10;
    for(*end = *start; (*end != buffer_end or just_started); *end = (*end + 1) % buffsize) {
      just_started = false;
      x_min = min(x_buffer[*end], x_min); x_max = max(x_buffer[*end], x_max);
      y_min = min(y_buffer[*end], y_min); y_max = max(y_buffer[*end], y_max);
      z_min = min(z_buffer[*end], z_min); z_max = max(z_buffer[*end], z_max);
//      log(String(t_buffer[*end]) + ' ' + *end);
      if(x_max - x_min > accel_peace_delta or y_max - y_min > accel_peace_delta or z_max - z_min > accel_peace_delta) break;
      if(t_buffer[*end] - t_buffer[*start] >= accel_peace_time)
        found = true;
    }
    if(found) break;
  }
  if(!found) return false;
  if(*end == buffer_end) *end = (buffer_end + buffsize - 1) % buffsize;
  return true;
}

bool calibrate_gyro_a_accel_ab() {
  log("Calibrating accelerometer and gyroscope ...");
  float ax_examples[gyro_accel_rotations + 1], ay_examples[gyro_accel_rotations + 1], az_examples[gyro_accel_rotations + 1];
  float rx_examples[gyro_accel_rotations], ry_examples[gyro_accel_rotations], rz_examples[gyro_accel_rotations];
  {
    xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
    long buffsize = accel_peace_time / 40;
    float *x_buffer = new float[buffsize];
    float *y_buffer = new float[buffsize];
    float *z_buffer = new float[buffsize];
    long *t_buffer = new long[buffsize];
    long buffer_start = 0, buffer_end = 0;
    bool buffer_filled = false;
    float rx = 0., ry = 0., rz = 0.;
    long t = micros();
    long start_time = millis();
    int i = 0;
    while(i <= gyro_accel_rotations) {
      for(int j = 0; j < 60; j ++) {
        float x = 0., y = 0., z = 0.;
        for(int _ = 0; _ < ACCEL_AVG; _ ++) {
          float newX, newY, newZ;
          lsm6dso32.readAcceleration(newX, newY, newZ);
          x += newX; y += newY; z += newZ;
          lsm6dso32.readGyroscope(newX, newY, newZ);
          long t2 = micros();
          float k = (t2 - t) * 1e-6;
          t = t2;
          rx += (newX * GYRO_K + gyro_bx) * k;
          ry += (newY * GYRO_K + gyro_by) * k;
          rz += (newZ * GYRO_K + gyro_bz) * k;
        }
        x_buffer[buffer_end] = x * ACCEL_K / (float)ACCEL_AVG + accel_bx;
        y_buffer[buffer_end] = y * ACCEL_K / (float)ACCEL_AVG + accel_by;
        z_buffer[buffer_end] = z * ACCEL_K / (float)ACCEL_AVG + accel_bz;
        t_buffer[buffer_end] = millis();
        buffer_end ++;
        if(buffer_end == buffsize) buffer_filled = true;
        buffer_end %= buffsize;
        if(buffer_filled) buffer_start = buffer_end;
      }
      long peace_start, peace_end;
      if(check_peace(x_buffer, y_buffer, z_buffer, t_buffer, buffsize, buffer_start, buffer_end, &peace_start, &peace_end)) {
        long n = 0;
        float x = 0, y = 0., z = 0.;
        for(long i = peace_start; i != peace_end; i = (i + 1) % buffsize) {
          if(t_buffer[i] - t_buffer[peace_start] >= accel_peace_edges and t_buffer[peace_end] - t_buffer[i] >= accel_peace_edges) {
            x += x_buffer[i]; y += y_buffer[i], z += z_buffer[i];
            n ++;
          }
        }
        x /= (float)n; y /= (float)n; z /= (float)n;
        bool fits = true;
        if(i > 0) {
          float sx = ax_examples[i - 1] - x, sy = ay_examples[i - 1] - y, sz = az_examples[i - 1] - z;
          fits = (sx * sx + sy * sy + sz * sz > accel_min_dist * accel_min_dist);
        }
        if(fits) {
          log("Got stationary point, acceleration: x = " + String(x, 5) + ", y = " + String(y, 5) + ", z = " + String(z, 5));
          ax_examples[i] = x;
          ay_examples[i] = y;
          az_examples[i] = z;
          if(i) {
            log("rotation: x = " + String(rx, 5) + ", y = " + String(ry, 5) + ", z = " + String(rz, 5));
            rx_examples[i - 1] = rx;
            ry_examples[i - 1] = ry;
            rz_examples[i - 1] = rz;
          }
          rx = ry = rz = 0.;
          i ++;
        }
      }
      if(millis() - start_time >= gyro_accel_calibration_timeout) {
        xSemaphoreGive(ag_deamon_lock);
        delete[] x_buffer;
        delete[] y_buffer;
        delete[] z_buffer;
        delete[] t_buffer;
        log("Calibration timeout, got " + String(i) + "/" + String(gyro_accel_rotations + 1) + " stationary points");
        return false;
      }
    }
    xSemaphoreGive(ag_deamon_lock);
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] z_buffer;
    delete[] t_buffer;
  }
  float A_p[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
  long ax_p = 0, ay_p = 0, az_p = 0;
  float A_n[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
  long ax_n = 0, ay_n = 0, az_n = 0;
  float R[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
  long rx = 0, ry = 0, rz = 0;
  for(int i = 0; i < gyro_accel_rotations + 1; i ++) {
    int shift = (abs(ay_examples[i]) > abs(ax_examples[i]) && abs(ay_examples[i]) > abs(az_examples[i])) +
            2 * (abs(az_examples[i]) > abs(ax_examples[i]) && abs(az_examples[i]) > abs(ay_examples[i]));
    int sgn = sign(ax_examples[i]) * (shift == 0) +
              sign(ay_examples[i]) * (shift == 1) +
              sign(az_examples[i]) * (shift == 2);
    if(sgn > 0) {
      A_p[    shift] += ax_examples[i];
      A_p[3 + shift] += ay_examples[i];
      A_p[6 + shift] += az_examples[i];
      if(shift == 0) ax_p ++;
      if(shift == 1) ay_p ++;
      if(shift == 2) az_p ++;
    } else {
      A_n[    shift] -= ax_examples[i];
      A_n[3 + shift] -= ay_examples[i];
      A_n[6 + shift] -= az_examples[i];
      if(shift == 0) ax_n ++;
      if(shift == 1) ay_n ++;
      if(shift == 2) az_n ++;
    }
    if(i) {
      int shift = (abs(ry_examples[i - 1]) > abs(rx_examples[i - 1]) && abs(ry_examples[i - 1]) > abs(rz_examples[i - 1])) +
              2 * (abs(rz_examples[i - 1]) > abs(rx_examples[i - 1]) && abs(rz_examples[i - 1]) > abs(ry_examples[i - 1]));
      int sgn = sign(rx_examples[i - 1]) * (shift == 0) +
                sign(ry_examples[i - 1]) * (shift == 1) +
                sign(rz_examples[i - 1]) * (shift == 2);
      R[    shift] += rx_examples[i - 1] * sgn;
      R[3 + shift] += ry_examples[i - 1] * sgn;
      R[6 + shift] += rz_examples[i - 1] * sgn;
      if(shift == 0) rx ++;
      if(shift == 1) ry ++;
      if(shift == 2) rz ++;
    }
  }
  if(!ax_p) { log("Calibration failed, did not get positive x acceleration"); return false; }
  if(!ay_p) { log("Calibration failed, did not get positive y acceleration"); return false; }
  if(!az_p) { log("Calibration failed, did not get positive z acceleration"); return false; }
  if(!ax_n) { log("Calibration failed, did not get negative x acceleration"); return false; }
  if(!ay_n) { log("Calibration failed, did not get negative y acceleration"); return false; }
  if(!az_n) { log("Calibration failed, did not get negative z acceleration"); return false; }
  if(!rx) { log("Calibration failed, did not get rotation around x axis"); return false; }
  if(!ry) { log("Calibration failed, did not get rotation around y axis"); return false; }
  if(!rz) { log("Calibration failed, did not get rotation around z axis"); return false; }
  float A[9];
  arrsum(A_p, A_n, A);
  for(int shift = 0; shift < 9; shift += 3) {
    A_p[shift    ] /= gravity * (float)ax_p;
    A_p[shift + 1] /= gravity * (float)ay_p;
    A_p[shift + 2] /= gravity * (float)az_p;
    A_n[shift    ] /= gravity * (float)ax_n;
    A_n[shift + 1] /= gravity * (float)ay_n;
    A_n[shift + 2] /= gravity * (float)az_n;
    A[shift    ] /= gravity * (float)(ax_p + ax_n);
    A[shift + 1] /= gravity * (float)(ay_p + ay_n);
    A[shift + 2] /= gravity * (float)(az_p + az_n);
    R[shift    ] /= 1.570796327 * (float)rx;
    R[shift + 1] /= 1.570796327 * (float)ry;
    R[shift + 2] /= 1.570796327 * (float)rz;
  }
  accel_bx = (A_n[0] - A_p[0]) * .5;
  accel_by = (A_n[4] - A_p[4]) * .5;
  accel_bz = (A_n[8] - A_p[8]) * .5;
  A[0] += accel_bx * (float)(ax_p - ax_n) / (float)(ax_p + ax_n); A[1] += accel_bx * (float)(ay_p - ay_n) / (float)(ay_p + ay_n); A[2] += accel_bx * (float)(az_p - az_n) / (float)(az_p + az_n);
  A[3] += accel_by * (float)(ax_p - ax_n) / (float)(ax_p + ax_n); A[4] += accel_by * (float)(ay_p - ay_n) / (float)(ay_p + ay_n); A[5] += accel_by * (float)(az_p - az_n) / (float)(az_p + az_n);
  A[6] += accel_bz * (float)(ax_p - ax_n) / (float)(ax_p + ax_n); A[7] += accel_bz * (float)(ay_p - ay_n) / (float)(ay_p + ay_n); A[8] += accel_bz * (float)(az_p - az_n) / (float)(az_p + az_n);
  float A2[9] = {1., 0., 0.,
                 0., 1., 0.,
                 0., 0., 1.};
  rmat(A, A2);
  float R2[9] = {1., 0., 0.,
                 0., 1., 0.,
                 0., 0., 1.};
  rmat(R, R2);
  log("acceleration biases: x + " + String(accel_bx, 5) + ", y + " + String(accel_by, 5) + ", z + " + String(accel_bz, 5));
  log("acceleration transform matrix:");
  log(String(A2[0], 5) + " " + String(A2[1], 5) + " " + String(A2[2], 5));
  log(String(A2[3], 5) + " " + String(A2[4], 5) + " " + String(A2[5], 5));
  log(String(A2[6], 5) + " " + String(A2[7], 5) + " " + String(A2[8], 5));
  log("rotation transform matrix:");
  log(String(R2[0], 5) + " " + String(R2[1], 5) + " " + String(R2[2], 5));
  log(String(R2[3], 5) + " " + String(R2[4], 5) + " " + String(R2[5], 5));
  log(String(R2[6], 5) + " " + String(R2[7], 5) + " " + String(R2[8], 5));
  memcpy(accel_transform, A2, sizeof(float) * 9);
  memcpy(gyro_transform, R2, sizeof(float) * 9);
//  log();
  save_calibration();
  return true;
}

bool calibrate_accel_ab() {
  log((String)"Accelerometer calibration started");
  xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
//  send_sms((String)"Accelerometer calibration started");
  long buffsize = accel_peace_time / 20;
  float *x_buffer = new float[buffsize];
  float *y_buffer = new float[buffsize];
  float *z_buffer = new float[buffsize];
  long *t_buffer = new long[buffsize];
  long buffer_start = 0, buffer_end = 0;
  bool buffer_filled = false;
//  long i = 0;
  int examples = 0;
  float x_examples[6], y_examples[6], z_examples[6];
  long t = millis();
  while(millis() - t < accel_calibration_timeout) {
    for(int j = 0; j < 6; j ++) {
      float x = 0., y = 0., z = 0.;
      for(int _ = 0; _ < ACCEL_AVG; _ ++) {
        float newX, newY, newZ;
//        while(!lsm6dso32.accelerationAvailable());
        lsm6dso32.readAcceleration(newX, newY, newZ);
        x += newX; y += newY; z += newZ;
      }
      x_buffer[buffer_end] = x * ACCEL_K / (float)ACCEL_AVG;
      y_buffer[buffer_end] = y * ACCEL_K / (float)ACCEL_AVG;
      z_buffer[buffer_end] = z * ACCEL_K / (float)ACCEL_AVG;
      t_buffer[buffer_end] = millis();
      buffer_end ++;
      if(buffer_end == buffsize) buffer_filled = true;
      buffer_end %= buffsize;
      if(buffer_filled) buffer_start = buffer_end;
    }
    long peace_start, peace_end;
    if(check_peace(x_buffer, y_buffer, z_buffer, t_buffer, buffsize, buffer_start, buffer_end, &peace_start, &peace_end)) {
      long n = 0;
      float x = 0, y = 0., z = 0.;
      for(long i = peace_start; i != peace_end; i = (i + 1) % buffsize) {
        if(t_buffer[i] - t_buffer[peace_start] >= accel_peace_edges and t_buffer[peace_end] - t_buffer[i] >= accel_peace_edges) {
          x += x_buffer[i]; y += y_buffer[i], z += z_buffer[i];
          n ++;
        }
      }
      x /= (float)n; y /= (float)n; z /= (float)n;
      bool fits = true;
      for(int i = 0; i < examples; i ++) {
        float rx = x_examples[i] - x, ry = y_examples[i] - y, rz = z_examples[i] - z;
        float r2 = rx * rx + ry * ry + rz * rz;
        if(r2 < accel_min_dist * accel_min_dist) {
          fits = false;
          break;
        }
      }
      if(fits) {
        log("Got calibration point, x = " + String(x, 5) + ", y = " + String(y, 5) + ", z = " + String(z, 5));
        x_examples[examples] = x;
        y_examples[examples] = y;
        z_examples[examples] = z;
        examples ++;
        if(examples >= 6) {
          xSemaphoreGive(ag_deamon_lock);
          bool solved = solve_accel_calibration(x_examples, y_examples, z_examples);
          if(solved) {
            save_calibration();
            log("Calibration solved successfully, (x + " + String(accel_bx, 5) + ") * " + String(accel_ax, 5) +
                                               ", (y + " + String(accel_by, 5) + ") * " + String(accel_ay, 5) +
                                               ", (z + " + String(accel_bz, 5) + ") * " + String(accel_az, 5));
          } else {
            log("Failed to solve calibration");
//            send_sms("Failed to solve calibration");
          }
          delete[] x_buffer;
          delete[] y_buffer;
          delete[] z_buffer;
          delete[] t_buffer;
          return solved;
        }
      }
    }
  }
  xSemaphoreGive(ag_deamon_lock);
  delete[] x_buffer;
  delete[] y_buffer;
  delete[] z_buffer;
  delete[] t_buffer;
  log((String)"Calibration timeout (got " + examples + "/6 points)");
//  send_sms((String)"Calibration timeout (got " + examples + "/6 points)");
  return false;
}

void calibrate_gyro_b() {
  log("Calibrating gyroscope, pls dont touch it");
  xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
  long t = millis();
  float x = 0., y = 0., z = 0.;
  long i;
  for(i = 0; millis() - t < gyro_calibration_time; i ++) {
    float x2, y2, z2;
    while(!lsm6dso32.gyroscopeAvailable());
    lsm6dso32.readGyroscope(x2, y2, z2);
    x += x2; y += y2; z += z2;
  }
  xSemaphoreGive(ag_deamon_lock);
  gyro_bx = -x * GYRO_K / i; gyro_by = -y * GYRO_K / i; gyro_bz = -z * GYRO_K / i;
  save_calibration();
  log("Gyroscope calibrated, x + " + String(gyro_bx, 5) + ", y + " + String(gyro_by, 5) + ", z + " + String(gyro_bz, 5));
}

bool get_acceleration(float &x, float &y, float &z) {
  bool retval = lsm6dso32.readAcceleration(x, y, z);
  if(isnan(x) || isnan(y) || isnan(z) || !retval) return false;
  float x0 = x * ACCEL_K + accel_bx;
  float y0 = y * ACCEL_K + accel_by;
  float z0 = z * ACCEL_K + accel_bz;
  x = x0 * accel_transform[0] + y0 * accel_transform[1] + z0 * accel_transform[2];
  y = x0 * accel_transform[3] + y0 * accel_transform[4] + z0 * accel_transform[5];
  z = x0 * accel_transform[6] + y0 * accel_transform[7] + z0 * accel_transform[8];
  return true;
}

bool get_rotation(float &x, float &y, float &z) {
  bool retval = lsm6dso32.readGyroscope(x, y, z);
  if(isnan(x) || isnan(y) || isnan(z) || !retval) return false;
  float x0 = x * GYRO_K + gyro_bx;
  float y0 = y * GYRO_K + gyro_by;
  float z0 = z * GYRO_K + gyro_bz;
  x = x0 * gyro_transform[0] + y0 * gyro_transform[1] + z0 * gyro_transform[2];
  y = x0 * gyro_transform[3] + y0 * gyro_transform[4] + z0 * gyro_transform[5];
  z = x0 * gyro_transform[6] + y0 * gyro_transform[7] + z0 * gyro_transform[8];
  return true;
}

float vx = 0.;
float vy = 0.;
float vz = 0.;
double rotation[] = {1., 0., 0.,
                     0., 1., 0.,
                     0., 0., 1.};
double start_rotation[] = {1., 0., 0.,
                           0., 1., 0.,
                           0., 0., 1.};
float x = 0.;
float y = 0.;
float z = 0.;
float current_gravity = gravity;
float approx_y = 0.;
float approx_vy = 0.;
float upwards_acceleration = 0.;
float smooth_ax = 0., smooth_ay = 0., smooth_az = 0.;
float smooth_rx = 0., smooth_ry = 0., smooth_rz = 0.;
float vibration = 0.;

void reset_position() {
  xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
  //delay(1000);
  vx = vy = vz = 0.;
  float ax = 0., ay = 0., az = 0.;
  int n = 0;
  for(int i = 0; i < 200; i ++) {
    delayMicroseconds(100);
    float _ax, _ay, _az;
    if(get_acceleration(_ax, _ay, _az)) {
      ax += _ax; ay += _ay; az += _az;
      n ++;
    }
  }
  float r1 = sqrt(ax*ax + ay*ay + az*az);
  ax /= r1; ay /= r1; az /= r1;
  current_gravity = r1 / n;
  rotation[3] = ax; rotation[4] = ay; rotation[5] = az;
  double A[9];
  memcpy(A, rotation, sizeof(double) * 9);
  double B[] = {1., 0., 0.};
  linsolve(A, B, 3);
  float r0 = sqrt(B[0]*B[0] + B[1]*B[1] + B[2]*B[2]);
  arrmul(B, 1. / r0, rotation, 3);
  memcpy(A, rotation, sizeof(double) * 9);
  B[0] = 0.; B[1] = 0.; B[2] = 1.;
  linsolve(A, B, 3);
  float r2 = sqrt(B[0]*B[0] + B[1]*B[1] + B[2]*B[2]);
  arrmul(B, 1. / r2, rotation + 6, 3);
  if(det(rotation) < 0.) {
    rotation[0] = -rotation[0]; rotation[1] = -rotation[1]; rotation[2] = -rotation[2];
  }
  x = y = z = 0.;
  approx_y = 0.;
  approx_vy = 0.;
  memcpy(start_rotation, rotation, sizeof(double) * 9);
  upwards_acceleration = 0.;
  max_a = 0;
  smooth_ax = smooth_ay = smooth_az = 0.;
  smooth_rx = smooth_ry = smooth_rz = 0.;
  vibration = 0.;
  //delay(60);
  xSemaphoreGive(ag_deamon_lock);
}

extern Adafruit_BMP280 bmp280;

bool lag_undetected = true;

void gyro_accel_deamon(void *_) {
  long h = millis();
  long t = -1;
  bool unexpected_lag;
  xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
  for(;;) {
    xSemaphoreGive(ag_deamon_lock);
    delayMicroseconds(10);
    unexpected_lag = true;
    if(!xSemaphoreTake(ag_deamon_lock, 0)) {
      unexpected_lag = false;
      xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
    }
    if(xSemaphoreTake(temp_request1, 0)) {
      temp = bmp280.readTemperature();
      xSemaphoreGive(temp_request2);
    }
    if(xSemaphoreTake(press_request1, 0)) {
      press = bmp280.readPressure();
      xSemaphoreGive(press_request2);
    }
    //Serial.println("gyro_accel_deamon");
    float ax, ay, az;
    while(!get_acceleration(ax, ay, az));
    float rx, ry, rz;
    while(!get_rotation(rx, ry, rz));
    long prev_t = t;
    t = micros();
    if(prev_t >= 0 && t - prev_t > 1000) {
      if(unexpected_lag) {
        log("gyro_accel_deamon: Lag spike: " + String(t - prev_t));
        if(lag_undetected) {
          send_sms("Lag in i2c communication with gyro/accel (probably caused by noise from gsm antenna)");
          lag_undetected = false;
        }
      }
      continue;
    }
    if(prev_t < 0 or t < prev_t) continue;
    float a = sqrt(ax * ax + ay * ay + az * az);
    if(a > max_a) max_a = max_a * .95 + a * .5;
    smooth_ax = smooth_ax * SMOOTH_K0 + ax * SMOOTH_K1;
    smooth_ay = smooth_ay * SMOOTH_K0 + ay * SMOOTH_K1;
    smooth_az = smooth_az * SMOOTH_K0 + az * SMOOTH_K1;
    float vib = sqrt((ax-smooth_ax)*(ax-smooth_ax) + (ay-smooth_ay)*(ay-smooth_ay) + (az-smooth_az)*(az-smooth_az));
    vibration *= .9995;
    if(vib > vibration) vibration = vib;
    smooth_rx = smooth_rx * SMOOTH_K0 + rx * SMOOTH_K1;
    smooth_ry = smooth_ry * SMOOTH_K0 + ry * SMOOTH_K1;
    smooth_rz = smooth_rz * SMOOTH_K0 + rz * SMOOTH_K1;
    float dt = (t - prev_t) * 1e-6;
    double r[9];
    get_rotation_matrix(rx * dt, ry * dt, rz * dt, r);
    double new_rotation[9];
    matmul(rotation, r, new_rotation);
    memcpy(rotation, new_rotation, sizeof(double) * 9);
    float ax2 = rotation[0] * ax + rotation[1] * ay + rotation[2] * az;
    float ay2 = rotation[3] * ax + rotation[4] * ay + rotation[5] * az - current_gravity;
    float az2 = rotation[6] * ax + rotation[7] * ay + rotation[8] * az;
    if(ay2 >= 0.) upwards_acceleration = upwards_acceleration * .98 + ay2 * .02;
    vx += ax2 * dt; vy += ay2 * dt; vz += az2 * dt;
    float approx_ay2 = start_rotation[3] * ax + start_rotation[4] * ay + start_rotation[5] * az - approx_gravity;
    approx_vy += approx_ay2 * dt;
    approx_y += approx_vy * dt;
//    float k1 = (1. - dt * 2.);
//    vx *= k1; vy *= k1; vz *= k1;
//    float k2 = (1. - dt * .5);
//    x *= k2; y *= k2; z *= k2;
    x += vx * dt; y += vy * dt; z += vz * dt;
    //delay(1);
  }
}

void get_rotation_matrix(double x, double y, double z, double *A) {
  double r = sqrt(x*x + y*y + z*z);
  if(!r) {
    A[0] = 1.; A[1] = 0.; A[2] = 0.;
    A[3] = 0.; A[4] = 1.; A[5] = 0.;
    A[6] = 0.; A[7] = 0.; A[8] = 1.;
    return;
  }
  x /= r; y /= r; z /= r;
  double c = cos(r); double s = sin(r);
  A[0] = c + (1.-c)*x*x;   A[1] = (1.-c)*x*y - s*z; A[2] = (1.-c)*x*z + s*y;
  A[3] = (1.-c)*y*x + s*z; A[4] = c + (1.-c)*y*y;   A[5] = (1.-c)*y*z - s*x;
  A[6] = (1.-c)*z*x - s*y; A[7] = (1.-c)*z*y + s*x; A[8] = c + (1.-c)*z*z;
}

void get_rotation_matrix_dx(double x, double y, double z, double *A) {
  double r = sqrt(x*x + y*y + z*z);
  if(!r) {
    A[0] = 0.; A[1] = 0.; A[2] =  0.;
    A[3] = 0.; A[4] = 0.; A[5] = -1.;
    A[6] = 0.; A[7] = 1.; A[8] =  0.;
    return;
  }
  x /= r; y /= r; z /= r;
  double c = cos(r); double s = sin(r);
  A[0] = (2.*(1.-c)/r - s) * x * (1. - x*x)             ; A[1] = (1.-c)/r * (y-2.*x*x*y) + s*x*x*y + x*z*(s/r-c) ; A[2] = (1.-c)/r * (z-2.*x*x*z) + s*x*x*z - x*y*(s/r-c) ;
  A[3] = (1.-c)/r * (y-2.*x*x*y) + s*x*x*y - x*z*(s/r-c); A[4] = -x*s*(1. - y*y) - 2.*(1. - c)*x*y*y/r           ; A[5] = (s - 2.*(1.-c)/r) * x*y*z - s*(1.-x*x)/r - c*x*x;
  A[6] = (1.-c)/r * (z-2.*x*x*z) + s*x*x*z + x*y*(s/r-c); A[7] = (s - 2.*(1.-c)/r) * x*y*z + s*(1.-x*x)/r + c*x*x; A[8] = -x*s*(1. - z*z) - 2.*(1. - c)*x*z*z/r           ;
}

void get_rotation_matrix_dy(double x, double y, double z, double *A) {
  double r = sqrt(x*x + y*y + z*z);
  if(!r) {
    A[0] =  0.; A[1] = 0.; A[2] = 1.;
    A[3] =  0.; A[4] = 0.; A[5] = 0.;
    A[6] = -1.; A[7] = 0.; A[8] = 0.;
    return;
  }
  x /= r; y /= r; z /= r;
  double c = cos(r); double s = sin(r);
  A[0] = -y*s*(1. - x*x) - 2.*(1. - c)*y*x*x/r           ; A[1] = (1.-c)/r * (x-2.*y*y*x) + s*y*y*x + y*z*(s/r-c); A[2] = (s - 2.*(1.-c)/r) * y*x*z + s*(1.-y*y)/r + c*y*y;
  A[3] = (1.-c)/r * (x-2.*y*y*x) + s*y*y*x - y*z*(s/r-c) ; A[4] = (2.*(1.-c)/r - s) * y * (1. - y*y)             ; A[5] = (1.-c)/r * (z-2.*y*y*z) + s*y*y*z + y*x*(s/r-c) ;
  A[6] = (s - 2.*(1.-c)/r) * y*x*z - s*(1.-y*y)/r - c*y*y; A[7] = (1.-c)/r * (z-2.*y*y*z) + s*y*y*z - y*x*(s/r-c); A[8] = -y*s*(1. - z*z) - 2.*(1. - c)*y*z*z/r           ;
}

void get_rotation_matrix_dz(double x, double y, double z, double *A) {
  double r = sqrt(x*x + y*y + z*z);
  if(!r) {
    A[0] = 0.; A[1] = -1.; A[2] = 0.;
    A[3] = 1.; A[4] =  0.; A[5] = 0.;
    A[6] = 0.; A[7] =  0.; A[8] = 0.;
    return;
  }
  x /= r; y /= r; z /= r;
  double c = cos(r); double s = sin(r);
  A[0] = -z*s*(1. - x*x) - 2.*(1. - c)*z*x*x/r           ; A[1] = (s - 2.*(1.-c)/r) * z*x*y - s*(1.-z*z)/r - c*z*z; A[2] = (1.-c)/r * (x-2.*z*z*x) + s*z*z*x - z*y*(s/r-c);
  A[3] = (s - 2.*(1.-c)/r) * z*x*y + s*(1.-z*z)/r + c*z*z; A[4] = -z*s*(1. - y*y) - 2.*(1. - c)*z*y*y/r           ; A[5] = (1.-c)/r * (y-2.*z*z*y) + s*z*z*y + z*x*(s/r-c);
  A[6] = (1.-c)/r * (x-2.*z*z*x) + s*z*z*x + z*y*(s/r-c) ; A[7] = (1.-c)/r * (y-2.*z*z*y) + s*z*z*y - z*x*(s/r-c) ; A[8] = (2.*(1.-c)/r - s) * z * (1. - z*z)             ;
}

void _solve_gyro_calibration(float *x_buffer, float *y_buffer, float *z_buffer, int buffsize, float *ax, float *ay, float *az) {
  double A[9] = {1., 0., 0.,
                 0., 1., 0.,
                 0., 0., 1.};
  double Adx[9] = {0., 0., 0.,
                   0., 0., 0.,
                   0., 0., 0.};
  double Ady[9] = {0., 0., 0.,
                   0., 0., 0.,
                   0., 0., 0.};
  double Adz[9] = {0., 0., 0.,
                   0., 0., 0.,
                   0., 0., 0.};
  for(int i = 0; i < buffsize; i ++) {
    double R[9], Rdx[9], Rdy[9], Rdz[9], temp[9];
    float rx = x_buffer[i] * *ax, ry = y_buffer[i] * *ay, rz = z_buffer[i] * *az;
    get_rotation_matrix(rx, ry, rz, R);
    get_rotation_matrix_dx(rx, ry, rz, Rdx); arrmul(Rdx, rx, Rdx);
    get_rotation_matrix_dy(rx, ry, rz, Rdy); arrmul(Rdy, ry, Rdy);
    get_rotation_matrix_dz(rx, ry, rz, Rdz); arrmul(Rdz, rz, Rdz);
    matmul(Adx, R, temp); memcpy(Adx, temp, sizeof(double) * 9);
    matmul(A, Rdx, temp); arrsum(Adx, temp, Adx);
    matmul(Ady, R, temp); memcpy(Ady, temp, sizeof(double) * 9);
    matmul(A, Rdy, temp); arrsum(Ady, temp, Ady);
    matmul(Adz, R, temp); memcpy(Adz, temp, sizeof(double) * 9);
    matmul(A, Rdz, temp); arrsum(Adz, temp, Adz);
    matmul(A, R, temp); memcpy(A, temp, sizeof(double) * 9);
  }
  double eqA[9] = {Adx[0] / *ax, Ady[0] / *ay, Adz[0] / *az,
                   Adx[4] / *ax, Ady[4] / *ay, Adz[4] / *az,
                   Adx[8] / *ax, Ady[8] / *ay, Adz[8] / *az};
  double eqB[3] = {1. - A[0], 1. - A[4], 1. - A[8]};
  linsolve(eqA, eqB, 3);
  *ax += eqB[0] * 2.;
  *ay += eqB[1] * 2.;
  *az += eqB[2] * 2.;
}

bool solve_gyro_calibration(float *x_buffer, float *y_buffer, float *z_buffer, int buffsize, float *ax, float *ay, float *az) {
  float ax2 = 1., ay2 = 1., az2 = 1.;
  for(int i = 0; i < 12; i ++) {
    float ax2_old = ax2, ay2_old = ay2, az2_old = az2;
    _solve_gyro_calibration(x_buffer, y_buffer, z_buffer, buffsize, &ax2, &ay2, &az2);
//    log(String(ax2, 5) + " " + String(ay2, 5) + " " + String(az2, 5));
    if(!(.72 < ax2 && ax2 < 1.4 && .72 < ay2 && ay2 < 1.4 && .72 < az2 && az2 < 1.4)) return false;
    if(abs(ax2 - ax2_old) < SOLVED_DELTA && abs(ay2 - ay2_old) < SOLVED_DELTA && abs(az2 - az2_old) < SOLVED_DELTA) {
      *ax = ax2;
      *ay = ay2;
      *az = az2;
      return true;
    }
  }
  return false;
}

bool calibrate_gyro_a(long buffsize = MAX_XYZ_BUFFSIZE) {
  log("Calibrating gyroscope (transform only)");
  xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
  float *x_buffer = new float[buffsize];
  float *y_buffer = new float[buffsize];
  float *z_buffer = new float[buffsize];
  log("Started buffering gyroscope data");
  long t = micros();
  for(int i = 0; i < MAX_XYZ_BUFFSIZE; i ++) {
    float x, y, z;
    while(!lsm6dso32.gyroscopeAvailable());
    lsm6dso32.readGyroscope(x, y, z);
    long t2 = micros();
    float k = (t2 - t) * 1e-6;
    t = t2;
    x_buffer[i] = (x * GYRO_K + gyro_bx) * k;
    y_buffer[i] = (y * GYRO_K + gyro_by) * k;
    z_buffer[i] = (z * GYRO_K + gyro_bz) * k;
  }
  xSemaphoreGive(ag_deamon_lock);
  log("Buffer filled, processing data ...");
  float ax, ay, az;
  if(solve_gyro_calibration(x_buffer, y_buffer, z_buffer, buffsize, &ax, &ay, &az)) {
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] z_buffer;
    log("Gyroscope calibrated: x * " + String(ax, 5) + ", y * " + String(ay, 5) + ", z * " + String(az, 5));
    for(int i = 0; i < 9; i ++) gyro_transform[i] = 0.;
    gyro_ax = ax; gyro_ay = ay; gyro_az = az;
    return true;
  }
  delete[] x_buffer;
  delete[] y_buffer;
  delete[] z_buffer;
  log("Gyroscope calibration failed");
  return false;
}

extern HardwareSerial gsm_uart;
extern SemaphoreHandle_t gsm_listener_lock;

void calibration_deamon(void *_) {
  String s;
  for(;;) {
    while(!Serial.available()) delay(20);
    char c = Serial.read();
    if(c == '\n') {
      if(s == "cal_gyro_a") calibrate_gyro_a();
      else if(s == "cal_gyro_b") calibrate_gyro_b();
      else if(s == "cal_accel") calibrate_accel_ab();
      else if(s == "cal_gyro_accel") calibrate_gyro_a_accel_ab();
      else if(s == "reset") {
        log("Reseting position");
        //xSemaphoreTake(ag_deamon_lock, portMAX_DELAY);
        reset_position();
        //xSemaphoreGive(ag_deamon_lock);
      } else if(s.startsWith("AT")) {
        xSemaphoreTake(gsm_listener_lock, portMAX_DELAY);
        log("Sending AT command: " + s);
        for(int i = 0; i < s.length(); i ++) {
          if(s.charAt(i) == '\\') s.setCharAt(i, '\n');
          if(s.charAt(i) == '#') s.setCharAt(i, (char)26);
        }
        gsm_uart.println(s);
        for(int i = 0; i < 100 && !gsm_uart.available(); i ++) delay(10);
        String s = get_gsm_responce();
        if(s.length()) log("sim800l responce: " + s);
        else log("Did not get a responce in 1 sec.");
        xSemaphoreGive(gsm_listener_lock);
      } else log("Unknown command: " + s);
      s = "";
    } else if(c != '\r') {
      s += c;
    }
  }
}

//void fix_rotation_matrix(double *A) {
//  double k0 = A[0] * A[0] + A[3] * A[3] + A[6] * A[6]; // (A0, A0)
//  k0 = 1. / sqrt(k0);
//  A[0] *= k0; A[3] *= k0; A[6] *= k0; // A0 /= sqrt((A0, A0))
//  double p = A[0] * A[1] + A[3] * A[4] + A[6] * A[7]; // (A0, A1)
//  A[1] -= A[0] * p; A[4] -= A[3] * p; A[7] -= A[6] * p; // A1 -= A0 * (A0, A1)
//  double k1 = A[1] * A[1] + A[4] * A[4] + A[7] * A[7]; // (A1, A1)
//  k1 = 1. / sqrt(k1);
//  A[1] *= k1; A[4] *= k1; A[7] *= k1; // A1 /= sqrt((A1, A1))
//  // A2 = A0 x A1:
//  A[2] = A[3] * A[7] - A[6] * A[4];
//  A[5] = A[6] * A[1] - A[0] * A[7];
//  A[8] = A[0] * A[4] - A[3] * A[1];
//}

double ax0, ay0, az0;

extern SemaphoreHandle_t file_writing_lock;

void save_calibration() {
  xSemaphoreTake(file_writing_lock, portMAX_DELAY);
  String new_config;
  File conf = SD.open(CONFIG, FILE_READ);
  while(conf.available()) {
    String line = conf.readStringUntil('\n');
    String name;
    String value;
    int retval = read_config_line(line, &name, &value);
    if(retval >= 0) {
      String comment;
      int i = line.indexOf("#");
      if(i >= 0) comment = " " + line.substring(i, line.length());
      if(name == "accel_transform") {
        new_config += "accel_transform = " + String(accel_transform[0], 5) + "," + String(accel_transform[1], 5) + "," + String(accel_transform[2], 5) + "," +
                                             String(accel_transform[3], 5) + "," + String(accel_transform[4], 5) + "," + String(accel_transform[5], 5) + "," +
                                             String(accel_transform[6], 5) + "," + String(accel_transform[7], 5) + "," + String(accel_transform[8], 5) + comment + "\n";
      } else if(name == "accel_bx") {
        new_config += "accel_bx = " + String(accel_bx, 5) + comment + "\n";
      } else if(name == "accel_by") {
        new_config += "accel_by = " + String(accel_by, 5) + comment + "\n";
      } else if(name == "accel_bz") {
        new_config += "accel_bz = " + String(accel_bz, 5) + comment + "\n";
      } else if(name == "gyro_transform") {
        new_config += "gyro_transform = " + String(gyro_transform[0], 5) + "," + String(gyro_transform[1], 5) + "," + String(gyro_transform[2], 5) + "," +
                                            String(gyro_transform[3], 5) + "," + String(gyro_transform[4], 5) + "," + String(gyro_transform[5], 5) + "," +
                                            String(gyro_transform[6], 5) + "," + String(gyro_transform[7], 5) + "," + String(gyro_transform[8], 5) + comment + "\n";
      } else if(name == "gyro_bx") {
        new_config += "gyro_bx = " + String(gyro_bx, 5) + comment + "\n";
      } else if(name == "gyro_by") {
        new_config += "gyro_by = " + String(gyro_by, 5) + comment + "\n";
      } else if(name == "gyro_bz") {
        new_config += "gyro_bz = " + String(gyro_bz, 5) + comment + "\n";
      } else {
        new_config += line + "\n";
      }
    } else {
      log("Detected illegal config line, commenting out: " + line);
      new_config += "# ?? " + line + "\n";
    }
  }
  conf.close();
  conf = SD.open(CONFIG, FILE_WRITE);
  conf.print(new_config);
  conf.close();
  xSemaphoreGive(file_writing_lock);
}
