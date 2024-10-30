//Including Libraries 
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//Values for Inertial Measurment Unit, it is recommended not to touch these or to create duplicates
unsigned long timer = 0;
const int MPU_ADDR = 0x68;
float roll, pitch, yaw, Ax, Ay, Az, Gx, Gy;
float FilteredAccelX,FilteredAccelY, FilteredAccelZ;
float elapsedTime = 0, time, previousTime = 0, currentTime ;
 
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
int16_t alpha1 = 0.85;
int16_t alpha2 = 0.5;

//Continuity Data/LEDs
int cnt1 = 2; 
int cnt2 = 4;
int LaunchDetectLED = 5;

//Analog Continuity
int read1, read2;

//Pyro-Channel Terminal Pins
int pyro1 = 7;
int pyro2 = 8;

int buz = 0;

int buzzer = 3;
int arduinoLED = 13;

int state;
double upper, lower;

float initialAltitude;
float PressureAtSea = 1003.80; //CHANGE VALUE DEPENDING ON YOUR LOCATION
float alpha = 0.85;
float maxAltitude = 0;
double alphaBaro = 1;
int ApogeeDelay = 0;          //CHANGE VALUE DEPENDING ON THE DELAY (in milliseconds) YOU WANT TO ADD AFTER APOGEE
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  !mpu.begin();
  !bmp.begin(0x76);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  Serial.begin(115200);
  pinMode(LaunchDetectLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pyro1, OUTPUT);
  pinMode(pyro2, OUTPUT);
  pinMode(cnt1, OUTPUT);
  pinMode(cnt2, OUTPUT);

  startup();
  protocol();

}

void angularCheck();
void launchdetect();
void loop() {

  // Calculate time elapsed since the last loop iteration
  unsigned long currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Read accelerometer values
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  launchdetect();
  int read1 = analogRead(A0);
  int read2 = analogRead(A1);
  if(read1>=200){
  digitalWrite(cnt1, HIGH);
  }
  else{
    digitalWrite(cnt1, LOW);
  }
    if(read2>=200){
  digitalWrite(cnt2, HIGH);
  }
  else{
    digitalWrite(cnt2, LOW);
  }
}
void deploy();
void flight() {
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);
  
  //Get accelerometer data
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;
  float FilteredAccelX = (( alpha2 * FilteredAccelX ) + accelX ) / ( alpha2 + 1 );
  float FilteredAccelY = (( alpha2 * FilteredAccelY ) + accelY ) / ( alpha2 + 1 );
  float FilteredAccelZ = (( alpha2 * FilteredAccelZ ) + accelZ ) / ( alpha2 + 1 );

  //Complementary Filter to smooth Gyroscopic data
  roll = alpha * (roll + (g.gyro.x * elapsedTime)) + (1 - alpha) * (atan2(-accelY, -accelZ) * RAD_TO_DEG);
  pitch = alpha * (pitch + (g.gyro.y * elapsedTime)) + (1 - alpha) * (atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG);
  yaw = alpha * (yaw + (g.gyro.z * elapsedTime)) + (1 - alpha) * (atan2(-accelY, -accelZ) * RAD_TO_DEG);

  float x = roll + 180.0;  // Convert roll to range 0-360
  float y = pitch + 180.0; // Convert pitch to range 0-360
  float z = yaw + 180.0;   // Convert yaw to range 0-360  
  float t = bmp.readTemperature();

  bmp.takeForcedMeasurement();

  int read1 = analogRead(A0);
  int read2 = analogRead(A1);
  float CurrentAltitude = bmp.readAltitude(PressureAtSea) - initialAltitude;

//============ - UNCOMMENT TO USE WITH CAtower -  ===============
//Datapackets will grab the values of the corresponding variables

Serial.print("@");                                          //New Data pack identifier
Serial.print(x); Serial.print("x");                         //gyro X
Serial.print(y); Serial.print("y");                         //gyro Y
Serial.print(z); Serial.print("z");                         //gyro Z
Serial.print(accelX); Serial.print("X");                    //acclerometer X
Serial.print(accelY); Serial.print("Y");                    //acclerometer Y
Serial.print(accelZ); Serial.print("Z");                    //acclerometer Z
Serial.print(t - 3.80); Serial.print("t");                  //Temperature
Serial.print(CurrentAltitude); Serial.print("A"); //Altitude (Auto sets to 0m during launch detect + or - 30cm)
Serial.print(bmp.readPressure()/100); Serial.print("P");    //Pressure
Serial.print(read1); Serial.print("c");                     //continuity 1
Serial.print(read2); Serial.print("C");                     //continuity 2
Serial.print(state); Serial.print("S");                     //State
Serial.print("\n");

//================ - Apogee detection by Altitude - ==================

    //Saves Highest Altitude in maxAltitude
    if (CurrentAltitude > maxAltitude) {
        maxAltitude = CurrentAltitude;
    }
    
    //Set by default with a 1 meter uncertainty, you may increases if needed
    if ((maxAltitude > CurrentAltitude + 1.00)&&(int(FilteredAccelY)>=8 && int(FilteredAccelY)<=14)) {
      deploy();
    }
}

//Bootup MAIN Buzzer
void startup(){
  
setBuzz(1800);
  
}

void setBuzz(int buzz) {
  while (buz != buzz) {

    if (buz < buzz ) buz += 10;
    if (buz > buzz ) buz -= 10;

    _setBuzz();
    delay(5);
    }
    if(buz==1800){
      noTone(buzzer);
      delay(500);
      tone(buzzer, 1800, 100);
      delay(100);
      noTone(buzzer);
      tone(buzzer, 1800, 100);
      delay(100);
      noTone(buzzer);
    }
}

void _setBuzz() {
  tone(buzzer, buz);
  initialAltitude = bmp.readAltitude(PressureAtSea);
}

//================ - LaunchDetection - ===============
void launchdetect () {

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Ax= a.acceleration.x;
  Ay= a.acceleration.y;

  //You can change Ay below - Increasing means it will need a larger acceleration to detect launch, 
  //lower means less acceleration...{Default set to 60 m/s^2}
  
  if (state == 0 && Ay  > 60) {
  digitalWrite(LaunchDetectLED, HIGH);
  state++;
  }
  
  if (state == 1) {
  flight();
}
}


void protocol () {

sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Ax= a.acceleration.x;
  Ay= a.acceleration.y;
  Gx= g.gyro.x;
  Gy= g.gyro.y;
  
  state == 0;
  delay(750);
  Serial.println("FC-Udev is Ready for Flight");
  int statusA;
  statusA = (a.acceleration.x + a.acceleration.y)/2;
  if (statusA = 0) {
    Serial.println("IMY Initialization Error");
    while (1) {
      }
  }
  int statusG;
  statusG = (g.gyro.x + g.gyro.y)/2;
  if (statusG = 0) {
    Serial.println("Gyro Initialization Error"); 
    while (1) {
      }
  }
  
}

void deploy(){
  tone(buzzer, 2000, 500);
  delay(ApogeeDelay);
  state++;
  digitalWrite(pyro1, HIGH);
  digitalWrite(pyro2, HIGH);
  tone(buzzer, 1800);
  //Duration of Pyrotechnic is set for 5 seconds (Default)
  delay(5000);
  state++;
  //Recovery Buzzer and LED
    while(state==3){
     digitalWrite(pyro1, LOW);
     digitalWrite(pyro2, LOW);
     digitalWrite(LaunchDetectLED, HIGH);
     digitalWrite(cnt1, LOW);
     digitalWrite(cnt2, LOW);
     tone(buzzer, 2000, 500);
     delay(500);
     digitalWrite(LaunchDetectLED, LOW);
     digitalWrite(cnt1, HIGH);
     digitalWrite(cnt2, HIGH);
     delay(100);
  }
}
