#include <SPI.h>
#include <SdFat.h>

const int chipSelect = 10;
SdFat sd;
File dataFile;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for the serial port to connect
  }

  Serial.println("Initializing SD card...");

  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println("Card initialization failed!");
    return;
  }

  Serial.println("Card initialized.");

  // Close the current file (if open)
  if (dataFile) {
    dataFile.close();
    Serial.println("Closed the current file.");
  }

  // Create a unique filename with a timestamp
  String filename = "flight_data_" + getTimestamp() + ".csv";
  dataFile = sd.open(filename, FILE_WRITE);

  if (!dataFile) {
    Serial.println("Error opening " + filename);
  } else {
    Serial.println(filename + " opened successfully");
    dataFile.println("Time,AccelerationX,AccelerationY,AccelerationZ,Altitude,OrientationX,OrientationY,OrientationZ");
  }
}

void loop() {
  // Simulated sensor data
  float accelerationX = 1.23;
  float accelerationY = 4.56;
  float accelerationZ = 7.89;
  float altitude = 123.45;
  float orientationX = 67.89;
  float orientationY = 45.67;
  float orientationZ = 89.01;

  unsigned long currentTime = millis();

  dataFile.print(currentTime);
  dataFile.print(",");
  dataFile.print(accelerationX);
  dataFile.print(",");
  dataFile.print(accelerationY);
  dataFile.print(",");
  dataFile.print(accelerationZ);
  dataFile.print(",");
  dataFile.print(altitude);
  dataFile.print(",");
  dataFile.print(orientationX);
  dataFile.print(",");
  dataFile.print(orientationY);
  dataFile.print(",");
  dataFile.println(orientationZ);

  // Flush the data to the file
  dataFile.flush();

  // Delay for demonstration purposes
  delay(1000);
}

String getTimestamp() {
  String timestamp = "";
  unsigned long now = millis();

  timestamp += (now / 3600000) % 24;
  timestamp += '-';
  timestamp += (now / 60000) % 60;
  timestamp += '-';
  timestamp += (now / 1000) % 60;

  return timestamp;
}
