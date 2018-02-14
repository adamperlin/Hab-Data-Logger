#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPL3115A2.h>
#include <utility/imumaths.h>
#include <SD.h>

/*
 * CONSTANTS
 */
#define SD_PIN 10 //can change
#define IMU_SENSOR_ID 55
#define SD_DATA_FILE "data.txt"
#define SEATTLE_BASELINE_ATM 102710.0


/*
 * GLOBAL classes
 */

Adafruit_BNO055 bno = Adafruit_BNO055(IMU_SENSOR_ID);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

File dataFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Beginning initialization...");
  devicesInit();
  Serial.println("Initialization Finished");

  for (int i = 0; i < 10; ++i) {
      pollSensors();
      delay(1000);
  }

  Serial.println("Test data read: ");
  testDataRead();
  
  delay(1000);
}

void testDataRead() {
  dataFile = SD.open(SD_DATA_FILE, FILE_READ);
  while (dataFile.available()) {
    Serial.write(dataFile.read());
  }
  dataFile.close();
}

void removeDataFileIfExists() {
  if (SD.exists("data.txt")) {
    SD.remove("data.txt");
  }
}

void initIMU() {
  if (!bno.begin()) {
    Serial.println("No BNO055 detected... idling");
    while(1);
  }
}
void initAltimeter(){
  if (!baro.begin()) {
      Serial.println("No Altimeter Detected... idling");
      while(1);
  }
  baro.setSeaPressure(SEATTLE_BASELINE_ATM);
}

void devicesInit() {
  initIMU();
  initAltimeter();
  initSD();
}

const char CSV_HEADER[] = {"time,linAccelX,linAccelY,linAccelZ,gravitAccelX,gravitAccelY,gravitAccelZ,alt,rad"};

void initSD() {
  pinMode(SD_PIN, OUTPUT);
  if (!SD.begin(SD_PIN)) {
    Serial.println("SD Card not present!");
    while(1);
  }

  removeDataFileIfExists();

  dataFile = SD.open(SD_DATA_FILE, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening SD data file!");
    while(1);
  }

  //write initial header to datafile
  dataFile.println(CSV_HEADER);
  dataFile.close();
}


long currentTime = 0L;


void loop() {
  // put your main code here, to run repeatedly:
  pollSensors();
  checkAltimeter();
  currentTime = millis();
  delay(1000);
}


#define PREVIOUS_POINTS 3 

// vectors for gravitational and linear acceleration.
imu::Vector<3> gravitAccel;
imu::Vector<3> linearAccel;

double altimeterValues[PREVIOUS_POINTS];
double alt;
unsigned int alt_index = 0;
double radiation = 0;

// do nothing for the moment. This will eventually actuate a servo at a certain altitude;


const char SEP[] = {", "};

void pollSensors() {
  dataFile = SD.open(SD_DATA_FILE, FILE_WRITE);
  
	linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gravitAccel = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  alt = baro.getAltitude();

  for (int i = 1; i < PREVIOUS_POINTS; i++) {
    altimeterValues[i] = altimeterValues[i - 1];
  }
  altimeterValues[0] = alt;

 /* dataFile.print(currentTime, DEC);
  dataFile.print(SEP);*/
  writeCSVPoint(currentTime);
  
  writeCSVPoint(linearAccel.x());  
  writeCSVPoint(linearAccel.y());
  writeCSVPoint(linearAccel.z());
  
  writeCSVPoint(gravitAccel.x());  
  writeCSVPoint(gravitAccel.y());
  writeCSVPoint(gravitAccel.z());
  
  writeCSVPoint(alt);
  writeCSVLastPoint(radiation);
  Serial.println("write finished");
}

void writeCSVPoint(double data) {
  dataFile.print(data, DEC);
  dataFile.print(SEP);
}

void writeCSVLastPoint(double data) {
  dataFile.println(data, DEC);
  dataFile.close();
}

void checkAltimeter() {
  return;
}

/*

// CONSTANTS USEFUL FOR DATA LOGGING
#define LINE_FMT "%s, %s, %s, %s, %s, %s, %s, %s"

// "n,n,n,n,n,n,n,n"
// six data points, space + comma, null terminator.
#define N_POINTS 8
#define PREC 6 
#define BUF_LEN (PREC + 3)
#define MAX_LINE_LENGTH N_POINTS * BUF_LEN + (N_POINTS-1) + 1  
char line[MAX_LINE_LENGTH];

char xAccel[BUF_LEN], yAccel[BUF_LEN], zAccel[BUF_LEN], gXAccel[BUF_LEN], gYAccel[BUF_LEN], gZAccel[BUF_LEN];
char altBuf[BUF_LEN];
char radBuf[BUF_LEN]; //not used for now.

void pollSensors() {
  linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gravitAccel = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  alt = baro.getAltitude();

  for (int i = 1; i < PREVIOUS_POINTS; i++) {
    altimeterValues[i] = altimeterValues[i - 1];
  }
  altimeterValues[0] = alt;

  convertDouble(linearAccel.x(), xAccel, sizeof xAccel);
  convertDouble(linearAccel.y(), yAccel, sizeof yAccel);
  convertDouble(linearAccel.z(), zAccel, sizeof zAccel);

  convertDouble(gravitAccel.x(), gXAccel, sizeof gXAccel);
  convertDouble(gravitAccel.y(), gYAccel, sizeof gYAccel);
  convertDouble(gravitAccel.z(), gZAccel, sizeof gZAccel);

  convertDouble(alt, altBuf, sizeof altBuf);
  convertDouble(radiation, radBuf, sizeof radBuf);

  Serial.println(linearAccel.x(), DEC);
  Serial.println(xAccel);
  Serial.println(gravitAccel.z());

  int n = snprintf(line, sizeof line, LINE_FMT, xAccel, yAccel, zAccel, gXAccel, gYAccel, gZAccel, altBuf, radBuf);
  Serial.println(line);
}

void convertDouble(double d, char *buf, size_t buf_len) {
  if (!buf) return;
  dtostrf(d, buf_len-1, 4, buf); 
  buf[buf_len-1] = '\0';
}*/

