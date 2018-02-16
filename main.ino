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

#define POLL_DELAY 15000

#define LOG_PERIOD 15000  //Logging period in milliseconds, recommended value 15000-60000.
#define MAX_PERIOD 60000
#define TIME_PERIOD_MULTIPLIER (MAX_PERIOD / LOG_PERIOD)

#define CPM_CONVERSION_FACTOR (1.0/151.0)
/*
 * GLOBAL classes
 */

Adafruit_BNO055 bno = Adafruit_BNO055(IMU_SENSOR_ID);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

File dataFile;

unsigned long currentTime = 0L;
unsigned long previousTime = 0L;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(F("Beginning initialization..."));
  devicesInit();
  Serial.println(F("Initialization Finished"));

  
  for (int i = 0; i < 3; ++i) {
      previousTime = currentTime;
      currentTime = millis();
      pollSensors();
      delay(POLL_DELAY);
  }

  Serial.println("Test data read: ");
  testDataRead();

  currentTime = 0L;
  previousTime = 0L;
  
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
  if (SD.exists(SD_DATA_FILE)) {
    SD.remove(SD_DATA_FILE);
  }
}

void initIMU() {
  if (!bno.begin()) {
    Serial.println(F("No BNO055 detected... idling"));
    while(1);
  }
}
void initAltimeter(){
  if (!baro.begin()) {
      Serial.println(F("No Altimeter Detected... idling"));
      while(1);
  }
  baro.setSeaPressure(SEATTLE_BASELINE_ATM);
}



const char CSV_HEADER[] = {"time,linAccelX,linAccelY,linAccelZ,gravitAccelX,gravitAccelY,gravitAccelZ,alt,temp,pressure,rad"};

void initSD() {
  pinMode(SD_PIN, OUTPUT);
  if (!SD.begin(SD_PIN)) {
    Serial.println(F("SD Card not present!"));
    while(1);
  }

  removeDataFileIfExists();

  dataFile = SD.open(SD_DATA_FILE, FILE_WRITE);
  if (!dataFile) {
    Serial.println(F("Error opening SD data file!"));
    while(1);
  }

  //write initial header to datafile
  dataFile.println(CSV_HEADER);
  dataFile.close();
}

int geigerCounts = 0;


void tube_impulse_handler() {
  geigerCounts++;
} 

void initGeiger() {
  attachInterrupt(0, tube_impulse_handler, FALLING);
}

void devicesInit() {
  initIMU();
  initAltimeter();
  initSD();
  initGeiger();
}


void loop() {
  // put your main code here, to run repeatedly:

  previousTime = currentTime;
  currentTime = millis();
  pollSensors();
  checkAltimeter();
  delay(POLL_DELAY);
}


#define PREVIOUS_POINTS 3 

// vectors for gravitational and linear acceleration.
imu::Vector<3> gravitAccel;
imu::Vector<3> linearAccel;

double altimeterValues[PREVIOUS_POINTS];
double alt;
unsigned int alt_index = 0;
double microSieverts = 0.0;
double temp = 0.0;
double pressure = 0.0;

// do nothing for the moment. This will eventually actuate a servo at a certain altitude;


const char SEP[] = {", "};

void pollSensors() {
  dataFile = SD.open(SD_DATA_FILE, FILE_WRITE);
  
	linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gravitAccel = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  alt = baro.getAltitude();
  temp = baro.getTemperature();
  pressure = baro.getPressure();

  for (int i = 1; i < PREVIOUS_POINTS; i++) {
    altimeterValues[i] = altimeterValues[i - 1];
  }
  altimeterValues[0] = alt;

  //read from geiger counter 

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
  writeCSVPoint(temp);
  writeCSVPoint(pressure);  

  if(currentTime - previousTime > LOG_PERIOD) {
    Serial.println(F("logging"));
    Serial.println(F("uSV: "));
    Serial.println(microSieverts, DEC);
    Serial.println(currentTime - previousTime, DEC);
    microSieverts = geigerCounts * TIME_PERIOD_MULTIPLIER * CPM_CONVERSION_FACTOR;
    // reset geigerCounts

    geigerCounts = 0;
  } 

  writeCSVLastPoint(microSieverts);


  Serial.println(F("write finished"));
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

