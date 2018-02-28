#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPL3115A2.h>
#include <utility/imumaths.h>
#include <SD.h>

/** Main Data logger sketch for HAB crew #2
 * 
 * Sensors in this version:
 * 
 * Adafruit Altimeter Combo MPL311A2
 * Adafruit BNO055 IMU
 * Geiger Counter
 * Adafruit microSD Card Shield
 */

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
#define SWITCH_PIN 4

/*
 * GLOBAL classes
 */

Adafruit_BNO055 bno = Adafruit_BNO055(IMU_SENSOR_ID);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// main output file
File dataFile;

unsigned long currentTime = 0L;
unsigned long previousTime = 0L;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Beginning initialization..."));
  devicesInit();

  pinMode(SWITCH_PIN, INPUT);
  Serial.println(F("Initialization Finished"));

  // this is a test; it simulates the arduino loop and writes 3 points to the SD Card before
  // reading them back with testDataRead()

  /**
   * Test
   */
  for (int i = 0; i < 3; ++i) {
      previousTime = currentTime;
      currentTime = millis();
      pollSensors();
      delay(POLL_DELAY);
  }

  Serial.println(F("Test data read: "));
  testDataRead();

  // reset timekeeping variables so we can actually start the program
  currentTime = 0L;
  previousTime = 0L;
  /**
   * End Test
   */
  
  delay(1000);
}

// check if switch is on or off
bool checkSwitch() {
  return digitalRead(SWITCH_PIN) == HIGH;
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

  // we use Seattle atmospheric pressure at Sea level as our baseline
  // In the end, it doesn't really matter; we're just looking at delta altitude
  baro.setSeaPressure(SEATTLE_BASELINE_ATM);
}



const char CSV_HEADER[] = {"time,linAccelX,linAccelY,linAccelZ,gravitAccelX,gravitAccelY,gravitAccelZ,alt,temp,pressure,rad"};

void initSD() {
  pinMode(SD_PIN, OUTPUT);
  if (!SD.begin(SD_PIN)) {
    Serial.println(F("SD Card not present!"));
    while(1);
  }

  // clear file so we don't add to existing
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

// attaches counter impulse handler to interrupt 0, which is on digital pin 2
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

// Main data variables

// this array is not used at the moment
double altimeterValues[PREVIOUS_POINTS];
double alt;
unsigned int alt_index = 0;
double microSieverts = 0.0;
double temp = 0.0;
double pressure = 0.0;

// csv seperator to write between each point
const char SEP[] = {", "};

// main polling function
void pollSensors() {
  // if the switch is on, return; we can't use the SD Card
  if (checkSwitch()) {
    Serial.println(F("On SD Lock Mode..."));
    return;
  }
  // re-open data file for writing
  dataFile = SD.open(SD_DATA_FILE, FILE_WRITE);

  //collect data
	linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gravitAccel = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  alt = baro.getAltitude();
  temp = baro.getTemperature();
  pressure = baro.getPressure();

  // update altimeter array
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

  //check to make sure we can write (this shouldn't be an issue if we're sleeping for 15s each loop,
  // it's just a safeguard in case that sleep time is changed
  if(currentTime - previousTime > LOG_PERIOD) {
    Serial.println(F("logging"));
    Serial.println(F("uSV: "));
    Serial.println(microSieverts, DEC);
    Serial.println(currentTime - previousTime, DEC);
    // do the conversion
    microSieverts = geigerCounts * TIME_PERIOD_MULTIPLIER * CPM_CONVERSION_FACTOR;
    // reset geigerCounts

    // reset to 0
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

