#include "CurieTimerOne.h"
#include "CurieIMU.h"
#include "MadgwickAHRS.h"

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

//const unsigned int oneSecInUsec = 1000000;  // A second in mirco second unit.

const unsigned int time = 20000; //us -> 50Hz
// const unsigned int dt = 10000; // 100Hz

const float dt = 0.02;

unsigned int toggle = 0;

Madgwick filter; // initialise Madgwick object
float ax, ay, az;
int axRaw, ayRaw, azRaw;
int gx, gy, gz;
volatile float vx, vy, vz;
volatile float sx, sy, sz;

float yaw;
float pitch;
float roll;
int factor = 800; // variable by which to divide gyroscope values, used to control sensitivity
// note that an increased baud rate requires an increase in value of factor

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 16.0) / 32768.0;

  return a;
}

void timedBlinkIsr()
{
  digitalWrite(13, toggle ? HIGH : LOW);
  toggle = (toggle + 1) & 0x01;

//  // read raw accel/gyro measurements from device
//  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
//  // use function from MagdwickAHRS.h to return quaternions
//  filter.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);
//  // functions to find yaw roll and pitch from quaternions
//  yaw = filter.getYaw();
//  roll = filter.getRoll();
//  pitch = filter.getPitch();

// read raw accelerometer measurements from device
  CurieIMU.readAccelerometer(axRaw, ayRaw, azRaw);

  // convert the raw accelerometer data to G's and m/s^2
  ax = convertRawAcceleration(axRaw) * 9.80665;
  ay = convertRawAcceleration(ayRaw) * 9.80665;
  az = convertRawAcceleration(azRaw) * 9.80665;

}

void setup() {
  vx = vy = vz = 0;
  sx = sy = sz = 0;
  
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // Initialize pin 13 as an output - onboard LED.
  pinMode(13, OUTPUT);

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);

  if (calibrateOffsets == 1) {
    // use the code below to calibrate accel/gyro offset values
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(X_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(Z_AXIS)); Serial.print("\t");
    Serial.println("");

    // The calibration below
    Serial.print("Starting Gyroscope calibration...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");
    Serial.print("Starting Acceleration calibration...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(X_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS)); Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(Z_AXIS)); Serial.print("\t");
    Serial.println("");
  }

  CurieTimerOne.start(time, &timedBlinkIsr);
}

void loop() {
  //delay();

  if(CurieTimerOne.rdRstTickCount() > 0) {
    vx += ax * dt; // m/s
    vy += ay * dt;
    //vz = 15;

    sx += vx * dt * 100; // m
    sy += vy * dt * 100; // m
    //sz += vz * dt;
  }

  if (Serial.available() > 0) {
    int val = Serial.read();
    noInterrupts();
    //if (val == 's') { // if incoming serial is "s"
      Serial.print(ax, 4);
      Serial.print(","); // print comma so values can be parsed
      Serial.print(ay, 4);
      Serial.print(","); // print comma so values can be parsed
      Serial.print(az, 4);
      Serial.print(","); // print comma so values can be parsed 
      Serial.print(vx, 4);
      Serial.print(","); // print comma so values can be parsed
      Serial.print(vy, 4);
      Serial.print(","); // print comma so values can be parsed
      Serial.print(vz, 4);
      Serial.print(","); // print comma so values can be parsed     
      Serial.print(sx, 4);
      Serial.print(","); // print comma so values can be parsed
      Serial.print(sy, 4);
      Serial.print(","); // print comma so values can be parsed
      Serial.println(sz, 4);
    //}
      interrupts();
  }

}
