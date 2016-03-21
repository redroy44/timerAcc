#include "CurieTimerOne.h"
#include "CurieIMU.h"
#include "MadgwickAHRS.h"

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

const unsigned int oneSecInUsec = 1000000;  // A second in mirco second unit.

const unsigned int time = 10000; //us -> 50Hz
// const unsigned int dt = 10000; // 100Hz

const float dt = 0.1;

unsigned int toggle = 0;

Madgwick filter; // initialise Madgwick object
float ax, ay, az;
float axf, ayf, azf;
int axRaw, ayRaw, azRaw;
int gx, gy, gz;
volatile float vx, vy, vz;
volatile float sx, sy, sz;

float yaw;
float pitch;
float roll;
int factor = 800; // variable by which to divide gyroscope values, used to control sensitivity
// note that an increased baud rate requires an increase in value of factor



float convertRawAcceleration(int* aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (*aRaw * 2.0) / 32768.0 * 9.80655;

  return a;
}

float kalmanFilter(float val) {
  float w = pow(0.0001, 2);
  float z = 0.000001;
  static float P = w;
  static float x = 0;
  float error = 0.0;
  float S = 0.0;
  float K = 0.0;

  x = x;
  P = P + w;
  error = val - x;
  S = P + z;
  K = P / S;
  x = x + K * error;
  P = P - (K * S * K);

  return x;
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


  //Serial.print(ayRaw);
  //Serial.print(","); // print comma so values can be parsed
  //Serial.println(azRaw);

  // convert the raw accelerometer data to G's and m/s^2
  ax = convertRawAcceleration(&axRaw);
  //ay = convertRawAcceleration(&ayRaw);
  //az = convertRawAcceleration(&azRaw);

  Serial.print(ax, 4);
  Serial.print(","); // print comma so values can be parsed

  axf = kalmanFilter(ax);
  Serial.println(axf, 4);
}

void setup() {
  vx = vy = vz = 0;
  sx = sy = sz = 0;

  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // Initialize pin 13 as an output - onboard LED.
  pinMode(13, OUTPUT);

  Serial.print("Time dt: ");
  Serial.println(dt, 2);
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
}

void serialEvent() {
  Serial.read();
  //  noInterrupts();

  //  Serial.print(","); // print comma so values can be parsed
  //  Serial.print(vx, 4);
  //  Serial.print(","); // print comma so values can be parsed
  //  Serial.print(vy, 4);
  //  Serial.print(","); // print comma so values can be parsed
  //  Serial.print(vz, 4);
  //  Serial.print(","); // print comma so values can be parsed
  //  Serial.print(sx, 4);
  //  Serial.print(","); // print comma so values can be parsed
  //  Serial.print(sy, 4);
  //  Serial.print(","); // print comma so values can be parsed
  //  Serial.println(sz, 4);
  //  interrupts();
}

