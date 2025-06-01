/*
  stty -F /dev/ttyACM0 115200 -echo
  cat  /dev/ttyACM0 > punch_log.csv
*/

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <MadgwickAHRS.h>
#include <NimBLEDevice.h>

#define SERVICE_UUID   "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_UUID_TX   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
NimBLECharacteristic* pTxChar;

/* ---------- IMU objects ---------- */
MPU6050  mpu;
Madgwick filter;

/* ---------- constants ---------- */
const float FS_HZ    = 500.0f;         // IMU sample rate (Hz)
const float DT_S     = 1.0f / FS_HZ;   // sample period (s)
const float G2MS2    = 9.80665f;       // 1 g → m/s²

/*  MPU6050 calibration biases  */
const float ACC_BIAS_X =  0.029126f;
const float ACC_BIAS_Y = -0.042134f;
const float ACC_BIAS_Z = -0.152346f;
const float GYR_BIAS_X = -3.573862f;
const float GYR_BIAS_Y =  0.405528f;
const float GYR_BIAS_Z =  1.292642f;

/*  punch detection thresholds (all in m/s²)  */
const float POS_PRE_THR  =  10.0f;   // small +Y “load” threshold (~+1 g)
const float NEG_IMPACT    = -20.0f;  // large –Y “impact” threshold (~–2 g)
const int   SEARCH_N      = 100;     // look up to 100 samples (200 ms) for the negative spike
const int   COOLOFF_N     = 100;     // after a punch, ignore next 100 samples (~200 ms)

uint32_t tLastUs;     // for 500 Hz timing
int      coolOff = 0; // countdown before next punch

/*  state for punch detector  */
bool  waitingNeg;     // true if we saw the +Y lobe and are now looking for –Y
int   searchCnt;      // how many samples we’ve searched so far
float minLinY;        // track the most-negative linY in this window

// store orientation/peak data at the moment of max negative linY
float driveLinX, driveLinY, driveLinZ;
float driveQ0, driveQ1, driveQ2, driveQ3;
float impactLinX, impactLinY, impactLinZ;
float impactQ0, impactQ1, impactQ2, impactQ3;


void setupBLE() {
  NimBLEDevice::init("Glove-R");
  NimBLEDevice::setMTU(128);
  NimBLEServer* pServer = NimBLEDevice::createServer();
  NimBLEService* pSvc   = pServer->createService(SERVICE_UUID);
  pTxChar = pSvc->createCharacteristic(
               CHAR_UUID_TX,
               NIMBLE_PROPERTY::READ |
               NIMBLE_PROPERTY::NOTIFY );
  pTxChar->setValue("ready");
  pSvc->start();
  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setName("Glove-R");
  pAdv->start();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(8, 9, 100000);   // SDA=8, SCL=9, 100 kHz
  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1) {
      delay(1000);
    }
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setFullScaleGyroRange (MPU6050_GYRO_FS_2000);
  filter.begin(FS_HZ);
  tLastUs = micros();
  setupBLE();
}

void loop() {
  while (micros() - tLastUs < (1e6 / FS_HZ));
  tLastUs += (uint32_t)(1e6 / FS_HZ);

  if (coolOff > 0) {
    coolOff--;
  }

  // Read raw IMU
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw counts → g (accel) and deg/s (gyro), then remove bias
  float aXg = ax / 2048.0f - ACC_BIAS_X;
  float aYg = ay / 2048.0f - ACC_BIAS_Y;  // forward component
  float aZg = az / 2048.0f - ACC_BIAS_Z;

  float gXd = gx / 16.4f - GYR_BIAS_X;
  float gYd = gy / 16.4f - GYR_BIAS_Y;
  float gZd = gz / 16.4f - GYR_BIAS_Z;

  // Madgwick update (gyro = deg/s, accel = g)
  filter.updateIMU(gXd, gYd, gZd, aXg, aYg, aZg);

  // Compute gravity‐free accel in m/s²
  float q0, q1, q2, q3;
  filter.getQuaternion(q0, q1, q2, q3);

  float g_bx =  2 * (q1*q3 - q0*q2);
  float g_by =  2 * (q0*q1 + q2*q3);
  float g_bz =  (q0*q0 - q1*q1 - q2*q2 + q3*q3);

  float linX = (aXg - g_bx) * G2MS2;
  float linY = (aYg - g_by) * G2MS2;
  float linZ = (aZg - g_bz) * G2MS2;

  // Punch detector: wait for small +Y, then look for big –Y
  if (!waitingNeg && coolOff == 0) {
    if (linY > POS_PRE_THR) {
      waitingNeg = true;
      searchCnt  = 0;
      minLinY    = linY;
    }
  }
  else if (waitingNeg) {
    if (linY < minLinY) {
      minLinY    = linY;
      impactLinX = linX; impactLinY = linY; impactLinZ = linZ;
      impactQ0 = q0;     impactQ1 = q1;     impactQ2 = q2;     impactQ3 = q3;
    }
    else if (driveLinY < linY) {
        driveLinX = linX; driveLinY = linY; driveLinZ = linZ;
        driveQ0 = q0;     driveQ1 = q1;     driveQ2 = q2;     driveQ3 = q3;
    }

    searchCnt++;
    if (searchCnt >= SEARCH_N || minLinY < NEG_IMPACT) {
      if (minLinY < NEG_IMPACT) {
        // valid punch detected
        coolOff = COOLOFF_N;

        char msg[200];
        int len = snprintf(
        msg, sizeof(msg),
            "%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,"
            "%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f",
            driveLinX, driveLinY, driveLinZ,
            driveQ0,   driveQ1,   driveQ2,   driveQ3,
            impactLinX, impactLinY, impactLinZ,
            impactQ0,   impactQ1,     impactQ2,    impactQ3
        );
        pTxChar->setValue((uint8_t*)msg, len);
        pTxChar->notify();
      }
      waitingNeg = false;
    }
  }
}
