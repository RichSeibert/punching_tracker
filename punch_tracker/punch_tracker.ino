/*
stty -F /dev/ttyACM0 115200 -echo
cat /dev/ttyACM0 > punch_log.csv
*/

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <MadgwickAHRS.h>

MPU6050  mpu;
Madgwick filter;

const float SAMPLE_FREQ = 500.0f;      // Hz
const float G_TO_MS2    = 9.80665f;    // 1 g → m/s²

const float ACC_BIAS_X  = 0.029126f;
const float ACC_BIAS_Y  = -0.042134f;
const float ACC_BIAS_Z  = -0.152346f;
const float GYRO_BIAS_X = -3.573862f;
const float GYRO_BIAS_Y = 0.405528f;
const float GYRO_BIAS_Z = 1.292642f;

uint32_t lastSampleUs;                 // keeps 500 Hz pace

void setup()
{
  Serial.begin(115200);
  Wire.begin(8, 9, 100000);                   // 100 kHz
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (true);
  }
  mpu.setFullScaleGyroRange (MPU6050_GYRO_FS_2000);   // 16.4 LSB / (° s⁻¹)
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);    // 2048 LSB / g

  filter.begin(SAMPLE_FREQ);

  lastSampleUs = micros();
}

void loop()
{
  while (micros() - lastSampleUs < (1e6 / SAMPLE_FREQ));
  lastSampleUs += (uint32_t)(1e6 / SAMPLE_FREQ);

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float aX_g = ax / 2048.0f - ACC_BIAS_X;
  float aY_g = ay / 2048.0f - ACC_BIAS_Y;
  float aZ_g = az / 2048.0f - ACC_BIAS_Z;

  float gX_dps = gx / 16.4f - GYRO_BIAS_X;
  float gY_dps = gy / 16.4f - GYRO_BIAS_Y;
  float gZ_dps = gz / 16.4f - GYRO_BIAS_Z;

  filter.updateIMU(gX_dps, gY_dps, gZ_dps, aX_g, aY_g, aZ_g);

  float q0, q1, q2, q3;
  filter.getQuaternion(q0, q1, q2, q3);

  float g_bx = 2*(q1*q3 - q0*q2);
  float g_by = 2*(q0*q1 + q2*q3);
  float g_bz =  (q0*q0 - q1*q1 - q2*q2 + q3*q3);

  float linX = (aX_g - g_bx) * G_TO_MS2;
  float linY = (aY_g - g_by) * G_TO_MS2;
  float linZ = (aZ_g - g_bz) * G_TO_MS2;
  float power = sqrtf(linX*linX + linY*linY + linZ*linZ);    // m/s²

  Serial.print(power);   Serial.print(',');
  Serial.print(linX);    Serial.print(',');
  Serial.print(linY);    Serial.print(',');
  Serial.print(linZ);    Serial.print(',');
  Serial.print(q0);      Serial.print(',');
  Serial.print(q1);      Serial.print(',');
  Serial.print(q2);      Serial.print(',');
  Serial.println(q3);
}
