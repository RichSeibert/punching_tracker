/*
  stty -F /dev/ttyACM0 115200 -echo
  cat  /dev/ttyACM0 > punch_log.csv
*/

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <MadgwickAHRS.h>

MPU6050 mpu;
Madgwick filter;

const float FS_HZ          = 500.0f;
const float DT_S           = 1.0f / FS_HZ;
const float G_TO_MS2       = 9.80665f;

const float ACC_BIAS_X     =  0.029126f;
const float ACC_BIAS_Y     = -0.042134f;
const float ACC_BIAS_Z     = -0.152346f;
const float GYR_BIAS_X     = -3.573862f;
const float GYR_BIAS_Y     =  0.405528f;
const float GYR_BIAS_Z     =  1.292642f;

const float POS_PRE_THR_MS2 = 10.0f;
const float NEG_THR_MS2     = -20.0f;
const int   SEARCH_SAMPS    = 100;
int cooloff_samples = 0;

uint32_t last_sample_us;

bool waitingNeg = false;
int searchCount = 0;

float minLinY = 0;
float peakLinX, peakLinY, peakLinZ;
float peakQ0, peakQ1, peakQ2, peakQ3;

void setup() {
    Serial.begin(115200);
    Wire.begin(8, 9, 100000);

    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    filter.begin(FS_HZ);
    last_sample_us = micros();
}

void loop() {
    while (micros() - last_sample_us < (1e6 / FS_HZ));
    last_sample_us += (uint32_t)(1e6 / FS_HZ);
    if (cooloff_samples) --cooloff_samples;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float aX_g = ax / 2048.0f - ACC_BIAS_X;
    float aY_g = ay / 2048.0f - ACC_BIAS_Y;
    float aZ_g = az / 2048.0f - ACC_BIAS_Z;

    float gX = gx / 16.4f - GYR_BIAS_X;
    float gY = gy / 16.4f - GYR_BIAS_Y;
    float gZ = gz / 16.4f - GYR_BIAS_Z;

    filter.updateIMU(gX, gY, gZ, aX_g, aY_g, aZ_g);

    float q0, q1, q2, q3;
    filter.getQuaternion(q0, q1, q2, q3);

    float g_bx = 2*(q1*q3 - q0*q2);
    float g_by = 2*(q0*q1 + q2*q3);
    float g_bz = (q0*q0 - q1*q1 - q2*q2 + q3*q3);

    float linX = (aX_g - g_bx) * G_TO_MS2;
    float linY = (aY_g - g_by) * G_TO_MS2;
    float linZ = (aZ_g - g_bz) * G_TO_MS2;

    if (false) {
        Serial.print(linX, 2);   Serial.print(',');
        Serial.print(linY, 2);   Serial.print(',');
        Serial.print(linZ, 2);   Serial.print(',');
        Serial.print(q0, 4);     Serial.print(',');
        Serial.print(q1, 4);     Serial.print(',');
        Serial.print(q2, 4);     Serial.print(',');
        Serial.println(q3, 4);
    }

    // punch detection based on peak search for y axis. Depending on if you punch a bag vs air, the results change but
    // generally there's a positive acceleration and then a large negative acceleration. If hitting a bag the 
    // negative acceleration is much larger
    if (!waitingNeg && (cooloff_samples == 0)) {
        if (linY > POS_PRE_THR_MS2) {
            waitingNeg = true;
            searchCount = 0;
            minLinY = linY;
            peakLinX = linX; peakLinY = linY; peakLinZ = linZ;
            peakQ0 = q0; peakQ1 = q1; peakQ2 = q2; peakQ3 = q3;
        }
    } else if (waitingNeg) {
        if (linY < minLinY) {
            minLinY = linY;
            peakLinX = linX; peakLinY = linY; peakLinZ = linZ;
            peakQ0 = q0; peakQ1 = q1; peakQ2 = q2; peakQ3 = q3;
        }

        if (++searchCount >= SEARCH_SAMPS || minLinY < NEG_THR_MS2) {
            if (minLinY < NEG_THR_MS2) {
                // punch detected
                cooloff_samples = 100;

                if (false) {
                    Serial.print("Punch detected: ");
                    Serial.print(minLinY, 1);    Serial.print(',');
                    Serial.print(peakLinX, 1);   Serial.print(',');
                    Serial.print(peakLinY, 1);   Serial.print(',');
                    Serial.print(peakLinZ, 1);   Serial.print(',');
                    Serial.print(peakQ0, 4);     Serial.print(',');
                    Serial.print(peakQ1, 4);     Serial.print(',');
                    Serial.print(peakQ2, 4);     Serial.print(',');
                    Serial.println(peakQ3, 4);
                }
            }
            waitingNeg = false;
        }
    }
}