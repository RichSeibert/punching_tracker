#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

constexpr float ACC_LSB_PER_G  = 2048.0f;
constexpr float GYR_LSB_PER_DPS = 16.4f;

constexpr int SAMPLES = 1500;          // 3 s

void setup()
{
  Serial.begin(115200);
  Wire.begin(8, 9, 100000);            // SDA 8, SCL 9

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");  while (1);
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setFullScaleGyroRange (MPU6050_GYRO_FS_2000);

  Serial.println("\n== Keep board flat & still for 3 s ==");
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;

  for (int i = 0; i < SAMPLES; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axSum += ax; aySum += ay; azSum += az;
    gxSum += gx; gySum += gy; gzSum += gz;
    delayMicroseconds(2000);           // 500 Hz
  }

  float accBiasX =  axSum / (SAMPLES * ACC_LSB_PER_G);   // g
  float accBiasY =  aySum / (SAMPLES * ACC_LSB_PER_G);
  float accBiasZ = (azSum / (SAMPLES * ACC_LSB_PER_G)) - 1.0f; // subtract 1 g

  float gyrBiasX = gxSum / (SAMPLES * GYR_LSB_PER_DPS);  // °/s
  float gyrBiasY = gySum / (SAMPLES * GYR_LSB_PER_DPS);
  float gyrBiasZ = gzSum / (SAMPLES * GYR_LSB_PER_DPS);

  Serial.printf("const float ACC_BIAS_X  = %.6ff;\n", accBiasX);
  Serial.printf("const float ACC_BIAS_Y  = %.6ff;\n", accBiasY);
  Serial.printf("const float ACC_BIAS_Z  = %.6ff;\n\n", accBiasZ);

  Serial.printf("const float GYRO_BIAS_X = %.6ff;\n", gyrBiasX);
  Serial.printf("const float GYRO_BIAS_Y = %.6ff;\n", gyrBiasY);
  Serial.printf("const float GYRO_BIAS_Z = %.6ff;\n", gyrBiasZ);
}

void loop() {}
