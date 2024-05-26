#include "MPU9250.h"

#define MPU9250_IMU_ADDRESS 0x68
#define MAGNETIC_DECLINATION 1.63
#define INTERVAL_MS_PRINT 1000

MPU9250 mpu;

unsigned long lastPrintMillis = 0;

float velocity = 0.0;
float position = 0.0;
float deltaTime = 0.0;
float prevAccel = 0.0;
float prevVelocity = 0.0;

void setup() {
  // 이전 코드 유지

  // ...
}

void loop() {
  unsigned long currentMillis = millis();

  if (mpu.update() && currentMillis - lastPrintMillis > INTERVAL_MS_PRINT) {
    // 가속도 측정
    float currentAccel = mpu.getAccelerationY();

    // 가속도를 적분하여 속도 계산
    velocity += (currentAccel + prevAccel) / 2.0 * deltaTime;

    // 속도를 적분하여 위치 계산
    position += (velocity + prevVelocity) / 2.0 * deltaTime;

    // 위치 및 속도 출력
    Serial.print("Position:\t");
    Serial.print(position, 4);
    Serial.print(" meters\t");

    Serial.print("Velocity:\t");
    Serial.print(velocity, 4);
    Serial.println(" m/s");

    lastPrintMillis = currentMillis;
    prevAccel = currentAccel;
    prevVelocity = velocity;
  }
}
