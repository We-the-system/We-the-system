#include <Wire.h>

const int MPU_ADDR = 0x68; // MPU-9250의 I2C 주소

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // MPU-9250 초기화
  writeToMPU9250(0x6B, 0); // MPU-9250을 활성화
  delay(100); // 잠시 대기
  writeToMPU9250(0x37, 0x02); // 바이패스 모드 활성화 (자이로스코프와 가속도계를 직접 읽음)
}

void loop() {
  // 가속도계, 자이로스코프, 지자기 데이터 읽기
  int16_t accelX = readMPU9250(0x3B) << 8 | readMPU9250(0x3C);
  int16_t accelY = readMPU9250(0x3D) << 8 | readMPU9250(0x3E);
  int16_t accelZ = readMPU9250(0x3F) << 8 | readMPU9250(0x40);
  
  int16_t gyroX = readMPU9250(0x43) << 8 | readMPU9250(0x44);
  int16_t gyroY = readMPU9250(0x45) << 8 | readMPU9250(0x46);
  int16_t gyroZ = readMPU9250(0x47) << 8 | readMPU9250(0x48);

  int16_t magX = readMPU9250(0x03) << 8 | readMPU9250(0x04);
  int16_t magY = readMPU9250(0x05) << 8 | readMPU9250(0x06);
  int16_t magZ = readMPU9250(0x07) << 8 | readMPU9250(0x08);

  // 데이터 출력
  Serial.print("Accel: ");
  Serial.print(accelX); Serial.print(", ");
  Serial.print(accelY); Serial.print(", ");
  Serial.print(accelZ); Serial.print(" | ");

  Serial.print("Gyro: ");
  Serial.print(gyroX); Serial.print(", ");
  Serial.print(gyroY); Serial.print(", ");
  Serial.print(gyroZ); Serial.print(" | ");

  Serial.print("Mag: ");
  Serial.print(magX); Serial.print(", ");
  Serial.print(magY); Serial.print(", ");
  Serial.println(magZ);

  delay(1000); // 1초에 한 번씩 출력
}

void writeToMPU9250(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte readMPU9250(byte reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}
