//VCC-5V
//GND-GND
//SCL-A5
//SDA-A4

#include <Wire.h>

#define MPU6050_ADDRESS 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// MPU6050의 스케일 팩터 설정
const float ACCEL_SCALE = 16384.0*9.81; // 16384 LSB/g
const float GYRO_SCALE = 131.0;    // 131 LSB/(º/s)

// 시간 간격 설정 (예: 0.01초)
const float dt = 0.01;

float accel_x_m_s2, accel_y_m_s2, accel_z_m_s2;
float gyro_x_rad_s, gyro_y_rad_s, gyro_z_rad_s;

float velocity_x = 0.0;
float velocity_y = 0.0;
float displacement_x = 0.0;
float displacement_y = 0.0;
float angleZ = 0.0;

// 이상치를 필터링하기 위한 최대 허용 가속도
const float error_accel = 0.02; // 예: 2 m/s^2
const float error_vel = 0.0002;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B); // PWR_MGMT_1 레지스터 주소
  Wire.write(0);    // 0을 쓰면 잠자기 모드에서 깨어납니다.
  Wire.endTransmission(true);
}

void loop() {
  // 가속도 데이터 읽기
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);

  accel_x_m_s2 = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  accel_y_m_s2 = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  accel_z_m_s2 = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;

  // 이상치 필터링
  if (abs(accel_x_m_s2) < error_accel) accel_x_m_s2 = 0.0;
  if (abs(accel_y_m_s2) < error_accel) accel_y_m_s2 = 0.0;
  //if (abs(accel_z_m_s2) > error_eccel) accel_z_m_s2 = 0.0;

  // 자이로스코프 데이터 읽기
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);

  gyro_x_rad_s = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE * (PI / 180.0);
  gyro_y_rad_s = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE * (PI / 180.0);
  gyro_z_rad_s = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE * (PI / 180.0);

  calculateDirection();
  calculateDisplacement();

  // 변환된 값을 출력
  Serial.print(displacement_x);
  Serial.print(" ");
  Serial.print(displacement_y);
  Serial.print(" ");
  Serial.print(angleZ);
  Serial.print(" ");

  Serial.print(accel_x_m_s2, 6);
  Serial.print(",");
  Serial.print(accel_y_m_s2, 6);
  Serial.print(",");
  Serial.print(accel_z_m_s2, 6);
  Serial.print(",");

  Serial.print(gyro_x_rad_s, 6);
  Serial.print(",");
  Serial.print(gyro_y_rad_s, 6);
  Serial.print(",");
  Serial.println(gyro_z_rad_s, 6);

  delay(10);
}

void calculateDisplacement() {
  // 속도 적분
  velocity_x = velocity_x + accel_x_m_s2 * dt;
  velocity_y = velocity_y + accel_y_m_s2 * dt;

  if (abs(velocity_x) < error_vel) velocity_x = 0.0;
  if (abs(velocity_y) < error_vel) velocity_y = 0.0;

  Serial.print(velocity_x);
  Serial.print(" ");
  Serial.print(velocity_y);
  Serial.print(" ");

  
  // 변위 적분
  displacement_x = displacement_x + velocity_x * dt;
  displacement_y = displacement_y + velocity_y * dt;
}

void calculateDirection() {
  // z축 각속도를 사용하여 z축 방향의 변화량(각도) 계산
  angleZ = angleZ + gyro_z_rad_s * dt;
}
