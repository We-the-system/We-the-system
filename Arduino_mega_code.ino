#include <SoftwareSerial.h>

#define ENC_COUNT_REV 330 //pulse per 1 rotation(360degree)
#define WHEEL_RADIUS 0.067

//Left Motor
#define Encoder_L_PulseA 18 // Yellow
#define Encoder_L_PulseB 19 // White
#define EN_L 9 //left velocity
#define Motor_L_pin1 2
#define Motor_L_pin2 3 //left motor 

//Right Motor
#define Encoder_R_PulseA 20 // Yellow
#define Encoder_R_PulseB 21 // White
#define Motor_R_pin1 4 
#define Motor_R_pin2 5 //right motor
#define EN_R 10 //right velocity

#define pwm_L 255

//mpu9250
#define ACCEL_CONFIG      0x1C
#define GYRO_CONFIG       0x1B
#define USER_CTRL         0x6A
#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40
#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

uint8_t acc_full_scale = 0x01; // Set accel scale (+-2g: 0x00, +-4g: 0x01, +-8g: 0x02, +- 16g: 0x03)
uint8_t gyro_full_scale = 0x02; // Set gyro scale (00 = +250dps, 01= +500 dps, 10 = +1000 dps, 11 = +2000 dps )
int crash = 99999999;
float   accel_scale, gyro_scale;
float   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
const float mpu_dt = 0.01; //time vary
float velocity_x = 0.0;
float velocity_y = 0.0;
float velocity_z = 0.0;
float displacement_x = 0.0;
float displacement_y = 0.0;
float displacement_z = 0.0;
float angle_x = 0.0;
float angle_y = 0.0;
float angle_z = 0.0;
char    accel_x_str[10], accel_y_str[10], accel_z_str[10], gyro_x_str[10], gyro_y_str[10], gyro_z_str[10];
uint8_t rawData_accel[6], rawData_gyro[6];
int16_t accelCount[3], gyroCount[3];
char    message[255] = {0,};

//relay_channel
#define relay1 PB6 //digital pin 12
#define relay2 PB7 //digital pin 13

volatile int EncoderCount_L = 0; //per one rotation
float kp, kd, ki;
double integral = 0.0;
long preT = 0;
double error=0.0;
double pre_error=0.0;

//communication
SoftwareSerial HC05_t(2, 3); // 아두이노 보드 기준(RX, TX), 블루투스 모듈 기준(TXD, RXD)으로 핀을 2번 3번으로 설정

void setup() {
  Serial.begin(115200);
  //left motor pinmode
  pinMode(Motor_L_pin1, OUTPUT);
  pinMode(Motor_L_pin2, OUTPUT);
  pinMode(EN_L,  OUTPUT);
  pinMode(Encoder_L_PulseA, INPUT);
  pinMode(Encoder_L_PulseB, INPUT);

  //right motor pinmode
  pinMode(Motor_R_pin1, OUTPUT);
  pinMode(Motor_R_pin2, OUTPUT);
  pinMode(EN_R,  OUTPUT);
  pinMode(Encoder_R_PulseA, INPUT_PULLUP);
  pinMode(Encoder_R_PulseB, INPUT_PULLUP);
  delay(5000); // delay 5 seconds to run

  attachInterrupt(digitalPinToInterrupt(Encoder_L_PulseA),EncoderPositionRead, RISING);

  digitalWrite(Motor_L_pin1, HIGH);
  digitalWrite(Motor_L_pin2, LOW); //확인해보기 앞 뒤 주행
  analogWrite(EN_L, pwm_L);
  PID_gain();

  DDRB |=  (1 << PB2) | (1 << PB3) | (1 << PB5);
  DDRB &= ~(1 << PB4); //NCS DISABLE for SPI communication
  SPCR |=   (1 << SPE)  | (1 << MSTR) | (1 << SPR0);
  SPCR &= ~((1 << DORD) | (1 << CPOL) | (1 << CPHA) | (1 << SPR1));
  PORTB |= (1 << PB2); 
  setup_scale(acc_full_scale, gyro_full_scale);
  //disable i2c
  uint8_t current_setting = transfer_SPI(USER_CTRL, 0x00, true); // Read USER_CTRL
  current_setting |= 0x10;   // Set I2C_IF_DIS to one (I2C_IF_DIS: bit 4 of USER_CTRL register)
  transfer_SPI(USER_CTRL, current_setting, false);   // Write USER_CTRL

  //pinMode(relay1, OUTPUT);
  //pinMode(relay2, OUTPUT);
  DDRB |= (1<<relay1)|(1<<relay2);

  //communication
  HC05_t.begin(38400); //아두이노-블루투스모듈간 통신 baud rate 설정
}

void loop() {
  long nowT = micros();
  double dt = ((double)(nowT - preT)/1.0e6);
  float velocity_L = Convert_CtoV(EncoderCount_L, dt);

  //pid
  pid(error, dt);

  //for next loop
  EncoderCount_L = 0;
  preT = nowT;

  //mpu9250
  read_AccelData();
  read_GyroData();
  sprintf(message, "accel_x = %s, accel_y = %s, accel_z = %s, gyro_x = %s, gyro_y = %s, gyro_z = %s,", accel_x_str, accel_y_str, accel_z_str, gyro_x_str, gyro_y_str, gyro_z_str);
  Serial.println(message);

  HC05_t.print("accel_x_str"); // 통신으로 보내고자 하는 정보를 입력
  HC05_t.print("accel_y_str");
  HC05_t.print("accel_z_str");
  HC05_t.print("gyro_x_str");
  HC05_t.print("gyro_y_str");
  HC05_t.print("gyro_z_str");
  //displacement value_x,y,z
  delay(1000); // 딜레이를 1000ms로 설정 즉, 1초마다 블루투스 모듈을 통해 정보 송신

  //relay channel switching
  relay_channel_on();
  delay(10000);
  relay_channel_off();
  delay(1000);
}

//반시계 방향이 정방향일 때 (Left Motor 기준)
void EncoderPositionRead() {
  if (digitalRead(Encoder_L_PulseA) == digitalRead(Encoder_L_PulseB)) {
    EncoderCount_L++;
  } 
  else {
    EncoderCount_L--;
  }
}

void PID_gain(){
  kp = 0.8;
  ki = 0.2;
  kd = 0.1;
}

double pid(double error, double dt)
{
  double proportional = error;
  integral = integral + error * dt;
  double derivative = (error - pre_error) / dt;
  pre_error = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}

float Convert_CtoV(int count, double dt){
  float rpm = (float)(count * 60.0 / ENC_COUNT_REV /dt);
  float circumference = 2 * 3.141592 * WHEEL_RADIUS;
  float velocity = rpm / 60.0 * circumference;
  return velocity;
}

uint8_t transfer_SPI(uint8_t registerAddress, uint8_t data, bool isRead) {
    uint8_t response = 0x00;
    // Set MSB = 1 for read
    registerAddress |= (isRead ? 0x80 : 0x00);    
    
    // SS: low (active)
    PORTB &= ~(1 << PB2);      
    
    // Register Address transfer
    SPDR = registerAddress;
    while (!(SPSR & (1 << SPIF)));    
    // Data transfer
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    response = SPDR;
    
    // SS: high (inactive)
    PORTB |= (1 << PB2);  
    return response;
}

void setup_scale(uint8_t scale_a, uint8_t scale_g) {
  uint8_t current_config_accel = transfer_SPI(ACCEL_CONFIG, 0x00, true);
  uint8_t current_config_gyro = transfer_SPI(GYRO_CONFIG, 0x00, true);
  
  current_config_accel &= ~0x18; // Set 00 to ACCEL_FS_SEL[1:0]
  current_config_gyro &= ~0x18;

  current_config_accel |= (scale_a << 3); // Set ACCEL_FS_SEL to scale
  current_config_gyro |= (scale_g << 3); // Set ACCEL_FS_SEL to scale


  transfer_SPI(ACCEL_CONFIG, current_config_accel, false); // Write accel config

  // Set resolution
  switch (scale_a) {
    case 0x00: // +- 2g
      accel_scale = 2.0f / 32768.0f;
      break;
    case 0x01: // +- 4g
      accel_scale = 4.0f / 32768.0f;
      break;
    case 0x02: // +- 8g
      accel_scale = 8.0f / 32768.0f;
      break;
    case 0x03: // +- 16g
      accel_scale = 16.0f / 32768.0f;
      break;
  }

  accel_scale = accel_scale * 9.80665f; //g to m/s^2

  transfer_SPI(GYRO_CONFIG, current_config_gyro, false); // Write accel config

    // Set resolution
  switch (scale_g) {
    case 0x00: // +- 250 deg
      gyro_scale = 250.0f / 32768.0f;
      break;
    case 0x01: // +- 500 deg
      gyro_scale = 500.0f / 32768.0f;
      break;
    case 0x02: // +- 1000 deg
      gyro_scale = 1000.0f / 32768.0f;
      break;
    case 0x03: // +- 16g
      gyro_scale = 2000.0f / 32768.0f;
      break;
  }

}

void read_AccelData() {
  rawData_accel[0] = transfer_SPI(ACCEL_XOUT_H, 0x00, true);
  rawData_accel[1] = transfer_SPI(ACCEL_XOUT_L, 0x00, true);
  rawData_accel[2] = transfer_SPI(ACCEL_YOUT_H, 0x00, true);
  rawData_accel[3] = transfer_SPI(ACCEL_YOUT_L, 0x00, true);
  rawData_accel[4] = transfer_SPI(ACCEL_ZOUT_H, 0x00, true);
  rawData_accel[5] = transfer_SPI(ACCEL_ZOUT_L, 0x00, true);
  
  accelCount[0] = (rawData_accel[0] << 8) | rawData_accel[1];
  accelCount[1] = (rawData_accel[2] << 8) | rawData_accel[3];
  accelCount[2] = (rawData_accel[4] << 8) | rawData_accel[5];

  accel_x = accelCount[0] * accel_scale;
  accel_y = accelCount[1] * accel_scale;
  accel_z = accelCount[2] * accel_scale;
  
  dtostrf(accel_x, 4, 2, accel_x_str);
  dtostrf(accel_y, 4, 2, accel_y_str);
  dtostrf(accel_z, 4, 2, accel_z_str);

  if (accel_x^2+accel_y^2+accel_z^2 > crash){
    relay_channel_off();
  }

  //accel_x = ((int16_t)(rawData[0] << 8 | rawData[1])) / ACCEL_SCALE + 0.002;
  //accel_y = ((int16_t)(rawData[2] << 8 | rawData[3])) / ACCEL_SCALE - 0.008;
  //accel_z = ((int16_t)(rawData[4] << 8 | rawData[5])) / ACCEL_SCALE + 0.020;
}

void read_GyroData() {
  rawData_gyro[0] = transfer_SPI(GYRO_XOUT_H, 0x00, true);
  rawData_gyro[1] = transfer_SPI(GYRO_XOUT_L, 0x00, true);
  rawData_gyro[2] = transfer_SPI(GYRO_YOUT_H, 0x00, true);
  rawData_gyro[3] = transfer_SPI(GYRO_YOUT_L, 0x00, true);
  rawData_gyro[4] = transfer_SPI(GYRO_ZOUT_H, 0x00, true);
  rawData_gyro[5] = transfer_SPI(GYRO_ZOUT_L, 0x00, true);

  gyroCount[0] = (rawData_gyro[0] << 8) | rawData_gyro[1];
  gyroCount[1] = (rawData_gyro[2] << 8) | rawData_gyro[3];
  gyroCount[2] = (rawData_gyro[4] << 8) | rawData_gyro[5];

  gyro_x = gyroCount[0] * gyro_scale;
  gyro_y = gyroCount[1] * gyro_scale;
  gyro_z = gyroCount[2] * gyro_scale;

  dtostrf(gyro_x, 4, 2, gyro_x_str);
  dtostrf(gyro_y, 4, 2, gyro_y_str);
  dtostrf(gyro_z, 4, 2, gyro_z_str);

  //gyro_x = ((int16_t)(rawData[0] << 8 | rawData[1])) / GYRO_SCALE * (PI / 180.0);
  //gyro_y = ((int16_t)(rawData[2] << 8 | rawData[3])) / GYRO_SCALE * (PI / 180.0);
  //gyro_z = ((int16_t)(rawData[4] << 8 | rawData[5])) / GYRO_SCALE * (PI / 180.0);
}

void calculateDisplacement() {
  // 속도 적분
  velocity_x += accel_x * mpu_dt;
  velocity_y += accel_y * mpu_dt;
  velocity_z += accel_z * mpu_dt;


  // 변위 적분
  displacement_x += velocity_x * mpu_dt;
  displacement_y += velocity_y * mpu_dt;
  displacement_z += velocity_z * mpu_dt;
}

void calculateDirection() {
  angle_x += gyro_x * mpu_dt;
  angle_y += gyro_y * mpu_dt;
  angle_z += gyro_z * mpu_dt;
  
}

void relay_channel_on() 
{
   //digitalWrite(relay1, HIGH);
   //digitalWrite(relay2, HIGH);
  // PORTB |= (1<<relay1)|(1<<relay2);
  PORTB |= (1<<relay1);
  PORTB |= (1<<relay2); //동시진행 안한 이유 회로의 안전성 때문
}

void relay_channel_off() 
{
   //digitalWrite(relay1, LOW);
   //digitalWrite(relay2, LOW);
  //PORTB &= ~((1<<relay1)|(1<<relay2))
  PORTB &= ~((1<<relay1));
  PORTB &= ~((1<<relay2)); //동시진행 안한 이유 회로의 안전성 때문
}
