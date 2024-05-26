#include <SoftwareSerial.h>

#define FOSC 16000000
#define BAUD 9600
#define ENC_COUNT_REV 330 // CPR
#define WHEEL_RADIUS 0.075

#define Encoder_L_PulseA (1<<PD3) //18
#define Encoder_R_PulseA (1<<PD1) //20 INT1
#define EN_L 9//(1<<PH6) //9 
#define EN_R 10//(1<<PB4) //10
#define Motor_L_pin2 (1<<PE5) //3,  forward
#define Motor_R_pin2 (1<<PE3) //5, forward

#define relay1 PB6 //digital pin 12
#define relay2 PB7 //digital pin 13

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

volatile long EncoderCount_L = 0;
volatile long EncoderCount_R = 0;

volatile long pre_EncoderCount_L = 0;
float e_integral_speed = 0.0;
float e_integral_pos = 0.0;
long preT = 0;
float pre_error_position = 0.0;
float pre_error_speed = 0.0;

volatile int pwm_L = 240; // 초기 pwm

uint8_t acc_full_scale = 0x01; // Set accel scale (+-2g: 0x00, +-4g: 0x01, +-8g: 0x02, +- 16g: 0x03)
uint8_t gyro_full_scale = 0x02; // Set gyro scale (00 = +250dps, 01= +500 dps, 10 = +1000 dps, 11 = +2000 dps )
int crash = 1000;
float   accel_scale, gyro_scale;
float   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
const float mpu_dt = 0.01; //time vary 10ms
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

unsigned long previousMillis = 0;
float filter_x=0;
float filter_y=0;
float filter_z=0;
int temp_motorcount = 0;
int temp_timecount = 0;

//communication
SoftwareSerial HC05_t(2, 3); // 아두이노 보드 기준(RX, TX), 블루투스 모듈 기준(TXD, RXD)으로 핀을 2번 3번으로 설정


void setup() {
  serial_baud(); //instead of serial.begin
  DDRE |= (Motor_L_pin2); //pinMode(Motor_L_pin2, OUTPUT);
  DDRE |= (Motor_R_pin2); //pinMode(Motor_R_pin2, OUTPUT);
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  DDRD &= ~(Encoder_L_PulseA); //pinMode(Encoder_L_PulseA, INPUT);
  DDRD &= ~(Encoder_R_PulseA); //pinMode(Encoder_R_PulseA, INPUT);
  PORTE |= (Motor_L_pin2); //digitalWrite(Motor_L_pin2, HIGH); //FORWARD
  PORTE |= (Motor_R_pin2);//digitalWrite(Motor_R_pin2, HIGH); //PORT 활용
  analogWrite(EN_L, pwm_L);

  init_INT();

//global interrupt enable
  SREG |= 0x01 << SREG_I;

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
  float kp_pos = 6.0;
  float ki_pos = 0.7;
  float kd_pos = 6.8;
  long nowT = micros();
  float dt = ((float)(nowT - preT) / (1.0e6));
  long current_count = EncoderCount_L - pre_EncoderCount_L;
  pre_EncoderCount_L = EncoderCount_L;

  float velocity_L = Convert_CtoV(current_count, dt); //m/s conversion
  float target_velocity_L = 1.5; //m/s target speed
  float output_speed = pid_speed(target_velocity_L - velocity_L, dt);

  Motor_L_Drive(output_speed, target_velocity_L, velocity_L);
  float output_pos = PID_pos_sync(EncoderCount_L, kp_pos, kd_pos, ki_pos);
  Motor_R_Drive(output_pos);


  //mpu9250
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= dt * 1000) {
    previousMillis = currentMillis;

    read_AccelData();
    read_GyroData();

    //filter_temp();
    lowpass_filter();
    thresholding_filter();
    calculateDisplacement();
  
    //Serial.print("accel_x = "); Serial.print(accel_x); Serial.print(", accel_y = "); Serial.print(accel_y); Serial.print(", accel_z = "); Serial.println(accel_z);
    //Serial.print("filter_x = "); Serial.print(filter_x); Serial.print(", filter_y = "); Serial.print(filter_y); Serial.print(", filter_z = "); Serial.println(filter_z);
    
    //Serial.print("velocity_x = "); Serial.print(velocity_x); Serial.print(", velocity_y = "); Serial.print(velocity_y); Serial.print(", velocity_z = "); Serial.println(velocity_z);
    Serial.print("displacement_x = "); Serial.print(displacement_x); Serial.print(", displacement_y = "); Serial.print(displacement_y); Serial.print(", displacement_z = "); Serial.println(displacement_z);
  }

  HC05_t.print("accel_x_str"); // 통신으로 보내고자 하는 정보를 입력
  HC05_t.print("accel_y_str");
  HC05_t.print("accel_z_str");
  HC05_t.print("gyro_x_str");
  HC05_t.print("gyro_y_str");
  HC05_t.print("gyro_z_str");

///////////////////////////////////////
  // //relay channel switching
  // relay_channel_on();
  // delay(10000);
  // relay_channel_off();
  // delay(1000);
///////////////////////////////////////

  // serial plotting
  Serial.print("Target:");
  Serial.print(target_velocity_L);
  Serial.print(",");
  Serial.print("Velocity:");
  Serial.print(velocity_L);
  Serial.print(",");
  Serial.print("PWM:");
  Serial.println(pwm_L);

  //Serial.print("EncoderCount_L:");
  //Serial.print(EncoderCount_L);
  //Serial.print(",");
  //Serial.print("EncoderCount_R:");
  //Serial.println(EncoderCount_R);
  preT = nowT;
  delay(100);
}

float pid_speed(float error, float dt) {
  float kp_speed = 1000.0;
  float kd_speed = 700.0;
  float ki_speed = 600.0;
  float proportional = error;
  e_integral_speed = e_integral_speed + error * dt;
  float derivative = (error - pre_error_speed) / dt;
  float output_speed = (kp_speed * proportional) + (ki_speed * e_integral_speed) + (kd_speed * derivative);

  //for next loop
  pre_error_speed = error;
  return output_speed;
}

void serial_baud(void){
  uint16_t ubrr = FOSC / 16 / BAUD - 1;
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B |= (1<<RXEN0) | (1<<TXEN0);
  UCSR0C |= (1<<USBS0);
  UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
}


float PID_pos_sync(int target, float kp_pos, float kd_pos, float ki_pos) {
  long currentTime = micros(); 
  float dt = ((float)(currentTime - preT)) / 1.0e6;
  int error = target - EncoderCount_R;
  e_integral_pos += error * dt;
  float e_derivative = (error - pre_error_position) / dt;
  float output_pos = (kp_pos * error) + (kd_pos * e_derivative) + (ki_pos * e_integral_pos);
  //for next loop
  pre_error_position = error;
  return output_pos;
}

//motor_l_drive by pid speed
void Motor_L_Drive(float output, float target, float real) {
  if (output < 0) output = -output;
  pwm_L = (int)output;

  if (target<real)
    pwm_L = 0;
  else if (pwm_L > 255) 
    pwm_L = 255;
  analogWrite(EN_L, pwm_L); 
}

//motor_r_drive by pid syn
void Motor_R_Drive(float output) {
  if (output < 0) output = -output;
  int pwm_R = (int)output;

  if (EncoderCount_R > EncoderCount_L) 
    pwm_R = 0;
  else if (pwm_R > 255) 
    pwm_R = 255;
  analogWrite(EN_R, pwm_R); //duty cycle 활용
}

//to velocity
float Convert_CtoV(int count, float dt) {
  float rpm = (float)(count * 60.0 / ENC_COUNT_REV / dt);
  float circumference = 2 * 3.141592 * WHEEL_RADIUS;
  float velocity = rpm / 60.0 * circumference;
  return velocity;
}

//interrupt function
void init_INT(){
  EICRA |= (1 << ISC30) | (1 << ISC31); //rising edge for int3
  EICRA |= (1 << ISC10) | (1 << ISC11); //rising edge for int1
  EIMSK |= (1 << INT3); //local interrpt for 3
  EIMSK |= (1 << INT1); //local interrpt for 1
}

//INT3 ISR
ISR(INT3_vect) {
  EncoderCount_L++;
}

//INT1 ISR
ISR(INT1_vect) {
  EncoderCount_R++;
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

  if (accel_x*accel_x +accel_y*accel_y + accel_z*accel_z > crash){
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

void filter_temp(){
  filter_x=accel_x;
  filter_y=accel_y;
  filter_z=accel_z;
}

void lowpass_filter() {
  float alpha = 0.5;
  filter_x = alpha * accel_x + (1 - alpha) * filter_x;
  filter_y = alpha * accel_y + (1 - alpha) * filter_y;
  filter_z = alpha * accel_z + (1 - alpha) * filter_z;
}

void thresholding_filter(){
  if (filter_x < 0.15)
    filter_x = 0;
  if (filter_y < 0.15)
    filter_y = 0;
  if (filter_z < 0.15)
    filter_z = 0;
  if (temp_timecount==500){
    if (temp_motorcount==EncoderCount_L)
      velocity_x=velocity_y=velocity_z=0;
    else
      temp_motorcount = EncoderCount_L;
  }
  temp_timecount ++;
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
