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

volatile long EncoderCount_L = 0;
volatile long EncoderCount_R = 0;

volatile long pre_EncoderCount_L = 0;
float e_integral_speed = 0.0;
float e_integral_pos = 0.0;
long preT = 0;
float pre_error_position = 0.0;
float pre_error_speed = 0.0;

volatile int pwm_L = 240; // 초기 pwm

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

