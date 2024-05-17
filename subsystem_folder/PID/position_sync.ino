#define Motor_L_pin2 3
#define EN_L 9 
#define Encoder_L_PulseA 18 // INT3
#define Motor_R_pin2 5
#define EN_R 10
#define Encoder_R_PulseA 20 // INT1

#define PWM_L 240

// encodercount
volatile long EncoderCount_L = 0;
volatile long EncoderCount_R = 0;

//pid variable
long preT = 0;
float pre_error = 0;
float e_integral = 0;

void setup() {
  Serial.begin(9600);

  //pinmode
  pinMode(Motor_L_pin2, OUTPUT);
  pinMode(Motor_R_pin2, OUTPUT);
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(Encoder_L_PulseA, INPUT);
  pinMode(Encoder_R_PulseA, INPUT);

  digitalWrite(Motor_L_pin2, HIGH);
  analogWrite(EN_L, PWM_L);
  init_INT();

  //global interrupt enable
  SREG |= 0x01 << SREG_I;
}

void loop() {
  float kp = 6.0;
  float ki = 0.7;
  float kd = 6.8;
  float output = PID_pos_sync(EncoderCount_L, kp, kd, ki);
  Motor_R_Drive(output);

  // for serial plot
  Serial.print("EncoderCount_L:");
  Serial.print(EncoderCount_L);
  Serial.print(",");
  Serial.print("EncoderCount_R:");
  Serial.println(EncoderCount_R);
  delay(100);
}

void init_INT(){
  EICRA |= (1 << ISC30) | (1 << ISC31); //falling
  EICRA |= (1 << ISC10) | (1 << ISC11); //falling
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

float PID_pos_sync(int target, float kp, float kd, float ki) {
  long currentTime = micros(); 
  float dt = ((float)(currentTime - preT)) / 1.0e6;
  int error = target - EncoderCount_R;
  e_integral += error * dt;
  float e_derivative = (error - pre_error) / dt;
  float output = (kp * error) + (kd * e_derivative) + (ki * e_integral);

  //for next loop
  preT = currentTime;
  pre_error = error;
  return output;
}

void Motor_R_Drive(float output) {
  if (output < 0) output = -output;
  int pwm_R = output;

  if (EncoderCount_R > EncoderCount_L) {
    pwm_R = 0;
  } else if (pwm_R > 255) {
    pwm_R = 255;
  }
  
  digitalWrite(Motor_R_pin2, HIGH); //PORT 활용
  analogWrite(EN_R, pwm_R); //duty cycle 활용
}


