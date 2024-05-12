//pin
#define Motor_L_pin2 3
#define EN_L 9
#define Encoder_L_PulseA 18
#define Motor_R_pin2 5
#define EN_R 10
#define Encoder_R_PulseA 20

#define PWM_L 240

//Encoder Count
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

  //left motor drive
  digitalWrite(Motor_L_pin2, HIGH);
  analogWrite(EN_L, PWM_L);

  attachInterrupt(digitalPinToInterrupt(Encoder_L_PulseA), EncoderPositionRead_L, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_R_PulseA), EncoderPositionRead_R, RISING);
}

void loop() {
  float kp = 6.0;
  float ki = 0.7;
  float kd = 6.8;
  float output = PID_pos_sync(EncoderCount_L, kp, kd, ki);
  Motor_R_Drive(output);
  //for serial plotter
  Serial.print("EncoderCount_L:");
  Serial.print(EncoderCount_L);
  Serial.print(",");
  Serial.print("EncoderCount_R:");
  Serial.println(EncoderCount_R);
}

void EncoderPositionRead_L() {
  EncoderCount_L++;
}
void EncoderPositionRead_R() {
  EncoderCount_R++;
}

float PID_pos_sync(int target, float kp, float kd, float ki) {
  long currentTime = micros(); //for package in function use
  float dt = ((float)(currentTime - preT)) / 1.0e6;
  //target and encodercount_r all integal
  int error = target - EncoderCount_R; 
  e_integral = e_integral + error * dt;
  float e_derivative = (error - pre_error) / dt;
  float output = (kp * error) + (kd * e_derivative) + (ki * e_integral);

  //for next loop
  preT = currentTime;
  pre_error = error;
  return output;
}

void Motor_R_Drive(float output){
  //for absolute
  if (output<0) output = output*(-1);
  int pwm_R = output;

  if (EncoderCount_R > EncoderCount_L){
    pwm_R = 0;
  }
  else if (pwm_R > 255){
    pwm_R = 255;
  }
  digitalWrite(Motor_R_pin2, HIGH);
  analogWrite(EN_R, pwm_R);
}

