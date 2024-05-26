#define ENC_COUNT_REV 330 // CPR
#define WHEEL_RADIUS 0.075

#define Encoder_L_PulseA 18 // Yellow
#define Encoder_L_PulseB 19 // White
#define EN_L 9 
#define Motor_L_pin1 2
#define Motor_L_pin2 3 // forward

volatile long EncoderCount_L = 0;
volatile long pre_EncoderCount_L = 0;
double integral = 0.0;
long preT = 0;
double pre_error = 0.0;
volatile int pwm_L = 240; // 초기 pwm

void setup() {
  Serial.begin(9600);

  pinMode(Motor_L_pin1, OUTPUT);
  pinMode(Motor_L_pin2, OUTPUT);
  pinMode(EN_L, OUTPUT);
  pinMode(Encoder_L_PulseA, INPUT);
  pinMode(Encoder_L_PulseB, INPUT);

  attachInterrupt(digitalPinToInterrupt(Encoder_L_PulseA), EncoderPositionRead, RISING);

  digitalWrite(Motor_L_pin1, LOW);
  digitalWrite(Motor_L_pin2, HIGH); //FORWARD
  analogWrite(EN_L, pwm_L);
}

void loop() {
  long nowT = micros();
  float dt = ((float)(nowT - preT) / (1.0e6));
  long current_count = EncoderCount_L - pre_EncoderCount_L;
  pre_EncoderCount_L = EncoderCount_L;

  float velocity_L = Convert_CtoV(current_count, dt); //m/s conversion
  float target_velocity_L = 2.0; //m/s target speed
  float output = pid_speed(target_velocity_L - velocity_L, dt);

  Motor_Drive(output, target_velocity_L, velocity_L);
  // serial plotting
  Serial.print("Target:");
  Serial.print(target_velocity_L);
  Serial.print(",");
  Serial.print("Velocity:");
  Serial.print(velocity_L);
  Serial.print(",");
  Serial.print("PWM:");
  Serial.println(pwm_L);
  preT = nowT;
  delay(100);
}

void Motor_Drive(float output, float target, float real) {
  if (output < 0) output = -output;
  pwm_L = (int)output;
  if (target<real){
    pwm_L = 0;
  else if (pwm_L > 255) pwm_L = 255;
  analogWrite(EN_L, pwm_L); 
}

void EncoderPositionRead() {
  EncoderCount_L++;
}

float pid_speed(float error, float dt) {
  float kp = 800.0;
  float kd = 700.0;
  float ki = 600.0;
  float proportional = error;
  integral = integral + error * dt;
  float derivative = (error - pre_error) / dt;
  float output = (kp * proportional) + (ki * integral) + (kd * derivative);

  //for next loop
  pre_error = error;
  return output;
}

float Convert_CtoV(int count, float dt) {
  float rpm = (float)(count * 60.0 / ENC_COUNT_REV / dt);
  float circumference = 2 * 3.141592 * WHEEL_RADIUS;
  float velocity = rpm / 60.0 * circumference;
  return velocity;
}
