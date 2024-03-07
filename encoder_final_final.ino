// motor control pins
const int motorDirPin1 = 8; // L298 Input 1 for motor 1
const int motorPWMPin1 = 9; // L298 Input 2 for motor 1
const int motorDirPin2 = 10; // L298 Input 1 for motor 2
const int motorPWMPin2 = 11; // L298 Input 2 for motor 2

// encoder pins
const int encoderPinA1 = 18; // encoder pin A for motor 1
const int encoderPinB1 = 19; // encoder pin B for motor 1
const int encoderPinA2 = 20; // encoder pin A for motor 2
const int encoderPinB2 = 21; // encoder pin B for motor 2

// Variables for PID control
float Kp = 0.1; // Proportional gain
float targetVelocity = 100.0; // Target velocity in degrees per second
float currentVelocity1 = 0.0; // Current velocity for motor 1
float currentVelocity2 = 0.0; // Current velocity for motor 2
float error1, error2; // Error terms for PID control
float output1, output2; // PID control outputs
unsigned long lastTime; // Time tracking for PID control
float dt = 0.1; // Time interval for PID control in seconds
int tempvel;



// Variables for encoder
volatile int encoderPos1 = 0; // Encoder position for motor 1
volatile int encoderPos2 = 0; // Encoder position for motor 2
unsigned long lastEncoderTime1 = 0; // Last time for encoder 1
unsigned long lastEncoderTime2 = 0; // Last time for encoder 2

void doMotor1(bool dir, int vel) {
  digitalWrite(motorDirPin1, dir);
  analogWrite(motorPWMPin1, vel);
}

void doMotor2(bool dir, int vel) {
  digitalWrite(motorDirPin2, dir);
  analogWrite(motorPWMPin1, vel);
}

void setup() {
  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);

  pinMode(motorDirPin1, OUTPUT);
  pinMode(motorPWMPin1, OUTPUT);
  pinMode(motorDirPin2, OUTPUT);
  pinMode(motorPWMPin2, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), doEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), doEncoder2, CHANGE);

  // Initialize PID control
  lastTime = millis();
}

void loop() {
  // Calculate velocity for motor 1
  currentVelocity1 = calculateVelocity(encoderPos1, lastEncoderTime1);
  // Calculate velocity for motor 2
  currentVelocity2 = calculateVelocity(encoderPos2, lastEncoderTime2);

  // PID control for motor 1
  error1 = targetVelocity - currentVelocity1;
  output1 = Kp * error1;

  // PID control for motor 2
  error2 = targetVelocity - currentVelocity2;
  output2 = Kp * error2;

  // Apply PID control to motors
  doMotor1(HIGH, 250);
  doMotor2(HIGH, 250);

  // Output data to serial monitor
  Serial.print("Motor 1 Encoder Vel: ");
  Serial.print(encoderPos1);
  Serial.print(" | Motor 2 Encoder Vel: ");
  Serial.print(encoderPos2);
  Serial.print(" | PID Output 1: ");
  Serial.print(output1);
  Serial.print(" | PID Output 2: ");
  Serial.println(output2);

  // Delay for PID control
  unsigned long now = millis();
  unsigned long elapsedTime = now - lastTime;
  if (elapsedTime >= dt * 1000) {
    lastTime = now;
    // Perform PID control every dt seconds
  }
}

float calculateVelocity(volatile int &encoderPos, unsigned long &lastEncoderTime) {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastEncoderTime;
  lastEncoderTime = currentTime;

  // Calculate velocity in degrees per second
  float velocity = (encoderPos / 360.0) / (elapsedTime / 1000.0);

  // Reset encoder position
  encoderPos = 0;

  return velocity;
}

void doEncoder1() {
  if (digitalRead(encoderPinA1) == digitalRead(encoderPinB1)) {
    encoderPos1--;
  } else {
    encoderPos1++;
  }
}

void doEncoder2() {
  if (digitalRead(encoderPinA2) == digitalRead(encoderPinB2)) {
    encoderPos2++;
  } else {
    encoderPos2--;
  }
}