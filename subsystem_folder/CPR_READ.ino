#define ENC_COUNT_REV 330 //

#define encoderPinA 18 // Yellow
#define encoderPinB 19 // White
#define ENA 9

int motor1pin1 = 2;
int motor1pin2 = 3;
//int motor2pin1 = 4; int motor2pin2 = 5;
volatile long EncoderCount = 0;

int timeinterval = 1000; //output time interval

long previousMillis = 0;
long currentMillis = 0;

float rpm_right = 0;
 

float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
const float wheel_diameter = 0.067;
///여기까지 시험



void EncoderPositionRead();




void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
//  pinMode(motor2pin1,  OUTPUT); pinMode(motor2pin2, OUTPUT);

  pinMode(ENA,  OUTPUT);  //pinMode(10, OUTPUT);
  //(Optional)
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA),EncoderPositionRead, RISING);
}

void loop() {


  analogWrite(ENA, 255); //ENA  pin //analogWrite(10, 200); //ENB pin
  digitalWrite(motor1pin1,  LOW);
  digitalWrite(motor1pin2, HIGH);
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of pulses
  float timedifference = currentMillis - previousMillis;
  //if (timedifference > timeinterval) {
  previousMillis = currentMillis;
    // Calculate revolutions per minute
  rpm_right = (float)(EncoderCount * 60 / ENC_COUNT_REV);
  ang_velocity_right = rpm_right * rpm_to_radians;   
  ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
     

  float wheel_speed = ang_velocity_right * wheel_diameter / 2;
  Serial.print(" cont: ");

  Serial.print(EncoderCount);
  Serial.println();
 
  EncoderCount = 0;
  
}


void EncoderPositionRead() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    EncoderCount++;
  } 
  else {
    EncoderCount--;
  }
}

