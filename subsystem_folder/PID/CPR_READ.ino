#define Motor_pin1 2 // Yellow
#define Motor_pin2 3 // White
#define Encoder_A 18
#define Encoder_B 19
#define ENA 9

volatile long EncoderCount = 0;

void setup() {
  pinMode(Motor_pin1, OUTPUT);
  pinMode(Motor_pin2, OUTPUT);
  pinMode(Encoder_A, INPUT);
  pinMode(Encoder_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Encoder_A), EncoderPositionRead, RISING);
  Serial.begin(9600);

}

void loop(){
  Serial.print("Encoder Count: ");
  Serial.println(EncoderCount);
  delay(1000);

}


void EncoderPositionRead() {
  if (digitalRead(Encoder_A) == digitalRead(Encoder_B)) {
    EncoderCount++;
  } 
  else {
    EncoderCount--;
  }
}
