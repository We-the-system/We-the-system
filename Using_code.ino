#define encoder0PinA 18
#define encoder0PinB 19

int encoder0Pos = 0;

void setup() {
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE); // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE); // encoder pin on interrupt 1 (pin 3)
  Serial.begin (115200);
}

void loop(){}

void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  Serial.println (encoder0Pos);
}

void doEncoderB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  Serial.println (encoder0Pos);
}

//타켓값을 정
//시간측정해서 단위 식간 동안  바뀐 pulse 330(한바퀴)

//pulse값을 통해 (m/s)를 구해야돼