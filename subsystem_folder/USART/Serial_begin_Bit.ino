#define FOSC 16000000 
#define BAUD 115200
void setup(){
  //instead of Serial.begin(9600);
  serial_baud();
}

void loop(){
  Serial.println("done");
  delay(1000);
}


void serial_baud(void){
  uint16_t ubrr = FOSC / 16 / BAUD - 1;
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B |= (1<<RXEN0) | (1<<TXEN0);
  UCSR0C |= (1<<USBS0);
  UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
}
