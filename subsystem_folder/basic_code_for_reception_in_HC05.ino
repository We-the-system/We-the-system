//수신용 (HC-05) 기본코드 구조

#include <SoftwareSerial.h>

//연결led
#define DIN 13
#define CS 11
#define CLK 10
#define LENGTH_OF_TRACK 12
#define NUMBER_OF_ROWS 8
int displacement_square, temp;

SoftwareSerial HC05_r(2, 3); // 아두이노 보드 기준(RX, TX), 블루투스 모듈 기준(TXD, RXD)으로 핀을 2번 3번으로 설정

void setup() {
  Serial.begin(9600); //PC-아두이노간 통신 baud rate 설정
  HC05_r.begin(38400); //아두이노-블루투스모듈간 통신 baud rate 설정

  //led setup
  pinMode(CLK,OUTPUT);
  pinMode(DIN,OUTPUT);
  pinMode(CS,OUTPUT);
  delay(100);

  bool verbose = false;
  write_Max7219(0x09, 0x00, verbose);
  write_Max7219(0x0A, 0x00, verbose);
  write_Max7219(0x0B, 0x07, verbose);
  write_Max7219(0x0C, 0x01, verbose);
  write_Max7219(0x0F, 0x00, verbose);

  for(uint8_t i=0;i<NUMBER_OF_ROWS;i++){
    write_Max7219(i+1,0x00,verbose);
  }

}

void loop() {
  /*
  if (HC05_r.available()) { //HC05_r에 연결된 아두이노 데이터가 들어오면 실행 되는 조건문
    String text = HC05_r.readStringUntil(0x0d); // 종료문자까지 데이터를 읽어서 text변수에 저장 이때, 0x0d는 carriage return
    Serial.println(text); // text 변수에 저장된 값을 시리얼 모니터에 출력
  }
  */

  if (HC05_r.available()) { //HC05_r에 연결된 아두이노 데이터가 들어오면 실행 되는 조건문
      String text = HC05_r.readStringUntil(0x0d); // 종료문자까지 데이터를 읽어서 text변수에 저장 이때, 0x0d는 carriage return
      int change = text.toInt(); //문자열을 정수형으로 변환하는 코드
      Serial.print("text:");
      Serial.println(change); // text 변수에 저장된 값을 시리얼 모니터에 출력
    }
  }
  Serial.println("");
  displacement_square=square(displacement_x,displacement_y,displacement_z);
  
  for(uint8_t i=0; i<LENGTH_OF_TRACK; i++){
    for(uint8_t j=0; j<NUMBER_OF_ROWS; j++){
      write_Max7219(j+1, matrix[i][j], true);
    }
    temp=displacement_square/mapping_unit
    if (temp>1){
      i=i+temp;
    }
  }
}

void write_Max7219(uint8_t address, uint8_t data, bool verbose){
  digitalWrite(CS,LOW);
  write_byte(address,verbose);
  write_byte(data,verbose);
  digitalWrite(CS,HIGH);
}

void write_byte(uint8_t data, bool verbose){
  bool bit;
  for (uint8_t i=0; i<8; i++){
    bit = (bool)(data & (1<<(7-i)));
    if (verbose){
      Serial.print(bit);
    }
    digitalWrite(DIN,bit);
    digitalWrite(CLK,HIGH);
    digitalWrite(CLK,LOW);
  }
  Serial.println("");
}

const uint8_t matrix[12][8] = {
  //{0x00, 0x18, 0x24, 0x42, 0x42, 0x24, 0x18, 0x00},
  {0x00, 0x18, 0x24, 0x42, 0x42, 0x24, 0x10, 0x00},
  {0x00, 0x18, 0x24, 0x42, 0x42, 0x20, 0x18, 0x00},
  {0x00, 0x18, 0x24, 0x42, 0x40, 0x24, 0x18, 0x00},
  {0x00, 0x18, 0x24, 0x40, 0x42, 0x24, 0x18, 0x00},
  {0x00, 0x18, 0x20, 0x42, 0x42, 0x24, 0x18, 0x00},
  {0x00, 0x10, 0x24, 0x42, 0x42, 0x24, 0x18, 0x00},
  {0x00, 0x08, 0x24, 0x42, 0x42, 0x24, 0x18, 0x00},
  {0x00, 0x18, 0x04, 0x42, 0x42, 0x24, 0x18, 0x00},
  {0x00, 0x18, 0x24, 0x02, 0x42, 0x24, 0x18, 0x00},
  {0x00, 0x18, 0x24, 0x42, 0x02, 0x24, 0x18, 0x00},
  {0x00, 0x18, 0x24, 0x42, 0x42, 0x04, 0x18, 0x00},
  {0x00, 0x18, 0x24, 0x42, 0x42, 0x24, 0x08, 0x00}
};

int square(int x,int y,int z){
  sq=x^2+y^2+z^2;
  return sq;
}
