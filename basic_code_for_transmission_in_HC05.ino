//송신용 (HC-05) 기본코드 구조

#include <SoftwareSerial.h>

SoftwareSerial HC05_t(2, 3); // 아두이노 보드 기준(RX, TX), 블루투스 모듈 기준(TXD, RXD)으로 핀을 2번 3번으로 설정

void setup() {
  Serial.begin(9600); //PC-아두이노간 통신 baud rate 설정
  HC05_t.begin(38400); //아두이노-블루투스모듈간 통신 baud rate 설정
}

void loop() {
  HC05_t.print("information"); // 통신으로 보내고자 하는 정보를 입력
  delay(1000); // 딜레이를 1000ms로 설정 즉, 1초마다 블루투스 모듈을 통해 정보 송신
  //HC05 블루투스 모듈의 경우 carriage return을 붙여서 정보를 전송

}