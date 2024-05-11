//수신용 (HC-05) 기본코드 구조

#include <SoftwareSerial.h>


SoftwareSerial HC05_r(2, 3); // 아두이노 보드 기준(RX, TX), 블루투스 모듈 기준(TXD, RXD)으로 핀을 2번 3번으로 설정

void setup() {
  Serial.begin(9600); //PC-아두이노간 통신 baud rate 설정
  HC05_r.begin(38400); //아두이노-블루투스모듈간 통신 baud rate 설정
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
}
