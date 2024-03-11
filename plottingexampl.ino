const int sensorPin1 = A0;  // 첫 번째 센서에 연결된 핀 번호
const int sensorPin2 = A1;  // 두 번째 센서에 연결된 핀 번호
int sensorValue1 = 0;      // 첫 번째 센서에서 읽은 값
int sensorValue2 = 0;      // 두 번째 센서에서 읽은 값

void setup() {
  Serial.begin(9600);      // 시리얼 통신 시작
}

void loop() {
  sensorValue1 = analogRead(sensorPin1);  // 첫 번째 센서에서 값을 읽음
  sensorValue2 = analogRead(sensorPin2);  // 두 번째 센서에서 값을 읽음

  // 변수들을 쉼표로 구분하여 시리얼 통신으로 전송
  Serial.print(sensorValue1);
  Serial.print(" 여기는 무시돼요 "); // 텍스트 추가
  Serial.print(sensorValue2);
  Serial.print(" 이것도 무시돼요 ");
  Serial.println();

  delay(1000);  // 1초 대기
}
