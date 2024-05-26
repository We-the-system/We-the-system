//차량에 내장된 서보모터의 회전 각도를 시험하기 위한 코드
//라이브러리와 내장 함수를 사용하는 경우

#include <Servo.h> //라이브러리 추가

Servo myservo; // Servo 클래스로 motor_servo 객체 생성
int servoPin = 7; // 모터 제어핀을  디지털 7번 핀으로 설정(PWM)
// #define servoPin 7 // 모터 제어핀을  디지털 7번 핀으로 설정(PWM)을 define을 통해 코딩
int first_angle = 90; //각도 조절 변수, 초기값을 직진 값으로 설정->이런식으로 전역변수 선언하면 루프에서 영향 갈듯


void setup() {
  pinMode(servoPin, OUTPUT); //servoPin을 출력으로 설정
  myservo.attach(servoPin); //서보 모터의 신호선을 디지털 7번핀에 연결
  myservo.write(first_angle); //처음 동작할 때 직진으로 서보모터를 설정
}

void loop() {
  //직진상태에서부터 10씩 증가하며 오른쪽 최대각도까지 확인
  int servo_angle;
  for(servo_angle=90; servo_angle<=150; servo_angle += 2){
    myservo.write(servo_angle);
    delay(70); //0.5초간 대기
  }
  delay(2500);
  //직진상태에서부터 10씩 감소하며 왼쪽 최대각도까지 확인
  for(servo_angle=150; servo_angle>=90; servo_angle -= 2){
    myservo.write(servo_angle);
    delay(70); //0.5초간 대기
  }
  delay(2500);
}
