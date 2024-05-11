//라이브러리와 내장함수를 사용하는 경우
//디지털 핀을 사용하는 경우
//서보모터용으로 개별적인 아두이노 우노를 사용하는 경우
//회전 방향을 탐지한 경우에는 최대 회전(왼쪽 최대 or 오른쪽 최대)으로 회전하도록 하는 코드

#include <Servo.h> //라이브러리 추가

Servo motor_servo; // Servo 클래스로 motor_servo 객체 생성
int servoPin = 7; // 모터 제어핀을  디지털 7번 핀으로 설정(PWM)
// #define servoPin 7 // 모터 제어핀을  디지털 7번 핀으로 설정(PWM)을 define을 통해 코딩
int servo_angle = 90; //각도 조절 변수, 초기값을 직진 값으로 설정

int infra_sensorPin_L = 2; //왼쪽 적외선 센서의 신호핀을 2번 핀에 연결
// #define infra_sensorPin_L 2 //왼쪽 적외선 센서의 신호핀을 2번 핀에 연결을 define을 통해 코딩
int infra_sensorPin_R = 3; //오른쪽 적외선 센서의 신호핀을 3번 핀에 연결
// #define infra_sensorPin_R 3 //오른쪽 적외선 센서의 신호핀을 3번 핀에 연결을 define을 통해 코딩
int infra_sensor_value_L = 0; //왼쪽 적외선 센서로 부터 얻은 아날로그 값에 대한 초기화
int infra_sensor_value_R = 0; //오른쪽 적외선 센서로 부터 얻은 아날로그 값에 대한 초기화


void setup() {
  pinMode(servoPin, OUTPUT); //servoPin을 출력으로 설정
  motor_servo.attach(7); //서보 모터의 신호선을 디지털 7번핀에 연결
  //Serial.begin(9600);
  pinMode(infra_sensorPin_L, INPUT); //왼쪽 적외선 센서의 신호핀을 입력으로 설정
  pinMode(infra_sensorPin_R, INPUT); //오른쪽 적외선 센서의 신호핀을 입력으로 설정
  motor_servo.write(servo_angle); //처음 동작할 때 직진으로 서보모터를 설정
}

void loop() {
  infra_sensor_value_L = digitalRead(infra_sensorPin_L); //왼쪽 적외선 센서로 부터 읽은 디지털 값을 변수에 저장
  infra_sensor_value_R = digitalRead(infra_sensorPin_R); //오른쪽 적외선 센서로 부터 읽은 디지털 값을 변수에 저장
  
  servo_angle = comparator(infra_sensor_value_L,infra_sensor_value_R); //comparator 함수를 통해 서보모터 회전 방향에 대한 정보를 servo_angle 변수에 저장
  motor_servo.write(servo_angle); //servo_angle 값을 통해 서보모터 회전
}

//매개변수로 전달된 양쪽 센서의 값에 대한 비교를 통해 라인의 색을 분석하고 이를 통해 회전 방향을 결정하는 함수
int comparator(int sensor_left, int sensor_right){
  if(sensor_left == 1){
    if(sensor_right != 1){
      return 0;
    }
    else{
      return 90;
    }
  }
  else if(sensor_right == 1){
    if(sensor_left != 1){
      return 180;
    }
    else{
      return 90;
    }
  }
  else{
    return 90;
  }
}  
