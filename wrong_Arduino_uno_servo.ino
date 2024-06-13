#include <Servo.h> //라이브러리 추가

//pin
#define SERVOPIN 7
#define INFRA_SENSOR_L_PIN A0 
#define INFRA_SENSOR_R_PIN A1
#define delay_servo 20

#define RIGHT_MAX_ANGLE 140 //서보 오른쪽 최대값
#define LEFT_MAX_ANGLE 40 // 서보 왼쪽 최대값

Servo motor_servo; // Servo 클래스로 motor_servo 객체 생성


void init_ADC(){
  //Voltage reference 설정 Vcc로
  ADMUX |= (0<<REFS1) | (1<<REFS0);

  //ADC데이터 오른쪽 정렬
  ADMUX |= (0<<ADLAR);

  //아날로그 채널 선택(A0핀의 경우)
  //ADMUX |= (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (0<<MUX0);

  //아날로그 채널 선택(A1핀의 경우)
  //ADMUX |= (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (1<<MUX0);

  //ADC Enable 역할
  ADCSRA |= (1<<ADEN);

  //Prescaler을 128로 설정
  ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

}


int readADC_L() {
  //left와 right를 구분하기 위해서 이 구문 꼭 넣어줘야함
  //ADMUX의 나머지 비트는 그대로 두고 마지막 비트를 0으로 초기화 해주는 구문
  ADMUX &= ~(1<<MUX0);
  //아날로그 채널 선택(A0핀의 경우)
  ADMUX |= (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (0<<MUX0);

  // ADC 변환 시작
  ADCSRA |= (1 << ADSC);
  
  // 변환이 끝날 때 까지 확인하는 while반복문
  while (ADCSRA & (1 << ADSC));
  
  // 변환 결과 반환
  return ADC;
}

int readADC_R() {
  //left와 right를 구분하기 위해서 이 구문 꼭 넣어줘야함
  //ADMUX의 나머지 비트는 그대로 두고 마지막 비트를 0으로 초기화 해주는 구문
  ADMUX &= ~(1<<MUX0);
  //아날로그 채널 선택(A1핀의 경우)
  ADMUX |= (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (1<<MUX0);

  // ADC 변환 시작
  ADCSRA |= (1 << ADSC);
  
  // 변환이 끝날 때 까지 확인하는 while반복문
  while (ADCSRA & (1 << ADSC));
  
  // 변환 결과 반환
  return ADC;
}


int comparator(int sensor_left, int sensor_right){
    if(sensor_left > sensor_right){
        if(sensor_left > 400){
            return LEFT_MAX_ANGLE;//왼쪽
        }
        else{
          return (RIGHT_MAX_ANGLE + LEFT_MAX_ANGLE)/2;//직진
        }
    }
    else if(sensor_left < sensor_right){
        if(sensor_right > 400){
            return RIGHT_MAX_ANGLE;//오른쪽
        }
        else{
          return (RIGHT_MAX_ANGLE + LEFT_MAX_ANGLE)/2;//직진
        }
    }
     else{
        return (RIGHT_MAX_ANGLE + LEFT_MAX_ANGLE)/2;//직진
    } 
}

//처음에 서보 모터 위치 90도로 맞추는 함수
void servo_init(){
  for(int servo_angle=40; servo_angle<=140; servo_angle += 2){
    motor_servo.write(servo_angle);
    delay(delay_servo); 
  }
  for(int servo_angle=140; servo_angle>=90; servo_angle -= 2){
    motor_servo.write(servo_angle);
    delay(delay_servo); 
  }
}

void servo_left(int angle){
  for(int servo_angle=90; servo_angle>=angle; servo_angle -= 2){
    motor_servo.write(servo_angle);
    delay(delay_servo); 
  }
  
  for(int servo_angle=angle; servo_angle<=90; servo_angle += 2){
    motor_servo.write(servo_angle);
    delay(delay_servo); 
  }
}

void servo_right(int angle){
  for(int servo_angle=90; servo_angle<=angle; servo_angle += 2){
    motor_servo.write(servo_angle);
    delay(delay_servo); 
  }
  
  for(int servo_angle=angle; servo_angle>=90; servo_angle -= 2){
    motor_servo.write(servo_angle);
    delay(delay_servo); 
  }
}

void setup() {
  pinMode(SERVOPIN, OUTPUT); 
  motor_servo.attach(SERVOPIN); 
  Serial.begin(9600);
  pinMode(INFRA_SENSOR_L_PIN, INPUT); 
  pinMode(INFRA_SENSOR_R_PIN, INPUT); 
  servo_init();
  init_ADC();
    
}

void loop(){
  int main_angle;
  int infra_sensor_value_L = readADC_L();
  //센서 값 잘 받아지는지 확인하는 용도
  //Serial.print("left:");
  //Serial.println(infra_sensor_value_L);
  //delay(1000);
  int infra_sensor_value_R = readADC_R();
  //센서 값 잘 받아지는지 확인하는 용도
  //Serial.print("right:");
  //Serial.println(infra_sensor_value_R);
  //delay(1000);

  main_angle = comparator(infra_sensor_value_L,infra_sensor_value_R);

  if(main_angle == LEFT_MAX_ANGLE){
    servo_left(main_angle);
  }
  else if(main_angle == RIGHT_MAX_ANGLE){
    servo_right(main_angle);
  }
    
}
