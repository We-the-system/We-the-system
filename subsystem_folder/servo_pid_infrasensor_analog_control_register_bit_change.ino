//레지스터와 비트변환을 통해 적외선 센서값 받아오는 코드
//아날로그 핀을 사용하는 경우
//서보모터용으로 개별적인 아두이노 우노를 사용하는 경우


#include <Servo.h> //라이브러리 추가

//pin
#define SERVOPIN 7
#define INFRA_SENSOR_L_PIN A0 
#define INFRA_SENSOR_R_PIN A1

#define RIGHT_MAX_ANGLE 150 //서보 오른쪽 최대값
#define LEFT_MAX_ANGLE 30 // 서보 왼쪽 최대값

Servo motor_servo; // Servo 클래스로 motor_servo 객체 생성
int first_angle = 90; //각도 조절 변수, 초기값을 직진 값으로 설정
//PID variable
double integral = 0.0;
double pre_error = 0.0;
double deriavative = 0.0;
double proportional =0.0;
long prevT = 0;
char present_position;

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

//int 형식으로도 써도 되는지 확인하기
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
        if(sensor_left > 500){
            return LEFT_MAX_ANGLE;//왼쪽
        }
    }
    else if(sensor_left < sensor_right){
        if(sensor_right > 500){
            return RIGHT_MAX_ANGLE;//오른쪽
        }
    }
     else{
        return (RIGHT_MAX_ANGLE + LEFT_MAX_ANGLE)/2;//직진
    } 
}


void setup() {
    pinMode(SERVOPIN, OUTPUT); 
    motor_servo.attach(SERVOPIN); 
    Serial.begin(9600);
    pinMode(INFRA_SENSOR_L_PIN, INPUT); 
    pinMode(INFRA_SENSOR_R_PIN, INPUT); 
    motor_servo.write(first_angle); 
    init_ADC();
    present_position = 'C';
}

void loop() {
    int servo_angle;
    double error = 0; //이거 뺄지 말지 정하기 누적되면 빼야함
    float kp=0.3;
    float ki=0.15;
    float kd=0.05;

    long currT = millis();

    //만약 차량이 움직이기 시작하면 (가속도 센서 값 여기에 붙이면 좋을 듯)
    int infra_sensor_value_L = readADC_L();
    Serial.print("left:");
    Serial.println(infra_sensor_value_L);
    delay(1000);
    int infra_sensor_value_R = readADC_R();
    Serial.print("right:");
    Serial.println(infra_sensor_value_R);
    delay(1000);
    int aver_sensor_value = (infra_sensor_value_L + infra_sensor_value_R)/2;

    servo_angle = comparator(infra_sensor_value_L,infra_sensor_value_R);
    
    if (aver_sensor_value>400){//완전히 선이 정중앙에 있을 때 측정값 넣어서 바꾸기(320->400)
        error = aver_sensor_value - 400;//(320->400)
        double dt = (double)(currT - prevT)/1000.0;
        proportional = error;
        integral = integral + error*dt;
        double derivative = (error-pre_error)/dt;
        int output = (int)((kp*proportional) + (ki*integral)+(kd*derivative));//아래 자리 버리기(int)
        if (servo_angle == LEFT_MAX_ANGLE)
            //motor_servo.write(servo_angle+output);//실험해서 바꾸기
            if(present_position == 'R'){
              present_position = RtoL(servo_angle+output);
            }
            else{
              present_position = CtoL(servo_angle+output);
            }
        else if (servo_angle == RIGHT_MAX_ANGLE)
            //motor_servo.write(servo_angle-output);//실험해서 바꾸기
            if(present_position == 'L'){
              present_position = LtoR(servo_angle-output);
            }
            else{
              present_position = CtoL(servo_angle-output);
            }

        else
            //motor_servo.write(servo_angle);
            if(present_position == 'L'){
              present_position = LtoC(servo_angle);
            }
            else if(present_position == 'R'){
              present_position = RtoC(servo_angle);
            }
            else{
              present_position = CtoC(servo_angle);
            }
    }
    else{
        //motor_servo.write(servo_angle); //servo_angle 값을 통해 서보모터 회전
        if(present_position == 'L'){
              present_position = LtoC(servo_angle);
            }
            else if(present_position == 'R'){
              present_position = RtoC(servo_angle);
            }
            else{
              present_position = CtoC(servo_angle);
            }

    }
    //for next loop
    pre_error = error;
    prevT = currT;
}

//모터 조절 함수 생성(큰 값을 파라미터로 했을 때 모터가 한 번에 안 돌아가므로 다음과 같이 해줘야함)
char CtoR(int angle){
  for(int servo_angle=90; servo_angle<=angle; servo_angle += 2){
    motor_servo.write(servo_angle);
    delay(70); //0.5초간 대기
  }
  return 'R';
}

char LtoR(int angle){
  for(int servo_angle=30; servo_angle<=angle; servo_angle += 2){
    motor_servo.write(servo_angle);
    delay(70); //0.5초간 대기
  }
  return 'R';
}

char CtoL(int angle){
  for(int servo_angle=90; servo_angle>=angle; servo_angle -= 2){
    motor_servo.write(servo_angle);
    delay(70); //0.5초간 대기
  }
  return 'L';
}

char RtoL(int angle){
  for(int servo_angle=150; servo_angle>=angle; servo_angle -= 2){
    motor_servo.write(servo_angle);
    delay(70); //0.5초간 대기
  }
  return 'L';
}

char RtoC(int angle){
  for(int servo_angle=150; servo_angle>=angle; servo_angle -= 2){
    motor_servo.write(servo_angle);
    delay(70); //0.5초간 대기
  }
  return 'C';
}

char LtoC(int angle){
  for(int servo_angle=30; servo_angle<=angle; servo_angle += 2){
    motor_servo.write(servo_angle);
    delay(70); //0.5초간 대기
  }
  return 'C';
}

char CtoC(int angle){
  motor_servo.write(angle);
  return 'C';
}
