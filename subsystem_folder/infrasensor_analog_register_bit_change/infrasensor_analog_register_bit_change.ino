//레지스터와 비트변환을 통해 적외선 센서값 받아오는 코드


int infra_sensor_value_L = 0; //왼쪽 적외선 센서로 부터 얻은 아날로그 값에 대한 초기화
int infra_sensor_value_R = 0; //오른쪽 적외선 센서로 부터 얻은 아날로그 값에 대한 초기화

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
  //아날로그 채널 선택(A1핀의 경우)
  //ADMUX |= (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (1<<MUX0);

  // ADC 변환 시작
  ADCSRA |= (1 << ADSC);
  
  // 변환이 끝날 때 까지 확인하는 while반복문
  while (ADCSRA & (1 << ADSC));
  
  // 변환 결과 반환
  return ADC;
}

void setup() {
  init_ADC();

}

void loop() {
  infra_sensor_value_L=readADC_L(); //왼쪽 적외선 센서로 부터 읽은 아날로그 값을 변수에 저장
  infra_sensor_value_R=readADC_R(); //오른쪽 적외선 센서로 부터 읽은 아날로그 값을 변수에 저장


}
