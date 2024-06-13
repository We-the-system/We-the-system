#define relay1 PB0 //digital pin8
#define relay2 PB1 //digital pin9

#define NCS PB2 //digital pin10
#define SDA PB3 //digital pin11
#define ADD PB4 //digital pin12
#define SCL PB5 //digital pin13

unsigned long startMillis;
unsigned long currentMillis;

#define ACCEL_CONFIG      0x1C
#define GYRO_CONFIG       0x1B
#define USER_CTRL         0x6A
#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40
#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

uint8_t acc_full_scale = 0x01; // Set accel scale (+-2g: 0x00, +-4g: 0x01, +-8g: 0x02, +- 16g: 0x03)
uint8_t gyro_full_scale = 0x02; // Set gyro scale (00 = +250dps, 01= +500 dps, 10 = +1000 dps, 11 = +2000 dps )
int crash = 40;
int flip = 20000;
float   accel_scale, gyro_scale;
float   accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
const float mpu_dt = 0.01; //time vary 10ms
float velocity_x = 0.0;
float velocity_y = 0.0;
float velocity_z = 0.0;
float displacement_x = 0.0;
float displacement_y = 0.0;
float displacement_z = 0.0;
float angle_x = 0.0;
float angle_y = 0.0;
float angle_z = 0.0;
char    accel_x_str[10], accel_y_str[10], accel_z_str[10], gyro_x_str[10], gyro_y_str[10], gyro_z_str[10];
uint8_t rawData_accel[6], rawData_gyro[6];
int16_t accelCount[3], gyroCount[3];
//char    message[255] = {0,}; //bluetooth

float filter_x=0;
float filter_y=0;
float filter_z=0;

void relay_channel_on() 
{
   //digitalWrite(relay1, HIGH);
   //digitalWrite(relay2, HIGH);
  // PORTB |= (1<<relay1)|(1<<relay2);
  PORTB &= ~(1<<relay1);
  PORTB |= (1<<relay2); //동시진행 안한 이유 회로의 안전성 때문
}

void relay_channel_off() 
{
   //digitalWrite(relay1, LOW);
   //digitalWrite(relay2, LOW);
  //PORTB &= ~((1<<relay1)|(1<<relay2))
  PORTB |= (1<<relay1);
  PORTB &= ~((1<<relay2)); //동시진행 안한 이유 회로의 안전성 때문
}

uint8_t transfer_SPI(uint8_t registerAddress, uint8_t data, bool isRead) {
    uint8_t response = 0x00;
    // Set MSB = 1 for read
    registerAddress |= (isRead ? 0x80 : 0x00);    
    
    // SS: low (active)
    PORTB &= ~(1 << NCS);      
    
    // Register Address transfer
    SPDR = registerAddress;
    while (!(SPSR & (1 << SPIF)));    
    // Data transfer
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    response = SPDR;
    
    // SS: high (inactive)
    PORTB |= (1 << PB2);  
    return response;
}

void setup_scale(uint8_t scale_a, uint8_t scale_g) {
  uint8_t current_config_accel = transfer_SPI(ACCEL_CONFIG, 0x00, true);
  uint8_t current_config_gyro = transfer_SPI(GYRO_CONFIG, 0x00, true);
  
  current_config_accel &= ~0x18; // Set 00 to ACCEL_FS_SEL[1:0]
  current_config_gyro &= ~0x18;

  current_config_accel |= (scale_a << 3); // Set ACCEL_FS_SEL to scale
  current_config_gyro |= (scale_g << 3); // Set ACCEL_FS_SEL to scale


  transfer_SPI(ACCEL_CONFIG, current_config_accel, false); // Write accel config

  // Set resolution
  switch (scale_a) {
    case 0x00: // +- 2g
      accel_scale = 2.0f / 32768.0f;
      break;
    case 0x01: // +- 4g
      accel_scale = 4.0f / 32768.0f;
      break;
    case 0x02: // +- 8g
      accel_scale = 8.0f / 32768.0f;
      break;
    case 0x03: // +- 16g
      accel_scale = 16.0f / 32768.0f;
      break;
  }

  accel_scale = accel_scale * 9.80665f; //g to m/s^2

  transfer_SPI(GYRO_CONFIG, current_config_gyro, false); // Write accel config

    // Set resolution
  switch (scale_g) {
    case 0x00: // +- 250 deg
      gyro_scale = 250.0f / 32768.0f;
      break;
    case 0x01: // +- 500 deg
      gyro_scale = 500.0f / 32768.0f;
      break;
    case 0x02: // +- 1000 deg
      gyro_scale = 1000.0f / 32768.0f;
      break;
    case 0x03: // +- 16g
      gyro_scale = 2000.0f / 32768.0f;
      break;
  }

}

void read_AccelData() {
  rawData_accel[0] = transfer_SPI(ACCEL_XOUT_H, 0x00, true);
  rawData_accel[1] = transfer_SPI(ACCEL_XOUT_L, 0x00, true);
  rawData_accel[2] = transfer_SPI(ACCEL_YOUT_H, 0x00, true);
  rawData_accel[3] = transfer_SPI(ACCEL_YOUT_L, 0x00, true);
  rawData_accel[4] = transfer_SPI(ACCEL_ZOUT_H, 0x00, true);
  rawData_accel[5] = transfer_SPI(ACCEL_ZOUT_L, 0x00, true);
  
  accelCount[0] = (rawData_accel[0] << 8) | rawData_accel[1];
  accelCount[1] = (rawData_accel[2] << 8) | rawData_accel[3];
  accelCount[2] = (rawData_accel[4] << 8) | rawData_accel[5];

  accel_x = accelCount[0] * accel_scale -0.18;
  accel_y = accelCount[1] * accel_scale -0.52;
  accel_z = accelCount[2] * accel_scale ;

  if (accel_x*accel_x +accel_y*accel_y > crash){
    relay_channel_on();
    infinite_loop();
  }

}

void read_GyroData() {
  rawData_gyro[0] = transfer_SPI(GYRO_XOUT_H, 0x00, true);
  rawData_gyro[1] = transfer_SPI(GYRO_XOUT_L, 0x00, true);
  rawData_gyro[2] = transfer_SPI(GYRO_YOUT_H, 0x00, true);
  rawData_gyro[3] = transfer_SPI(GYRO_YOUT_L, 0x00, true);
  rawData_gyro[4] = transfer_SPI(GYRO_ZOUT_H, 0x00, true);
  rawData_gyro[5] = transfer_SPI(GYRO_ZOUT_L, 0x00, true);

  gyroCount[0] = (rawData_gyro[0] << 8) | rawData_gyro[1];
  gyroCount[1] = (rawData_gyro[2] << 8) | rawData_gyro[3];
  gyroCount[2] = (rawData_gyro[4] << 8) | rawData_gyro[5];

  gyro_x = gyroCount[0] * gyro_scale + 3.94;
  gyro_y = gyroCount[1] * gyro_scale + 0.1;
  gyro_z = gyroCount[2] * gyro_scale + 0.5;

  if (gyro_x*gyro_x + gyro_y*gyro_y > flip){
    relay_channel_on();
    infinite_loop();
  }
}

void calculateDisplacement() {
  // 속도 적분
  velocity_x += filter_x * mpu_dt;
  velocity_y += filter_y * mpu_dt;
  velocity_z += filter_z * mpu_dt;


  // 변위 적분
  displacement_x += velocity_x * mpu_dt;
  displacement_y += velocity_y * mpu_dt;
  displacement_z += velocity_z * mpu_dt;
}

void calculateDirection() {
  angle_x += gyro_x * mpu_dt;
  angle_y += gyro_y * mpu_dt;
  angle_z += gyro_z * mpu_dt;
  
}

void filter_temp(){
  filter_x=accel_x;
  filter_y=accel_y;
  filter_z=accel_z;
}

void filter(){
  float alpha = 0.7;
  filter_x = alpha * accel_x + (1 - alpha) * filter_x;
  filter_y = alpha * accel_y + (1 - alpha) * filter_y;
  filter_z = alpha * accel_z + (1 - alpha) * filter_z;

  if (filter_x < 0.15)
    filter_x = 0;
  if (filter_y < 0.15)
    filter_y = 0;
  if (filter_z < 0.15)
    filter_z = 0;
}

void infinite_loop() {
  //when crash or flip occurs
  while (true) {
    Serial.println("Entering infinite loop...");
    delay(1000);
  }
}
 
void setup(){
  Serial.begin(9600);

  DDRB |=  (1 << NCS) | (1 << SDA) | (1 << SCL); //pin number above when defiend
  DDRB &= ~(1 << ADD); //NCS DISABLE for SPI communication
  SPCR |=   (1 << SPE)  | (1 << MSTR) | (1 << SPR0); //enable spi, master, SPR[1:0] prescaler 16 1mhz
  SPCR &= ~((1 << DORD) | (1 << CPOL) | (1 << CPHA) | (1 << SPR1)); //DORD 0 MSB first, CPOL, CPHA 1,1 > SPI Mode0, gives data at falling edge
  PORTB |= (1 << NCS);
  setup_scale(acc_full_scale, gyro_full_scale);
  //disable i2c
  uint8_t current_setting = transfer_SPI(USER_CTRL, 0x00, true); // Read USER_CTRL
  current_setting |= 0x10;   // Set I2C_IF_DIS to one (I2C_IF_DIS: bit 4 of USER_CTRL register)
  transfer_SPI(USER_CTRL, current_setting, false);   // Write USER_CTRL

  //pinMode(relay1, OUTPUT);
  //pinMode(relay2, OUTPUT);
  DDRB |= (1<<relay1)|(1<<relay2);
  
  startMillis = millis(); 
}

void loop(){

  read_AccelData();
  read_GyroData();

  filter_temp();
  filter();
  

  calculateDisplacement();
  calculateDirection();
  
  //Serial.print("accel_x = "); Serial.print(accel_x); Serial.print(", accel_y = "); Serial.print(accel_y); Serial.print(", accel_z = "); Serial.println(accel_z);
  Serial.print("filter_x = "); Serial.print(filter_x); Serial.print(", filter_y = "); Serial.print(filter_y); Serial.print(", filter_z = "); Serial.println(filter_z);
    
  //Serial.print("velocity_x = "); Serial.print(velocity_x); Serial.print(", velocity_y = "); Serial.print(velocity_y); Serial.print(", velocity_z = "); Serial.println(velocity_z);
  //Serial.print("displacement_x = "); Serial.print(displacement_x); Serial.print(", displacement_y = "); Serial.print(displacement_y); Serial.print(", displacement_z = "); Serial.println(displacement_z);
  
  //Serial.print("gyro_x = "); Serial.print(gyro_x); Serial.print(", gyro_y = "); Serial.print(gyro_y); Serial.print(", gyro_z = "); Serial.println(gyro_z);
  //Serial.print("angle_x = "); Serial.print(angle_x); Serial.print(", angle_y = "); Serial.print(angle_y); Serial.print(", angle_z = "); Serial.println(angle_z);
  currentMillis=millis();

  if (currentMillis - startMillis >= 5000 && currentMillis - startMillis < 8000){
    relay_channel_on();
  } else if (currentMillis - startMillis >= 8000){
    relay_channel_off();
    startMillis = millis();
  }
}
