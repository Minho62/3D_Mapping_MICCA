/*
 front
1     3
2     4
  back
*/
#define HALL1_A 15  //PCINT9
#define HOLL1_B 14  //PCINT10
#define IN1_1 6     //7
#define IN1_2 7     //6
#define PWM1 11
#define STBY1 42

#define HALL2_A 19  //INT2
#define HOLL2_B 18  //INT3
#define IN2_1 8     //9
#define IN2_2 9     //8
#define PWM2 13
#define STBY2 44

#define HALL3_A 21  //INT0
#define HOLL3_B 20  //INT1
#define IN3_1 5
#define IN3_2 4
#define PWM3 12
#define STBY3 46

#define HALL4_A 50  //PCINT3
#define HOLL4_B 53  //PCINT0
#define IN4_1 3
#define IN4_2 2
#define PWM4 10
#define STBY4 48

int new_speed1 = 0;
int new_speed2 = 0;
int new_speed3 = 0;
int new_speed4 = 0;

int max_speed = 255;
char bt_data = NULL;

byte gbpTxBuffer[128];
unsigned long auto_time;

enum PM  // Print Menu
{
  MAIN,
  MEASUREMENT,
  TUNNEL,
  CAVE,
  PROCESSING,
  SETTING,
  SETTING_ANGLE_MAX,
  SETTING_ANGLE_MIN,
  SELECT_USB,
};

void motor(int motor1_speed, int motor2_speed, int motor3_speed, int motor4_speed) {
  if (motor1_speed > 0) {
    digitalWrite(IN1_1, HIGH);
    digitalWrite(IN1_2, LOW);
    analogWrite(PWM1, motor1_speed);
  } else if (motor1_speed < 0) {
    new_speed1 = -motor1_speed;
    digitalWrite(IN1_1, LOW);
    digitalWrite(IN1_2, HIGH);
    analogWrite(PWM1, new_speed1);
  }
  if (motor2_speed > 0) {
    digitalWrite(IN2_1, HIGH);
    digitalWrite(IN2_2, LOW);
    analogWrite(PWM2, motor2_speed);
  } else if (motor2_speed < 0) {
    new_speed2 = -motor2_speed;
    digitalWrite(IN2_1, LOW);
    digitalWrite(IN2_2, HIGH);
    analogWrite(PWM2, new_speed2);
  }

  if (motor3_speed > 0) {
    digitalWrite(IN3_1, HIGH);
    digitalWrite(IN3_2, LOW);
    analogWrite(PWM3, motor3_speed);
  } else if (motor3_speed < 0) {
    new_speed3 = -motor3_speed;
    digitalWrite(IN3_1, LOW);
    digitalWrite(IN3_2, HIGH);
    analogWrite(PWM3, new_speed3);
  }
  if (motor4_speed > 0) {
    digitalWrite(IN4_1, HIGH);
    digitalWrite(IN4_2, LOW);
    analogWrite(PWM4, motor4_speed);
  } else if (motor4_speed < 0) {
    new_speed4 = -motor4_speed;
    digitalWrite(IN4_1, LOW);
    digitalWrite(IN4_2, HIGH);
    analogWrite(PWM4, new_speed4);
  }
}

void motor_stop() {
  analogWrite(PWM1, LOW);
  analogWrite(PWM2, LOW);
  analogWrite(PWM3, LOW);
  analogWrite(PWM4, LOW);
}

void send_data(Stream &serialport, byte buf_0, byte buf_1, byte buf_2) {
  gbpTxBuffer[0] = 0xff;
  gbpTxBuffer[1] = 0xff;
  gbpTxBuffer[2] = buf_0;
  gbpTxBuffer[3] = buf_1;
  gbpTxBuffer[4] = buf_2;

  char checksum = 0;
  for (int i = 2; i < 5; i++) {
    checksum += gbpTxBuffer[i];
  }
  gbpTxBuffer[5] = ~checksum;

  for (int i = 0; i < 6; i++) {
    serialport.write(gbpTxBuffer[i]);
    serialport.flush();
  }
}


void Packet() {
  if (Serial2.available()) {
    bt_data = Serial2.read();

    // Serial.print("bt_data : ");
    // Serial.println(bt_data);
  }

  if (bt_data == 'w') {  //직진
    motor(max_speed, max_speed, max_speed, max_speed);
  } else if (bt_data == 'q') {  //직좌회전
    motor(max_speed / 2, max_speed / 2, max_speed, max_speed);
  } else if (bt_data == 'e') {  //직우회전
    motor(max_speed, max_speed, max_speed / 2, max_speed / 2);
  } else if (bt_data == 's') {  //후진
    motor(-max_speed, -max_speed, -max_speed, -max_speed);
  } else if (bt_data == 'z') {  //후좌회전
    motor(-max_speed / 2, -max_speed / 2, -max_speed, -max_speed);
  } else if (bt_data == 'c') {  //후우회전
    motor(-max_speed, -max_speed, -max_speed / 2, -max_speed / 2);
  } else if (bt_data == 'a') {  //제자리 좌회전
    motor(-max_speed, -max_speed, max_speed, max_speed);
  } else if (bt_data == 'd') {  //제자리 우회전
    motor(max_speed, max_speed, -max_speed, -max_speed);
  } else if (bt_data == 'n') {  //정지
    motor_stop();
  }

  //Jetson Orin Nano
  else if (bt_data == '1') {  //터널 측정모드
    send_data(Serial, 0x02, 0x00, 0x02);
    auto_time = millis();
  }

  else if (bt_data == '2') {
    // SEND TUNNEL 측정 시작, PROCESSING
    send_data(Serial, 0x02, 0x00, 0x03);
  }

  else if (bt_data == '3') {  //CAVE 측정모드
    // SEND CAVE 측정 시작
    send_data(Serial, 0x02, 0x00, 0x05);
    auto_time = millis();
  }

  else if (bt_data == '4') {  //CAVE 측정시작
    //cave측정 시작
    send_data(Serial, 0x02, 0x00, 0x07);
  }

  else if (bt_data == '5') {  //CAVE 측정정지
    send_data(Serial, 0x02, 0x00, 0x04);
    // SEND CAVE 측정 중지
    send_data(Serial, 0x03, 0x00, 0x01);
    // SEND USB 연결 유지
  } else
    bt_data = NULL;
}

void motor_init() {
  pinMode(IN1_1, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(IN3_1, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_1, OUTPUT);
  pinMode(IN4_2, OUTPUT);

  pinMode(STBY1, OUTPUT);
  pinMode(STBY2, OUTPUT);
  pinMode(STBY3, OUTPUT);
  pinMode(STBY4, OUTPUT);

  digitalWrite(STBY1, HIGH);
  digitalWrite(STBY2, HIGH);
  digitalWrite(STBY3, HIGH);
  digitalWrite(STBY4, HIGH);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM2, OUTPUT);
}


void setup() {
  motor_init();
  Serial.begin(1000000);
  Serial2.begin(9600);
}

void loop() {

  Packet();
}
