#include <Encoder.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x04

int data_to_send = 0b1011101001; 
byte data[32] = {0};

double kp = 1.404;
double ki = .4404;
double i = 0;
double e = 0;
double e_past = 0;
double ts = 0;
double tc = millis() / 1000;
double r = 0;
double y = 0;
double theta = 0;
double count = 0;
double u = 0;
double currentTime = 0;
double fullroto = 3200;
int pwm = 0;
int dir = 0;
int zero = 0;
const int umax = 7;
int command;

const int input_A = 2;
const int input_B = 5;
const int camera = 12;
const int M1_speed = 9;
const int M1_dir = 7;
const int m_enable = 4;
const int topi = 13;


Encoder wheel(input_A, input_B);

void setup() {
  pinMode(topi, OUTPUT);
  pinMode(camera, INPUT);
  pinMode(M1_speed, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(m_enable, OUTPUT);
  digitalWrite(m_enable, HIGH);  //set enable to high
  Serial.begin(115200);          //set buad rate
  zero = wheel.read();

  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendBlock);
}

void loop() {
  switch (command) {
    case 0:
      r = 0;
      break;

    case 1:
      r = PI / 2;
      break;

    case 2:
      r = PI;
      break;

    case 3:
      r = 3 * PI / 2;
      break;
  }
  spin(0, 1);
  count = (wheel.read() - zero) % 3200;
  r = command;
  y = (-count / fullroto * 2 * PI);
  e = r - y;
  i = i + ts * e;
  u = kp * e + ki * i;

//  data[0] = y >> 56;
//  data[1] = y >> 48;
//  data[2] = y >> 40;
//  data[3] = y >> 32;
//  data[4] = y >> 24;
//  data[5] = y >> 16;
//  data[6] = y >> 8;
//  data[7] = y & 0xFF;

  if (abs(u) > umax) {
    u = signbit(u) * umax;
    e = signbit(e) * min(umax / kp, abs(e));
    i = (u - kp * e) / ki;
  }

  if (u < 0) {
    dir = 0;
  } else {
    dir = 1;
  }

  pwm = abs((u / 7) * 256);

  spin(pwm, dir);
  Serial.print(e);
  Serial.print("\t");
  Serial.print(u);
  Serial.print("\t");
  Serial.print(pwm);
  Serial.print("\n");
  ts = (millis() / 1000) - tc;
  tc = millis() / 1000;
}

void spin(int PWM, int dir) {
  analogWrite(M1_speed, PWM);  //set duty to half pin 9
  digitalWrite(M1_dir, dir);
}

// callback for received data
void receiveData(int byteCount){

Serial.print("data received: ");
while(Wire.available()) {
  command = Wire.read();
  Serial.println(command);
  }
}



// callback for sending data
void sendBlock(){
  Serial.print("sending data: ");
//  Serial.println(data[0]);
//  Serial.println(data[1]);
  Wire.write(data, 8); // 8 is number of bytes in data
}
