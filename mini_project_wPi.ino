#include <Encoder.h>
#include <Wire.h>

//used for I2C
#define SLAVE_ADDRESS 0x04
byte data[2] = { 0 };

//variable declarations
double kp = 4;  //1.404
double ki = 0;  //.4404
double i = 0;
double e = 0;
double e_past = 0;
double ts = 0;
double tc = 0;
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
const int umax = 8;
int command;
int temp_command;

//variables for encoder
const int input_A = 2;
const int input_B = 5;
const int camera = 12;
const int M1_speed = 9;
const int M1_dir = 7;
const int m_enable = 4;
const int topi = 13;
Encoder wheel(input_A, input_B);

void setup() {
  //setting desired pins to input or output, setting baud rate
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
  //case to determine what value pi is relaying
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

//calculating radians position based off wheel count and values for controller
  count = (wheel.read() - zero) % 3200;
  y = ((-count / fullroto) * 2 * PI);
  e = r - y;
  i = i + ts * e;
  u = kp * e + ki * i;

//transfering wheel counts to pi
  data[0] = int(count) >> 8;
  data[1] = int(count) & 0xFF;

//determining if wheel should spin c or cc
  if (u < 0) {
    dir = 1;
  } else {
    dir = 0;
  }

//converts voltage value to pwm value
  pwm = abs((u / 8) * 255);
  Serial.println(pwm);

  //limiter for if PWM value is greater than 255
  if (pwm > 255) {
    pwm = 255;
  }

//writing direction 
  analogWrite(M1_speed, pwm);
  digitalWrite(M1_dir, dir);

//recalculating
  ts = (millis() / 1000) - tc;
  tc = millis() / 1000;
  delay(50);
}

void spin(int PWM, int dir) {
  analogWrite(M1_speed, PWM);  //set duty to half pin 9
  digitalWrite(M1_dir, dir);
}

// callback for received data
void receiveData(int byteCount) {

  while (Wire.available()) {
    temp_command = Wire.read();
    if (temp_command != 5) {
      command = temp_command;
    }
    Serial.println(r);
    Serial.println(command);
  }
}



// callback for sending data
void sendBlock() {
  Wire.write(data, 2);  // 2 is number of bytes in data
}
