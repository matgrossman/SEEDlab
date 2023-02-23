//mini demo
#include <Encoder.h>

const int input_A = 2;
const int input_B = 5;
int State_A = 0;
int State_B = digitalRead(input_B);
double count = 0;
int pwm = 150;
int dir = 1;

const int M1_speed = 9, M1_dir = 7;
const int m_enable = 4;

const int camera = 12;  //input pin for camera info
double theta = 0;
double thetaPrev = 0;
double fullroto = 3210;
int currentTime = 0;
int timePrev = 0;
double velo = 0;


Encoder wheel(input_A, input_B);

void setup() {
  // put your setup code here, to run once:
  pinMode(input_A, INPUT_PULLUP);
  pinMode(input_B, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(input_A), rotate, CHANGE);
  digitalWrite(input_A, HIGH);
  digitalWrite(input_B, HIGH);
  Serial.begin(9600);  //set buad rate

  pinMode(M1_speed, OUTPUT);  //pins 9,7,10,8 to output
  pinMode(M1_dir, OUTPUT);

  pinMode(m_enable, OUTPUT);
  digitalWrite(m_enable, HIGH);  //set enable to high

  pinMode(camera, INPUT);  //setting pin 12 to input
}

void loop() {  //need to get the motor to stop and be able to self correct itself*************
  currentTime = millis() + 1;
  count = wheel.read();                                                              //gives wheel count
  theta = (-count / fullroto * 2 * PI);                                              //check sign
  velo = (theta - thetaPrev) / (((double(currentTime) - double(timePrev)) / 1000));  //calc velo

  if (currentTime >= 1000 && currentTime < 2000) {
    spin(pwm, dir);
    Serial.print(theta);
    Serial.print("\t");
    Serial.print(pwm);
    Serial.print("\t");
    Serial.print(currentTime);
    Serial.print("\t");
    Serial.print(velo);
    Serial.print("\n");
  }

  thetaPrev = theta;
  timePrev = currentTime;
}

void spin(int PWM, int dir) {
  analogWrite(M1_speed, PWM);  //set duty to half pin 9
  digitalWrite(M1_dir, dir);
}
