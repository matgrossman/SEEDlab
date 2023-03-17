//included libraries
#include <Encoder.h>
#include <Wire.h>
//given angle and distance targets
double target_d = 120 * 0.9;
double target_a = 180;

//current angle and position
double current_dtot = 0;
double current_a = 0;

//variables for distance controller
double kp = 1.2;
double ki = 0;
double kp1 = 0;
double kp2 = 0;
double u = 0;
double u1 = 0;
double u2 = 0;
double e = 0;

//variables for angle controller
double kpa = 0.1;
double kia = 0;
double ua = 0;
double offset = 0;
double ts = 0;
double tc = 0;
double e_a = 0;

//variables for motor 1 power and encoder
const double mFudge = 0.97;
const int maxPWM = 225;
const int input_A1 = 3;
const int input_B1 = 6;
const int M1_dir = 7;
const int M1_speed = 9;
const int M1_enable = 4;
const int M1_count = 0;
int M1_zero = 0;
double current_d1 = 0;
Encoder wheel1(input_A1, input_B1);

//variables for motor 1 control
int dir1 = 0;
int pwm1 = 0;
double i = 0;
int pwmoffset = 0;

//variables for motor 2 power and encoder
const int input_A2 = 2;
const int input_B2 = 5;
const int M2_dir = 8;
const int M2_speed = 10;
const int M2_enable = 11;
const int M2_count = 0;
int M2_zero = 0;
double current_d2 = 0;
Encoder wheel2(input_A2, input_B2);

//variables for motor 2 control
int pwm2 = 0;
int dir2 = 0;

//pin assignments for camera and pi
const int camera = 12;
const int topi = 13;

//variables for calculations
double fullroto = 3200;
double r = 2.87;
int prevCount1 = 0;
int currentCount1 = 0;
int prevCount2 = 0;
int currentCount2 = 0;
const double d_between = 13.3;  //13.3< x<13.6
bool ang = false;

void setup() {
  //configuring pins for motors and encoders
  pinMode(M1_speed, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M1_enable, OUTPUT);
  digitalWrite(M1_enable, HIGH);

  pinMode(M2_speed, OUTPUT);
  pinMode(M2_dir, OUTPUT);
  pinMode(M2_enable, OUTPUT);
  digitalWrite(M2_enable, HIGH);

  //configuring pins to communicate to camera and pi
  pinMode(topi, OUTPUT);
  pinMode(camera, INPUT);

  //zero values at start
  current_dtot = 0;
  current_a = 0;
  M1_zero = wheel1.read();
  M2_zero = wheel2.read();

  Serial.begin(31250);
}

void loop() {
  if (!ang) {
    currentCount1 = wheel1.read() * -1;
    currentCount2 = wheel2.read();
    current_d1 = 2 * PI * r * (currentCount1 - M1_zero) * (1 / fullroto);
    current_d2 = 2 * PI * r * (currentCount2 - M2_zero) * (1 / fullroto);
    current_a = ((current_d1 - current_d2) / d_between) * (180 / PI);
    e_a = target_a - current_a;
    e = target_d;
    offset = kpa * e_a;

    if (offset > 0) {
      dir1 = 1;
      dir2 = 0;
    } else {
      dir1 = 0;
      dir2 = 1;
    }

    pwmoffset = abs((offset / 8) * 255);
    if (pwmoffset < 50) pwmoffset = 50;
    pwm1 = round((pwmoffset / 2) * mFudge);
    pwm2 = pwmoffset / 2;
    if (pwm1 > maxPWM*mFudge) {
      pwm1 = round(maxPWM*mFudge);
    }
    if (pwm2 > maxPWM) {
      pwm2 = maxPWM;
    }
    if (abs(e_a) <= 0.1) {
      delay(500);
      ang = true;
      M1_zero = wheel1.read() * -1;
      M2_zero = wheel2.read();
      pwm1 = 0;
      pwm2 = 0;
      current_d1 = 0;
      current_d2 = 0;
    }
  }

  if (ang) {
    currentCount1 = wheel1.read() * -1;
    currentCount2 = wheel2.read();
    current_d1 = 2 * PI * r * (currentCount1 - M1_zero) * (1 / fullroto);
    current_d2 = 2 * PI * r * (currentCount2 - M2_zero) * (1 / fullroto);
    Serial.print(current_d1);
    Serial.print('\t');
    Serial.print(current_d2);
    Serial.print('\t');
    current_dtot = (current_d1 + current_d2) / 2;
    e = target_d - current_dtot;

    //using controller values to get voltage output
    // i = i + ts * e;
    u = kp * e;

    //determine if u value means forwards or backwards
    if (u < 0) {
      dir1 = 0;
    } else {
      dir1 = 1;
    }
    dir2 = dir1;

    //limit pwm to max 255             ISSUES WITH INDIVDUAL PWMS
    

    //redo with individual controllers
    pwm2 = abs(((u / 8) * 255));
    pwm1 = round(pwm2  * mFudge);
    if (pwm1 > maxPWM) {
      pwm1 = round(maxPWM*mFudge);
    }
    if (pwm2 > maxPWM) {
      pwm2 = maxPWM;
    }
  }

  //recalculating
  ts = (millis() / 1000) - tc;
  tc = millis() / 1000;
  prevCount1 = currentCount1;
  prevCount2 = currentCount2;
  analogWrite(M1_speed, pwm1);
  analogWrite(M2_speed, pwm2);
  digitalWrite(M1_dir, dir1);
  digitalWrite(M2_dir, dir2);

  Serial.print(e_a);
  Serial.print("\t");
  Serial.print(e);
  Serial.print("\t");
  Serial.print(target_a);
  Serial.print("\t");
  Serial.print(current_a);
  Serial.print("\t");
  Serial.print(pwm1);
  Serial.print("\t");
  Serial.println(pwm2);
  // Serial.print("\t");
  //Serial.println(current_dtot);
  // Serial.print(currentCount1);
  // Serial.print("\t");
  // Serial.println(currentCount2);
  delay(25);  //GET BETTER DELAY VALUE
}
