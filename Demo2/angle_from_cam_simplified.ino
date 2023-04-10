//included libraries
#include <Encoder.h>
#include <Wire.h>

//used for I2C
#define SLAVE_ADDRESS 0x04
uint8_t dataIn[4];
int temp_length;
int data_length;
double temp_command = 0;
bool one_foot = false;
double temp_angle;

//given angle and distance targets
double target_d = 0 * 0.9;
double target_a = 180;

//current angle and position
double current_dtot = 0;
double current_a = 0;
double search_a = 45;

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
double kpa = 0.3;  //0.1 originally
double kia = 0;
double ua = 0;
double offset = 0;
double ts = 0;
double tc = 0;
double e_a = 0;
double search_e = 0;

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
double angle = 90;
bool marker_detected = false;
double read_d = 0;

//variables for calculations
double fullroto = 3200;
double r = 2.87;
int prevCount1 = 0;
int currentCount1 = 0;
int prevCount2 = 0;
int currentCount2 = 0;
const double d_between = 13.3;  //13.3< x<13.6
bool ang = false;
double e_a_n = 0;
double angle_2 = 0;

//variables for camera reading
double pi_a = 0;
bool marker_found = false;
uint8_t State = 0
double temp = 0; 

// for angle read in
double right_d = 0;

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

  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveBlock);
  Wire.onRequest(sendBlock);

  Serial.begin(115200);  // start serial for output
}

void loop() {

  //read in from pi
  switch (State) {
    //seach case, spin in 45 degree turns then stop for moment to look for marker before continuing
    case (0):
      currentCount1 = wheel1.read() * -1;
      currentCount2 = wheel2.read();

      //angle calculations
      current_d1 = 2 * PI * r * (currentCount1 - M1_zero) * (1 / fullroto);
      current_d2 = 2 * PI * r * (currentCount2 - M2_zero) * (1 / fullroto);
      current_a = ((current_d1 - current_d2) / d_between) * (180 / PI);
      search_e = search_a - current_a;
      e = target_d;
      offset = kpa * search_e;

      //determine spin left or right
      if (offset > 0) {
        dir1 = 1;
        dir2 = 0;
      } else {
        dir1 = 0;
        dir2 = 1;
      }

      //setting pwm from offset
      pwmoffset = abs((offset / 8) * 255);
      if (pwmoffset < 50) pwmoffset = 50;
      pwm1 = round((pwmoffset / 2) * mFudge);
      pwm2 = pwmoffset / 2;
      if (pwm1 > maxPWM * mFudge) {
        pwm1 = round(maxPWM * mFudge);
      }
      if (pwm2 > maxPWM) {
        pwm2 = maxPWM;
      }

      //delay during search when within 1 degree
      if (abs(search_e) <= 1.0) {
        // ang = true;
        M1_zero = wheel1.read() * -1;
        M2_zero = wheel2.read();
        pwm1 = 0;
        pwm2 = 0;
        analogWrite(M1_speed, pwm1);
        analogWrite(M2_speed, pwm2);
        digitalWrite(M1_dir, dir1);
        digitalWrite(M2_dir, dir2);
        delay(500);
        current_d1 = 0;
        current_d2 = 0;
      }

      //if marker detected pause robot and send to next state
      if (marker_found) {
        pi_a = angle;  //READ IN FROM PI
        //State = 1;
        M1_zero = wheel1.read() * -1;
        M2_zero = wheel2.read();
        pwm1 = 0;
        pwm2 = 0;
        analogWrite(M1_speed, pwm1);
        analogWrite(M2_speed, pwm2);
        digitalWrite(M1_dir, dir1);
        digitalWrite(M2_dir, dir2);
        delay(100);
        current_d1 = 0;
        current_d2 = 0;
        State = 1;
      }
      break;

    //state to center onto marker
    case (1):
      //recieve initial angle from marker and rotate to that position
      currentCount1 = wheel1.read() * -1;
      currentCount2 = wheel2.read();
      current_d1 = 2 * PI * r * (currentCount1 - M1_zero) * (1 / fullroto);
      current_d2 = 2 * PI * r * (currentCount2 - M2_zero) * (1 / fullroto);
      current_a = ((current_d1 - current_d2) / d_between) * (180 / PI);
      e_a = pi_a - current_a;
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
      if (pwmoffset < 70) pwmoffset = 70;
      pwm1 = round((pwmoffset / 2) * mFudge);
      pwm2 = pwmoffset / 2;
      if (pwm1 > maxPWM * mFudge) {
        pwm1 = round(maxPWM * mFudge);
      }
      if (pwm2 > maxPWM) {
        pwm2 = maxPWM;
      }
      //if within 1 degree slow down rotatiion then stop when locked on and go to state 2
      if (abs(e_a) <= 1) {  //was at .1
        while (angle >= 0.5) {
          pwm1 = 20;
          pwm2 = 20;
          analogWrite(M1_speed, pwm1);
          analogWrite(M2_speed, pwm2);
          digitalWrite(M1_dir, dir1);
          digitalWrite(M2_dir, dir2);
        }
        M1_zero = wheel1.read() * -1;
        M2_zero = wheel2.read();
        pwm1 = 0;
        pwm2 = 0;
        current_a = 0;
        current_d1 = 0;
        current_d2 = 0;
        target_d = read_d;
        State = 2;
      }
      break;

    //go forward to how far marker is, recieved once from pi at start
    case (2):
      currentCount1 = wheel1.read() * -1;
      currentCount2 = wheel2.read();
      current_d1 = 2 * PI * r * (currentCount1 - M1_zero) * (1 / fullroto);
      current_d2 = 2 * PI * r * (currentCount2 - M2_zero) * (1 / fullroto);
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


      //redo with individual controllers
      if (abs(angle) >= 1.0) {
        State = 1;
        M1_zero = wheel1.read() * -1;
        M2_zero = wheel2.read();
        pwm1 = 0;
        pwm2 = 0;
        pi_a = angle;
        current_a = 0;
        current_d1 = 0;
        current_d2 = 0;
      }

      //set pwm from controler
      pwm2 = abs(((u / 8) * 255));
      pwm1 = round(pwm2 * mFudge);
      if (pwm1 > maxPWM) {
        pwm1 = round(maxPWM * mFudge);
      }
      if (pwm2 > maxPWM) {
        pwm2 = maxPWM;
      }

      //if within 1 inch stop
      if (abs(e) < 1.0) {
        State = 4;
      }
      break;

    //stop robot from moving
    case (4):
      M1_zero = wheel1.read() * -1;
      M2_zero = wheel2.read();
      pwm1 = 0;
      pwm2 = 0;
      current_a = 0;
      current_d1 = 0;
      current_d2 = 0;
      target_d = read_d;
      analogWrite(M1_speed, pwm1);
      analogWrite(M2_speed, pwm2);
      digitalWrite(M1_dir, dir1);
      digitalWrite(M2_dir, dir2);
      break;
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
}

// callback for received data
void receiveBlock(int byteCount) {
  angle = 0;
  temp = 0;
  int i = 0;
  while (Wire.available()) {
    if (i == 0) {
      temp_length = Wire.read();
      if (temp_length != 0) {
        data_length = temp_length;
      }
      i++;
    } else {
      uint8_t in = Wire.read();
      dataIn[i - 1] = in;
      uint8_t shift = 8 * (data_length - i);
      temp += (in << shift);
      i++;
    }
  }

  //detmining if data is distance or angle from length of info
  if (data_length == 2) {
    if (one_foot) {
      read_d = (((temp / 100) - 8) * 0.9);
    }
    if (!one_foot) {
      read_d = (((temp / 100) - 3.5) * 0.9);
    }
  }


  else {
    temp_angle = ((temp / 100.0) - 26.75) * -1;
    if (temp_angle <  1) {
      Serial.println("uh oh");
    }else{
      angle = temp_angle;
    }
  }
  marker_found = true;
  Serial.print("data received: ");
  Serial.println(angle);
}


// callback for sending data
void sendBlock() {
  Wire.write(dataIn, data_length);
}
