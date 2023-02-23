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

// 0.00	150	1940	0.00
// 0.00	150	1959	0.00
// 0.00	150	1979	0.00
// 0.00	150	1999	0.00
// 0.00	150	1000	0.00
// 0.00	150	1001	0.00
// 0.00	150	1002	0.00
// 0.00	150	1003	0.00
// 0.01	150	1011	0.73
// 0.05	150	1031	2.06
// 0.11	150	1051	3.03
// 0.19	150	1071	4.01
// 0.29	150	1090	5.15
// 0.40	150	1109	5.77
// 0.51	150	1130	5.59
// 0.64	150	1149	6.49
// 0.76	150	1169	6.36
// 0.90	150	1189	6.66
// 1.03	150	1209	6.85
// 1.17	150	1228	7.31
// 1.31	150	1249	6.71
// 1.46	150	1268	7.52
// 1.60	150	1288	7.14
// 1.74	150	1307	7.52
// 1.89	150	1328	6.90
// 2.03	150	1347	7.62
// 2.18	150	1367	7.24
// 2.32	150	1387	7.24
// 2.47	150	1406	7.73
// 2.61	150	1426	7.24
// 2.76	150	1445	7.73
// 2.90	150	1466	6.90
// 3.05	150	1485	7.73
// 3.20	150	1505	7.24
// 3.35	150	1525	7.44
// 3.49	150	1545	7.34
// 3.64	150	1564	7.73
// 3.79	150	1585	6.99
// 3.93	150	1604	7.83
// 4.08	150	1624	7.34
// 4.23	150	1643	7.83
// 4.38	150	1663	7.34
// 4.52	150	1683	7.34
// 4.67	150	1702	7.73
// 4.82	150	1723	7.08
// 4.97	150	1742	7.83
// 5.12	150	1762	7.54
// 5.27	150	1782	7.54
// 5.42	150	1802	7.54
// 5.57	150	1821	7.73
// 5.71	150	1841	7.34
// 5.86	150	1861	7.34
// 6.01	150	1881	7.44
// 6.16	150	1900	7.93
// 6.31	150	1921	7.08
// 6.46	150	1940	7.93
// 6.61	150	1959	7.83
// 6.75	150	1979	7.34
// 6.90	150	1999	7.44
