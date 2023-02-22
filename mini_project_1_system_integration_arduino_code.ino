#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

void setup() {
pinMode(13, OUTPUT);
Serial.begin(115200); // start serial for output
// initialize i2c as slave
Wire.begin(SLAVE_ADDRESS);

// define callbacks for i2c communication
Wire.onReceive(receiveData);
Wire.onRequest(sendBlock);
}

// this is the command to direct the motor
int command; 

// replace this with raw counts from motor position
int data_to_send = 0b1011101001; 

byte data[32] = {0};

void loop() {
delay(100);

data[0] = data_to_send >> 8;
data[1] = data_to_send & 0xFF;

}


// callback for received data
void receiveData(int byteCount){

Serial.print("data received: ");
while(Wire.available()) {
  command = Wire.read();
  Serial.println(command);
  //Serial.print(' ');
  }
  //Serial.println(' ');
}



// callback for sending data
void sendBlock(){
  Serial.print("sending data: ");
  Serial.println(data[0]);
  Serial.println(data[1]);
  Wire.write(data, 2); // 2 is number of bytes in data
}
