#include "i2c.h"


int temp_command;

// callback for received data
void receiveData(offsetBit, int& commandVar) {
  while (Wire.available()) {
    temp_command = Wire.read();
    if (temp_command != offsetBit) {
      commandVar = temp_command;
    }
  }
}



// callback for sending data
void sendBlock(byteArray, numBytes) {
  Wire.write(data, numBytes);
}