Final Demo code. Uses same basic structure as Demo 2 code, except with the states looping to search for the next marker.
The Arduino takes care of keeping track of the marker number variable with the pi requesting I2C communication with the Arduino
every second. After finding the 6 markers, the motor voltage is set to 0.