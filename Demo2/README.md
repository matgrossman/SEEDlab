Demo 2 code for both the Arduino and Raspberry Pi. The Arduino starts by moving the robot in 45 degree angles with a set delay.
This continues until the Pi Camera detects a marker. It then calculates the angle offset for the Arduino and sends angle and distance.
The Arduino then moves the robot to center the robot on the marker and then get an accurate distance from the Pi. The robot then moves either on top of the marker or 1 foot 
away from the marker depending on a boolean statement.