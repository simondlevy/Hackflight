This repository contains a simple Arduino library for bare-bones interaction with the Invensense MPU6050
Inertial Measurement Unit (IMU).  Unlike more sophisticated MPU6050 
[libraries](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050), this one has just two 
methods: 
<ul>
<li> begin(), allowing you to specify the acceleromter and gyroscope ranges
<p><li>getMotion6Counts(), wich outputs the raw X,Y,Z values for the accelerometer and gyroscope
</ul>

I have tested this library with an Arduino Uno, Teensy 3.2, and Teensy 3.6.
