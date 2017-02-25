/*
* CoffeeMate - Arduino controlled self-stabilizing platform, gyro / accelerometer as sensors.
 CopyRight Kjetil Næss

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

The code here is based on stuff you can find on internet, but the Kalman filter implementation is private. You can 
find a good implementation to use here: https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/blob/master/IMU6DOF/MPU6050/Kalman.h
The MPU6050 class is from Jeff Rowbergs library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
*/


#include <avr/io.h>
#include <Arduino.h>
#include <Wire/Wire.h>
#include <util/delay.h>

#include <../I2C/MPU6050/MPU6050.h>
#include <../Servo/Servo.h>
#include "Kalman.h"

//main components of input. sensor and the raw values
MPU6050 mpu6050;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle, accZangle;   // Angle calculate using the accelerometer

//controls XYZ axises. 
Servo servoX,servoY, servoZ;

uint32_t timer;


int main(void) {
  init();
  setup();
  
  while(1) {
    loop();
  }
  
  return 0;
}


void setup() {
  Serial.begin(57600);
  Serial.println("Searching for mpu6050 I2C device...");
  
  sei();

  mpu6050.initialize();
  _delay_ms(50);
  Serial.println("mpu6050 init done");
  Serial.println("Testing device connections...");
  Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  //initialize the sensor readings
  mpu6050.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);

  //initialize the angles for first run. (setting up/calibrating)
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accZangle = (atan2(accX, accY) + PI) * RAD_TO_DEG;

  //initialize timer for calculation of acceleration
  timer = micros();


  //attach the servos to their respective pins for outputs
  servoX.attach(6);
  servoY.attach(7);
  servoZ.attach(8);

  //initialize servo position. 
  servoX.writeMicroseconds(/*FILL IN VALUE*/); 
  servoY.writeMicroseconds(/*FILL IN VALUE*/);
  servoZ.writeMicroseconds(/*FILL IN VALUE*/);

  //give the servos some time to adjust
  delay(5000);  
}


void loop() {
  //gets the raw values from the MPU6050
  mpu6050.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);

  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accZangle = (atan2(accX, accY) + PI) * RAD_TO_DEG;

  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);
  double gyroZrate = -((double)gyroY / 131.0);

  //kalman filter is used to get the angle to move the servos at. 
  //can change to Fast Fourier Transform. 
  //found that a lot of people use kalman filters for their simplicity. 
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); 
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);

  //used to get the acceleration rate to get to the certain angle. 
  //keeps a timer on the movement. 
  timer = micros();

  
  kalAngleX-=180; //Reverse angles for compensation by servos
  kalAngleY-=180;
  kalAngleZ-=180;

  servoY.writeMicroseconds(1410+kalAngleY*18.55);// Angles converted to microsecs and sent as position to servos
  servoX.writeMicroseconds(1400+kalAngleX*18.55); 
  servoZ.writeMicroseconds(1400+kalAngleZ*18.55); 

  //refresh the servos to keep them alive. 
  servoX.refresh();
  servoY.refresh();
}








