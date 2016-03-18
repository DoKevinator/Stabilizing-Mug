#include <avr/io.h>
#include <Arduino.h>
#include <util/delay.h>

#include <MPU6050.h>
#include <Servo.h>
#include "Kalman.h"


#define MOTOR_1_MID 1580
#define MOTOR_2_MID 1350
#define MOTOR_3_MID 1430
#define MOTOR_4_MID 1490
#define MOTOR_BACKUP_MID 1450


MPU6050 mpu6050;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle, accZangle;   // Angle calculate using the accelerometer
double kalAngleX, kalAngleY, kalAngleZ;   // Calculate the angle using a Kalman filter
Kalman kalmanX; 
Kalman kalmanY;
Kalman kalmanZ;

Servo servoX,servoY, servoZ;
Servo servoYmirror;

const int buttonPin = 13;
int buttonState = 0;

int pos_prev, pos_new;

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

  pinMode(buttonPin, INPUT);
  
  Serial.begin(57600);
  Serial.println("Searching for mpu6050 I2C device...");
  
  sei();    //enables interrupts on arduino

  mpu6050.initialize();
  _delay_ms(50);
  Serial.println("mpu6050 init done");
  Serial.println("Testing device connections...");
  Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  mpu6050.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);
  
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accZangle = (atan2(accZ, accX) + PI) * RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  kalmanZ.setAngle(accZangle);
  
  timer = micros();
  
  servoX.attach(3);   //CONNECTS TO BACKUP MOTOR  
  servoY.attach(5);   //CONNECTS TO MOTOR 1
  servoZ.attach(6);   //MOTOR 4
  servoYmirror.attach(9); //CONNECTS TO MOTOR 2
  
  servoX.writeMicroseconds(MOTOR_3_MID); // Position servos at 0 degrees. Value found by experimenting
  servoY.writeMicroseconds(MOTOR_1_MID);
  servoYmirror.writeMicroseconds(MOTOR_2_MID);
  servoZ.writeMicroseconds(MOTOR_4_MID); //TESTING VALUE ONLY. 
  delay(5000);   //delay for 5 seconds to allow setup to complete
}


void loop() {

  buttonState = digitalRead(buttonPin);
  
  mpu6050.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);

  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accZangle = (atan2(accZ,accX) + PI) * RAD_TO_DEG;

  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);
  double gyroZrate = (double)gyroZ / 131.0;

  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); 
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (double)(micros() - timer) / 1000000);
  
  timer = micros();

  kalAngleX -= 180; //Reverse angles for compensation by servos
  kalAngleY -= 180;
  kalAngleZ -= 180;

  pos_prev = pos_new;
  pos_new = MOTOR_3_MID + kalAngleX * 18.55;
  
  if (buttonState == HIGH) {
    servoX.writeMicroseconds(1750); 
    int position = pos_prev;

    //slowly move the mug down to drinking level
    if( position < 2000 ) {
      position += 10;
    } else {
      position = 2000;
    }
    
    servoX.writeMicroseconds(position); 
 } else {
    servoX.writeMicroseconds(MOTOR_3_MID + kalAngleX * 18.55); 
  }

  servoY.writeMicroseconds(MOTOR_1_MID + kalAngleY * 18.55);// Angles converted to microsecs and sent as position to servos
  servoYmirror.writeMicroseconds(2000 - (MOTOR_2_MID + kalAngleY * 18.55) + 1500);
  servoZ.writeMicroseconds(MOTOR_4_MID + kalAngleZ * 18.55);
}
