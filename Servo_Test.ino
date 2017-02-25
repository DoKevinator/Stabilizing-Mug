#include <Servo.h>

 Servo myservo;

void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 2 to the servo object 
} 

void loop() 
{  
  myservo.writeMicroseconds(1000);                  
  delay(1000);                       
  
  myservo.writeMicroseconds(1500);                  
  delay(1000); 

  myservo.writeMicroseconds(2000);                  
  delay(1000); 
                         
  
} 
