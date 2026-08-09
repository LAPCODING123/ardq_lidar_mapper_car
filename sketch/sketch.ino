#include <Servo.h>
#include <Wire.h>
#include "LIDARLite.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "mapper_car.h"
#include <Arduino_RouterBridge.h>


// #define SWEEPER_PIN 3
// #define TRUE_ZERO_SERV_DEG 7
// #define TRUE_NINETY_SERV_DEG 85
// #define TRUE_ONEEIGHTY_SERV_DEG 166


//Servo sweeper; //180 degree servo

// LIDARLite lidar;

// int pos = 0;

// int bias_degrees = 0;

 MapperCar robocar;


void setup() {
  // put your setup code here, to run once:
 // sweeper.attach(SWEEPER_PIN);
  Serial.begin(9600);
  
 // sweeper.write(166);  
  // lidar.begin(0, true);
  // lidar.configure(0);
  robocar.init();

}

void loop() {
  robocar.telemeter_health();
  delay(500);
  
  // Serial.print("Bias corrected measurment: ");
  // Serial.println(lidar.distance());


  // for(int i = 0; i < 99; i++)
  // {
  //   Serial.print("Non bias corrected measurements: ");
  //   Serial.println(lidar.distance(false));
  // }
  
  // put your main code here, to run repeatedly:

 // for (pos = 0; pos <= 90; pos += 1) { // goes from 0 degrees to 180 degrees
 //    // in steps of 1 degree
 //    sweeper.write(pos);              // tell servo to go to position in variable 'pos'
 //    delay(15);                       // waits 15ms for the servo to reach the position
 //  }
 //  Serial.println("Servo: 90 deg");

 //  delay(2000);

 //  for (pos = 90; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
 //    // in steps of 1 degree
 //    sweeper.write(pos);              // tell servo to go to position in variable 'pos'
 //    delay(15);                       // waits 15ms for the servo to reach the position
 //  }
 //  Serial.println("Servo: 180 deg");
 //  delay(2000);
  
 //  for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
 //    sweeper.write(pos);              // tell servo to go to position in variable 'pos'
 //    delay(15);                       // waits 15ms for the servo to reach the position
 //  }

 //  Serial.println("Servo: 90 deg");
 //    delay(2000);

 //   for (pos = 90; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
 //    sweeper.write(pos);              // tell servo to go to position in variable 'pos'
 //    delay(15);                       // waits 15ms for the servo to reach the position
 //  }
 //  Serial.println("Servo: 0 deg");
 //    delay(2000);
}
