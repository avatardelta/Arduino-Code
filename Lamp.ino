#include <Adafruit_MotorShield.h>
#include <Wire.h>
#import <String.h>
#include "utility/Adafruit_PWMServoDriver.h"



/**************************************************************************/
/*!
 @brief  Initialises all the sensors used by this example
 */
/**************************************************************************/
//Motor Shield Initialization
Adafruit_MotorShield ledpan = Adafruit_MotorShield(0x60); //LED driven as DC motor


Adafruit_DCMotor *led0 = ledpan.getMotor(1);
Adafruit_DCMotor *led1 = ledpan.getMotor(2);
Adafruit_DCMotor *led2 = ledpan.getMotor(3);
Adafruit_DCMotor *led3 = ledpan.getMotor(4);
Adafruit_DCMotor* LedArray[3];


void setup(){
  Serial.begin(115200);
  Serial.println("Serial Begin");
  ledpan.begin();
//  pinMode(13, OUTPUT);
  LedArray[0] = led0; //Webcam Up/Down
  LedArray[1] = led1; //Webcam Left/Right
  LedArray[2] = led2; //Kinect Up/Down
  LedArray[3] = led3; //Kinect Left/Right

  for (int i=0; i < 4; i++){ //Initialize the Cathode/DC Motor Array
    LedArray[i]->setSpeed(255);
    LedArray[i]->run(FORWARD);
    LedArray[i]->run(RELEASE);
   }
}

void lights(int led,int power){
if (power > 255){ //Cap the power
      power = 255;
}
if (power < 0){ //floor the power
    power = 0;
}
if (led < 0 || led > 4){ //Errors turn it off
  led = 0; // switch address to all leds
  power = 0; //turn off the lights
}


if (led != 0 ){
  LedArray[led - 1]->setSpeed(power);
  LedArray[led - 1]->run(FORWARD);
  if (power == 0){
      LedArray[led - 1]->run(RELEASE);
    }
}
else {
  LedArray[0]->setSpeed(power);
  LedArray[1]->setSpeed(power);
  LedArray[2]->setSpeed(power);
  LedArray[3]->setSpeed(power);
  LedArray[0]->run(FORWARD);
  LedArray[1]->run(FORWARD);
  LedArray[2]->run(FORWARD);
  LedArray[3]->run(FORWARD);
  if (power == 0){
    LedArray[0]->run(RELEASE);
    LedArray[1]->run(RELEASE);
    LedArray[2]->run(RELEASE);
    LedArray[3]->run(RELEASE);
  }

  
}  

}



void parse(String inData){
  String work = inData;
  int ledindex = work.indexOf("L") + 1; //Find the L parameter, give the index of the number following it
  int speedindex = work.indexOf("S") + 1; //find the S parameter, give the index of the number following it.
  String lednum = work.substring(ledindex , ledindex + 1); //Find 1 character after L
  String ledspeed = work.substring(speedindex, speedindex + 3); //find 3 characters after S
  Serial.print(lednum);
  Serial.print(ledspeed);
  lights(lednum.toInt(), ledspeed.toInt());
}

void loop(){
if (Serial.available() > 0)
  {
    String inData = Serial.readStringUntil(';');
    if (inData != " "){ //if the string isn't empty, parse it
    Serial.println(inData);
    parse(inData);
    }
    inData = " ";
  }
}
  






