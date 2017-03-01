#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#import <String.h>
#include "utility/Adafruit_PWMServoDriver.h"
//char parsebuffer;
/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

const float pi = 3.14;

/**************************************************************************/
/*!
 @brief  Initialises all the sensors used by this example
 */
/**************************************************************************/
void moveme(void);
void setzero();
void multipass();


void initSensors()
{
  if(!accel.begin())  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("I DUNNO WHATS HAPPENING TO ME. NO LSM303! \n"));
    }
   }
 


//Motor Shield Initialization
Adafruit_MotorShield shirly = Adafruit_MotorShield(0x60); //Stepper shield
Adafruit_MotorShield bob = Adafruit_MotorShield(0x61); //LED driven as DC motor

Adafruit_StepperMotor *mot_0 = shirly.getStepper(200, 1);
Adafruit_StepperMotor *mot_1 = shirly.getStepper(200, 2);

Adafruit_DCMotor *cam2 = bob.getMotor(1);
Adafruit_DCMotor *cam3 = bob.getMotor(2);
Adafruit_DCMotor *cam0 = bob.getMotor(3);
Adafruit_DCMotor *cam1 = bob.getMotor(4);
Adafruit_DCMotor* CathodeArray[3];

void forward0(){
  mot_0->onestep(FORWARD, MICROSTEP);
}
void backward0(){
  mot_0->onestep(BACKWARD, MICROSTEP);
}
void forward1(){
  mot_1->onestep(FORWARD, MICROSTEP);
}
void backward1(){
  mot_1->onestep(BACKWARD, MICROSTEP);
}

AccelStepper stepper0(forward0, backward0);
AccelStepper stepper1(forward1, backward1);

AccelStepper MotorArray[2]; //Change this to add more motors.
int motorstate[] = {
  0,0,0,0};
void(* reset) (void) = 0; //Reset Function


int index[] = {
  0,0,0,0,0,0  }; //Start, first break, second break, third break, end
String werk = "";
String inData = "";
char received = ' ';
int ledstate = 0;
int blstate = 0;
int num_command = 4;
double MULTI = 16/1.8; //Input degrees, turn into number of steps. Used in movement.
double origin[] = { 
  -4.0, 0.0, 134.5 };
double current_pos[] = {
  0, 0, 0};
//Sensor Vectors, right?
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;

void setup(){
  Serial.begin(115200);
  Serial.println("Serial Begin");
  initSensors();
  shirly.begin();
  bob.begin();
  pinMode(13, OUTPUT);
  MotorArray[0]= stepper0; //Laser Left/Right
  MotorArray[1]= stepper1; //Laser Up/Down
  CathodeArray[0] = cam0; //Webcam Up/Down
  CathodeArray[1] = cam1; //Webcam Left/Right
  CathodeArray[2] = cam2; //Kinect Up/Down
  CathodeArray[3] = cam3; //Kinect Left/Right

  for (int i=0; i < 2; i++){ //Initialize the Steppers, set speed to 250
    MotorArray[i].setMaxSpeed(500.0);
    MotorArray[i].setAcceleration(250.0);
    MotorArray[i].setCurrentPosition(0);
    delay(1);
  }
  for (int i=0; i < 4; i++){ //Initialize the Cathode/DC Motor Array
    CathodeArray[i]->setSpeed(255);
    CathodeArray[i]->run(FORWARD);
    CathodeArray[i]->run(RELEASE);
    delay(1);
  }

  getpos(0); //Get starting position

}
void multipass(int address){
  Serial.print("multipass ran");
//  if (address == 0){
//    digitalWrite(11, LOW);
//    digitalWrite(12, LOW);
//  }
//  if (address == 1){
//    digitalWrite(11,LOW);
//    digitalWrite(12,HIGH);
//  }
//  if (address == 2){
//    digitalWrite(11,HIGH);
//    digitalWrite(12, LOW);
//  }
}
void parse(String inData){
  int n = 0; //n is the working index
  int i=0;
  int len = inData.length()+1;
  char parsebuffer[len];

  inData.toCharArray(parsebuffer,len);
  while ( inData.substring(i,i+1) != "" ) {
    char tchar = parsebuffer[i]; //Move array value to character
    if (tchar == '{' || tchar == ';' || tchar == '}'){
      index[n] = i; //Store the index of the data array.
      n++;
    }	
    i++;
  }
  for (int x = 0; x <= num_command; x++)
  {
    werk = inData.substring(index[x]+1,index[x+1]); //find the value for the xth index of the motostate array
    motorstate[x] = werk.toInt();
  }
  ////////////Data is parsed and stored. Let's use it!
  //release, set values
  moveme(motorstate[0], motorstate[1], motorstate[2], motorstate[3]); //pass parsed values to the moveme function
}

void delta(int heading, int pitch){
  moveme(1, double(heading), double(pitch), 1); 
}

void moveme( int cgroup, int rpos, int ppos, int data){
  //  int cgroup = motorstate[0]; //Motor group. Motors 0-1 are steppers, 2 is cathode 3-5 are DC motors/Cathodes
  //  int rpos = motorstate[1]; //Desired Position for Pitch
  //  int ppos = motorstate[2]; //Desired Position for Roll
  //  int data = motorstate[3]; //Data bit, woo!
  //Recieving { Motornumber ; position ; Speed } in motorstate
  //Reset Current position
  //Neow We're going to determine where we are.
//200/64; // Um, a
  //find out how far away we are from 
  if (cgroup == 1){ //direct differential laser movement
    setzero();       //Reset local Position
    MotorArray[0].moveTo(rpos * MULTI); //assign position
    MotorArray[1].moveTo(-ppos * MULTI); //assign position
  }
  if (cgroup == 2){ //With blocking!
    setzero();
    MotorArray[0].runToNewPosition(rpos * MULTI);
    MotorArray[1].runToNewPosition(-ppos * MULTI);

    //    MotorArray[0].moveTo(rpos*MULTI);
    //    MotorArray[1].moveTo(-ppos*MULTI);
  }
  //Motor Index
  int m;
  if (cgroup == 3 || cgroup == 4){ //DC motor assembly specific
    if (cgroup == 3){//Pan-tilt assembly for webcam, cam0, cam1
      //   Serial.print("Webcam Assembly 1"); 
      m = 0; 
    }
    if (cgroup == 4){
      // Serial.println("Webcam Assembly 2");
      m = 2;
    }
    if (rpos < 0){
      CathodeArray[m]->run(FORWARD);
    }
    if (rpos > 0){
      CathodeArray[m]->run(BACKWARD);
    }
    if (ppos < 0){
      CathodeArray[m+1]->run(FORWARD);
    }
    if (ppos > 0){
      CathodeArray[m+1]->run(BACKWARD);
    }

    if (rpos == 0){
      CathodeArray[m]->run(RELEASE);
    }
    if (ppos == 0){
      CathodeArray[m+1]->run(RELEASE);
    }
    if ( data == 0){
      data = 1;
    }
    delay(data*1000); //wait data (seconds) Default wait time is 1
    for (int i = 0; i < 4; i++){
      CathodeArray[i]->run(RELEASE); //Keep the things under control!
    }
  }
  if (cgroup == 5){
    delta(rpos, ppos);
  }


}



void led(){
  Serial.print("led");
  if (ledstate != 1){
    digitalWrite(13, HIGH);
    ledstate = 1;
  }
  else{
    digitalWrite(13,LOW);
    ledstate = 0;
  }
}
void idle(){
  delay(1);
   mot_0->release();
  mot_1->release();
}
void restart(){
  Serial.println("resetting...");
  reset();

}
void getpos(int v){
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  /* Use the new fusionGetOrientation function to merge accel/mag data */
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    current_pos[1] = orientation.pitch;
    current_pos[0] = orientation.roll;
    current_pos[2] = orientation.heading;
    if ( v == 1){
      //    pitch = orientation.pitch;
      //    roll = orientation.roll;
      //    heading = orientation.heading;
      Serial.println("");
      Serial.print(F("Orientation: r:  "));
      Serial.print(orientation.roll);
      Serial.print(F(" p: "));
      Serial.print(orientation.pitch);
      Serial.print(F(" h: "));
      Serial.print(orientation.heading);

    } 
  }
}


void runsteppers(){
  MotorArray[0].run();
  MotorArray[1].run();
  //delay half a second?
}
void setzero(){
  for (int i=0; i < 2; i++){ //Initialize the motors, set speed to 0
    MotorArray[i].setCurrentPosition(0);

  }
}

void loop(){
  //This part is for the IMU
  while (Serial.available() > 0)
  {
    received = Serial.read();
    //	Serial.println(received);
    inData += received;
    delay(5);
    if (received == '}'){ //If character was the end terminator
      parse(inData);
      //  //escaaaape //Process the command string.
      //	Serial.println(inData);

    }
  delay(5);
  //Process other cases for input.
  //if (inData != ""){
  //  Serial.println("recieved: " + inData);
  //}
  //Serial.print(inData);
  if (inData == "led"){
    led(); 
  }

  if (inData == "setzero"){
    Serial.println("set zero");
    setzero();
  }


  if (inData == "reset"){ //reset function
    restart();
  }
  if (inData == "givepos"){
    getpos(1);
  }

  if (inData == "idle"){
  idle();
  }
 
}

  runsteppers();
 inData = ""; 
}




