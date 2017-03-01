#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>


#include "utility/Adafruit_PWMServoDriver.h"
//char parsebuffer;
/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

unsigned long Timer;
/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void moveme(void);
void setzero();
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
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
double pitch = 0.0;
double roll = 0.0;
double heading = 0.0;
int num_command = 4;

double origin[] = { -4.0, -10.5, 134.5 };
double current_pos[] = {0, 0, 0};
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
  MotorArray[0]= stepper0;
  MotorArray[1]= stepper1;
  CathodeArray[0] = cam0; //Webcam
  CathodeArray[1] = cam1; //Webcam
  CathodeArray[2] = cam2;
  CathodeArray[3] = cam3;

  for (int i=0; i < 2; i++){ //Initialize the Steppers, set speed to 250
    MotorArray[i].setMaxSpeed(250.0);
    MotorArray[i].setAcceleration(100.0);
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

void takemehome(){
  moveme(2, origin[0], origin[1], 0);
}

void moveme( int cgroup, int rpos, int ppos, int data){
  //  int cgroup = motorstate[0]; //Motor group. Motors 0-1 are steppers, 2 is cathode 3-5 are DC motors/Cathodes
  //  int rpos = motorstate[1]; //Desired Position for Pitch
  //  int ppos = motorstate[2]; //Desired Position for Roll
  //  int data = motorstate[3]; //Data bit, woo!
  //Recieving { Motornumber ; position ; Speed } in motorstate
  //Reset Current position
  //Neow We're going to determine where we are.
    int MULTI = 200/16; // Um, a
  //find out how far away we are from 
  if (cgroup == 1){ //direct differential laser movement
      idle();       //Reset local Position
      MotorArray[0].moveTo(rpos * MULTI); //assign position
      MotorArray[1].moveTo(-ppos * MULTI); //assign position
  }
  if (cgroup == 2){ //Using 'absolute' coordinates
 
     idle(); //idle motors, clear local orientation
     

       getpos(0); //Where are we approximately?
       double delpitch = ppos - current_pos[1]; //How far away?
       double delroll = rpos - current_pos[0]; //how far away?
        Serial.println("dp: ");// Here, have some numbers.
        Serial.print(delpitch);
        Serial.print(F(" dr: "));
        Serial.print(delroll);
        Serial.print(F(" dp: "));
        Serial.print(delroll);
        Serial.print(F("h: "));
        Serial.print(current_pos[2]);
        Serial.println("This feature is unimplemented");
 //   MotorArray[0].moveTo(delroll*MULTI);
 //   MotorArray[1].moveTo(-delpitch*MULTI);
    }
    //Motor Index
    int m;
  if (cgroup > 2){
   
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

}


//  //Set upper and lower bounds for speed
//  if (cgroup < 2){
//  MotorArray[cgroup].setAcceleration(ppos);
//  MotorArray[cgroup].moveTo(rpos);
//  }
//  if (cgroup > 2){
//   // CathodeArray[cgroup - 2]->setSpeed(ppos); //If we want speed control. These are sloooow motors and the cathode requires 12V and isn't very tolerant. 
//   
//   if (rpos < 0){
//     CathodeArray[cgroup - 2]->run(BACKWARD);
//      }
//   if (rpos > 0){
//     CathodeArray[cgroup-2]->run(FORWARD);
//   }
//   if (rpos == 0){
//     CathodeArray[cgroup-2]->run(RELEASE);
//   }
//  }


void led_toggle(){
     if (ledstate != 1){
       digitalWrite(13, HIGH);
       ledstate = 1;
    }
    else{
       digitalWrite(13,LOW);
       ledstate = 0;
    }
}
void bl_toggle(){
 if (blstate != 1){
       cam2->run(FORWARD);
       blstate = 1;
    }
    else{
       cam2->run(RELEASE);
       blstate = 0;
    }
}
void idle(){
  delay(1);
  setzero();
  mot_0->release();
  mot_1->release();
}
void restart(){
  Serial.println("resetting...");
  blstate = 1;
  ledstate = 1;
  led_toggle();
  bl_toggle();
  idle();

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
 }
//Process other cases for input.
//if (inData != ""){
//  Serial.println("recieved: " + inData);
//}
if (inData == "blacklight"){
   bl_toggle();
}
if (inData == "led"){
  led_toggle(); 
}
if (inData == "setzero"){
  Serial.println("set zero");
  idle();
}


if (inData == "reset"){ //reset function
  restart();
  }
if (inData == "givepos"){
  getpos(1);
}
  
if (inData == "gohome"){
    takemehome();
}
  inData = ""; 




runsteppers();

}



