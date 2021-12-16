/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Mastermind                                       */
/*    Created:      Thu Jul 22 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         4               
// LeftBack             motor         16              
// RightFront           motor         9               
// RightBack            motor         7               
// LeftLift             motor         19              
// RightLift            motor         10              
// HookArm              motor         13              
// BackLift             motor         12              
// Controller1          controller                    
// Inertial             inertial      20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
int getSign (double inputValue) {
 if (inputValue > 0){
 return 1;
 }
 else if (inputValue < 0){
 return -1;
 }
 else return 0;
}

void DriverPID (double target, double kP, double kI, double kD, double maxIntegral, double tolerance, double minSpeed, double maxSpeed, motor inputMotor) {
 double error = target;
 double prevError = error;
 double integral = 0;
 double derivative = 0;
 double total = 0;
 if (fabs(error)>fabs(tolerance)){
 prevError = error;
 error = target - inputMotor.velocity(percent);
 integral = integral += error;
 derivative = prevError-error;
 total = inputMotor.velocity(percent) + error*kP + integral*kI + derivative*kD;
 inputMotor.setVelocity(total, percent);

 if (fabs(total) < fabs(minSpeed)){
 inputMotor.setVelocity(getSign(total)*minSpeed, percent);
 }
 if (fabs(total) > fabs(maxSpeed)){
 inputMotor.setVelocity(getSign(total)* maxSpeed, percent);
 }
 inputMotor.spin(forward);
 }
}

void PID (double kP, double kI, double kD, double maxIntegral, double tolerance, double maximumSpeed, double minimumSpeed, double target){
 double error = target;
 double derivative = 0;
 double integral = 0;
 double LastError=error;
 double total = 0;
 LeftBack.setPosition(0, turns);
 while(fabs(tolerance)<fabs(error)){
  double SensorValue = LeftBack.position(turns)*4*M_PI;
  error = target - SensorValue;
  integral = integral + error;
  if(fabs(integral)>fabs(maxIntegral)){
    integral=getSign(integral)*maxIntegral;
  }
  derivative = error-LastError;
  LastError = error;
  total = kP*error + kI*integral + kD*derivative;
  if(fabs(total) > fabs(maximumSpeed)){
    LeftBack.setVelocity(getSign(total)*maximumSpeed, percent);
    RightBack.setVelocity(getSign(total)*maximumSpeed, percent);
    LeftFront.setVelocity(getSign(total)*maximumSpeed, percent);
    RightFront.setVelocity(getSign(total)*maximumSpeed, percent);
  }

  else if(fabs(total) < fabs(minimumSpeed)){
    LeftBack.setVelocity(getSign(total)*minimumSpeed, percent);
    RightBack.setVelocity(getSign(total)*minimumSpeed, percent);
    LeftFront.setVelocity(getSign(total)*minimumSpeed, percent);
    RightFront.setVelocity(getSign(total)*minimumSpeed, percent);
  }
  else{
    LeftBack.setVelocity(total, percent);
    RightBack.setVelocity(total, percent);
    LeftFront.setVelocity(total, percent);
    RightFront.setVelocity(total,percent);
  }
  LeftBack.spin(forward);
  RightBack.spin(forward);
  LeftFront.spin(forward);
  RightFront.spin(forward);
 }
 LeftBack.setStopping(brake);
 RightBack.setStopping(brake);
 RightFront.setStopping(brake);
 LeftFront.setStopping(brake);
 LeftBack.stop();
 RightBack.stop();
 RightFront.stop();
 LeftFront.stop();
 wait(0.5, sec);
}
 

void TurnClockwisePID (double kP, double kI, double kD, double maxIntegral, double tolerance, double maximumSpeed, double minimumSpeed, double target){
  double error = target;
  double derivitive = 0;
  double integral = 0;
  double LastError=error;
  double total = 0;
  Inertial.setRotation(0, degrees);
  while(tolerance<error){
    double SensorValue = Inertial.rotation(degrees);
    Controller1.Screen.print(SensorValue);
    error = target - SensorValue;
    integral = integral + error;
    if(integral>maxIntegral){
      integral=maxIntegral;
    }
    if(-integral<-maxIntegral){
      integral = -maxIntegral;
    }
    derivitive = error-LastError;
    LastError = error;
    total = kP*error + kI*integral + kD*derivitive;
  if(fabs(total) > fabs(maximumSpeed)){
    LeftBack.setVelocity(getSign(total)*maximumSpeed, percent);
    RightBack.setVelocity(getSign(total)*maximumSpeed, percent);
    LeftFront.setVelocity(getSign(total)*maximumSpeed, percent);
    RightFront.setVelocity(getSign(total)*maximumSpeed, percent);
  }

  else if(fabs(total) < fabs(minimumSpeed)){
    LeftBack.setVelocity(getSign(total)*minimumSpeed, percent);
    RightBack.setVelocity(getSign(total)*minimumSpeed, percent);
    LeftFront.setVelocity(getSign(total)*minimumSpeed, percent);
    RightFront.setVelocity(getSign(total)*minimumSpeed, percent);
  }
  else{
    LeftBack.setVelocity(total, percent);
    RightBack.setVelocity(total, percent);
    LeftFront.setVelocity(total, percent);
    RightFront.setVelocity(total,percent);
  }
  LeftBack.spin(forward);
  RightBack.spin(reverse);
  LeftFront.spin(forward);
  RightFront.spin(reverse);
 }
 LeftBack.setStopping(brake);
 RightBack.setStopping(brake);
 RightFront.setStopping(brake);
 LeftFront.setStopping(brake);
 LeftBack.stop();
 RightBack.stop();
 RightFront.stop();
 LeftFront.stop();
 wait(0.5, sec);
}

void TurnCounterClockwisePID (double kP, double kI, double kD, double maxIntegral, double tolerance, double maximumSpeed, double minimumSpeed, double target){
  double error = target;
  double derivitive = 0;
  double integral = 0;
  double LastError=error;
  double total = 0;
  Inertial.setRotation(0, degrees);
  while(tolerance<error){
    double SensorValue = Inertial.rotation(degrees);
    Controller1.Screen.print(SensorValue);
    error = target - SensorValue;
    integral = integral + error;
    if(integral>maxIntegral){
      integral=maxIntegral;
    }
    if(-integral<-maxIntegral){
      integral = -maxIntegral;
    }
    derivitive = error-LastError;
    LastError = error;
    total = kP*error + kI*integral + kD*derivitive;
  if(fabs(total) > fabs(maximumSpeed)){
    LeftBack.setVelocity(getSign(total)*maximumSpeed, percent);
    RightBack.setVelocity(getSign(total)*maximumSpeed, percent);
    LeftFront.setVelocity(getSign(total)*maximumSpeed, percent);
    RightFront.setVelocity(getSign(total)*maximumSpeed, percent);
  }

  else if(fabs(total) < fabs(minimumSpeed)){
    LeftBack.setVelocity(getSign(total)*minimumSpeed, percent);
    RightBack.setVelocity(getSign(total)*minimumSpeed, percent);
    LeftFront.setVelocity(getSign(total)*minimumSpeed, percent);
    RightFront.setVelocity(getSign(total)*minimumSpeed, percent);
  }
  else{
    LeftBack.setVelocity(total, percent);
    RightBack.setVelocity(total, percent);
    LeftFront.setVelocity(total, percent);
    RightFront.setVelocity(total,percent);
  }
  LeftBack.spin(reverse);
  RightBack.spin(forward);
  LeftFront.spin(reverse);
  RightFront.spin(forward);
 }
 LeftBack.setStopping(brake);
 RightBack.setStopping(brake);
 RightFront.setStopping(brake);
 LeftFront.setStopping(brake);
 LeftBack.stop();
 RightBack.stop();
 RightFront.stop();
 LeftFront.stop();
 wait(0.5, sec);
}

void simpleDrive(){
 LeftBack.setVelocity(Controller1.Axis3.position(percent), percent);
 RightBack.setVelocity(Controller1.Axis2.position(percent), percent);
 LeftFront.setVelocity(Controller1.Axis3.position(percent), percent);
 RightFront.setVelocity(Controller1.Axis2.position(percent), percent);
 LeftBack.spin(forward);
 LeftFront.spin(forward);
 RightBack.spin(forward);
 RightFront.spin(forward);
}
 
void armLift(){
 if (Controller1.ButtonR1.pressing()) {
   LeftLift.setVelocity(40, percent);
   RightLift.setVelocity(40, percent);
   LeftLift.spin(forward);
   RightLift.spin(forward);
 }
 else if (Controller1.ButtonR2.pressing()){
   LeftLift.setVelocity(40, percent);
   RightLift.setVelocity(40, percent);
   LeftLift.spin(reverse);
   RightLift.spin(reverse);
 }
 else{
   RightLift.setStopping(hold);
   LeftLift.setStopping(hold);
   RightLift.stop();
   LeftLift.stop();
 }
}

void hookLift() {
  if(Controller1.ButtonL1.pressing()){
    HookArm.setVelocity(50,percent);
    HookArm.spin(forward);
  }
  else if(Controller1.ButtonL2.pressing()){
    HookArm.setVelocity(50, percent);
    HookArm.spin(reverse);
  }
  else{
    HookArm.setStopping(hold);
    HookArm.stop();
  }
}

void backGoal(){
 if (Controller1.ButtonUp.pressing()){
   BackLift.setVelocity(50, percent);
   BackLift.spin(forward);
 }
 else if (Controller1.ButtonDown.pressing() && BackLift.torque(Nm)<50){
   BackLift.setVelocity(50, percent);
   BackLift.spin(reverse);
 }
 else{
   BackLift.setStopping(hold);
   BackLift.stop();
  }
}
 
 
int main(){
  //Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //(double kP, double kI, double kD, double maxIntegral, double tolerance, double maximumSpeed, double minimumSpeed, double target)
  /*PID(0.9, 0.01, 0.05, 500, 0.5, 90, 40, -22);
  BackLift.spinFor(reverse, 360, degrees, true);
  PID(0.9, 0.01, 0.05, 500, 0.5, 90, 40, 22);
  BackLift.spinFor(forward, 330, degrees, true);*/
  PID(1.5, 0.01, 0.05, 500, 0.5, 90, 40, 55);
  BackLift.spinFor(reverse, 360, degrees, true);
  PID(1.5, 0.01, 0.05, 500, 0.5, 90, 40, -55);
  BackLift.spinFor(forward, 330, degrees, true);  
  
  while(true){
   simpleDrive();
   armLift();
   hookLift();
   backGoal();
   wait(15, msec);
  }
}
 


