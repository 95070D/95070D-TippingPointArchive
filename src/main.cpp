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
<<<<<<< Updated upstream
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
=======
// LeftFront            motor         15              
// LeftBack             motor         17              
// RightFront           motor         18              
// RightBack            motor         20              
// RightLift            motor         21              
// Clamp                motor         4               
// Inertial             inertial      11              
// Controller1          controller                    
// backPiston           digital_out   D               
// LeftMiddle           motor         16              
// RightMiddle          motor         19              
>>>>>>> Stashed changes
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
<<<<<<< Updated upstream
=======
// A global instance of competition
competition Competition;
//Function for determining whether input is positive, negative, or 0

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
=======

/*Our code uses PID, a control loop used to help the robot move efficiently and accurately
without overshooting its target position. PID takes in input based on the sensors
in the V5 Motors and uses a function to output the target speed for the motors. The "P" in PID
stands for proportional. It makes the motors move based on the distance to the target value.
The "I" in PID stands for integral. It calculates how far the motors have already moved to
give it a little push when proportional control cannot get the robot to its final destination.
The "D" in PID stands for derivative. The derivative calculates how fast the robot has been
accelerating and slows it down if it has been accelerating too rapidly. Combined, these
features create a powerful control loop that keeps our robot's performance consistently high.*/
>>>>>>> Stashed changes
 

void TurnClockwisePID (double kP, double kI, double kD, double maxIntegral, double tolerance, double maximumSpeed, double minimumSpeed, double target){
  double error = target;
  double derivitive = 0;
  double integral = 0;
  double LastError=error;
  double total = 0;
  Inertial.setRotation(0, degrees);
<<<<<<< Updated upstream
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
=======
  while(fabs(tolerance)<fabs(error)){
    LeftBack.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    RightFront.spin(forward);
    LeftMiddle.spin(forward);
    RightMiddle.spin(forward);
    double SensorValue = LeftBack.position(turns)*3.25*5/3*M_PI;
    error = target - SensorValue;
    integral = integral + error;
    if(fabs(integral)>fabs(maxIntegral)){
      integral=getSign(integral)*maxIntegral;
    }
    derivative = error-LastError;
    LastError = error;
    total = kP*error + kI*integral + kD*derivative;
    double amountOff = Inertial.rotation(degrees);
    if(-1 < amountOff < 1){
      amountOff = 0;
    }
    if(fabs(total) > fabs(maximumSpeed)){
      LeftBack.setVelocity(getSign(total)*maximumSpeed - 0.5*amountOff, percent);
      RightBack.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
      LeftFront.setVelocity(getSign(total)*maximumSpeed - 0.5*amountOff, percent);
      RightFront.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
      LeftMiddle.setVelocity(getSign(total)*maximumSpeed - 0.5*amountOff, percent);
      RightMiddle.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
    }
    else if(fabs(total) < fabs(minimumSpeed)){
      LeftBack.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightBack.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftFront.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightFront.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftMiddle.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightMiddle.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
    }
    else{
      LeftBack.setVelocity(total - 0.5*amountOff, percent);
      RightBack.setVelocity(total + 0.5*amountOff, percent);
      LeftFront.setVelocity(total - 0.5*amountOff, percent);
      RightFront.setVelocity(total + 0.5*amountOff,percent);
      LeftMiddle.setVelocity(total - 0.5*amountOff, percent);
      RightMiddle.setVelocity(total + 0.5*amountOff,percent);
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

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
=======
  LeftBack.stop(brake);
  RightBack.stop(brake);
  RightFront.stop(brake);
  LeftFront.stop(brake);
  LeftMiddle.stop(brake);
  RightMiddle.stop(brake);
}
//Void that controls the drivetrain based on inputs from the joysticks

int speedFactor = 1;

void platformMode() {
  if(Controller1.ButtonX.pressing()){
    speedFactor = 2;
  }
  else if(Controller1.ButtonB.pressing()){
    speedFactor = 1;
  }
}

void goSlow(){
  if(Controller1.ButtonX.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(forward, 50, percent);
    LeftMiddle.spin(forward, 50, percent);
    RightMiddle.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonB.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(reverse, 50, percent);
    LeftMiddle.spin(reverse, 50, percent);
    RightMiddle.spin(reverse, 50, percent);
  }
  else if(Controller1.ButtonY.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(forward, 50, percent);
    LeftMiddle.spin(reverse, 50, percent);
    RightMiddle.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonA.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(reverse, 50, percent);
    LeftMiddle.spin(forward, 50, percent);
    RightMiddle.spin(reverse, 50, percent);
  }
  else{
    RightBack.stop(hold);
    RightFront.stop(hold);
    LeftBack.stop(hold);
    LeftFront.stop(hold);
    RightMiddle.stop(hold);
    LeftMiddle.stop(hold);
  }
}

void simpleDrive(){
  if(abs(Controller1.Axis2.position(percent))>abs(5)){ 
    RightBack.setVelocity(Controller1.Axis2.position(percent)/speedFactor, percent);
    RightFront.setVelocity(Controller1.Axis2.position(percent)/speedFactor, percent);
    RightMiddle.setVelocity(Controller1.Axis2.position(percent)/speedFactor, percent);    
    RightBack.spin(forward);
    RightFront.spin(forward);
    RightMiddle.spin(forward);
  }
  else{
    RightBack.stop(coast);
    RightFront.stop(coast);
    RightMiddle.stop(coast);
  }
  if(abs(Controller1.Axis3.position(percent))>abs(5)){
    LeftBack.setVelocity(Controller1.Axis3.position(percent)/speedFactor, percent);
    LeftFront.setVelocity(Controller1.Axis3.position(percent)/speedFactor, percent);
    LeftMiddle.setVelocity(Controller1.Axis3.position(percent)/speedFactor, percent);
    LeftBack.spin(forward);
    LeftFront.spin(forward);
    LeftMiddle.spin(forward);
  }
  else{
    LeftBack.stop(coast);
    LeftFront.stop(coast);
    LeftMiddle.stop(coast);
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

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
=======
//Void that controls the movement of the 4-bar lift
void armLift(){
  if (Controller1.ButtonR1.pressing()) {
    RightLift.setVelocity(90, percent);
    RightLift.spin(forward);
  }
  else if (Controller1.ButtonR2.pressing()){
    RightLift.setVelocity(90, percent);
    RightLift.spin(reverse);
  }
  else{
    RightLift.setStopping(hold);
    RightLift.stop();
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
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
  PID(0.9, 0.01, 0.05, 500, 0.5, 90, 40, -22);
  BackLift.spinFor(reverse, 360, degrees, true);
// wait(0.1, sec);
//  BackLift.spinFor(forward, 360, degrees, true);
//  wait(0.1,sec);
//  BackLift.spinFor(reverse, 380, degrees, true);
  PID(0.9, 0.01, 0.05, 500, 0.5, 90, 40, 22);
  BackLift.spinFor(forward, 330, degrees, true);
  while(true){
   simpleDrive();
   armLift();
   hookLift();
   backGoal();
   wait(15, msec);
=======
    Clamp.setVelocity(50, percent);
    Clamp.spin(reverse);
  }
  else{
    Clamp.setStopping(hold);
    Clamp.stop();
  }
}

void turnCounterClockwise(double amount){
  Inertial.setRotation(0, degrees);
  while(fabs(Inertial.rotation(degrees)) < amount){
    double error = amount - fabs(Inertial.rotation(degrees));
    LeftBack.spin(reverse, 0.2*error + 5, percent);
    RightBack.spin(forward, 0.2*error + 5, percent);
    LeftFront.spin(reverse, 0.2*error + 5, percent);
    RightFront.spin(forward, 0.2*error + 5, percent);
    LeftMiddle.spin(reverse, 0.2*error + 5, percent);
    RightMiddle.spin(forward, 0.2*error + 5, percent);    
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftMiddle.setStopping(hold);
  RightMiddle.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  LeftMiddle.stop();
  RightMiddle.stop();
  wait(0.1, sec);
}

void turnClockwise(double amount){
  Inertial.setRotation(0, degrees);
  while(fabs(Inertial.rotation(degrees))< amount){
    double error = amount - fabs(Inertial.rotation(degrees));
    LeftBack.spin(forward, 0.2*error + 5, percent);
    RightBack.spin(reverse, 0.2*error + 5, percent);
    LeftFront.spin(forward, 0.2*error + 5, percent);
    RightFront.spin(reverse, 0.2*error + 5, percent);
    LeftMiddle.spin(forward, 0.2*error + 5, percent);
    RightMiddle.spin(reverse, 0.2*error + 5, percent);
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftMiddle.setStopping(hold);
  RightMiddle.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  LeftMiddle.stop();
  RightMiddle.stop();
  wait(0.1, sec);
}

void backClamp(){
  if(Controller1.ButtonUp.pressing()){
    backPiston.set(true);
    wait(50, msec);
  }
  else if(Controller1.ButtonDown.pressing()){
    backPiston.set(false);
    wait(50, msec);
  }
}

void pre_auton(void) {
 // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  RightLift.stop(hold);
  Inertial.calibrate();
  wait(3, sec);
  Controller1.Screen.print("Ready");
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*  This task is used to control the robot during the autonomous phase of    */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  int a=7;
  switch(a){
    case 1:
      Clamp.setVelocity(20, percent);
      Clamp.spinFor(forward, 90, degrees);
      PID(0.5, 0.01, 0, 500, 0.5, 90, 20, 12);
      Clamp.spin(reverse, 100, percent);
      wait(0.3, sec);
      Clamp.stop(hold);
      PID(1, 0.01, 0, 500, 0.5, 90, 20, -16);
      Clamp.spinFor(forward, 110, degrees);
      break;
    case 2:
      Clamp.spinFor(forward, 60, degrees, false);
      RightLift.spinFor(reverse, 35, degrees, false);
      PID(5, 0.08, 0.01, 500, 0.5, 95, 60, 42);
      wait(10, msec);
      Clamp.spin(reverse, 80, percent);
      wait(0.3, sec);
      Clamp.stop(hold);
      PID(2, 0.01, 0.05, 500, 0.5, 90, 40, -45);
      break;
    case 3:
      Clamp.spinFor(forward, 50, degrees, false);
      RightLift.setVelocity(50, percent);
      PID(5, 0.05, 0.01, 1000, 0.5, 100, 70, 40);
      RightLift.spinFor(reverse, 40, degrees, false);
      Clamp.spin(reverse, 95, percent);
      wait(0.2, sec);
      Clamp.stop(hold);
      wait(10, msec);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 50, -24);
      Clamp.spinFor(forward, 50, degrees);
      wait(10, msec);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 50, -15);
      wait(10, msec);
      turnCounterClockwise(28);
      wait(10, msec);
      PID(2, 0.05, 0.1, 500, 0.5, 95, 60, 56);
      Clamp.spin(reverse, 80, percent);
      wait(0.2, sec);
      Clamp.stop();
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 60, -40);
      wait(10, msec);
      turnClockwise(160);
      wait(10, msec);
      PID(3, 0.05, 0.1, 1000, 0.5, 95, 80, 30);
      Clamp.spinFor(forward, 80, degrees);
      wait(10, msec);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 80, -10);
      turnCounterClockwise(110);
      break;
    case 4:
      Clamp.setVelocity(15, percent);
      Clamp.spinFor(forward, 90, degrees);
      PID(0.75, 0.01, 0.05, 500, 0.5, 90, 20, 16);
      Clamp.spin(reverse, 100, percent);
      wait(2, sec);
      Clamp.spinFor(forward, 80, degrees);
      PID(0.75, 0.01, 0.05, 500, 0.5, 90, 20, -12);
      break;
    case 5:
      backPiston.set(true);
      Clamp.setVelocity(20, percent);
      Clamp.spinFor(forward, 60, degrees, false);
      PID(4, 0, 0, 1000, 0.5, 95, 40, -50);
      backPiston.set(false);
      wait(10, msec);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 40, 28);
      wait(0.25, sec);
      turnCounterClockwise(87);
      wait(10, msec);
      Clamp.spinFor(forward, 30, degrees);
      PID(1, 0.05, 0.1, 500, 0.5, 95, 20, 16);
      Clamp.spin(reverse, 100, percent);
      wait(0.2, sec);
      Clamp.stop(hold);
      wait(0.1, sec);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -40);
      Clamp.spinFor(forward, 80, degrees);
      break;
    case 6:
      backPiston.set(true);
      Clamp.spinFor(reverse, 5, degrees, false);
      PID(4, 0, 0, 1000, 0.5, 95, 80, -51);
      backPiston.set(false);
      wait(10, msec);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 80, 40);
      wait(10, msec);
      turnClockwise(143);
      wait(10, msec);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 80, 56);
      wait(10, msec);
      Clamp.spin(reverse, 80, percent);
      wait(0.2, sec);
      Clamp.stop(hold);
      wait(10, msec);
      RightLift.spinFor(reverse, 35, degrees, false);
      Clamp.spinFor(reverse, 3, degrees, false);
      turnCounterClockwise(5);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 80, -40);
      wait(10, msec);
      turnCounterClockwise(150);
      wait(10, msec);
      RightLift.spinFor(forward, 35, degrees, false);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 90, 16);
      Clamp.spinFor(forward, 80, degrees);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 60, -2);
      break;
    case 7:
      Clamp.spinFor(forward, 80, degrees, false);
      PID(1, 0.05, 0.1, 500, 0.5, 40, 30, 6);
      wait(10, msec);
      RightLift.spinFor(reverse, 105, degrees);
      RightLift.stop(hold);
      Clamp.spin(reverse, 90, percent);
      wait(0.25, sec);
      Clamp.stop(hold);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 50, 40, -12);
      wait(10, msec);
      RightLift.spinFor(forward, 98, degrees);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 50, 40, 15);
      wait(10, msec);
      turnCounterClockwise(89);
      wait(10, msec);
      //Go for first yellow goal
      backPiston.set(true);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 40, 20);
      wait(10, msec);
      turnCounterClockwise(177);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 60, -100);
      backPiston.set(false);
      wait(10, msec);
      turnClockwise(80);
      wait(10, msec);
      //Go for another alliance goal
      PID(1, 0.05, 0.1, 500, 0.5, 90, 20, 12);
      wait(10, msec);
      Clamp.spinFor(forward, 80, degrees);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 40, -12);
      wait(10, msec);
      turnCounterClockwise(30);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 40, 24);
      wait(10, msec);
      Clamp.spin(reverse, 90, percent);
      wait(0.25, sec);
      Clamp.stop(hold);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 40, -24);
      wait(10, msec);
      turnCounterClockwise(48);
      wait(10, msec);
      //Go place on platform
      PID(1, 0.05, 0.1, 500, 0.5, 90, 60, 92);
      wait(10, msec);
      turnCounterClockwise(88);
      wait(10, msec);
      backPiston.set(true);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 60, 29);
      wait(10, msec);
      RightLift.spinFor(reverse, 600, degrees);
      wait(10, msec);
      turnClockwise(88);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 20, 6.5);
      wait(10, msec);
      Clamp.spinFor(forward, 90, degrees);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 20, -5.5);
      wait(10, msec);
      turnClockwise(87);
      wait(10, msec);
      RightLift.spinFor(forward, 590, degrees, false);
      wait(10, msec);
      //Get another alliance + yellow
      PID(1, 0.05, 0.1, 500, 0.5, 90, 60, -63);
      wait(10, msec);
      backPiston.set(false);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 40, 21);
      wait(10, msec);
      turnClockwise(88);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 60, 28);
      wait(10, msec);
      Clamp.spin(reverse, 90, percent);
      wait(0.25, sec);
      Clamp.stop(hold);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 60, -30);
      wait(10, msec);
      //Place on platform
      turnCounterClockwise(88);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 60, 30);
      wait(10, msec);
      RightLift.spinFor(reverse, 600, degrees);
      wait(10, msec);
      turnCounterClockwise(88);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.25, 90, 20, 6);
      wait(10, msec);
      Clamp.spinFor(forward, 80, degrees);
      wait(10, msec);
      PID(1, 0.05, 0.1, 500, 0.5, 90, 40, -70);
      break;
    case 8:
      Clamp.setVelocity(90, percent);
      Clamp.spinFor(forward, 50, degrees, false);
      RightLift.spinFor(reverse, 40, degrees, false);
      PID(5, 0.05, 0.1, 1000, 0.5, 90, 70, 40);
      wait(10, msec);
      Clamp.spin(reverse, 95, percent);
      wait(0.2, sec);
      Clamp.stop(hold);
      PID(3, 0.05, 0.1, 500, 0.5, 90, 50, -24);
      wait(10, msec);
      turnCounterClockwise(88);
      wait(10, msec);
      backPiston.set(true);
      wait(10, msec);
      PID(3, 0.05, 0.1, 500, 0.5, 90, 50, -16);
      wait(10, msec);
      backPiston.set(false);
      wait(10, msec);
      PID(3, 0.05, 0.1, 500, 0.5, 90, 50, 40);
      wait(10, msec);
      Clamp.spinFor(forward, 80, degrees);
      wait(10, msec);
      PID(3, 0.05, 0.1, 500, 0.5, 90, 50, -24);
      wait(10, msec);
      turnClockwise(43);
      wait(10, msec);
      PID(3, 0.05, 0.1, 500, 0.5, 90, 50, 43);
      wait(10, msec);
      Clamp.spin(reverse, 90, percent);
      wait(0.25, sec);
      Clamp.stop(hold);
      PID(3, 0.05, 0.1, 500, 0.5, 90, 50, -42);
  }
}

/*  turnCounterClockwise(145);
      wait(0.5, sec);
      PID(1, 0.05, 0.1, 1000, 0.5, 60, 10, 33);
      Clamp.spinFor(reverse, 75, degrees);
      turnClockwise(90);
      wait(0.5, sec);
      PID(1, 0.05, 0.1, 1000, 0.5, 60, 10, 24);
      wait(0.5, sec);
      turnClockwise(82);
      wait(0.5, sec);
      PID(1, 0.05, 0.1, 1000, 0.5, 60, 10, 75);
      wait(0.5, sec);
      Clamp.spin(forward, 40, percent);
      wait(1, sec);
      Clamp.stop(hold);
      PID(1, 0.05, 0.1, 1000, 0.5, 60, 10, -85);
      wait(0.5, sec);
      Clamp.spinFor(reverse, 90, degrees);*/
/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
 // User control code here, inside the loop
 while (1) {
    Controller1.Screen.print(RightLift.temperature(celsius));
    simpleDrive();
    armLift();
    hookLift();
    backClamp();
    platformMode();
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3,1);
    if(Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()){
      RightLift.stop(hold);
      Clamp.stop(hold);
      while((Controller1.ButtonY.pressing() && Controller1.ButtonA.pressing()) == false){
        goSlow();
        wait(10, msec);
      }
    }
>>>>>>> Stashed changes
  }
}
 


