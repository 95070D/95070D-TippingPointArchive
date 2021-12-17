#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         4               
// LeftBack             motor         7               
// RightFront           motor         8               
// RightBack            motor         16              
// LeftLift             motor         6               
// RightLift            motor         10              
// Clamp                motor         9               
// Forklift             motor         20              
// Inertial             inertial      19              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----
using namespace vex;
// A global instance of competition
competition Competition;
//Function for determining whether input is positive, negative, or 0
int getSign (double inputValue) {
  if (inputValue > 0){
    return 1;
  }
  else if (inputValue < 0){
    return -1;
  }
  else return 0;
}
/*Our code uses PID, a control loop used to help the robot move efficiently and accurately
without overshooting its target position. PID takes in input based on the sensors
in the V5 Motors and uses a function to output the target speed for the motors. The "P" in PID
stands for proportional. It makes the motors move based on the distance to the target value.
The "I" in PID stands for integral. It calculates how far the motors have already moved to
give it a little push when proportional control cannot get the robot to its final destination.
The "D" in PID stands for derivative. The derivative calculates how fast the robot has been
accelerating and slows it down if it has been accelerating too rapidly. Combined, these
features create a powerful control loop that keeps our robot's performance consistently high.*/
 
//PID to make the robot drive a certain distance during the autonomous period
void PID (double kP, double kI, double kD, double maxIntegral, double tolerance, double maximumSpeed, double minimumSpeed, double target){
  double error = target;
  double derivative = 0;
  double integral = 0;
  double LastError=error;
  double total = 0;
  LeftBack.setPosition(0, turns);
  Inertial.setRotation(0, degrees);
  while(fabs(tolerance)<fabs(error)){
    LeftBack.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    RightFront.spin(forward);
    double SensorValue = LeftBack.position(turns)*5.6*M_PI;
    error = target - SensorValue;
    integral = integral + error;
    if(fabs(integral)>fabs(maxIntegral)){
      integral=getSign(integral)*maxIntegral;
    }
    derivative = error-LastError;
    LastError = error;
    total = kP*error + kI*integral + kD*derivative;
    if(fabs(total) > fabs(maximumSpeed)){
      LeftBack.setVelocity(getSign(total)*maximumSpeed - 0.5*Inertial.rotation(degrees), percent);
      RightBack.setVelocity(getSign(total)*maximumSpeed + 0.5*Inertial.rotation(degrees), percent);
      LeftFront.setVelocity(getSign(total)*maximumSpeed - 0.5*Inertial.rotation(degrees), percent);
      RightFront.setVelocity(getSign(total)*maximumSpeed + 0.5*Inertial.rotation(degrees), percent);
    }
    else if(fabs(total) < fabs(minimumSpeed)){
      LeftBack.setVelocity(getSign(total)*minimumSpeed - 0.5*Inertial.rotation(degrees), percent);
      RightBack.setVelocity(getSign(total)*minimumSpeed + 0.5*Inertial.rotation(degrees), percent);
      LeftFront.setVelocity(getSign(total)*minimumSpeed - 0.5*Inertial.rotation(degrees), percent);
      RightFront.setVelocity(getSign(total)*minimumSpeed + 0.5*Inertial.rotation(degrees), percent);
    }
    else{
      LeftBack.setVelocity(total - 0.5*Inertial.rotation(degrees), percent);
      RightBack.setVelocity(total + 0.5*Inertial.rotation(degrees), percent);
      LeftFront.setVelocity(total - 0.5*Inertial.rotation(degrees), percent);
      RightFront.setVelocity(total + 0.5*Inertial.rotation(degrees),percent);
    }
  }
  LeftBack.setStopping(brake);
  RightBack.setStopping(brake);
  RightFront.setStopping(brake);
  LeftFront.setStopping(brake);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
}
//Void that controls the drivetrain based on inputs from the joysticks
void simpleDrive(){
  if(abs(Controller1.Axis2.position(percent))>abs(5)){ 
    RightBack.setVelocity(Controller1.Axis2.position(percent), percent);
    RightFront.setVelocity(Controller1.Axis2.position(percent), percent);
    RightBack.spin(forward);
    RightFront.spin(forward);
  }
  else{
    RightBack.stop(coast);
    RightFront.stop(coast);
  }
  if(abs(Controller1.Axis3.position(percent))>abs(5)){
    LeftBack.setVelocity(Controller1.Axis3.position(percent), percent);
    LeftFront.setVelocity(Controller1.Axis3.position(percent), percent);
    LeftBack.spin(forward);
    LeftFront.spin(forward);
  }
  else{
    LeftBack.stop(coast);
    LeftFront.stop(coast);
  }
}
//Void that controls the movement of the 4-bar lift
void armLift(){
  if (Controller1.ButtonR1.pressing()) {
    LeftLift.setVelocity(90, percent);
    RightLift.setVelocity(90, percent);
    RightLift.spin(forward);
    LeftLift.spin(forward);
  }
  else if (Controller1.ButtonR2.pressing()){
    LeftLift.setVelocity(90, percent);
    RightLift.setVelocity(90, percent);
    LeftLift.spin(reverse);
    RightLift.spin(reverse);
  }
  else{
    LeftLift.setStopping(hold);
    RightLift.setStopping(hold);
    LeftLift.stop();
    RightLift.stop();
  }
}
//Void that controls movement of the hook grabber at the end of the 6-bar
void hookLift() {
  if(Controller1.ButtonL1.pressing()){
    Clamp.setVelocity(50,percent);
    Clamp.spin(forward);
  }
  else if(Controller1.ButtonL2.pressing()){
    Clamp.setVelocity(50, percent);
    Clamp.spin(reverse);
  }
  else{
    Clamp.setStopping(hold);
    Clamp.stop();
  }
}
//Void that controls movement of the goal manipulator at the back of the robot
void backGoal(){
  if (Controller1.ButtonDown.pressing()){
    Forklift.setVelocity(95, percent);
    Forklift.spin(forward);
  }
  else if (Controller1.ButtonUp.pressing() && Forklift.torque(Nm)<2){
    Forklift.setVelocity(95, percent);
    Forklift.spin(reverse);
  }
  else{
    Forklift.setStopping(hold);
    Forklift.stop();
  }
}
void turnCounterClockwise(double amount){
  Inertial.setRotation(0, degrees);
  while(fabs(Inertial.rotation(degrees)) < amount){
    double error = amount - fabs(Inertial.rotation(degrees));
    LeftBack.spin(reverse, 0.25*error + 5, percent);
    RightBack.spin(forward, 0.25*error + 5, percent);
    LeftFront.spin(reverse, 0.25*error + 5, percent);
    RightFront.spin(forward, 0.25*error + 5, percent);
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  wait(0.5, sec);
}

void turnClockwise(double amount){
  Inertial.setRotation(0, degrees);
  while(fabs(Inertial.rotation(degrees))< amount){
    double error = amount - fabs(Inertial.rotation(degrees));
    LeftBack.spin(forward, 0.25*error + 5, percent);
    RightBack.spin(reverse, 0.25*error + 5, percent);
    LeftFront.spin(forward, 0.25*error + 5, percent);
    RightFront.spin(reverse, 0.25*error + 5, percent);
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  wait(0.5, sec);
}
void pre_auton(void) {
 // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertial.calibrate();
  RightLift.stop(hold);
  LeftLift.stop(hold);
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*  This task is used to control the robot during the autonomous phase of    */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  int a=6;
  switch(a){
    case 1:
      PID(1, 0.01, 0, 500, 0.5, 90, 20, 20);
      Clamp.spin(forward, 40, percent);
      wait(0.5, sec);
      Clamp.stop(hold);
      Clamp.spinFor(reverse, 45, degrees);
      PID(2, 0.01, 0, 500, 0.5, 90, 40, -16);
      wait(0.5, sec);
      turnClockwise(175);
      Forklift.spinFor(forward, 720, degrees);
      wait(0.5, sec);
      PID(2, 0.01, 0, 500, 0.5, 90, 40, -18);
      wait(0.5, sec);
      Forklift.spinFor(reverse, 270, degrees);
      PID(2, 0.01, 0, 500, 0.5, 90, 40, 22);
      break;
    case 2:
      Clamp.spinFor(forward, 40, degrees, false);
      PID(4, 0.01, 0.01, 500, 0.5, 95, 40, 43);
      Clamp.spin(forward, 80, percent);
      wait(0.25, sec);
      Clamp.stop(hold);
      PID(2, 0.01, 0.05, 500, 0.5, 90, 40, -43);
      Clamp.spinFor(reverse, 60, degrees, true);
      break;
    case 3:
      Clamp.spinFor(forward, 40, degrees,false);
      PID(4, 0, 0, 1000, 0.5, 95, 40, 43);
      Clamp.spin(forward, 80, percent);
      wait(0.25, sec);
      Clamp.stop(hold);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -30);
      Clamp.spinFor(reverse, 90, degrees);
      wait(0.5, sec);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -9);
      wait(0.5, sec);
      turnCounterClockwise(30);
      PID(1, 0.05, 0.1, 500, 0.5, 95, 20, 58);
      Clamp.spin(forward, 80, percent);
      wait(0.75, sec);
      Clamp.stop();
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -35);
      break;
    case 4:
      PID(0.75, 0.01, 0.05, 500, 0.5, 90, 20, 24);
      Clamp.spin(forward, 40, percent);
      wait(1, sec);
      Clamp.spinFor(reverse, 60, degrees);
      break;
    case 5:
      break;
    case 6:
      Clamp.spinFor(forward, 20, degrees,false);
      PID(4, 0.01, 0.01, 1000, 0.5, 95, 20, 43);
      Clamp.spin(forward, 80, percent);
      wait(0.4, sec);
      Clamp.stop(hold);
      Forklift.spinFor(forward, 1050, degrees, false);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -20);
      LeftLift.spin(reverse, 25, percent);
      RightLift.spin(reverse, 25, percent);
      wait(0.25, sec);
      LeftLift.stop(hold);
      RightLift.stop(hold);
      turnClockwise(125);
      wait(0.25, sec);
      Clamp.spinFor(reverse, 40, degrees);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -35);
      Forklift.spinFor(reverse, 270, degrees, false);
      wait(0.5, sec);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 20, -45);
      wait(0.5, sec);
      turnClockwise(55);
      wait(0.5, sec);
      Forklift.spinFor(forward, 270, degrees);
      wait(0.25, sec);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 20, 30);
      Clamp.spinFor(forward, 35, degrees);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 20, 30);      
    case 7:
      Clamp.spinFor(forward, 20, degrees,false);
      PID(4, 0.01, 0.01, 1000, 0.5, 95, 20, 43);
      Clamp.spin(forward, 80, percent);
      wait(0.4, sec);
      Clamp.stop(hold);
      Forklift.spinFor(forward, 1050, degrees, false);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -20);
      LeftLift.spin(reverse, 25, percent);
      RightLift.spin(reverse, 25, percent);
      wait(0.25, sec);
      LeftLift.stop(hold);
      RightLift.stop(hold);
      turnClockwise(125);
      wait(0.25, sec);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -35);
      Forklift.spinFor(reverse, 540, degrees, false);
      LeftLift.spin(reverse, 25, percent);
      RightLift.spin(reverse, 25, percent);
      wait(0.5, sec);
      LeftLift.stop(hold);
      RightLift.stop(hold);
      wait(0.5, sec);
      PID(2, 0.05, 0.1, 1000, 0.5, 95, 20, 20);
      turnCounterClockwise(160);
      PID(1, 0.05, 0.1, 1000, 0.5, 95, 20, -20);
      break;
    case 8:
      PID(2, 0.05, 0.05, 500, 0.5, 95, 20, 21);
      Clamp.spin(forward, 80, percent);
      wait(0.4, sec);
      Clamp.stop(hold);
      turnClockwise(45);
      PID(2, 0.05, 0.05, 500, 0.5, 95, 20, -93);
      turnCounterClockwise(90);
      Forklift.spinFor(reverse, 540, degrees);
      break;
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
    Controller1.Screen.print(LeftBack.temperature(percent));
    simpleDrive();
    armLift();
    hookLift();
    backGoal();
    Controller1.Screen.clearLine(3);
    Controller1.Screen.setCursor(3,1);
    if(Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()){
      RightBack.stop(hold);
      RightFront.stop(hold);
      LeftBack.stop(hold);
      LeftFront.stop(hold);
      Forklift.stop(hold);
      RightLift.stop(hold);
      LeftLift.stop(hold);
      Clamp.stop(hold);
      while((Controller1.ButtonY.pressing() && Controller1.ButtonA.pressing()) == false){
        wait(1, sec);
      }
    }
  }
    wait(15, msec);
   // Sleep the task for a short amount of time to prevent wasted resources.
}

int main() {
 // Set up callbacks for autonomous and driver control periods.
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);
 pre_auton();
 // Prevent main from exiting with an infinite loop.
 while (true) {
   wait(100, msec);
 }
}
