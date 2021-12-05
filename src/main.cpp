// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         4               
// LeftBack             motor         7               
// RightFront           motor         8               
// RightBack            motor         16              
// LeftLift             motor         6               
// HookArm              motor         9               
// BackLift             motor         20              
// Controller1          controller                    
// Inertial             inertial      19              
// RightLift            motor         10              
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
int main(){
<<<<<<< Updated upstream
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
  }
=======
  Inertial.calibrate();
  wait(2.5, sec);
  PID(2, 0.05, 0.2, 1000, 0.5, 90, 50, -54);
  BackLift.spinFor(reverse, 330, degrees);
  PID(1.5, 0.05, 0.1, 1000, 0.5, 90, 50, 36);
  BackLift.spinFor(forward, 330, degrees);
  PID(2, 0.05, 0.1, 1000, 0.5, 90, 50, 18);  
  turnCounterClockwise(25);
  PID(1.5, 0.05, 0.1, 1000, 0.5, 90, 50, -63.8122);
  BackLift.spinFor(reverse, 330, degrees);
  turnCounterClockwise(116.586);
  PID(2, 0.05, 0.1, 1000, 0.5, 90, 50, -38.58756);
  BackLift.spinFor(forward, 330, degrees);
  turnClockwise(118.5);
  PID(1.5, 0.05, 0.1, 1000, 0.5, 90, 50, -38.58756);
  BackLift.spinFor(reverse, 330, degrees);
  turnCounterClockwise(148);
  PID(1.5, 0.05, 0.1, 1000, 0.5, 90, 50, -33);
  BackLift.spinFor(forward, 330, degrees);
  turnClockwise(90);
  PID(3, 0.05, 0.1, 1000, 0.5, 90, 50, -24);
  turnClockwise(82);
  PID(1.5, 0.05, 0.1, 1000, 0.5, 90, 50, -75);
  BackLift.spinFor(reverse, 345, degrees);
  PID(1.5, 0.05, 0.1, 1000, 0.5, 90, 50, 85);
  BackLift.spinFor(forward, 330, degrees);
>>>>>>> Stashed changes
}
