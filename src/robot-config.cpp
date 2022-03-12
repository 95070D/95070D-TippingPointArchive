#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
<<<<<<< Updated upstream
motor LeftFront = motor(PORT4, ratio18_1, true);
motor LeftBack = motor(PORT16, ratio18_1, true);
motor RightFront = motor(PORT9, ratio18_1, false);
motor RightBack = motor(PORT7, ratio18_1, false);
motor LeftLift = motor(PORT19, ratio36_1, false);
motor RightLift = motor(PORT10, ratio36_1, true);
motor HookArm = motor(PORT13, ratio36_1, false);
motor BackLift = motor(PORT12, ratio36_1, false);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT20);
=======
motor LeftFront = motor(PORT15, ratio18_1, true);
motor LeftBack = motor(PORT17, ratio18_1, true);
motor RightFront = motor(PORT18, ratio18_1, false);
motor RightBack = motor(PORT20, ratio18_1, false);
motor RightLift = motor(PORT21, ratio36_1, true);
motor Clamp = motor(PORT4, ratio36_1, false);
inertial Inertial = inertial(PORT11);
controller Controller1 = controller(primary);
digital_out backPiston = digital_out(Brain.ThreeWirePort.D);
motor LeftMiddle = motor(PORT16, ratio18_1, true);
motor RightMiddle = motor(PORT19, ratio18_1, false);
>>>>>>> Stashed changes

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}