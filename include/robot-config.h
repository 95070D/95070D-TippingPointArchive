using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftFront;
extern motor LeftBack;
extern motor RightFront;
extern motor RightBack;
extern motor RightLift;
<<<<<<< Updated upstream
extern motor HookArm;
extern motor BackLift;
extern controller Controller1;
extern inertial Inertial;
=======
extern motor Clamp;
extern inertial Inertial;
extern controller Controller1;
extern digital_out backPiston;
extern motor LeftMiddle;
extern motor RightMiddle;
>>>>>>> Stashed changes

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );